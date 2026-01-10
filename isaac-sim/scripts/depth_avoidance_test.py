#!/usr/bin/env python3
"""
Depth Estimation and Avoidance Navigation Test.

Two modes:
1. Ground-truth mode (default): Uses Isaac Sim's depth buffer directly
   for isolated navigation/avoidance algorithm testing without perception noise.

2. Full pipeline mode (--with-ros): Uses actual ROS 2 nodes (flyby_depth +
   local_avoidance) for end-to-end integration testing of the complete stack.

Features:
- Spawns pillar_field scenario (8 massive pillars)
- Ground-truth or ROS 2 depth estimation
- VFH+ obstacle avoidance (simulated or via ROS 2 node)
- Records video with split-screen HUD overlay
- Logs navigation metrics

Usage:
    # Ground-truth depth (isolated nav test)
    ./scripts/depth_avoidance_test.py --scenario pillar_field --headless

    # Full ROS 2 pipeline (integration test)
    ./scripts/depth_avoidance_test.py --scenario pillar_field --with-ros

Author: Finley Holt
"""

from isaacsim import SimulationApp

import sys
import argparse

# Parse args before SimulationApp
parser = argparse.ArgumentParser(description="Depth + Avoidance Navigation Test")
parser.add_argument("--scenario", type=str, default="pillar_field",
                    help="Scenario name (default: pillar_field)")
parser.add_argument("--headless", action="store_true", default=True,
                    help="Run headless (default)")
parser.add_argument("--gui", action="store_true",
                    help="Run with GUI (overrides --headless)")
parser.add_argument("--with-ros", action="store_true",
                    help="Use ROS 2 nodes for depth/avoidance (full pipeline test)")
parser.add_argument("--output-dir", type=str, default="/workspace/output/depth_avoidance_tests",
                    help="Output directory for results")
parser.add_argument("--max-steps", type=int, default=3000,
                    help="Maximum simulation steps")
parser.add_argument("--save-benchmark", action="store_true",
                    help="Save benchmark dataset for offline testing")
parser.add_argument("--benchmark-interval", type=int, default=30,
                    help="Frame interval for benchmark capture (default: every 30 frames)")
args = parser.parse_args()

HEADLESS = not args.gui

simulation_config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 0,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(simulation_config)

import omni
import omni.timeline
from omni.isaac.core.world import World
from pxr import UsdGeom, UsdPhysics, UsdLux, UsdShade, Sdf, Gf, PhysxSchema
import omni.physx
from isaacsim.sensors.camera import Camera
import time
import numpy as np
import os
import json
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Tuple
from scipy.spatial.transform import Rotation
import cv2

# Add workspace to path
sys.path.insert(0, "/workspace")

from environments.obstacle_world import ObstacleWorldGenerator, NavigationScenario
from environments.depth_avoidance_hud import (
    DepthAvoidanceHUD,
    DepthData,
    AvoidanceData,
    TelemetryData,
)
from collections import deque
from enum import Enum, auto

# Import MVP scale corrector (simplified warmup-then-freeze approach)
from mvp_scale_corrector import WarmupScaleCorrector

# Import trajectory rollout planner (proactive path planning)
from trajectory_planner import TrajectoryRolloutPlanner, TrajectoryPlanResult

# ROS 2 imports (optional - for actual ROS integration)
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Range
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    print("[Warning] ROS 2 not available - running in standalone mode", flush=True)


# =============================================================================
# GROUND PLANE FILTERING
# RANSAC-based ground plane detection to prevent ground from appearing as obstacle
# =============================================================================


class GroundFilter:
    """RANSAC-based ground plane filtering for depth maps.

    Detects and filters ground plane pixels to prevent them from
    triggering obstacle avoidance when camera is tilted downward.
    """

    def __init__(
        self,
        inlier_threshold: float = 0.5,
        min_ground_normal_z: float = 0.8,
        ransac_iterations: int = 50,
        sample_stride: int = 8,
    ):
        """Initialize ground filter.

        Args:
            inlier_threshold: Distance threshold for RANSAC inliers (meters)
            min_ground_normal_z: Minimum z-component of ground plane normal
            ransac_iterations: Number of RANSAC iterations
            sample_stride: Pixel stride for sparse sampling (performance)
        """
        self.inlier_threshold = inlier_threshold
        self.min_normal_z = min_ground_normal_z
        self.ransac_iterations = ransac_iterations
        self.sample_stride = sample_stride

        # Cached ground plane for temporal consistency
        self._last_plane = None
        self._plane_age = 0
        self._max_plane_age = 10  # Re-estimate every N frames

    def filter_ground(
        self,
        depth_map: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        camera_pitch_deg: float,
    ) -> Tuple[np.ndarray, Optional[Tuple[float, float, float, float]]]:
        """Filter ground plane from depth map.

        Args:
            depth_map: Metric depth map (H, W) in meters
            fx, fy: Focal lengths in pixels
            cx, cy: Principal point in pixels
            camera_pitch_deg: Camera pitch angle in degrees (negative = looking down)

        Returns:
            (filtered_depth, plane_coeffs) - depth with ground set to inf, plane (a,b,c,d) or None
        """
        h, w = depth_map.shape

        # Convert to 3D points (sparse sampling for speed)
        rows, cols = np.mgrid[0:h:self.sample_stride, 0:w:self.sample_stride]
        depths = depth_map[rows, cols]
        valid = (depths > 0.5) & (depths < 100)

        if valid.sum() < 100:
            return depth_map, None  # Not enough points

        # Project to camera frame (z = depth along optical axis)
        z = depths[valid]
        x = (cols[valid] - cx) * z / fx
        y = (rows[valid] - cy) * z / fy

        # Rotate by camera pitch to approximate world frame
        # In camera frame: +Y is down, +Z is forward
        # After pitch rotation: Y becomes vertical component
        pitch = np.radians(camera_pitch_deg)
        cos_p, sin_p = np.cos(pitch), np.sin(pitch)

        # Rotate around X axis (pitch)
        y_world = y * cos_p - z * sin_p
        z_world = y * sin_p + z * cos_p

        points = np.column_stack([x, y_world, z_world])  # (N, 3)

        # RANSAC plane fitting
        best_plane = self._ransac_plane(points)

        if best_plane is None:
            self._last_plane = None
            return depth_map, None

        # Check if plane is approximately horizontal (ground-like)
        # In world frame, Y is up, so normal should have |n_y| > threshold
        a, b, c, d = best_plane
        if abs(b) < self.min_normal_z:  # b is y-component of normal
            self._last_plane = None
            return depth_map, None

        # Cache valid plane
        self._last_plane = best_plane
        self._plane_age = 0

        # Apply plane filter at full resolution
        filtered = self._apply_plane_filter(
            depth_map, best_plane, fx, fy, cx, cy, camera_pitch_deg
        )

        return filtered, best_plane

    def _ransac_plane(self, points: np.ndarray) -> Optional[Tuple[float, float, float, float]]:
        """RANSAC plane fitting.

        Args:
            points: (N, 3) array of 3D points

        Returns:
            Plane coefficients (a, b, c, d) where ax + by + cz + d = 0, or None
        """
        n_points = len(points)
        if n_points < 3:
            return None

        best_plane = None
        best_inlier_count = 0

        for _ in range(self.ransac_iterations):
            # Sample 3 random points
            idx = np.random.choice(n_points, 3, replace=False)
            p1, p2, p3 = points[idx]

            # Compute plane normal via cross product
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal = normal / norm

            # Plane equation: normal . (p - p1) = 0
            # ax + by + cz + d = 0 where d = -normal . p1
            d = -np.dot(normal, p1)
            plane = (normal[0], normal[1], normal[2], d)

            # Count inliers
            distances = np.abs(np.dot(points, normal) + d)
            inlier_count = np.sum(distances < self.inlier_threshold)

            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_plane = plane

        return best_plane

    def _apply_plane_filter(
        self,
        depth_map: np.ndarray,
        plane: Tuple[float, float, float, float],
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        camera_pitch_deg: float,
    ) -> np.ndarray:
        """Apply plane filter to full resolution depth map.

        Args:
            depth_map: Metric depth map (H, W)
            plane: Plane coefficients (a, b, c, d)
            fx, fy, cx, cy: Camera intrinsics
            camera_pitch_deg: Camera pitch in degrees

        Returns:
            Filtered depth map with ground pixels set to infinity
        """
        h, w = depth_map.shape
        a, b, c, d = plane

        # Create coordinate grids
        rows, cols = np.mgrid[0:h, 0:w]

        # Project all pixels to 3D
        z = depth_map
        x = (cols - cx) * z / fx
        y = (rows - cy) * z / fy

        # Rotate to world frame
        pitch = np.radians(camera_pitch_deg)
        cos_p, sin_p = np.cos(pitch), np.sin(pitch)
        y_world = y * cos_p - z * sin_p
        z_world = y * sin_p + z * cos_p

        # Distance to plane
        distances = np.abs(a * x + b * y_world + c * z_world + d)

        # Mark ground as infinity
        ground_mask = (distances < self.inlier_threshold) & (depth_map > 0.5)
        filtered = depth_map.copy()
        filtered[ground_mask] = np.inf

        return filtered


# =============================================================================
# 3D VECTOR FIELD HISTOGRAM
# Enhanced VFH with elevation bins for vertical escape directions
# =============================================================================


class DepthFieldNavigator:
    """Depth field-based navigation that analyzes the actual depth image.

    Instead of hardcoded steering values, this class:
    1. Analyzes depth image columns to find obstacles and openings
    2. Identifies the best path (largest opening toward goal)
    3. Computes steering to aim at that opening
    4. Continuously re-evaluates based on current depth field

    The depth field IS the navigation - we steer toward where depth shows clear space.

    Includes hysteresis to prevent oscillation between similarly-scored openings.
    """

    def __init__(
        self,
        hfov_deg: float = 160.0,
        vfov_deg: float = 80.0,
        safety_dist: float = 8.0,
        num_sectors: int = 72,
        hysteresis_bonus: float = 15.0,
        min_switch_improvement: float = 10.0,
    ):
        """Initialize depth field navigator.

        Args:
            hfov_deg: Horizontal field of view in degrees
            vfov_deg: Vertical field of view in degrees
            safety_dist: Minimum safe distance to consider "blocked" (meters)
            num_sectors: Number of angular sectors to analyze in depth image
            hysteresis_bonus: Score bonus for staying with previous opening choice
            min_switch_improvement: Minimum score improvement to switch openings
        """
        self.hfov = np.radians(hfov_deg)
        self.vfov = np.radians(vfov_deg)
        self.hfov_deg = hfov_deg
        self.safety_dist = safety_dist
        self.num_sectors = num_sectors

        # Hysteresis parameters
        self.hysteresis_bonus = hysteresis_bonus
        self.min_switch_improvement = min_switch_improvement
        self._prev_opening_idx: Optional[int] = None
        self._prev_heading: Optional[float] = None

        # Precompute sector angles (from left to right in image)
        # Left edge = +hfov/2, right edge = -hfov/2 (camera convention)
        self.sector_angles = np.linspace(
            self.hfov / 2, -self.hfov / 2, num_sectors
        )
        self.sector_width = self.hfov / num_sectors

    def compute_avoidance(
        self,
        depth_map: np.ndarray,
        goal_direction: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> AvoidanceData:
        """Compute avoidance by analyzing the depth field directly.

        This method:
        1. Samples depth along horizontal sectors
        2. Finds the minimum depth (closest obstacle) in each sector
        3. Identifies clear sectors where depth > safety_dist
        4. Finds the best opening that leads toward goal
        5. Returns heading to steer toward center of that opening

        No hardcoded steering magnitudes - steering is derived from where
        the depth field shows clear space.

        Args:
            depth_map: Metric depth map (H, W) in meters
            goal_direction: 3D unit vector toward goal in camera frame
            fx, fy: Focal lengths in pixels
            cx, cy: Principal point in pixels

        Returns:
            AvoidanceData with heading derived from depth field analysis
        """
        h, w = depth_map.shape

        # Sample the middle portion of the image (avoid ground in lower part)
        # Use vertical band from 30% to 70% of image height
        y_start = int(h * 0.30)
        y_end = int(h * 0.70)
        depth_band = depth_map[y_start:y_end, :]

        # Compute minimum depth for each horizontal sector
        sector_min_depths = np.zeros(self.num_sectors)
        sector_width_pixels = w / self.num_sectors

        for i in range(self.num_sectors):
            x_start = int(i * sector_width_pixels)
            x_end = int((i + 1) * sector_width_pixels)
            sector_slice = depth_band[:, x_start:x_end]

            # Get minimum depth in this sector (closest obstacle)
            valid = sector_slice[(sector_slice > 0.5) & (sector_slice < 200)]
            if len(valid) > 0:
                sector_min_depths[i] = np.min(valid)
            else:
                sector_min_depths[i] = 200.0  # No valid depth = assume clear

        # Find blocked sectors (obstacle closer than safety distance)
        blocked = sector_min_depths < self.safety_dist
        blocked_count = int(blocked.sum())
        closest_dist = float(sector_min_depths.min())

        # Determine goal sector from goal direction
        # goal_direction is in camera frame: +X forward, +Y left, +Z up
        goal_azimuth = np.arctan2(-goal_direction[1], goal_direction[0])

        # Map goal azimuth to sector index
        # sector_angles goes from +hfov/2 (left, index 0) to -hfov/2 (right, index n-1)
        goal_sector = int((self.hfov / 2 - goal_azimuth) / self.sector_width)
        goal_sector = np.clip(goal_sector, 0, self.num_sectors - 1)

        # Check if goal direction is blocked
        goal_blocked = blocked[goal_sector]

        # Also check sectors around goal (±3 sectors for early warning)
        adjacent_blocked = False
        for adj in range(-3, 4):
            adj_sector = goal_sector + adj
            if 0 <= adj_sector < self.num_sectors and blocked[adj_sector]:
                adjacent_blocked = True
                break

        path_blocked = goal_blocked or adjacent_blocked

        # Find best opening in depth field
        recommended_heading = 0.0

        if path_blocked:
            # Analyze the depth field to find the best opening
            # An "opening" is a contiguous run of unblocked sectors

            # Find all openings (contiguous clear sectors)
            openings = []  # List of (start_sector, end_sector, avg_depth, center_angle)
            in_opening = False
            start_sector = 0

            for i in range(self.num_sectors):
                if not blocked[i]:
                    if not in_opening:
                        start_sector = i
                        in_opening = True
                else:
                    if in_opening:
                        # End of opening
                        end_sector = i - 1
                        opening_sectors = range(start_sector, end_sector + 1)
                        avg_depth = sector_min_depths[start_sector:end_sector + 1].mean()
                        center_idx = (start_sector + end_sector) / 2
                        center_angle = self.sector_angles[int(center_idx)]
                        width = end_sector - start_sector + 1
                        openings.append((start_sector, end_sector, avg_depth, center_angle, width))
                        in_opening = False

            # Handle opening that extends to right edge
            if in_opening:
                end_sector = self.num_sectors - 1
                avg_depth = sector_min_depths[start_sector:end_sector + 1].mean()
                center_idx = (start_sector + end_sector) / 2
                center_angle = self.sector_angles[int(center_idx)]
                width = end_sector - start_sector + 1
                openings.append((start_sector, end_sector, avg_depth, center_angle, width))

            if openings:
                # Score each opening based on:
                # 1. Width (wider is better)
                # 2. Proximity to goal direction (closer is better)
                # 3. Depth (deeper is better - more room)
                # 4. Hysteresis bonus for staying with previous choice
                goal_angle = self.sector_angles[goal_sector]
                opening_scores = []

                for idx, opening in enumerate(openings):
                    start_s, end_s, avg_d, center_a, width = opening

                    # Angular distance from goal
                    angle_diff = abs(center_a - goal_angle)

                    # Base score: prioritize width, penalize angle deviation
                    score = (
                        width * 10.0  # Wider openings are much better
                        + avg_d * 0.5  # Deeper is slightly better
                        - angle_diff * 20.0  # Penalize deviation from goal
                    )

                    # Hysteresis bonus: prefer previous opening to prevent oscillation
                    if self._prev_opening_idx is not None and idx == self._prev_opening_idx:
                        score += self.hysteresis_bonus

                    opening_scores.append((score, idx, opening))

                # Sort by score (highest first)
                opening_scores.sort(reverse=True, key=lambda x: x[0])
                best_score, best_idx, best_opening = opening_scores[0]

                # Apply hysteresis: only switch if improvement is significant
                if self._prev_opening_idx is not None and best_idx != self._prev_opening_idx:
                    # Check if previous opening still exists
                    prev_entries = [(s, i, o) for s, i, o in opening_scores
                                    if i == self._prev_opening_idx]
                    if prev_entries:
                        prev_entry = prev_entries[0]
                        prev_score_without_bonus = prev_entry[0] - self.hysteresis_bonus

                        # Only switch if new opening is significantly better
                        if best_score < prev_score_without_bonus + self.min_switch_improvement:
                            # Stay with previous opening
                            best_opening = prev_entry[2]
                            best_idx = self._prev_opening_idx

                # Update state
                self._prev_opening_idx = best_idx

                if best_opening:
                    _, _, _, center_angle, _ = best_opening
                    # Steer toward the center of the best opening
                    recommended_heading = center_angle
                    self._prev_heading = center_angle

                    print(f"[DepthNav] Found opening: center={np.degrees(center_angle):.1f}° "
                          f"width={best_opening[4]} sectors, depth={best_opening[2]:.1f}m", flush=True)
            else:
                # No horizontal openings - all sectors blocked
                # Would need vertical escape (not implemented in this version)
                print("[DepthNav] All sectors blocked - no horizontal escape", flush=True)
        else:
            # Path is clear - head toward goal
            # But if we're still near an obstacle, blend toward opening
            if closest_dist < self.safety_dist * 2:
                # Find the clearest direction and blend toward it
                clearest_sector = int(np.argmax(sector_min_depths))
                clearest_angle = self.sector_angles[clearest_sector]
                goal_angle = self.sector_angles[goal_sector]

                # Blend: weight toward clearest direction based on proximity
                blend = 1.0 - (closest_dist / (self.safety_dist * 2))
                blend = np.clip(blend, 0, 0.5)  # Max 50% blend toward clear
                recommended_heading = goal_angle * (1 - blend) + clearest_angle * blend

        # Safety level classification based on depth field
        if closest_dist > 20:
            safety = 0  # CLEAR
        elif closest_dist > 10:
            safety = 1  # CAUTION
        elif closest_dist > 5:
            safety = 2  # WARNING
        else:
            safety = 3  # CRITICAL

        return AvoidanceData(
            closest_obstacle_distance=closest_dist,
            path_blocked=path_blocked,
            recommended_heading=recommended_heading,
            recommended_pitch=0.0,  # Vertical escape not yet implemented
            recommendation_confidence=0.9 if not path_blocked else 0.75,
            blocked_bins=blocked_count,
            total_bins=self.num_sectors,
            candidate_directions=self.num_sectors - blocked_count,
            safety_level=safety,
            processing_time_ms=1.0,
        )



# =============================================================================
# TACTICAL PLANNER STATE MACHINE
# Orchestrates navigation with recovery behaviors
# =============================================================================


class TacticalState(Enum):
    """States for the tactical planner."""
    NAVIGATE = auto()       # Normal goal-directed flight
    AVOID = auto()          # Active obstacle avoidance
    STUCK = auto()          # Detected stuck condition
    RECOVER_CLIMB = auto()  # Altitude increase to escape
    RECOVER_ORBIT = auto()  # Rotate in place to scan
    RECOVER_REVERSE = auto()  # Back up from dead end


@dataclass
class TacticalCommand:
    """Command output from tactical planner."""
    heading: float = 0.0      # Heading offset in radians
    pitch: float = 0.0        # Pitch angle in degrees
    speed: float = 8.0        # Target speed m/s
    vertical: bool = False    # Whether this is a vertical maneuver
    rotate_only: bool = False # Whether to only rotate (no translation)


class OscillationDetector:
    """Detects oscillation patterns indicating stuck behavior."""

    def __init__(
        self,
        reversal_threshold: int = 4,
        window_sec: float = 2.0,
        hz: float = 30.0,
    ):
        """Initialize oscillation detector.

        Args:
            reversal_threshold: Number of reversals to trigger stuck
            window_sec: Time window for reversal counting
            hz: Update frequency
        """
        self.history = deque(maxlen=int(window_sec * hz))
        self.reversal_threshold = reversal_threshold

    def update(self, heading_cmd: float) -> bool:
        """Update with new heading command and check for oscillation.

        Args:
            heading_cmd: Heading command in radians

        Returns:
            True if oscillation detected
        """
        if abs(heading_cmd) < 0.01:
            # Ignore near-zero commands
            return False

        self.history.append(np.sign(heading_cmd))

        if len(self.history) < 10:
            return False

        # Count sign changes (direction reversals)
        signs = np.array(self.history)
        reversals = int(np.sum(np.abs(np.diff(signs)) > 0))

        return reversals >= self.reversal_threshold

    def reset(self):
        """Reset oscillation history."""
        self.history.clear()


class TacticalPlanner:
    """State machine for tactical navigation with recovery behaviors.

    The depth field navigator handles the actual steering decisions by
    analyzing the depth image. This planner just manages state transitions
    and recovery when the navigator can't find any openings.
    """

    def __init__(
        self,
        orbit_steps: int = 36,
        backup_distance: float = 5.0,
        climb_timeout_frames: int = 300,
        clear_frames_threshold: int = 5,
    ):
        """Initialize tactical planner.

        Args:
            orbit_steps: Steps for full orbit during recovery
            backup_distance: Distance to reverse during recovery (meters)
            climb_timeout_frames: Frames before giving up on climb
            clear_frames_threshold: Consecutive clear frames to exit avoidance
        """
        self.state = TacticalState.NAVIGATE
        self.orbit_steps = orbit_steps
        self.backup_distance = backup_distance
        self.climb_timeout = climb_timeout_frames
        self.clear_threshold = clear_frames_threshold

        # State tracking
        self.clear_frames = 0
        self.climb_frames = 0
        self.orbit_step = 0
        self.reverse_start = None
        self.recovery_attempts = 0
        self.last_state = None
        self.oscillation_detector = OscillationDetector()

    def update(
        self,
        avoidance_data: AvoidanceData,
        position: np.ndarray,
        goal: np.ndarray,
    ) -> TacticalCommand:
        """Update tactical planner and return command.

        Args:
            avoidance_data: Current avoidance sensor data
            position: Current position [x, y, z]
            goal: Goal position [x, y, z]

        Returns:
            TacticalCommand with heading, pitch, speed
        """
        # Log state transitions
        if self.state != self.last_state:
            print(f"[TACTICAL] State: {self.last_state} -> {self.state}", flush=True)
            self.last_state = self.state

        # Compute goal heading
        to_goal = goal - position
        goal_heading = np.arctan2(to_goal[1], to_goal[0])

        if self.state == TacticalState.NAVIGATE:
            return self._handle_navigate(avoidance_data, goal_heading)

        elif self.state == TacticalState.AVOID:
            return self._handle_avoid(avoidance_data, goal_heading)

        elif self.state == TacticalState.STUCK:
            return self._handle_stuck(position)

        elif self.state == TacticalState.RECOVER_CLIMB:
            return self._handle_recover_climb(avoidance_data)

        elif self.state == TacticalState.RECOVER_ORBIT:
            return self._handle_recover_orbit(avoidance_data, position)

        elif self.state == TacticalState.RECOVER_REVERSE:
            return self._handle_recover_reverse(avoidance_data, position)

        return TacticalCommand()

    def _handle_navigate(
        self, avoidance_data: AvoidanceData, goal_heading: float
    ) -> TacticalCommand:
        """Handle NAVIGATE state.

        The depth field navigator continuously analyzes the depth image.
        We simply follow its recommended heading - it steers toward openings.
        """
        if avoidance_data.path_blocked:
            self.state = TacticalState.AVOID
            self.clear_frames = 0

        # Always use the depth navigator's recommended heading
        # It continuously analyzes the depth field and steers toward openings
        speed = self._compute_safety_speed(avoidance_data)
        return TacticalCommand(
            heading=avoidance_data.recommended_heading,
            pitch=0,
            speed=speed
        )

    def _handle_avoid(
        self, avoidance_data: AvoidanceData, goal_heading: float
    ) -> TacticalCommand:
        """Handle AVOID state.

        The depth field navigator analyzes the depth image and returns
        a heading toward the best opening. We just follow it.
        """
        # Check if path is clear
        if not avoidance_data.path_blocked:
            self.clear_frames += 1
            if self.clear_frames >= self.clear_threshold:
                self.state = TacticalState.NAVIGATE
        else:
            self.clear_frames = 0

            # Only go to STUCK if there are literally no openings in the depth field
            if avoidance_data.candidate_directions == 0:
                print("[TACTICAL] No openings found in depth field - transitioning to STUCK", flush=True)
                self.state = TacticalState.STUCK
                return TacticalCommand(heading=0, pitch=0, speed=0)

        # Follow the depth navigator's recommendation
        speed = self._compute_safety_speed(avoidance_data)
        return TacticalCommand(
            heading=avoidance_data.recommended_heading,
            pitch=np.degrees(avoidance_data.recommended_pitch),
            speed=speed,
            vertical=abs(avoidance_data.recommended_pitch) > 0.01,
        )

    def _handle_stuck(self, position: np.ndarray) -> TacticalCommand:
        """Handle STUCK state - transition to recovery."""
        self.state = TacticalState.RECOVER_CLIMB
        self.climb_frames = 0
        self.recovery_attempts += 1
        print(f"[TACTICAL] Recovery attempt #{self.recovery_attempts}", flush=True)
        return TacticalCommand(heading=0, pitch=0, speed=0)

    def _handle_recover_climb(self, avoidance_data: AvoidanceData) -> TacticalCommand:
        """Handle RECOVER_CLIMB state - climb to escape."""
        self.climb_frames += 1

        # Success - path is now clear
        if not avoidance_data.path_blocked:
            print("[TACTICAL] Climb successful - path clear", flush=True)
            self.state = TacticalState.NAVIGATE
            self.recovery_attempts = 0
            self.oscillation_detector.reset()
            return TacticalCommand(heading=0, pitch=0, speed=4.0)

        # Timeout - try orbit instead
        if self.climb_frames >= self.climb_timeout:
            print("[TACTICAL] Climb timeout - trying orbit", flush=True)
            self.state = TacticalState.RECOVER_ORBIT
            self.orbit_step = 0
            return TacticalCommand(heading=0, pitch=0, speed=0)

        # Continue climbing
        return TacticalCommand(heading=0, pitch=15, speed=3.0, vertical=True)

    def _handle_recover_orbit(
        self, avoidance_data: AvoidanceData, position: np.ndarray
    ) -> TacticalCommand:
        """Handle RECOVER_ORBIT state - rotate to find opening."""
        self.orbit_step += 1

        # Success - found clear path
        if not avoidance_data.path_blocked:
            print("[TACTICAL] Orbit successful - path clear", flush=True)
            self.state = TacticalState.NAVIGATE
            self.recovery_attempts = 0
            self.oscillation_detector.reset()
            return TacticalCommand(heading=0, pitch=0, speed=4.0)

        # Completed full orbit - try reverse
        if self.orbit_step >= self.orbit_steps:
            print("[TACTICAL] Orbit complete - trying reverse", flush=True)
            self.state = TacticalState.RECOVER_REVERSE
            self.reverse_start = position.copy()
            return TacticalCommand(heading=0, pitch=0, speed=0)

        # Continue rotating (10 deg/step)
        return TacticalCommand(
            heading=np.radians(10),
            pitch=0,
            speed=0,
            rotate_only=True,
        )

    def _handle_recover_reverse(
        self, avoidance_data: AvoidanceData, position: np.ndarray
    ) -> TacticalCommand:
        """Handle RECOVER_REVERSE state - back up from obstacle."""
        # Success - path is now clear
        if not avoidance_data.path_blocked:
            print("[TACTICAL] Reverse successful - path clear", flush=True)
            self.state = TacticalState.NAVIGATE
            self.recovery_attempts = 0
            self.oscillation_detector.reset()
            return TacticalCommand(heading=0, pitch=0, speed=4.0)

        # Check if we've backed up enough
        if self.reverse_start is not None:
            dist_reversed = np.linalg.norm(position[:2] - self.reverse_start[:2])
            if dist_reversed >= self.backup_distance:
                print(f"[TACTICAL] Reversed {dist_reversed:.1f}m - retrying", flush=True)
                self.state = TacticalState.STUCK  # Will trigger another recovery cycle
                return TacticalCommand(heading=0, pitch=0, speed=0)

        # Continue reversing
        return TacticalCommand(heading=np.pi, pitch=0, speed=2.0)

    def _compute_safety_speed(self, avoidance_data: AvoidanceData) -> float:
        """Compute speed based on safety level."""
        if avoidance_data.safety_level >= 3:  # CRITICAL
            return 2.0
        elif avoidance_data.safety_level >= 2:  # WARNING
            return 4.0
        elif avoidance_data.safety_level >= 1:  # CAUTION
            return 6.0
        return 8.0

    @property
    def state_name(self) -> str:
        """Get current state name for logging."""
        return self.state.name


class SimpleTacticalPlanner:
    """
    Minimal 2-state planner for MVP with heading smoothing and oscillation detection.

    Simplified alternative to TacticalPlanner that only handles NAVIGATE and AVOID.
    Includes:
    - EMA smoothing on heading to prevent jitter
    - Rate limiting to prevent physically impossible turn rates
    - Oscillation detection with commitment to break deadlocks
    """

    NAVIGATE = 0
    AVOID = 1

    def __init__(
        self,
        clear_frames: int = 3,
        heading_alpha: float = 0.4,
        max_heading_rate: float = 0.5,
        oscillation_threshold: float = 0.3,
        commitment_duration: int = 30,
    ):
        """
        Initialize simple planner with smoothing.

        Args:
            clear_frames: Consecutive clear frames needed to exit AVOID state
            heading_alpha: EMA smoothing factor (0=no change, 1=instant)
            max_heading_rate: Maximum heading change per frame (rad/frame)
            oscillation_threshold: Std dev of heading history indicating oscillation
            commitment_duration: Frames to hold heading when oscillation detected
        """
        self.state = self.NAVIGATE
        self._clear_count = 0
        self._clear_threshold = clear_frames

        # Heading smoothing
        self.heading_alpha = heading_alpha
        self.max_heading_rate = max_heading_rate
        self._smoothed_heading: Optional[float] = None

        # Oscillation detection
        self._heading_history: List[float] = []
        self._history_size: int = 30  # 1 second at 30Hz
        self._oscillation_threshold = oscillation_threshold
        self._committed_heading: Optional[float] = None
        self._commitment_frames: int = 0
        self._commitment_duration = commitment_duration

    @property
    def state_name(self) -> str:
        """Get current state name for logging."""
        return "NAVIGATE" if self.state == self.NAVIGATE else "AVOID"

    def _detect_oscillation(self) -> bool:
        """Detect if we're oscillating (high heading variance)."""
        if len(self._heading_history) < self._history_size:
            return False
        recent = self._heading_history[-self._history_size:]
        return float(np.std(recent)) > self._oscillation_threshold

    def _apply_smoothing(self, raw_heading: float) -> float:
        """Apply EMA smoothing and rate limiting to heading."""
        if self._smoothed_heading is None:
            return raw_heading

        # EMA smoothing
        smoothed = self.heading_alpha * raw_heading + (1 - self.heading_alpha) * self._smoothed_heading

        # Rate limiting
        delta = smoothed - self._smoothed_heading
        if abs(delta) > self.max_heading_rate:
            smoothed = self._smoothed_heading + np.sign(delta) * self.max_heading_rate

        return smoothed

    def update(self, avoidance_data: AvoidanceData) -> TacticalCommand:
        """
        Update planner state and return navigation command.

        Args:
            avoidance_data: Current avoidance analysis from depth navigator

        Returns:
            TacticalCommand with heading, speed, and flags
        """
        raw_heading = avoidance_data.recommended_heading

        # Track heading history for oscillation detection
        self._heading_history.append(raw_heading)
        if len(self._heading_history) > self._history_size * 2:
            self._heading_history.pop(0)

        # Determine smoothed heading
        if self._commitment_frames > 0:
            # Currently committed - hold heading
            self._commitment_frames -= 1
            smoothed_heading = self._committed_heading
            if self._commitment_frames % 10 == 0:
                print(f"[SimplePlanner] COMMITTED: holding {np.degrees(smoothed_heading):.1f}° "
                      f"({self._commitment_frames} frames left)", flush=True)
        elif self._detect_oscillation():
            # Start commitment
            self._committed_heading = self._smoothed_heading if self._smoothed_heading else raw_heading
            self._commitment_frames = self._commitment_duration
            smoothed_heading = self._committed_heading
            print(f"[SimplePlanner] OSCILLATION DETECTED: committing to {np.degrees(smoothed_heading):.1f}°",
                  flush=True)
        else:
            # Normal smoothing
            smoothed_heading = self._apply_smoothing(raw_heading)

        self._smoothed_heading = smoothed_heading

        # State machine logic
        if self.state == self.NAVIGATE:
            if avoidance_data.path_blocked:
                self.state = self.AVOID
                self._clear_count = 0
                print(f"[SimplePlanner] NAVIGATE -> AVOID (path blocked)", flush=True)

            return TacticalCommand(
                heading=smoothed_heading,
                pitch=0.0,
                speed=8.0,
                vertical=False,
                rotate_only=False,
            )

        else:  # AVOID state
            if not avoidance_data.path_blocked:
                self._clear_count += 1
                if self._clear_count >= self._clear_threshold:
                    self.state = self.NAVIGATE
                    print(f"[SimplePlanner] AVOID -> NAVIGATE (path clear)", flush=True)
            else:
                self._clear_count = 0

            # Slower speed during avoidance based on safety level
            if avoidance_data.safety_level >= 3:  # CRITICAL
                speed = 2.0
            elif avoidance_data.safety_level >= 2:  # WARNING
                speed = 4.0
            else:  # CAUTION
                speed = 6.0

            return TacticalCommand(
                heading=smoothed_heading,
                pitch=avoidance_data.recommended_pitch,
                speed=speed,
                vertical=False,
                rotate_only=False,
            )

    def reset(self):
        """Reset planner state."""
        self.state = self.NAVIGATE
        self._clear_count = 0
        self._smoothed_heading = None
        self._heading_history.clear()
        self._committed_heading = None
        self._commitment_frames = 0


# =============================================================================
# ROS DEPTH MODE COMPONENTS
# Implements Depth Anything V2 with rangefinder calibration for --with-ros mode
# Based on Gremsy VIO payload specifications (co-located camera + rangefinder)
# =============================================================================


class FixedGimbal:
    """Fixed gimbal at navigation tilt - no calibration sweeps.

    Keeps camera at -10° pitch (slight downward look to see obstacles ahead).
    The rangefinder will hit ground/obstacles in front of the drone,
    providing continuous scale calibration data to the ScaleCorrector.

    Based on Gremsy VIO co-located camera + rangefinder design.
    """

    NAV_TILT = -15.0  # Slight downward pitch to see obstacles ahead

    def __init__(self, camera_prim_path: str, stage):
        """Initialize fixed gimbal.

        Args:
            camera_prim_path: USD path to camera prim
            stage: USD stage reference
        """
        self.camera_path = camera_prim_path
        self.stage = stage
        self.current_tilt = self.NAV_TILT
        self.mode = "NAVIGATION"  # Always navigation

        # Get camera orient op for physical rotation
        camera_prim = stage.GetPrimAtPath(camera_prim_path)
        self._orient_op = None
        if camera_prim.IsValid():
            xform = UsdGeom.Xformable(camera_prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    self._orient_op = op
                    break
            if self._orient_op is None:
                self._orient_op = xform.AddOrientOp()

        # Set initial orientation
        self._update_camera_orientation()
        print(f"  [Gimbal] Fixed at {self.NAV_TILT}° pitch (no calibration sweeps)", flush=True)

    def update(self, sim_time: float, scale_confidence: float, dt: float) -> Tuple[float, bool]:
        """Update gimbal state (no-op for fixed gimbal).

        Returns:
            (current_tilt, is_calibrating) - always returns nav tilt and False
        """
        # No calibration sweeps - always in navigation mode
        # Rangefinder data is always valid for scale correction
        return self.current_tilt, False

    def _update_camera_orientation(self):
        """Apply tilt to camera prim orientation using vector-based approach.

        Uses geometric construction instead of euler angles to avoid
        convention mismatches between scipy and Isaac Sim.
        """
        if self._orient_op is None:
            return

        # Convert tilt to look direction
        # Positive tilt = looking forward-up, negative = forward-down
        tilt_rad = np.radians(self.current_tilt)

        # Look direction in drone body frame
        # X = forward, Y = left, Z = up
        cos_tilt, sin_tilt = np.cos(tilt_rad), np.sin(tilt_rad)
        look_dir = np.array([cos_tilt, 0.0, sin_tilt])
        look_dir = look_dir / np.linalg.norm(look_dir)

        # Build orthonormal camera frame
        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(look_dir, world_up)
        right_norm = np.linalg.norm(right)
        if right_norm < 1e-6:
            right = np.array([0.0, 1.0, 0.0])
        else:
            right = right / right_norm

        up = np.cross(right, look_dir)
        up = up / np.linalg.norm(up)

        # Camera looks down -Z in its local frame, so negate look_dir
        R_cam_to_world = np.column_stack([right, up, -look_dir])
        world_rot = Rotation.from_matrix(R_cam_to_world)
        quat_xyzw = world_rot.as_quat()
        self._orient_op.Set(Gf.Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]))


class InProcessDepthEstimator:
    """Depth Anything V2 inference with affine depth correction.

    Runs monocular depth estimation in-process within Isaac Sim,
    using rangefinder readings to calibrate metric scale AND shift.

    Depth Anything V2 outputs AFFINE-INVARIANT inverse depth, meaning
    the output has unknown scale AND shift that must be recovered:
        model_output = a * true_inverse_depth + b

    We use least-squares fitting on accumulated rangefinder samples to
    recover both parameters: metric_depth = scale * model_output + shift
    """

    def __init__(self, model_size: str = "small"):
        """Initialize depth estimator.

        Args:
            model_size: Model size ('small', 'base', 'large')
        """
        self.model_size = model_size
        self.model = None
        self.scale_corrector = None
        self._load_model()

    def _load_model(self):
        """Load Depth Anything V2 RELATIVE model for rangefinder fusion.

        Uses relative depth models that output normalized disparity (0-1).
        The rangefinder is used to calibrate scale and convert to metric depth.

        Pipeline:
        1. Model outputs disparity (higher = closer)
        2. Invert to get relative depth (higher = farther)
        3. Normalize to 0-1 range
        4. Scale corrector calibrates using rangefinder
        5. Output: metric depth in meters
        """
        try:
            import torch
            from transformers import pipeline

            # Use RELATIVE models - output normalized disparity
            # These require rangefinder fusion to get metric depth
            model_names = {
                'small': 'depth-anything/Depth-Anything-V2-Small-hf',
                'base': 'depth-anything/Depth-Anything-V2-Base-hf',
                'large': 'depth-anything/Depth-Anything-V2-Large-hf',
            }

            model_name = model_names.get(self.model_size, model_names['small'])
            print(f"  [DepthEstimator] Loading Depth Anything V2 RELATIVE ({self.model_size})...", flush=True)
            print(f"  [DepthEstimator] Model: {model_name}", flush=True)
            print(f"  [DepthEstimator] Output: Disparity (higher=closer), requires rangefinder fusion", flush=True)

            device = 0 if torch.cuda.is_available() else -1
            print(f"  [DepthEstimator] Device: {'CUDA' if device == 0 else 'CPU'}", flush=True)

            self.model = pipeline(
                task='depth-estimation',
                model=model_name,
                device=device,
            )
            print(f"  [DepthEstimator] Model loaded successfully", flush=True)

            # Initialize scale corrector for rangefinder fusion
            # This converts normalized relative depth to metric depth
            self.scale_corrector = WarmupScaleCorrector(
                warmup_frames=60,      # 2 seconds at 30Hz
                min_samples=20,
                min_depth_cv=0.02,     # 2% coefficient of variation
                alpha=0.1,             # Slow EMA for fallback
                min_valid_range=1.0,
                max_valid_range=200.0,
            )
            self.use_metric_model = False  # Requires rangefinder fusion
            print(f"  [DepthEstimator] Scale corrector initialized (rangefinder fusion enabled)", flush=True)

        except ImportError as e:
            print(f"  [DepthEstimator] ERROR: Failed to import dependencies: {e}", flush=True)
            print(f"  [DepthEstimator] Install with: pip install transformers torch", flush=True)
            self.model = None
        except Exception as e:
            print(f"  [DepthEstimator] ERROR: Failed to load model: {e}", flush=True)
            import traceback
            traceback.print_exc()
            self.model = None

    def estimate_depth(
        self,
        rgb_image: np.ndarray,
        rangefinder_reading: float,
        rangefinder_valid: bool
    ) -> Tuple[Optional[np.ndarray], DepthData, Optional[np.ndarray]]:
        """Run depth inference and return metric depth with metadata.

        Args:
            rgb_image: RGB image (H, W, 3) uint8
            rangefinder_reading: Distance from rangefinder in meters
            rangefinder_valid: Whether rangefinder reading should be used for scale

        Returns:
            (metric_depth, depth_data, model_output) - depth map, metadata, and raw model output
            model_output is the normalized (0-1) raw output before affine correction
        """
        if self.model is None:
            # Return fallback data if model not loaded
            return None, DepthData(
                scale_factor=1.0,
                scale_confidence=0.0,
                rangefinder_reading=rangefinder_reading,
                rangefinder_valid=False,
                min_depth=0.5,
                max_depth=100.0,
                mean_depth=50.0,
                inference_time_ms=0.0,
            ), None

        start = time.perf_counter()

        try:
            # Run depth inference
            from PIL import Image as PILImage
            pil_image = PILImage.fromarray(rgb_image)
            result = self.model(pil_image)
            inference_time = (time.perf_counter() - start) * 1000

            # =========================================================
            # RELATIVE DEPTH MODEL + RANGEFINDER FUSION PIPELINE
            # =========================================================
            # The pipeline returns result['depth'] as a PIL Image (8-bit grayscale)
            # This is DISPARITY: higher values = CLOSER objects
            # We need to: 1) Invert to depth, 2) Normalize, 3) Scale with rangefinder
            #
            # DEBUG: Log what the pipeline returns on first frame
            if not hasattr(self, '_logged_keys'):
                self._logged_keys = True
                print(f"[DEBUG] Pipeline result keys: {list(result.keys())}", flush=True)

            # Step 1: Get raw disparity output (8-bit grayscale image)
            raw_disparity = np.array(result['depth']).astype(np.float32)

            # Normalize disparity to 0-1 range
            if raw_disparity.max() > 1.0:
                raw_disparity = raw_disparity / 255.0

            # Debug first frame
            if not hasattr(self, '_logged_depth'):
                self._logged_depth = True
                print(f"[DEBUG] raw_disparity shape={raw_disparity.shape} "
                      f"range=[{raw_disparity.min():.3f}, {raw_disparity.max():.3f}] "
                      f"(higher=closer)", flush=True)

            # Step 2: Invert disparity to get relative depth (higher = farther)
            # depth = 1 / disparity, but need to handle zeros
            # DO NOT normalize - scale corrector needs proportional relationship
            eps = 0.001  # Prevent division by zero
            relative_depth = 1.0 / (raw_disparity + eps)

            # Debug on first frame
            if not hasattr(self, '_logged_rel'):
                self._logged_rel = True
                print(f"[DEBUG] relative_depth range=[{relative_depth.min():.3f}, {relative_depth.max():.3f}] "
                      f"(higher=farther)", flush=True)

            # Step 3: Update scale corrector with rangefinder
            # scale = rangefinder_reading / center_relative_depth
            # metric = scale * relative_depth
            # This works because relative_depth is proportional to true depth
            scale_estimate = self.scale_corrector.update(
                relative_depth, rangefinder_reading, rangefinder_valid
            )

            # Debug: Print scale status periodically
            if not hasattr(self, '_frame_count'):
                self._frame_count = 0
            self._frame_count += 1
            if self._frame_count % 30 == 0:
                frozen_str = "FROZEN" if scale_estimate.frozen else "warmup"
                center_val = self.scale_corrector._sample_center(relative_depth)
                print(f"[SCALE] scale={scale_estimate.scale:.1f} conf={scale_estimate.confidence:.2f} "
                      f"status={frozen_str} center={center_val:.3f} range_gt={rangefinder_reading:.1f}m",
                      flush=True)

            # Step 4: Apply scale to get metric depth
            # metric_depth = scale * relative_depth
            metric_depth = self.scale_corrector.apply(relative_depth, min_depth=0.5)

            # Clamp to reasonable range
            metric_depth = np.clip(metric_depth, 0.5, 200.0)

            # Debug: Check if depth makes sense
            if self._frame_count % 30 == 0:
                print(f"[DEPTH] min={metric_depth.min():.1f}m max={metric_depth.max():.1f}m "
                      f"mean={metric_depth.mean():.1f}m", flush=True)

            # Compute stats
            valid_mask = metric_depth > 0.5

            depth_data = DepthData(
                scale_factor=scale_estimate.scale,
                scale_confidence=scale_estimate.confidence,
                rangefinder_reading=rangefinder_reading,
                rangefinder_valid=rangefinder_valid,
                min_depth=float(metric_depth[valid_mask].min()) if valid_mask.any() else 0.5,
                max_depth=float(metric_depth[valid_mask].max()) if valid_mask.any() else 200.0,
                mean_depth=float(metric_depth[valid_mask].mean()) if valid_mask.any() else 100.0,
                inference_time_ms=inference_time,
            )

            # Return metric depth, metadata, and relative depth for debugging
            return metric_depth, depth_data, relative_depth

        except Exception as e:
            print(f"  [DepthEstimator] Inference error: {e}", flush=True)
            return None, DepthData(
                scale_factor=1.0,
                scale_confidence=0.0,
                rangefinder_reading=rangefinder_reading,
                rangefinder_valid=False,
                min_depth=0.5,
                max_depth=100.0,
                mean_depth=50.0,
                inference_time_ms=0.0,
            ), None


class DepthBasedVFH:
    """Vector Field Histogram obstacle avoidance from depth map.

    Converts depth map to polar histogram and finds safe navigation directions.
    """

    def __init__(self, hfov_deg: float = 160.0, num_bins: int = 36, safety_dist: float = 8.0):
        """Initialize VFH.

        Args:
            hfov_deg: Horizontal field of view in degrees
            num_bins: Number of angular bins in histogram
            safety_dist: Minimum safe distance in meters
        """
        self.hfov = np.radians(hfov_deg)
        self.num_bins = num_bins
        self.bin_width = self.hfov / num_bins
        self.safety_dist = safety_dist

    def compute_avoidance(
        self,
        depth_map: np.ndarray,
        goal_direction: np.ndarray,
        fx: float,
        cx: float
    ) -> AvoidanceData:
        """Compute VFH+ avoidance from depth map.

        Args:
            depth_map: Metric depth map (H, W) in meters
            goal_direction: 3D unit vector toward goal
            fx: Focal length in pixels
            cx: Principal point x in pixels

        Returns:
            AvoidanceData with recommended heading and safety status
        """
        h, w = depth_map.shape

        # Build polar histogram - minimum depth per angular bin
        histogram = np.full(self.num_bins, float('inf'))

        for col in range(w):
            # Column azimuth angle relative to image center
            azimuth = np.arctan2(col - cx, fx)
            bin_idx = int((azimuth + self.hfov / 2) / self.bin_width)
            bin_idx = np.clip(bin_idx, 0, self.num_bins - 1)

            # Minimum depth in column (closest obstacle at this azimuth)
            col_depth = depth_map[:, col]
            valid = col_depth > 0.5
            if valid.any():
                min_depth = col_depth[valid].min()
                histogram[bin_idx] = min(histogram[bin_idx], min_depth)

        # Find blocked bins (obstacle closer than safety distance)
        blocked = histogram < self.safety_dist
        blocked_count = int(blocked.sum())

        # Determine goal bin from goal direction
        # Note: goal_direction is in camera frame where +X is forward, +Y is left
        # But column mapping uses positive azimuth for right side of image
        # So we negate Y to match: left columns = negative azimuth, right = positive
        goal_azimuth = np.arctan2(-goal_direction[1], goal_direction[0])
        goal_bin = int((goal_azimuth + self.hfov / 2) / self.bin_width)
        goal_bin = np.clip(goal_bin, 0, self.num_bins - 1)

        # Check if direct path to goal is blocked - also check adjacent bins for early warning
        # This gives the drone time to steer before getting too close
        goal_blocked = blocked[goal_bin] if 0 <= goal_bin < self.num_bins else False

        # Also check adjacent bins (±2 bins = ~18° cone around goal)
        adjacent_blocked = False
        for adj in range(-2, 3):
            adj_bin = goal_bin + adj
            if 0 <= adj_bin < self.num_bins and blocked[adj_bin]:
                adjacent_blocked = True
                break

        # Trigger path_blocked if goal OR adjacent bins are blocked
        path_blocked = goal_blocked or adjacent_blocked

        # Debug: Print goal bin depth and blocked bin range
        goal_bin_depth = histogram[goal_bin] if 0 <= goal_bin < self.num_bins else float('inf')
        blocked_indices = np.where(blocked)[0]
        blocked_range = f"{blocked_indices.min()}-{blocked_indices.max()}" if len(blocked_indices) > 0 else "none"
        center_bins = histogram[15:21]  # bins around center (18 is center of 36)
        print(f"[VFH-DEBUG] goal_bin={goal_bin} goal_depth={goal_bin_depth:.1f}m blocked_range={blocked_range} "
              f"center_depths=[{','.join(f'{d:.1f}' for d in center_bins)}]", flush=True)

        # Find best heading - search outward from goal direction
        recommended_heading = 0.0
        if path_blocked:
            for offset in range(1, self.num_bins // 2):
                left_bin = goal_bin - offset
                right_bin = goal_bin + offset
                if 0 <= left_bin < self.num_bins and not blocked[left_bin]:
                    recommended_heading = -offset * self.bin_width
                    break
                if 0 <= right_bin < self.num_bins and not blocked[right_bin]:
                    recommended_heading = offset * self.bin_width
                    break

        # Closest obstacle distance
        closest_dist = float(histogram.min()) if histogram.min() < float('inf') else 100.0

        # Safety level classification
        if closest_dist > 20:
            safety = 0  # CLEAR
        elif closest_dist > 10:
            safety = 1  # CAUTION
        elif closest_dist > 5:
            safety = 2  # WARNING
        else:
            safety = 3  # CRITICAL

        return AvoidanceData(
            closest_obstacle_distance=closest_dist,
            path_blocked=path_blocked,
            recommended_heading=recommended_heading,
            recommended_pitch=0.0,
            recommendation_confidence=0.9 if not path_blocked else 0.75,
            blocked_bins=blocked_count,
            total_bins=self.num_bins,
            candidate_directions=self.num_bins - blocked_count,
            safety_level=safety,
            processing_time_ms=1.0,
        )


# =============================================================================
# END ROS DEPTH MODE COMPONENTS
# =============================================================================


print("=" * 70, flush=True)
print("Depth + Avoidance Navigation Test", flush=True)
print("=" * 70, flush=True)
print(f"  Scenario: {args.scenario}", flush=True)
print(f"  Headless: {HEADLESS}", flush=True)
if args.with_ros:
    print(f"  Mode: FULL PIPELINE (ROS 2 depth + avoidance nodes)", flush=True)
else:
    print(f"  Mode: GROUND TRUTH (Isaac Sim depth, isolated nav test)", flush=True)
print(f"  Output: {args.output_dir}", flush=True)

# Create output directory with timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_DIR = f"{args.output_dir}/{args.scenario}_{timestamp}"
os.makedirs(OUTPUT_DIR, exist_ok=True)


class VideoWriter:
    """Simple video writer using OpenCV."""

    def __init__(self, output_path: str, fps: int = 30, resolution: Tuple[int, int] = (1280, 720)):
        self.output_path = output_path
        self.fps = fps
        self.resolution = resolution
        self.writer = None
        self.frame_count = 0
        self._closed = False

        try:
            import cv2
            self._cv2 = cv2
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(output_path, fourcc, fps, resolution)
            if self.writer.isOpened():
                print(f"  [Video] Writer initialized: {output_path}", flush=True)
            else:
                print(f"  [Video] Warning: Writer failed to open", flush=True)
                self.writer = None
        except ImportError:
            print("  [Video] OpenCV not available", flush=True)
            self._cv2 = None

    def write_frame(self, frame: np.ndarray):
        """Write BGR frame (cv2 native format)."""
        if self.writer is not None and not self._closed:
            if frame.shape[:2] != (self.resolution[1], self.resolution[0]):
                frame = self._cv2.resize(frame, self.resolution)
            # Frame is already BGR from HUD (cv2 drawing functions use BGR)
            self.writer.write(frame)
            self.frame_count += 1
            if self.frame_count % 300 == 0:
                print(f"  [Video] Written {self.frame_count} frames", flush=True)

    def close(self):
        """Release video writer - MUST be called before process exit."""
        if self.writer is not None and not self._closed:
            self._closed = True
            self.writer.release()
            self.writer = None
            print(f"  [Video] Saved {self.frame_count} frames to {self.output_path}", flush=True)
            # Verify file was written correctly
            import os
            if os.path.exists(self.output_path):
                size_mb = os.path.getsize(self.output_path) / (1024 * 1024)
                print(f"  [Video] File size: {size_mb:.1f} MB", flush=True)

    def __del__(self):
        """Destructor - ensure writer is released."""
        self.close()


@dataclass
class NavigationMetrics:
    """Metrics collected during navigation test."""
    scenario_name: str
    success: bool
    goal_reached: bool
    collision_count: int
    flight_time_seconds: float
    path_length_meters: float
    min_clearance_meters: float
    avg_clearance_meters: float
    total_steps: int
    depth_fps_avg: float
    vfh_fps_avg: float
    error_message: str = ""


class DepthAvoidanceTestRunner:
    """
    Runs depth + avoidance navigation test in Isaac Sim.

    Two modes:
    - Ground-truth: Uses Isaac Sim depth buffer directly (isolated nav test)
    - Full pipeline: Uses ROS 2 nodes for depth estimation + avoidance

    Integrates:
    - Procedural obstacle world generation
    - Ground-truth or ROS 2 depth estimation
    - VFH+ avoidance (simulated or via ROS 2)
    - HUD rendering with depth visualization
    - Video recording
    - Metrics collection
    """

    def __init__(
        self,
        world: World,
        scenario: NavigationScenario,
        output_dir: str,
        max_steps: int = 3000,
        use_ros: bool = False,
        save_benchmark: bool = False,
        benchmark_interval: int = 30,
    ):
        self.world = world
        self.scenario = scenario
        self.output_dir = output_dir
        self.max_steps = max_steps
        self.use_ros = use_ros
        self.save_benchmark = save_benchmark
        self.benchmark_interval = benchmark_interval

        # Simulation state
        self.step_count = 0
        self.sim_time = 0.0
        self.dt = 1.0 / 60.0

        # Drone state
        self.position = np.array(scenario.start_position)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.heading = 0.0  # radians

        # Goal
        self.goal = np.array(scenario.goal_position)

        # Camera
        self.camera = None
        self.camera_resolution = (640, 480)

        # HUD and video
        self.hud = DepthAvoidanceHUD(output_resolution=(1280, 720))
        self.video_writer = VideoWriter(
            f"{output_dir}/video.mp4",
            fps=30,
            resolution=(1280, 720),
        )

        # Simulated perception data (in real test, from ROS 2 topics)
        self.depth_data = DepthData()
        self.avoidance_data = AvoidanceData()

        # Metrics
        self.path_length = 0.0
        self.min_clearance = float('inf')
        self.total_clearance = 0.0
        self.clearance_samples = 0
        self.collision_count = 0
        self.depth_times: List[float] = []
        self.vfh_times: List[float] = []

        # Obstacle prims for collision checking
        self.obstacle_prims: List[str] = []

        # ROS depth mode components (initialized after camera setup)
        self.depth_estimator: Optional[InProcessDepthEstimator] = None
        self.calibration_gimbal: Optional[FixedGimbal] = None
        self.depth_vfh: Optional[DepthBasedVFH] = None
        self.gimbal_tilt: float = 15.0  # Default nav tilt (matches NAV_TILT)
        self._current_metric_depth: Optional[np.ndarray] = None
        self._last_valid_avoidance: Optional[AvoidanceData] = None

        # Enhanced navigation components
        self.ground_filter: Optional[GroundFilter] = None
        self.vfh3d: Optional[DepthFieldNavigator] = None  # Depth field navigator
        self.tactical_planner: Optional[TacticalPlanner] = None

    def initialize_ros_mode(self, stage):
        """Initialize ROS depth mode components.

        Called after camera setup when use_ros=True.
        Loads Depth Anything model and sets up gimbal controller.
        """
        if not self.use_ros:
            return

        print("\n[ROS Mode] Initializing depth estimation pipeline...", flush=True)

        # Initialize depth estimator (loads model - may take 5-10s)
        self.depth_estimator = InProcessDepthEstimator(model_size="small")

        # Initialize gimbal controller
        self.calibration_gimbal = FixedGimbal(
            self.camera_prim_path, stage
        )

        # Initialize depth-based VFH (legacy, kept for fallback)
        self.depth_vfh = DepthBasedVFH(hfov_deg=160.0, num_bins=36, safety_dist=15.0)  # 15m for earlier reaction at 8m/s

        # Initialize enhanced navigation components
        print("[ROS Mode] Initializing enhanced navigation components...", flush=True)

        # Ground filter - DISABLED for MVP (not needed at 30m altitude)
        # self.ground_filter = GroundFilter(...)
        self.ground_filter = None
        print("  [GroundFilter] DISABLED for MVP", flush=True)

        # Depth Field Navigator - analyzes depth image to find openings (FALLBACK)
        # MVP: Reduced to 36 sectors (from 72) for simplicity
        self.vfh3d = DepthFieldNavigator(
            hfov_deg=160.0,
            vfov_deg=80.0,
            safety_dist=8.0,  # meters - closer safety for more responsive avoidance
            num_sectors=36,   # Reduced for MVP (was 72)
        )
        print("  [DepthNav] Depth field navigator initialized (36 sectors, 8m safety) - FALLBACK", flush=True)

        # Trajectory Rollout Planner - proactive path planning (PRIMARY)
        # 5-second lookahead at 8 m/s = 40m planning horizon
        self.trajectory_planner = TrajectoryRolloutPlanner(
            horizon_seconds=5.0,        # Plan 5 seconds ahead
            timestep=0.5,               # Evaluate every 0.5s along trajectory
            num_candidates=21,          # 21 different turn rates to sample
            max_turn_rate=0.5,          # Max 30°/s turn rate
            speed=8.0,                  # Nominal flight speed
            safety_margin=10.0,         # Prefer 10m clearance
            collision_threshold=3.0,    # Reject trajectories < 3m clearance
            hfov_deg=160.0,             # Match camera FOV
        )
        self.use_trajectory_planner = True  # Use proactive planning by default
        print("  [TrajPlan] Trajectory rollout planner initialized (5s horizon, 40m lookahead) - PRIMARY", flush=True)

        # Tactical planner - simplified 2-state FSM for MVP
        self.tactical_planner = SimpleTacticalPlanner(
            clear_frames=3,  # Quick state transitions
        )
        print("  [SimplePlanner] 2-state planner initialized (NAVIGATE/AVOID)", flush=True)

        print("[ROS Mode] Initialization complete", flush=True)

    def setup_camera(self, simulation_app):
        """Create camera with explicit Replicator API for headless capture.

        The Camera class internal methods (get_rgba, get_current_frame) return None
        in headless mode. We use the explicit omni.replicator API with render products
        and annotators for reliable RGB+depth capture.
        """
        import omni.replicator.core as rep
        import isaacsim.core.utils.numpy.rotations as rot_utils
        from isaacsim.core.utils.stage import get_current_stage

        # Create camera as independent prim - not parented to drone
        camera_path = "/World/DroneCamera"
        self.camera_prim_path = camera_path
        self._camera_translate_op = None
        self._render_product = None
        self._rgb_annotator = None
        self._depth_annotator = None

        try:
            # Camera orientation: euler angles [0, -10, 0] = 10 degree downward pitch
            # Slight downward tilt to see obstacles ahead and ground in front of drone
            # (Positive Y-euler = upward, negative = downward in camera frame)
            import isaacsim.core.utils.numpy.rotations as rot_utils
            initial_orient = rot_utils.euler_angles_to_quats(
                np.array([0, -15, 0]), degrees=True
            )

            # Offset camera forward (+X) and slightly down (-Z) from drone center
            camera_offset = np.array([0.5, 0.0, -0.2])  # 0.5m forward, 0.2m down
            camera_position = self.position + camera_offset

            self.camera = Camera(
                prim_path=camera_path,
                position=camera_position,
                frequency=30,
                resolution=self.camera_resolution,
                orientation=initial_orient,
            )

            # CRITICAL: Double initialization required for camera to work
            # Per Isaac Sim examples, first call creates resources, second finalizes
            print(f"  [Camera] Double initialization...", flush=True)
            self.camera.initialize()
            simulation_app.update()
            self.camera.initialize()
            simulation_app.update()

            # Set focal length for wide FOV
            # FOV = 2 * atan(aperture / (2 * focal_length))
            # With 50mm aperture and 4.0mm focal: ~160° horizontal FOV
            self.camera.set_focal_length(4.0)
            self.camera.set_clipping_range(0.1, 500.0)

            # Set horizontal aperture for wider FOV
            stage = get_current_stage()
            camera_prim = stage.GetPrimAtPath(camera_path)
            if camera_prim.IsValid():
                usd_camera = UsdGeom.Camera(camera_prim)
                if usd_camera:
                    # Increase aperture for wider FOV (50mm with 4mm focal ≈ 160° HFOV)
                    usd_camera.GetHorizontalApertureAttr().Set(50.0)
                    print(f"  [Camera] Set horizontal aperture to 50.0mm (FOV ~160°)", flush=True)

            # EXPLICIT REPLICATOR API - this is what works in headless mode
            # Both ground-truth and ROS mode need RGB capture for frame rendering
            print(f"  [Camera] Setting up Replicator render product and annotators...", flush=True)

            # Disable automatic capture on play - we want manual control
            rep.orchestrator.set_capture_on_play(False)

            # Create render product from camera prim path
            self._render_product = rep.create.render_product(
                camera_path, self.camera_resolution
            )
            print(f"  [Camera] Created render product", flush=True)

            # Create and attach RGB annotator (needed for both modes)
            self._rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            self._rgb_annotator.attach([self._render_product])
            print(f"  [Camera] Attached RGB annotator", flush=True)

            if not self.use_ros:
                # Ground-truth mode: also capture depth from Isaac Sim
                self._depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
                self._depth_annotator.attach([self._render_product])
                print(f"  [Camera] Attached depth annotator (distance_to_image_plane)", flush=True)
                print(f"  [Camera] Created with Replicator API (ground-truth mode)", flush=True)
            else:
                # ROS mode: depth comes from Depth Anything V2
                print(f"  [Camera] Created with Replicator API (ROS mode - depth from Depth Anything)", flush=True)

            # Pre-fetch translate op for efficient updates (pattern from drone_functions_check.py)
            if camera_prim.IsValid():
                camera_xform = UsdGeom.Xformable(camera_prim)
                for op in camera_xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        self._camera_translate_op = op
                        break
                if self._camera_translate_op is None:
                    self._camera_translate_op = camera_xform.AddTranslateOp()
                print(f"  [Camera] Translate op ready for position updates", flush=True)

        except Exception as e:
            print(f"  [Camera] Warning: {e}", flush=True)
            import traceback
            traceback.print_exc()
            self.camera = None

    def update_camera_pose(self):
        """Update camera world pose to follow drone using USD transforms."""
        if self.camera is None or self._camera_translate_op is None:
            return

        try:
            # Camera mounted on drone nose: 0.5m forward, 0.2m down from drone center
            cam_pos = Gf.Vec3d(
                float(self.position[0] + 0.5),   # 0.5m forward (+X)
                float(self.position[1]),
                float(self.position[2] - 0.2)   # 0.2m down (-Z)
            )
            self._camera_translate_op.Set(cam_pos)
        except Exception:
            pass

    def warmup_calibration(self, max_warmup_steps: int = 180) -> bool:
        """
        Hover in place, rotating to gather diverse depth samples for scale calibration.

        The drone rotates slowly to see different depths (obstacles at varying distances),
        which helps the scale corrector accumulate diverse samples and freeze the scale.

        Args:
            max_warmup_steps: Maximum frames for warmup (default 180 = 6 seconds at 30Hz)

        Returns:
            True if scale was successfully frozen, False if using EMA fallback
        """
        if not self.use_ros or self.depth_estimator is None:
            print("[Warmup] Skipping - not in ROS mode", flush=True)
            return True

        print("\n[Warmup] Starting scale calibration (hover and rotate)...", flush=True)

        rotation_step = np.radians(10)  # 10 degrees per step
        steps_per_rotation = 6  # Rotate every 6 frames (0.2s at 30Hz)
        initial_heading = self.heading

        for step in range(max_warmup_steps):
            # Step physics
            self.world.step(render=True)
            self.update_camera_pose()

            # Rotate slowly to see different depths
            if step % steps_per_rotation == 0:
                self.heading += rotation_step

            # Get frame and update depth estimation
            self.update_depth_data()

            # For metric models, no warmup needed - they output meters directly
            if hasattr(self.depth_estimator, 'use_metric_model') and self.depth_estimator.use_metric_model:
                # Return to initial heading immediately
                self.heading = initial_heading
                print(f"[Warmup] Using METRIC model - no scale calibration needed", flush=True)
                return True

            # Check if scale frozen (for legacy scale corrector models)
            if (self.depth_estimator.scale_corrector is not None and
                self.depth_estimator.scale_corrector.is_frozen):
                # Return to initial heading
                self.heading = initial_heading
                print(f"[Warmup] Scale calibrated at step {step} "
                      f"(scale={self.depth_estimator.scale_corrector.scale:.3f})", flush=True)
                return True

            # Log progress (only for scale corrector models)
            if step % 30 == 0 and self.depth_estimator.scale_corrector is not None:
                scale = self.depth_estimator.scale_corrector.scale
                samples = self.depth_estimator.scale_corrector.sample_count
                print(f"[Warmup] Step {step}: scale={scale:.3f}, samples={samples}", flush=True)

        # Return to initial heading
        self.heading = initial_heading
        print("[Warmup] WARNING: Scale not frozen, continuing with EMA fallback", flush=True)
        return False

    def spawn_obstacles_and_ground(self, stage):
        """Spawn scenario obstacles AND ground with materials applied immediately.

        This matches the working camera_orientation_test.py pattern:
        1. Create materials first
        2. Create prims
        3. Apply materials immediately after each prim
        All BEFORE world.reset()
        """
        import omni.isaac.core.utils.prims as prim_utils

        self.obstacle_prims.clear()

        # Create materials container and materials FIRST
        stage.DefinePrim("/World/Looks", "Scope")
        self._create_material(stage, "obstacle_gray", Gf.Vec3f(0.5, 0.5, 0.55), roughness=0.6)
        self._create_material(stage, "ground_green", Gf.Vec3f(0.1, 0.5, 0.1), roughness=0.8)
        print("  Created materials", flush=True)

        # Create ground plane (matching orientation test: /World/Ground, Cube at z=-0.5)
        ground_path = "/World/Ground"
        prim_utils.create_prim(
            prim_path=ground_path,
            prim_type="Cube",
            position=np.array([0.0, 0.0, -0.5]),
            scale=np.array([500.0, 500.0, 1.0]),
        )
        # Apply material immediately
        ground_prim = stage.GetPrimAtPath(ground_path)
        if ground_prim.IsValid():
            self._apply_material(stage, ground_prim, "ground_green")
        print(f"  Created ground at {ground_path}", flush=True)

        # Create obstacles
        for i, obstacle in enumerate(self.scenario.obstacles):
            prim_path = f"/World/Obstacles/{obstacle.name or f'obstacle_{i}'}"

            # Spawn cylinder or box based on type
            prim_type = "Cylinder" if obstacle.obstacle_type.value == "cylinder" else "Cube"

            prim_utils.create_prim(
                prim_path=prim_path,
                prim_type=prim_type,
                position=np.array([
                    obstacle.position[0],
                    obstacle.position[1],
                    obstacle.position[2] + obstacle.size[2] / 2,
                ]),
                scale=np.array([
                    obstacle.size[0] / 2,
                    obstacle.size[1] / 2,
                    obstacle.size[2] / 2,
                ]),
            )

            # Apply material IMMEDIATELY after creating prim (like orientation test)
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                self._apply_material(stage, prim, "obstacle_gray")

                # Add collision so physics raycasts (rangefinder) can hit the obstacle
                from pxr import UsdPhysics
                UsdPhysics.CollisionAPI.Apply(prim)

            self.obstacle_prims.append(prim_path)
            print(f"  Spawned {obstacle.name} at ({obstacle.position[0]:.0f}, {obstacle.position[1]:.0f})", flush=True)

    def setup_lighting(self, stage):
        """Setup scene lighting for headless RGB rendering.

        In headless mode, scenes have no lighting by default - RGB will be black.
        This adds a dome light (ambient sky with HDRI) and distant light (sun).

        NOTE: Materials are created in spawn_obstacles_and_ground(), not here.
        """
        print("[Lighting] Setting up scene lights...", flush=True)

        # Create lighting parent prim
        lighting_path = "/World/Lighting"
        stage.DefinePrim(lighting_path, "Xform")

        # Dome light with HDRI texture - provides sky background AND sun lighting
        # Single light source from HDRI = single shadow (no separate distant light needed)
        dome_path = f"{lighting_path}/DomeLight"
        dome_light = UsdLux.DomeLight.Define(stage, dome_path)
        dome_light.CreateIntensityAttr(1500.0)

        # Use HDRI texture for realistic sky and sun
        hdri_paths = [
            "/workspace/extensions/forest_generator/models/textures/sky_2k.hdr",
            "/isaac-sim/exts/omni.replicator.core/omni/replicator/core/data/dome_lights/studio_small_08_4k.hdr",
        ]
        hdri_found = False
        for hdri_path in hdri_paths:
            import os
            if os.path.exists(hdri_path):
                dome_light.CreateTextureFileAttr().Set(hdri_path)
                dome_light.CreateTextureFormatAttr().Set("latlong")
                print(f"  Using HDRI: {hdri_path}", flush=True)
                hdri_found = True
                break

        if not hdri_found:
            dome_light.CreateColorAttr(Gf.Vec3f(0.6, 0.75, 1.0))
            print(f"  Using fallback blue sky color", flush=True)

        dome_light.CreateSpecularAttr(1.0)
        print(f"  Created DomeLight (HDRI sky + sun) at {dome_path}", flush=True)

        print("[Lighting] Scene lights configured", flush=True)

    def _create_material(self, stage, name: str, color: Gf.Vec3f, roughness: float = 0.5):
        """Create a PBR material optimized for headless rendering."""
        mat_path = f"/World/Looks/{name}_mat"
        material = UsdShade.Material.Define(stage, mat_path)

        # Create PBR shader
        shader_path = f"{mat_path}/Shader"
        shader = UsdShade.Shader.Define(stage, shader_path)
        shader.CreateIdAttr("UsdPreviewSurface")

        # Core properties
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        # Add specular for proper lighting response
        shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.04, 0.04, 0.04)  # Dielectric default
        )
        shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(0)

        # Ensure opacity
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)

        # Connect shader to material surface
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        return material

    def _apply_material(self, stage, prim, material_name: str):
        """Apply a material to a prim."""
        mat_path = f"/World/Looks/{material_name}_mat"
        material = UsdShade.Material.Get(stage, mat_path)
        if material:
            UsdShade.MaterialBindingAPI(prim).Bind(material)

    def apply_obstacle_materials(self, stage):
        """Apply gray material to all spawned obstacles."""
        print("[Materials] Applying gray material to obstacles...", flush=True)
        for prim_path in self.obstacle_prims:
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                self._apply_material(stage, prim, "obstacle_gray")
        print(f"  Applied gray material to {len(self.obstacle_prims)} obstacles", flush=True)

    def create_visible_ground(self, stage, size: float = 2000.0):
        """Create or update ground plane with green material.

        If ground plane already exists, just apply the material.
        Otherwise creates a flattened Cube prim for reliable headless rendering.
        """
        import omni.isaac.core.utils.prims as prim_utils

        ground_path = "/World/GroundPlane"

        # Check if ground plane already exists
        existing_prim = stage.GetPrimAtPath(ground_path)
        if existing_prim.IsValid():
            # Ground plane exists, just apply material
            self._apply_material(stage, existing_prim, "ground_green")
            print(f"  [Ground] Applied green material to existing ground plane", flush=True)
            return ground_path

        # Create as Cube scaled flat (more reliable than Plane prim in headless)
        prim_utils.create_prim(
            prim_path=ground_path,
            prim_type="Cube",
            position=np.array([0.0, 0.0, -0.5]),  # Top face at Z=0
            scale=np.array([size, size, 1.0]),     # Large flat box
        )

        # Apply green material immediately
        ground_prim = stage.GetPrimAtPath(ground_path)
        if ground_prim.IsValid():
            self._apply_material(stage, ground_prim, "ground_green")
            print(f"  [Ground] Created {size}x{size}m ground plane at {ground_path}", flush=True)
        else:
            print(f"  [Ground] WARNING: Failed to create ground plane", flush=True)

        return ground_path

    def spawn_drone(self, stage):
        """Spawn simple drone representation (camera setup must be done separately after world.reset)."""
        import omni.isaac.core.utils.prims as prim_utils

        drone_path = "/World/Drone"
        prim_utils.create_prim(
            prim_path=drone_path,
            prim_type="Sphere",
            position=self.position,
            scale=np.array([0.5, 0.5, 0.5]),
        )

        self.drone_prim_path = drone_path
        # NOTE: Camera setup is done AFTER world.reset() but BEFORE timeline.play()
        # Do NOT call self.setup_camera() here - it must be called in the correct order
        print(f"  [Drone] Spawned at {self.position}", flush=True)

    def simulate_rangefinder(self) -> Tuple[float, bool]:
        """Raycast along camera optical axis for rangefinder simulation.

        Simulates Gremsy VIO laser rangefinder:
        - Range: 0-1200m @ 10Hz
        - Precision: ±0.1-0.5m
        - Co-located with camera (points same direction as gimbal)

        Returns:
            (distance, valid) - distance in meters, validity flag
        """
        from omni.physx import get_physx_scene_query_interface
        import carb

        # Camera/rangefinder position (mounted on drone nose)
        # Same position since VIO has co-located sensors
        cam_pos = self.position + np.array([0.5, 0.0, -0.2])

        # Ray direction = gimbal tilt (pitch) + drone heading (yaw)
        # gimbal_tilt: negative = looking down, positive = looking up
        gimbal_tilt = getattr(self, 'gimbal_tilt', -10.0)  # Default: -10° (matching camera)
        pitch_rad = np.radians(gimbal_tilt)
        yaw_rad = self.heading

        # Forward direction with pitch applied
        direction = np.array([
            np.cos(yaw_rad) * np.cos(pitch_rad),
            np.sin(yaw_rad) * np.cos(pitch_rad),
            np.sin(pitch_rad)  # Positive tilt = looking up (Z+)
        ])

        # Gremsy VIO rangefinder max range
        MAX_RANGE = 1200.0  # meters
        MIN_RANGE = 0.5     # meters (minimum valid reading)

        try:
            hit = get_physx_scene_query_interface().raycast_closest(
                carb.Float3(float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2])),
                carb.Float3(float(direction[0]), float(direction[1]), float(direction[2])),
                MAX_RANGE
            )

            if hit["hit"]:
                distance = hit["distance"]
                # Add small noise for realism (±0.1m per Gremsy spec)
                distance += np.random.normal(0, 0.1)
                valid = MIN_RANGE <= distance <= MAX_RANGE
                return max(MIN_RANGE, min(distance, MAX_RANGE)), valid

        except Exception as e:
            # Fallback to simple ground distance if raycast fails
            pass

        # No hit = pointing at sky or beyond max range
        # Fallback: use simple altitude as ground distance
        ground_distance = self.position[2]
        if ground_distance < MAX_RANGE:
            return ground_distance, True
        return MAX_RANGE, False

    def get_ground_truth_depth(self) -> Optional[np.ndarray]:
        """Get ground-truth depth using Replicator annotator (primary) or Camera class (fallback)."""
        # Primary: Use Replicator depth annotator (works in headless mode)
        if self._depth_annotator is not None:
            try:
                depth = self._depth_annotator.get_data()
                if depth is not None and depth.size > 0:
                    # Handle shape: may be (H, W) or (H, W, 1)
                    if len(depth.shape) == 3 and depth.shape[-1] == 1:
                        depth = depth.squeeze(-1)
                    return depth.astype(np.float32)
            except Exception:
                pass

        # Fallback: Camera class API (may not work in headless)
        if self.camera is not None:
            try:
                depth = self.camera.get_distance_to_image_plane()
                if depth is not None and depth.size > 0 and len(depth.shape) >= 2 and depth.max() > 0:
                    return depth.astype(np.float32)

                frame_data = self.camera.get_current_frame()
                if frame_data and "distance_to_image_plane" in frame_data:
                    depth = frame_data["distance_to_image_plane"]
                    if depth is not None and depth.size > 0 and depth.max() > 0:
                        return depth.astype(np.float32)
            except Exception:
                pass

        return None

    def _save_depth_comparison(self, rgb_frame: np.ndarray, da_depth: np.ndarray, range_dist: float):
        """Save depth comparison images for debugging.

        Saves:
        1. RGB frame
        2. Depth Anything metric depth (colorized)
        3. Ground-truth depth (colorized)
        4. Side-by-side comparison with stats
        """
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        # Get ground-truth depth for comparison
        gt_depth = self.get_ground_truth_depth()

        # Create output directory
        debug_dir = os.path.join(self.output_dir, "depth_debug")
        os.makedirs(debug_dir, exist_ok=True)

        # Create figure with 2x2 subplots
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))

        # 1. RGB frame
        axes[0, 0].imshow(rgb_frame)
        axes[0, 0].set_title(f"RGB Frame (step {self.step_count})")
        axes[0, 0].axis('off')

        # 2. Depth Anything metric depth
        # Clip to reasonable range for visualization
        da_clipped = np.clip(da_depth, 0, 100)
        im1 = axes[0, 1].imshow(da_clipped, cmap='turbo', vmin=0, vmax=100)
        axes[0, 1].set_title(f"Depth Anything V2 (scale={self.depth_data.scale_factor:.2f})")
        axes[0, 1].axis('off')
        plt.colorbar(im1, ax=axes[0, 1], label='Depth (m)')

        # Add depth stats as text
        valid_da = da_depth[(da_depth > 0.5) & (da_depth < 500)]
        if len(valid_da) > 0:
            da_stats = f"min={valid_da.min():.1f}m max={valid_da.max():.1f}m mean={valid_da.mean():.1f}m"
        else:
            da_stats = "No valid depths"
        axes[0, 1].text(10, 30, da_stats, color='white', fontsize=10,
                        bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))

        # 3. Ground-truth depth
        if gt_depth is not None:
            gt_clipped = np.clip(gt_depth, 0, 100)
            im2 = axes[1, 0].imshow(gt_clipped, cmap='turbo', vmin=0, vmax=100)
            axes[1, 0].set_title("Ground Truth Depth (Isaac Sim)")
            axes[1, 0].axis('off')
            plt.colorbar(im2, ax=axes[1, 0], label='Depth (m)')

            # Add GT stats
            valid_gt = gt_depth[(gt_depth > 0.5) & (gt_depth < 500)]
            if len(valid_gt) > 0:
                gt_stats = f"min={valid_gt.min():.1f}m max={valid_gt.max():.1f}m mean={valid_gt.mean():.1f}m"
            else:
                gt_stats = "No valid depths"
            axes[1, 0].text(10, 30, gt_stats, color='white', fontsize=10,
                            bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
        else:
            axes[1, 0].text(0.5, 0.5, "GT depth not available", transform=axes[1, 0].transAxes,
                           ha='center', va='center', fontsize=14)
            axes[1, 0].axis('off')

        # 4. Difference map (if GT available)
        if gt_depth is not None:
            # Resize DA depth to match GT if needed
            if da_depth.shape != gt_depth.shape:
                from scipy.ndimage import zoom
                scale_h = gt_depth.shape[0] / da_depth.shape[0]
                scale_w = gt_depth.shape[1] / da_depth.shape[1]
                da_resized = zoom(da_depth, (scale_h, scale_w), order=1)
            else:
                da_resized = da_depth

            # Compute difference (GT - DA)
            diff = gt_depth - da_resized
            diff_clipped = np.clip(diff, -50, 50)
            im3 = axes[1, 1].imshow(diff_clipped, cmap='RdBu', vmin=-50, vmax=50)
            axes[1, 1].set_title("Difference (GT - DA)")
            axes[1, 1].axis('off')
            plt.colorbar(im3, ax=axes[1, 1], label='Depth diff (m)')

            # Add position and rangefinder info
            info_text = f"Pos: ({self.position[0]:.1f}, {self.position[1]:.1f}, {self.position[2]:.1f})\n"
            info_text += f"Rangefinder: {range_dist:.1f}m"
            axes[1, 1].text(10, 30, info_text, color='white', fontsize=10,
                           bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
        else:
            axes[1, 1].axis('off')

        # Save figure
        fig_path = os.path.join(debug_dir, f"depth_comparison_step{self.step_count:04d}.png")
        plt.tight_layout()
        plt.savefig(fig_path, dpi=150)
        plt.close(fig)

        print(f"[DEBUG] Saved depth comparison to {fig_path}", flush=True)

    def _save_benchmark_frame(
        self,
        rgb_frame: np.ndarray,
        model_output: np.ndarray,
        gt_depth: np.ndarray,
        rangefinder: float,
        rangefinder_valid: bool,
    ):
        """Save frame for offline benchmarking.

        Creates a benchmark dataset that can be used for rapid offline
        validation of depth correction algorithms without running Isaac Sim.

        Args:
            rgb_frame: RGB image (H, W, 3) uint8
            model_output: Raw normalized model output (H, W) float32
            gt_depth: Ground truth depth from Isaac Sim (H, W) float32
            rangefinder: Rangefinder reading in meters
            rangefinder_valid: Whether rangefinder reading is valid
        """
        # Create benchmark directory
        benchmark_dir = os.path.join(self.output_dir, "benchmark", "frames", f"{self.step_count:06d}")
        os.makedirs(benchmark_dir, exist_ok=True)

        # Save RGB frame
        rgb_path = os.path.join(benchmark_dir, "rgb.png")
        cv2.imwrite(rgb_path, cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR))

        # Save ground truth depth
        gt_path = os.path.join(benchmark_dir, "depth_gt.npy")
        np.save(gt_path, gt_depth.astype(np.float32))

        # Save raw model output (normalized, before affine correction)
        model_path = os.path.join(benchmark_dir, "model_output.npy")
        np.save(model_path, model_output.astype(np.float32))

        # Save rangefinder data
        rf_path = os.path.join(benchmark_dir, "rangefinder.json")
        rf_data = {
            "range": float(rangefinder),
            "valid": bool(rangefinder_valid),
            "step": int(self.step_count),
            "position": [float(x) for x in self.position],
            "heading": float(self.heading),
        }
        with open(rf_path, "w") as f:
            json.dump(rf_data, f, indent=2)

        if self.step_count % 100 == 0:
            print(f"[BENCHMARK] Saved frame {self.step_count} to {benchmark_dir}", flush=True)

    def update_depth_data(self):
        """Update depth data from ground-truth or ROS 2 depth estimation."""
        if self.use_ros and self.depth_estimator is not None:
            # ROS MODE: Use Depth Anything V2 with gimbal calibration

            # Update gimbal state (fixed at +15° pitch)
            if self.calibration_gimbal is not None:
                self.gimbal_tilt, _ = self.calibration_gimbal.update(
                    self.sim_time, self.depth_data.scale_confidence, self.dt
                )
            else:
                self.gimbal_tilt = -15.0  # Default nav tilt (negative = downward, matching camera)

            # Physics raycast rangefinder (uses self.gimbal_tilt)
            range_dist, range_valid = self.simulate_rangefinder()

            # Debug: Print rangefinder reading
            if self.step_count % 30 == 0:
                print(f"[RANGE] step={self.step_count} range={range_dist:.1f}m valid={range_valid} tilt={self.gimbal_tilt:.1f}°", flush=True)

            # Always use rangefinder for scale correction when valid
            # At +15° pitch, rangefinder hits ground/obstacles ahead of drone
            use_range_for_scale = range_valid

            # Capture RGB frame
            rgb_frame = self.capture_frame()
            if rgb_frame is not None:
                # Run Depth Anything V2 inference
                self._current_metric_depth, self.depth_data, model_output = self.depth_estimator.estimate_depth(
                    rgb_frame, range_dist, use_range_for_scale
                )
                self.depth_times.append(self.depth_data.inference_time_ms)

                # Debug: Print scale factor
                if self.step_count % 30 == 0:
                    print(f"[SCALE] step={self.step_count} scale={self.depth_data.scale_factor:.2f} conf={self.depth_data.scale_confidence:.2f}", flush=True)

                # DEBUG: Save depth comparison at specific steps
                if self.step_count in [100, 200, 300, 400]:
                    self._save_depth_comparison(rgb_frame, self._current_metric_depth, range_dist)

                # Save benchmark frame for offline testing
                if self.save_benchmark and self.step_count % self.benchmark_interval == 0:
                    gt_depth = self.get_ground_truth_depth()
                    if gt_depth is not None and model_output is not None:
                        self._save_benchmark_frame(
                            rgb_frame, model_output, gt_depth, range_dist, use_range_for_scale
                        )
            else:
                # No frame available
                self.depth_times.append(0.0)
        else:
            # GROUND-TRUTH MODE: Use Isaac Sim depth buffer directly
            inference_time = 0.1  # Essentially instant
            self.depth_times.append(inference_time)

            # Get rangefinder (still useful for HUD display)
            range_dist, range_valid = self.simulate_rangefinder()

            # Get actual depth stats from ground-truth
            gt_depth = self.get_ground_truth_depth()
            if gt_depth is not None:
                valid_mask = (gt_depth > 0.1) & (gt_depth < 500.0)
                if valid_mask.sum() > 0:
                    min_depth = float(gt_depth[valid_mask].min())
                    max_depth = float(gt_depth[valid_mask].max())
                    mean_depth = float(gt_depth[valid_mask].mean())
                else:
                    min_depth, max_depth, mean_depth = 0.5, 100.0, 50.0

                # Ground-truth: perfect scale factor
                scale_factor = 1.0
                scale_confidence = 1.0

                # Store for visualization
                self._current_metric_depth = gt_depth
            else:
                # No depth available - use estimates
                min_depth = max(0.5, range_dist * 0.3)
                max_depth = min(100.0, range_dist * 3)
                mean_depth = range_dist * 1.2
                scale_factor = 1.0
                scale_confidence = 0.0

            self.depth_data = DepthData(
                scale_factor=scale_factor,
                scale_confidence=scale_confidence,
                rangefinder_reading=range_dist,
                rangefinder_valid=range_valid,
                min_depth=min_depth,
                max_depth=max_depth,
                mean_depth=mean_depth,
                inference_time_ms=inference_time,
            )

    def update_avoidance_data(self):
        """Update avoidance data from ground-truth geometry or ROS 2 depth-based VFH."""
        if self.use_ros and self.vfh3d is not None:
            # ROS MODE: Use enhanced VFH3D with ground filtering
            # Run depth-based VFH if we have a depth map
            if self._current_metric_depth is not None:
                start_time = time.perf_counter()

                # Camera intrinsics (160 deg HFOV, 640 width, ~80 deg VFOV)
                # For 160° HFOV: fx = cx / tan(80°) = 320 / 5.67 ≈ 56.4
                h, w = self._current_metric_depth.shape
                fx = 56.4  # pixels (horizontal)
                fy = 56.4  # pixels (vertical, assuming square pixels)
                cx = w / 2.0  # Image center x
                cy = h / 2.0  # Image center y

                # Get camera pitch from gimbal
                camera_pitch = self.gimbal_tilt if hasattr(self, 'gimbal_tilt') else -15.0

                # Step 1: Apply ground filtering to remove ground plane from depth map
                filtered_depth = self._current_metric_depth
                if self.ground_filter is not None:
                    filtered_depth, plane = self.ground_filter.filter_ground(
                        self._current_metric_depth,
                        fx, fy, cx, cy,
                        camera_pitch
                    )
                    if plane is not None and self.step_count % 60 == 0:
                        print(f"[GroundFilter] Plane detected, normal=({plane[0]:.2f},{plane[1]:.2f},{plane[2]:.2f})", flush=True)

                # Step 2: Compute goal direction in CAMERA FRAME
                goal_dir_world = self.goal[:2] - self.position[:2]
                goal_dist = np.linalg.norm(goal_dir_world)
                if goal_dist > 0:
                    goal_dir_world = goal_dir_world / goal_dist

                # Rotate from world frame to camera/body frame using drone heading
                cos_h = np.cos(-self.heading)
                sin_h = np.sin(-self.heading)
                goal_camera_x = goal_dir_world[0] * cos_h - goal_dir_world[1] * sin_h
                goal_camera_y = goal_dir_world[0] * sin_h + goal_dir_world[1] * cos_h
                goal_dir_3d = np.array([goal_camera_x, goal_camera_y, 0.0])

                # Step 3: Compute avoidance - PRIMARY: Trajectory Planner, FALLBACK: VFH3D
                goal_azimuth = np.arctan2(-goal_dir_3d[1], goal_dir_3d[0])

                if self.use_trajectory_planner:
                    # Use proactive trajectory rollout planner
                    traj_result = self.trajectory_planner.plan(
                        filtered_depth, goal_azimuth, fx, fy, cx, cy
                    )

                    # Check if trajectory planner found a safe path
                    if traj_result.min_clearance >= 2.0:
                        # Use trajectory planner result
                        self.avoidance_data = AvoidanceData(
                            closest_obstacle_distance=traj_result.min_clearance,
                            path_blocked=traj_result.min_clearance < 8.0,
                            recommended_heading=traj_result.recommended_heading,
                            recommended_pitch=0.0,
                            recommendation_confidence=0.9,
                            blocked_bins=0 if traj_result.min_clearance > 10 else 10,
                            total_bins=21,
                            candidate_directions=len([c for c in traj_result.all_candidates if c.clearances.min() > 3]),
                            safety_level=0 if traj_result.min_clearance > 20 else (1 if traj_result.min_clearance > 10 else 2),
                            processing_time_ms=traj_result.planning_time_ms,
                        )

                        if self.step_count % 30 == 0:
                            best = traj_result.best_candidate
                            print(f"[TrajPlan] step={self.step_count} goal_az={np.degrees(goal_azimuth):.1f}° "
                                  f"turn_rate={best.turn_rate:.2f} rad/s "
                                  f"min_clear={traj_result.min_clearance:.1f}m "
                                  f"goal_prog={best.goal_progress:.1f}m "
                                  f"heading={np.degrees(traj_result.recommended_heading):.1f}° "
                                  f"speed={traj_result.recommended_speed:.1f}m/s "
                                  f"time={traj_result.planning_time_ms:.1f}ms", flush=True)
                    else:
                        # Trajectory planner couldn't find safe path - fall back to VFH
                        print(f"[TrajPlan] FALLBACK to VFH: best clearance {traj_result.min_clearance:.1f}m < 2.0m", flush=True)
                        self.avoidance_data = self.vfh3d.compute_avoidance(
                            filtered_depth, goal_dir_3d, fx, fy, cx, cy
                        )
                else:
                    # Use reactive VFH3D
                    self.avoidance_data = self.vfh3d.compute_avoidance(
                        filtered_depth, goal_dir_3d, fx, fy, cx, cy
                    )

                processing_time = (time.perf_counter() - start_time) * 1000
                self.avoidance_data.processing_time_ms = processing_time

                # Debug: Log decisions periodically (VFH mode or fallback)
                if self.step_count % 30 == 0 and not self.use_trajectory_planner:
                    goal_azimuth_deg = np.degrees(goal_azimuth)
                    tactical_state = self.tactical_planner.state_name if self.tactical_planner else "N/A"
                    print(f"[VFH3D] step={self.step_count} goal_az={goal_azimuth_deg:.1f}° "
                          f"blocked={self.avoidance_data.path_blocked} "
                          f"closest={self.avoidance_data.closest_obstacle_distance:.1f}m "
                          f"blocked_bins={self.avoidance_data.blocked_bins}/{self.avoidance_data.total_bins} "
                          f"heading={np.degrees(self.avoidance_data.recommended_heading):.1f}° "
                          f"pitch={np.degrees(self.avoidance_data.recommended_pitch):.1f}° "
                          f"tactical={tactical_state}", flush=True)

                # Cache valid avoidance for use during calibration
                self._last_valid_avoidance = self.avoidance_data

                self.vfh_times.append(processing_time)

                # Update clearance metrics
                self.min_clearance = min(self.min_clearance, self.avoidance_data.closest_obstacle_distance)
                self.total_clearance += max(0, self.avoidance_data.closest_obstacle_distance)
                self.clearance_samples += 1
                return
            else:
                # No depth map - use placeholder avoidance (conservative: assume blocked)
                self.vfh_times.append(0.1)
                # Set default avoidance data when no depth available in ROS mode
                if self._last_valid_avoidance is not None:
                    self.avoidance_data = self._last_valid_avoidance
                    self.avoidance_data.processing_time_ms = 0.1
                else:
                    # No prior avoidance data - create conservative default
                    self.avoidance_data = AvoidanceData(
                        closest_obstacle_distance=5.0,  # Assume close obstacle
                        path_blocked=True,
                        recommended_heading=0.0,
                        recommended_pitch=0.0,
                        recommendation_confidence=0.1,
                        blocked_bins=18,
                        total_bins=36,
                        candidate_directions=18,
                        safety_level=2,  # WARNING
                        processing_time_ms=0.1,
                    )
                return  # Exit early in ROS mode
        else:
            # GROUND-TRUTH MODE: Compute from known obstacle positions
            processing_time = 0.1  # Essentially instant
            self.vfh_times.append(processing_time)

        # Calculate actual clearance and path status
        goal_dir = self.goal[:2] - self.position[:2]
        goal_distance = np.linalg.norm(goal_dir)
        if goal_distance > 0:
            goal_dir = goal_dir / goal_distance

        closest_dist = float('inf')
        closest_obstacle = None
        path_blocked = False
        blocked_bins = 0

        for obstacle in self.scenario.obstacles:
            obs_pos = np.array(obstacle.position)
            to_obs = obs_pos[:2] - self.position[:2]
            dist_2d = np.linalg.norm(to_obs)
            clearance = dist_2d - obstacle.size[0] / 2

            if clearance < closest_dist:
                closest_dist = clearance
                closest_obstacle = obstacle

            # Check if obstacle blocks path to goal (larger detection cone)
            if dist_2d > 0:
                dot = np.dot(to_obs / dist_2d, goal_dir)
                # More aggressive blocking detection: within 60 degrees and closer than 25m
                if dot > 0.5 and clearance < 25:
                    path_blocked = True
                    blocked_bins += 3  # Approximate blocked bins

        # Determine safety level
        if closest_dist > 20:
            safety = 0  # CLEAR
        elif closest_dist > 10:
            safety = 1  # CAUTION
        elif closest_dist > 5:
            safety = 2  # WARNING
        else:
            safety = 3  # CRITICAL

        # Recommended heading - proper VFH-style avoidance
        recommended_heading = 0.0
        if path_blocked and closest_obstacle is not None:
            obs_pos = np.array(closest_obstacle.position)
            to_obs = obs_pos[:2] - self.position[:2]
            obs_dist = np.linalg.norm(to_obs)

            if obs_dist > 0:
                # Calculate avoidance direction using cross product
                # This tells us which side of the obstacle to go around
                obs_dir = to_obs / obs_dist
                cross = goal_dir[0] * obs_dir[1] - goal_dir[1] * obs_dir[0]

                # Calculate required avoidance angle based on distance and obstacle size
                # Closer = need bigger steering angle
                obstacle_radius = closest_obstacle.size[0] / 2
                safety_margin = 8.0  # meters to pass beside obstacle

                # Required perpendicular offset to clear obstacle
                required_offset = obstacle_radius + safety_margin

                # Calculate steering angle: bigger when closer, smaller when far
                if closest_dist < 20:
                    # Aggressive steering when close
                    steering_magnitude = np.arctan2(required_offset, max(1.0, closest_dist))
                    # Clamp to reasonable range
                    steering_magnitude = min(steering_magnitude, np.radians(60))
                    steering_magnitude = max(steering_magnitude, np.radians(20))
                else:
                    # Gentler steering when far
                    steering_magnitude = np.radians(15)

                # Determine direction: go opposite to obstacle offset from goal line
                if cross > 0:
                    recommended_heading = -steering_magnitude  # Steer left (negative yaw)
                else:
                    recommended_heading = steering_magnitude   # Steer right (positive yaw)

        self.avoidance_data = AvoidanceData(
            closest_obstacle_distance=closest_dist,
            path_blocked=path_blocked,
            recommended_heading=recommended_heading,
            recommended_pitch=0.0,
            recommendation_confidence=0.9 if not path_blocked else 0.75,
            blocked_bins=blocked_bins,
            total_bins=36,
            candidate_directions=36 - blocked_bins,
            safety_level=safety,
            processing_time_ms=processing_time,
        )

        # Update metrics
        self.min_clearance = min(self.min_clearance, closest_dist)
        self.total_clearance += max(0, closest_dist)
        self.clearance_samples += 1

        # Collision detection: drone has ~0.5m radius, detect when within 1m of obstacle surface
        # closest_dist is already surface-to-center distance (dist - obstacle_radius)
        # So collision when drone center gets within ~0.5m of obstacle surface
        DRONE_RADIUS = 0.5  # meters
        if closest_dist < DRONE_RADIUS:
            self.collision_count += 1
            if self.collision_count <= 5:  # Log first few collisions
                print(f"[COLLISION] clearance={closest_dist:.2f}m < {DRONE_RADIUS}m at step {self.step_count}", flush=True)

    def update_drone(self):
        """Update drone position toward goal with avoidance."""
        # Simple waypoint following with avoidance adjustment
        to_goal = self.goal - self.position
        goal_dist = np.linalg.norm(to_goal)

        if goal_dist < self.scenario.goal_tolerance_meters:
            return True  # Goal reached

        # Base direction to goal
        goal_direction = to_goal / goal_dist

        # Use SimpleTacticalPlanner in ROS mode with simplified 2-state navigation
        if self.use_ros and self.tactical_planner is not None:
            # Get command from tactical planner (SimpleTacticalPlanner only takes avoidance_data)
            cmd = self.tactical_planner.update(self.avoidance_data)

            # Apply command based on mode
            if cmd.rotate_only:
                # Rotate in place (for orbit recovery)
                self.heading += cmd.heading
                speed = 0.0
                direction = goal_direction.copy()
            else:
                # Apply heading offset to direction
                direction = goal_direction.copy()
                if abs(cmd.heading) > 0.01:
                    cos_h = np.cos(cmd.heading)
                    sin_h = np.sin(cmd.heading)
                    new_x = direction[0] * cos_h - direction[1] * sin_h
                    new_y = direction[0] * sin_h + direction[1] * cos_h
                    direction[0] = new_x
                    direction[1] = new_y
                    direction = direction / np.linalg.norm(direction)

                speed = cmd.speed

            # Apply vertical motion if climbing/descending
            vertical_speed = 0.0
            if cmd.vertical and abs(cmd.pitch) > 0.1:
                # Convert pitch angle to vertical speed
                vertical_speed = cmd.speed * np.sin(np.radians(cmd.pitch))
                speed = cmd.speed * np.cos(np.radians(cmd.pitch))  # Horizontal component

            # Update position
            old_pos = self.position.copy()
            self.velocity = np.array([
                direction[0] * speed,
                direction[1] * speed,
                vertical_speed
            ])
            self.position = self.position + self.velocity * self.dt

            # Update heading (unless rotating only, which was handled above)
            if not cmd.rotate_only and speed > 0.1:
                self.heading = np.arctan2(direction[1], direction[0])

            # Track path length
            self.path_length += np.linalg.norm(self.position - old_pos)

        else:
            # LEGACY MODE: Original steering logic for ground-truth mode
            direction = goal_direction.copy()

            # Initialize persistent steering state if needed
            if not hasattr(self, '_steering_active'):
                self._steering_active = False
                self._steering_heading = 0.0
                self._steering_cooldown = 0

            # Update steering state
            if self.avoidance_data.path_blocked and abs(self.avoidance_data.recommended_heading) > 0.01:
                # New steering recommendation - activate/update
                self._steering_active = True
                self._steering_heading = self.avoidance_data.recommended_heading
                self._steering_cooldown = 30  # Continue steering for at least 30 steps after clear

            # Apply steering if active
            if self._steering_active:
                if self.avoidance_data.path_blocked or self._steering_cooldown > 0:
                    # Rotate direction by steering heading
                    cos_h = np.cos(self._steering_heading)
                    sin_h = np.sin(self._steering_heading)
                    new_x = direction[0] * cos_h - direction[1] * sin_h
                    new_y = direction[0] * sin_h + direction[1] * cos_h
                    direction[0] = new_x
                    direction[1] = new_y
                    direction = direction / np.linalg.norm(direction)

                    if not self.avoidance_data.path_blocked:
                        self._steering_cooldown -= 1
                else:
                    # Path clear and cooldown expired - deactivate steering
                    self._steering_active = False
                    self._steering_heading = 0.0

            # Slow down near obstacles for safer maneuvering
            speed = 8.0  # m/s base speed
            if self.avoidance_data.path_blocked and self.avoidance_data.blocked_bins >= self.avoidance_data.total_bins:
                # ALL directions blocked - full stop (no crawling into obstacles)
                speed = 0.0
                print(f"[AVOID] step={self.step_count} FULL STOP - all bins blocked", flush=True)
            elif self.avoidance_data.safety_level >= 3:  # CRITICAL
                speed = 2.0
            elif self.avoidance_data.safety_level >= 2:  # WARNING
                speed = 4.0
            elif self.avoidance_data.safety_level >= 1:  # CAUTION
                speed = 6.0

            # Log when avoidance steering is applied
            if self._steering_active:
                print(f"[AVOID] step={self.step_count} STEERING {np.degrees(self._steering_heading):.1f}° speed={speed:.1f}m/s cooldown={self._steering_cooldown}", flush=True)

            # Update position
            old_pos = self.position.copy()
            self.velocity = direction * speed
            self.position = self.position + self.velocity * self.dt

            # Update heading
            self.heading = np.arctan2(direction[1], direction[0])

            # Track path length
            self.path_length += np.linalg.norm(self.position - old_pos)

        # Update drone prim position
        try:
            from pxr import UsdGeom
            from isaacsim.core.utils.stage import get_current_stage
            stage = get_current_stage()
            drone_prim = stage.GetPrimAtPath(self.drone_prim_path)
            if drone_prim.IsValid():
                xform = UsdGeom.Xformable(drone_prim)
                ops = xform.GetOrderedXformOps()
                if ops:
                    ops[0].Set(Gf.Vec3d(float(self.position[0]),
                                        float(self.position[1]),
                                        float(self.position[2])))
        except Exception:
            pass

        # Update camera to follow drone
        self.update_camera_pose()

        return False

    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture camera frame using Replicator annotators (primary) or Camera class (fallback)."""
        import omni.replicator.core as rep

        # Primary: Use Replicator RGB annotator (works in headless mode)
        if self._rgb_annotator is not None:
            try:
                # Step the orchestrator to ensure annotators are updated
                rep.orchestrator.step(rt_subframes=0, pause_timeline=False)

                rgb_data = self._rgb_annotator.get_data()
                if rgb_data is not None and rgb_data.size > 0 and rgb_data.max() > 0:
                    # Replicator returns RGBA (H, W, 4), extract RGB
                    if len(rgb_data.shape) == 3 and rgb_data.shape[2] >= 3:
                        return rgb_data[:, :, :3].astype(np.uint8)
            except Exception:
                pass

        # Fallback: Camera class API (may not work in headless)
        if self.camera is not None:
            try:
                rgba = self.camera.get_rgba()
                if rgba is not None and rgba.size > 0 and len(rgba.shape) >= 2:
                    if rgba.max() > 0:
                        return rgba[:, :, :3].astype(np.uint8)

                frame_data = self.camera.get_current_frame()
                if frame_data:
                    if "rgb" in frame_data:
                        rgb = frame_data["rgb"]
                        if rgb is not None and rgb.size > 0 and rgb.max() > 0:
                            if len(rgb.shape) == 3 and rgb.shape[2] >= 3:
                                return rgb[:, :, :3].astype(np.uint8)
            except Exception:
                pass

        return None

    def get_depth_for_visualization(self) -> np.ndarray:
        """Get depth map for HUD visualization."""
        # ROS mode: use estimated depth from Depth Anything
        if self.use_ros and self._current_metric_depth is not None:
            return self._current_metric_depth

        # Ground-truth mode: use Isaac Sim depth buffer
        if not self.use_ros:
            gt_depth = self.get_ground_truth_depth()
            if gt_depth is not None:
                return gt_depth

        # Fallback: generate synthetic depth gradient (vectorized for speed)
        h, w = self.camera_resolution[1], self.camera_resolution[0]

        # Create coordinate grids
        y_coords, x_coords = np.ogrid[:h, :w]
        center_dist = np.sqrt((x_coords - w/2)**2 + (y_coords - h/2)**2) / (w/2)

        # Depth increases from center (closer) to edges (farther)
        depth = self.depth_data.mean_depth * (0.5 + 0.5 * center_dist)

        return depth.astype(np.float32)

    def step(self) -> Tuple[bool, bool]:
        """
        Execute one simulation step.

        Returns:
            (done, goal_reached) - whether episode is done and if goal was reached
        """
        self.step_count += 1
        self.sim_time += self.dt

        # Step physics
        self.world.step(render=True)

        # Update perception data (ground-truth or from ROS 2)
        self.update_depth_data()
        self.update_avoidance_data()

        # Update drone
        goal_reached = self.update_drone()

        # Capture and record video frame
        if self.step_count % 2 == 0:  # 30fps from 60Hz
            rgb_frame = self.capture_frame()
            if rgb_frame is None:
                # No fallback - camera MUST work for depth-based navigation
                if self.step_count > 10:  # Allow some warmup frames
                    raise RuntimeError(
                        f"Camera capture failed at step {self.step_count}. "
                        "The camera must work for depth-based obstacle avoidance. "
                        "Check initialization order: camera must be created BEFORE world.reset()."
                    )
                # During warmup, use a placeholder frame
                bgr_frame = np.zeros((self.camera_resolution[1], self.camera_resolution[0], 3), dtype=np.uint8)
                cv2.putText(bgr_frame, "WARMING UP...", (50, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            else:
                # Camera returns RGB, convert to BGR for cv2-based HUD
                bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

            depth_frame = self.get_depth_for_visualization()

            telemetry = TelemetryData(
                position=tuple(self.position),
                heading_deg=np.degrees(self.heading),
                goal_distance=np.linalg.norm(self.goal - self.position),
                step=self.step_count,
                sim_time=self.sim_time,
            )

            hud_frame = self.hud.render(
                bgr_image=bgr_frame,
                depth_image=depth_frame,
                depth_data=self.depth_data,
                avoidance_data=self.avoidance_data,
                telemetry=telemetry,
            )

            self.video_writer.write_frame(hud_frame)

        # Logging
        if self.step_count % 60 == 0:
            goal_dist = np.linalg.norm(self.goal - self.position)
            safety_name = ["CLEAR", "CAUTION", "WARNING", "CRITICAL"][self.avoidance_data.safety_level]
            print(f"  [T+{self.sim_time:.1f}s] POS:({self.position[0]:.0f},{self.position[1]:.0f},{self.position[2]:.0f}) "
                  f"GOAL:{goal_dist:.0f}m SAFETY:{safety_name} CLOSEST:{self.avoidance_data.closest_obstacle_distance:.1f}m",
                  flush=True)

        # Check termination
        done = False
        if goal_reached:
            print(f"  [T+{self.sim_time:.1f}s] GOAL REACHED!", flush=True)
            done = True
        elif self.collision_count > 0:
            print(f"  [T+{self.sim_time:.1f}s] COLLISION DETECTED!", flush=True)
            done = True
        elif self.step_count >= self.max_steps:
            print(f"  [T+{self.sim_time:.1f}s] MAX STEPS REACHED", flush=True)
            done = True
        elif self.sim_time > self.scenario.max_time_seconds:
            print(f"  [T+{self.sim_time:.1f}s] TIMEOUT", flush=True)
            done = True

        return done, goal_reached

    def get_metrics(self, goal_reached: bool) -> NavigationMetrics:
        """Collect final metrics."""
        avg_clearance = self.total_clearance / max(1, self.clearance_samples)

        depth_fps = float(1000.0 / np.mean(self.depth_times)) if self.depth_times else 0.0
        vfh_fps = float(1000.0 / np.mean(self.vfh_times)) if self.vfh_times else 0.0

        success = bool(
            goal_reached and
            self.collision_count == 0 and
            self.min_clearance >= self.scenario.min_clearance_meters
        )

        return NavigationMetrics(
            scenario_name=self.scenario.name,
            success=success,
            goal_reached=bool(goal_reached),
            collision_count=int(self.collision_count),
            flight_time_seconds=float(self.sim_time),
            path_length_meters=float(self.path_length),
            min_clearance_meters=float(self.min_clearance) if self.min_clearance < float('inf') else 0.0,
            avg_clearance_meters=float(avg_clearance),
            total_steps=int(self.step_count),
            depth_fps_avg=depth_fps,
            vfh_fps_avg=vfh_fps,
        )

    def close(self):
        """Cleanup resources."""
        self.video_writer.close()


# =============================================================================
# MAIN EXECUTION
# =============================================================================

# Create world
print("\n[World] Creating simulation world...", flush=True)
timeline = omni.timeline.get_timeline_interface()
world = World(stage_units_in_meters=1.0)

# Create a large ground plane (1000m x 1000m) instead of default small one
from isaacsim.core.utils.stage import get_current_stage
stage = get_current_stage()

import omni.isaac.core.utils.prims as prim_utils
print("[World] Creating large ground plane (1000m x 1000m)...", flush=True)
prim_utils.create_prim(
    prim_path="/World/GroundPlane",
    prim_type="Plane",
    position=np.array([0.0, 0.0, 0.0]),
    scale=np.array([1000.0, 1000.0, 1.0]),  # 1000m x 1000m
)
# Add collision to ground
from pxr import UsdPhysics as UsdPhysicsLocal
ground_prim = stage.GetPrimAtPath("/World/GroundPlane")
if ground_prim.IsValid():
    UsdPhysicsLocal.CollisionAPI.Apply(ground_prim)

# CRITICAL: Pre-warmup for renderer initialization (required for camera to work)
for _ in range(10):
    simulation_app.update()

# Create physics scene
physics_path = "/World/PhysicsScene"
if not stage.GetPrimAtPath(physics_path):
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_path))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

# Generate scenario
print(f"\n[Scenario] Loading {args.scenario}...", flush=True)
generator = ObstacleWorldGenerator(seed=42)
scenario = generator.create_scenario(args.scenario)

print(f"  Name: {scenario.name}", flush=True)
print(f"  Description: {scenario.description}", flush=True)
print(f"  Start: {scenario.start_position}", flush=True)
print(f"  Goal: {scenario.goal_position}", flush=True)
print(f"  Obstacles: {len(scenario.obstacles)}", flush=True)

# Create test runner
runner = DepthAvoidanceTestRunner(
    world=world,
    scenario=scenario,
    output_dir=OUTPUT_DIR,
    max_steps=args.max_steps,
    use_ros=args.with_ros,
    save_benchmark=args.save_benchmark,
    benchmark_interval=args.benchmark_interval,
)

# MATCHING camera_orientation_test.py ORDER EXACTLY:
# 1. Setup lighting (creates dome light, sun light)
# 2. Create scene objects with materials applied immediately
# 3. Create cameras
# 4. world.reset()
# 5. timeline.play()

# Setup scene lighting first (dome light + sun)
print("\n[Scene] Setting up lighting...", flush=True)
runner.setup_lighting(stage)

# Spawn obstacles AND ground with materials applied immediately
# This is the key fix - materials must be applied BEFORE world.reset()
print("\n[Scene] Creating obstacles and ground with materials...", flush=True)
runner.spawn_obstacles_and_ground(stage)

print("\n[Spawn] Creating drone...", flush=True)
runner.spawn_drone(stage)

# Create camera BEFORE world.reset()
print("\n[Camera] Setting up camera BEFORE world.reset...", flush=True)
runner.setup_camera(simulation_app)

# Initialize ROS depth mode components if using --with-ros
if args.with_ros:
    print("\n[ROS Mode] Initializing depth estimation components...", flush=True)
    runner.initialize_ros_mode(stage)

print("\n[Init] world.reset()...", flush=True)
world.reset()

print("\n[Init] timeline.play()...", flush=True)
timeline.play()

# Camera warmup (need more frames for camera to stabilize)
# Isaac Sim cameras need substantial warmup to start rendering
print("\n[Camera] Warming up (300 frames)...", flush=True)
warmup_rgb_success = False
warmup_depth_success = False
for i in range(300):
    world.step(render=True)
    # Update camera position during warmup
    runner.update_camera_pose()
    if i % 50 == 0:
        # Check camera is working via Replicator API
        rgba = runner.capture_frame()
        depth = runner.get_ground_truth_depth() if not runner.use_ros else None
        if rgba is not None:
            warmup_rgb_success = True
            depth_info = ""
            if depth is not None:
                warmup_depth_success = True
                valid_mask = (depth > 0.1) & (depth < 500.0)
                if valid_mask.sum() > 0:
                    depth_info = f", depth=[{depth[valid_mask].min():.1f}, {depth[valid_mask].max():.1f}]m"
            print(f"  Frame {i}: RGB shape={rgba.shape}, max={rgba.max()}{depth_info}", flush=True)
        else:
            print(f"  Frame {i}: Waiting for camera warmup...", flush=True)

if warmup_rgb_success:
    print(f"  Camera warmup complete: RGB={'OK' if warmup_rgb_success else 'FAILED'}, Depth={'OK' if warmup_depth_success else 'FAILED'}", flush=True)
else:
    print(f"  WARNING: Camera may not be working properly", flush=True)

# MVP: Run scale calibration warmup (hover and rotate)
if args.with_ros:
    scale_calibrated = runner.warmup_calibration(max_warmup_steps=180)  # 6 seconds max
    if scale_calibrated:
        print("[Warmup] Scale calibration successful", flush=True)
    else:
        print("[Warmup] Scale calibration incomplete - using fallback", flush=True)

# Run test
print("\n" + "=" * 70, flush=True)
print(f"RUNNING: {scenario.name}", flush=True)
print("=" * 70, flush=True)

done = False
goal_reached = False

while not done:
    done, goal_reached = runner.step()

# Get metrics
metrics = runner.get_metrics(goal_reached)

# CRITICAL: Save metrics FIRST before closing video (Isaac Sim may crash during cleanup)
metrics_path = f"{OUTPUT_DIR}/metrics.json"
metrics_temp = f"{OUTPUT_DIR}/metrics.json.tmp"
metrics_json = json.dumps(asdict(metrics), indent=2)
with open(metrics_temp, 'w') as f:
    f.write(metrics_json)
    f.flush()
    os.fsync(f.fileno())  # Force write to disk
os.rename(metrics_temp, metrics_path)  # Atomic on POSIX
print(f"\n[Cleanup] Saved metrics to {metrics_path}", flush=True)

# Now close video writer - this may trigger Isaac Sim shutdown crash
print("[Cleanup] Finalizing video...", flush=True)
runner.close()

# Print results
print("\n" + "=" * 70, flush=True)
print("TEST RESULTS", flush=True)
print("=" * 70, flush=True)

print(f"\n  Scenario: {metrics.scenario_name}", flush=True)
print(f"  Success: {'PASS' if metrics.success else 'FAIL'}", flush=True)
print(f"  Goal Reached: {metrics.goal_reached}", flush=True)
print(f"  Collisions: {metrics.collision_count}", flush=True)
print(f"  Flight Time: {metrics.flight_time_seconds:.1f}s", flush=True)
print(f"  Path Length: {metrics.path_length_meters:.1f}m", flush=True)
print(f"  Min Clearance: {metrics.min_clearance_meters:.1f}m", flush=True)
print(f"  Avg Clearance: {metrics.avg_clearance_meters:.1f}m", flush=True)
print(f"  Depth FPS: {metrics.depth_fps_avg:.1f}", flush=True)
print(f"  VFH FPS: {metrics.vfh_fps_avg:.1f}", flush=True)

print(f"\nOutput files:", flush=True)
print(f"  Video: {OUTPUT_DIR}/video.mp4", flush=True)
print(f"  Metrics: {metrics_path}", flush=True)

print("\n" + "=" * 70, flush=True)
if metrics.success:
    print("TEST PASSED!", flush=True)
else:
    print("TEST FAILED - see metrics for details", flush=True)
print("=" * 70, flush=True)

# Cleanup
timeline.stop()
simulation_app.close()

sys.exit(0 if metrics.success else 1)
