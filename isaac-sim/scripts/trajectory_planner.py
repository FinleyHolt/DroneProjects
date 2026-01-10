"""
Trajectory rollout planner for proactive obstacle avoidance.

Instead of reactive VFH (looking 20m ahead), this planner:
1. Generates candidate arc trajectories (5-10 second horizon)
2. Ray-casts each trajectory against depth field
3. Scores by clearance + goal progress + smoothness
4. Returns best trajectory for execution

Works directly with depth images - no OctoMap needed.

Author: Finley Holt
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class TrajectoryCandidate:
    """A candidate trajectory to evaluate."""
    turn_rate: float          # Turn rate used (rad/s)
    headings: np.ndarray      # Heading at each timestep (rad)
    positions: np.ndarray     # XY positions at each timestep (m)
    clearances: np.ndarray    # Min clearance at each point (m)
    goal_progress: float      # Progress toward goal (m)
    smoothness: float         # Heading change penalty
    score: float              # Combined score


@dataclass
class TrajectoryPlanResult:
    """Result of trajectory planning."""
    recommended_heading: float      # Heading to execute (rad, relative to current)
    recommended_speed: float        # Speed to execute (m/s)
    min_clearance: float            # Minimum clearance along best trajectory
    best_candidate: TrajectoryCandidate
    all_candidates: List[TrajectoryCandidate]
    planning_time_ms: float


class TrajectoryRolloutPlanner:
    """
    Proactive trajectory planner using depth field evaluation.

    Generates candidate trajectories as circular arcs with different
    turn rates, evaluates each against the depth field, and returns
    the best trajectory for execution.

    Key insight: Instead of asking "what's blocking me now?" (VFH),
    we ask "where does each possible path lead in 5 seconds?"
    """

    def __init__(
        self,
        horizon_seconds: float = 5.0,       # Planning horizon
        timestep: float = 0.5,              # Evaluation timestep
        num_candidates: int = 21,           # Number of heading options
        max_turn_rate: float = 0.5,         # Max rad/s turn rate
        speed: float = 8.0,                 # Forward speed (m/s)
        safety_margin: float = 10.0,        # Desired min clearance (m)
        collision_threshold: float = 3.0,   # Hard collision threshold (m)
        hfov_deg: float = 160.0,            # Camera horizontal FOV
    ):
        """
        Initialize trajectory rollout planner.

        Args:
            horizon_seconds: How far ahead to plan (seconds)
            timestep: Time between trajectory evaluation points
            num_candidates: Number of different turn rates to sample
            max_turn_rate: Maximum turn rate to consider (rad/s)
            speed: Nominal forward speed (m/s)
            safety_margin: Preferred minimum clearance from obstacles
            collision_threshold: Hard limit - trajectory rejected below this
            hfov_deg: Camera horizontal field of view
        """
        self.horizon = horizon_seconds
        self.dt = timestep
        self.num_candidates = num_candidates
        self.max_turn_rate = max_turn_rate
        self.speed = speed
        self.safety_margin = safety_margin
        self.collision_threshold = collision_threshold
        self.hfov = np.radians(hfov_deg)

        # Generate turn rates to sample (centered on 0 = straight)
        self.turn_rates = np.linspace(
            -max_turn_rate, max_turn_rate, num_candidates
        )

        # Number of steps in trajectory
        self.num_steps = int(horizon_seconds / timestep)

        # State for hysteresis
        self._prev_turn_rate: Optional[float] = None
        self._hysteresis_bonus = 20.0  # Score bonus for consistency

    def plan(
        self,
        depth_map: np.ndarray,
        goal_direction: float,  # Direction to goal relative to camera (rad)
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> TrajectoryPlanResult:
        """
        Plan best trajectory given current depth field.

        Args:
            depth_map: Metric depth (H, W) in meters
            goal_direction: Direction to goal relative to camera forward (rad)
                           Positive = goal is to the left
            fx, fy, cx, cy: Camera intrinsics

        Returns:
            TrajectoryPlanResult with recommended heading and all candidates
        """
        import time
        start_time = time.time()

        candidates = []

        for turn_rate in self.turn_rates:
            candidate = self._evaluate_trajectory(
                depth_map, turn_rate, goal_direction, fx, fy, cx, cy
            )
            candidates.append(candidate)

        # Apply hysteresis bonus to previous turn rate
        if self._prev_turn_rate is not None:
            for candidate in candidates:
                if abs(candidate.turn_rate - self._prev_turn_rate) < 0.1:
                    candidate.score += self._hysteresis_bonus

        # Sort by score (higher is better)
        candidates.sort(key=lambda c: c.score, reverse=True)
        best = candidates[0]

        # Update hysteresis state
        self._prev_turn_rate = best.turn_rate

        # Compute recommended heading (relative change for first timestep)
        # The heading at step 1 is the immediate heading to follow
        recommended_heading = best.turn_rate * self.dt

        # Reduce speed if clearance is low
        min_clearance = float(np.min(best.clearances))
        if min_clearance < self.safety_margin:
            speed_factor = min_clearance / self.safety_margin
            recommended_speed = self.speed * max(0.3, speed_factor)
        else:
            recommended_speed = self.speed

        planning_time = (time.time() - start_time) * 1000

        return TrajectoryPlanResult(
            recommended_heading=recommended_heading,
            recommended_speed=recommended_speed,
            min_clearance=min_clearance,
            best_candidate=best,
            all_candidates=candidates,
            planning_time_ms=planning_time,
        )

    def _evaluate_trajectory(
        self,
        depth_map: np.ndarray,
        turn_rate: float,
        goal_direction: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> TrajectoryCandidate:
        """Evaluate a single trajectory candidate."""
        h, w = depth_map.shape

        headings = np.zeros(self.num_steps + 1)
        positions = np.zeros((self.num_steps + 1, 2))
        clearances = np.full(self.num_steps + 1, 200.0)

        headings[0] = 0.0  # Start facing forward (camera frame)
        positions[0] = [0, 0]  # Start at origin

        # Simulate trajectory
        for i in range(self.num_steps):
            # Update heading (accumulate turn)
            headings[i + 1] = headings[i] + turn_rate * self.dt

            # Update position (forward motion in heading direction)
            # In camera frame: +X is forward, +Y is left
            dx = self.speed * self.dt * np.cos(headings[i + 1])
            dy = self.speed * self.dt * np.sin(headings[i + 1])
            positions[i + 1] = positions[i] + [dx, dy]

            # Check clearance at this position by ray-casting into depth
            clearances[i + 1] = self._check_clearance(
                depth_map, positions[i + 1], fx, cx, w, h
            )

        clearances[0] = clearances[1]  # Copy first clearance

        # Compute goal progress (projection onto goal direction)
        # Final position dotted with goal direction unit vector
        goal_progress = (
            positions[-1, 0] * np.cos(goal_direction) +
            positions[-1, 1] * np.sin(goal_direction)
        )

        # Smoothness penalty (prefer straight paths)
        smoothness = -abs(turn_rate) * 10.0

        # Compute score
        min_clearance = float(np.min(clearances))
        avg_clearance = float(np.mean(clearances))

        if min_clearance < self.collision_threshold:
            # Trajectory gets too close to obstacle - heavy penalty
            # Still score based on clearance so "least bad" can be chosen
            score = -1000.0 + min_clearance * 100
        else:
            # Good trajectory - balance clearance vs goal progress
            score = (
                min_clearance * 5.0 +       # Min clearance bonus
                avg_clearance * 2.0 +       # Average clearance
                goal_progress * 3.0 +       # Goal progress
                smoothness                  # Smoothness (negative for turns)
            )

        return TrajectoryCandidate(
            turn_rate=turn_rate,
            headings=headings,
            positions=positions,
            clearances=clearances,
            goal_progress=goal_progress,
            smoothness=smoothness,
            score=score,
        )

    def _check_clearance(
        self,
        depth_map: np.ndarray,
        position: np.ndarray,
        fx: float,
        cx: float,
        w: int,
        h: int,
    ) -> float:
        """
        Check clearance at a trajectory point by sampling depth field.

        Projects the trajectory point into camera frame and samples depth
        at that angle to determine if there's an obstacle.

        Args:
            depth_map: Metric depth (H, W)
            position: XY position in camera frame (X forward, Y left)
            fx: Focal length X
            cx: Principal point X
            w, h: Image dimensions

        Returns:
            Clearance in meters (depth at that angle minus distance to point)
        """
        distance = np.linalg.norm(position)
        if distance < 0.1:
            return 200.0  # At origin, assume clear

        # Angle to this point from camera (positive = left)
        angle_to_point = np.arctan2(position[1], position[0])

        # Check if point is within FOV
        if abs(angle_to_point) > self.hfov / 2:
            # Outside FOV - we can't see it, conservatively assume some clearance
            # but not infinite (we don't know what's there)
            return 50.0

        # Project angle to image column
        # Camera convention: angle=0 is center, positive angle = left = lower u
        # u = cx - angle * (fx / something) but simpler:
        # Map angle from [-hfov/2, +hfov/2] to [w-1, 0]
        normalized_angle = angle_to_point / (self.hfov / 2)  # -1 to +1
        u = int(cx - normalized_angle * cx)  # Map to image
        u = np.clip(u, 0, w - 1)

        # Sample depth in a vertical band (avoid ground/sky)
        v_start = int(h * 0.25)
        v_end = int(h * 0.75)

        # Sample region around projected column (±15 pixels for robustness)
        margin = 15
        u_start = max(0, u - margin)
        u_end = min(w, u + margin)

        region = depth_map[v_start:v_end, u_start:u_end]

        # Find minimum valid depth in region
        valid_mask = (region > 0.5) & (region < 200)
        if not np.any(valid_mask):
            return 200.0  # No valid depth readings, assume clear

        depth_at_angle = float(np.min(region[valid_mask]))

        # Clearance = depth minus how far along the trajectory this point is
        clearance = depth_at_angle - distance

        return max(0.1, clearance)

    def reset(self):
        """Reset planner state (call when starting new navigation)."""
        self._prev_turn_rate = None

    def get_debug_info(self) -> dict:
        """Get debug information about planner state."""
        return {
            "horizon_seconds": self.horizon,
            "num_candidates": self.num_candidates,
            "num_steps": self.num_steps,
            "speed": self.speed,
            "safety_margin": self.safety_margin,
            "prev_turn_rate": self._prev_turn_rate,
        }


# Convenience function to visualize trajectory candidates
def visualize_trajectories(
    candidates: List[TrajectoryCandidate],
    best_idx: int = 0,
) -> np.ndarray:
    """
    Create a simple bird's-eye visualization of trajectory candidates.

    Returns:
        Image (H, W, 3) with trajectories drawn
    """
    import cv2

    img_size = 400
    scale = 3.0  # pixels per meter
    img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

    # Draw all candidates (gray)
    for i, candidate in enumerate(candidates):
        color = (100, 100, 100) if i != best_idx else (0, 255, 0)
        thickness = 1 if i != best_idx else 2

        for j in range(len(candidate.positions) - 1):
            p1 = candidate.positions[j]
            p2 = candidate.positions[j + 1]

            # Convert to image coordinates (origin at center-bottom)
            x1 = int(img_size // 2 + p1[1] * scale)
            y1 = int(img_size - 20 - p1[0] * scale)
            x2 = int(img_size // 2 + p2[1] * scale)
            y2 = int(img_size - 20 - p2[0] * scale)

            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    # Draw drone position
    cv2.circle(img, (img_size // 2, img_size - 20), 5, (255, 255, 255), -1)

    return img
