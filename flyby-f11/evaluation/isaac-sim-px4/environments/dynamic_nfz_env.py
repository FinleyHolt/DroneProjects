#!/usr/bin/env python3
"""
Canonical Problem 2: Dynamic No-Fly Zone Avoidance

Mission: Complete a 3km point-to-point reconnaissance transit while
dynamically avoiding a No-Fly Zone that appears mid-mission (simulating
friendly aircraft corridor activation). Re-plan route in real-time while
maintaining mission timeline within 20% of original estimate.

Key Challenges:
- Dynamic NFZ activation mid-flight
- Real-time path replanning
- Timeline adherence under constraints
- NFZ buffer zone management

Per ONTOLOGY_FOUNDATION.qmd Section 1.2
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Tuple

from .base_isr_env import (
    BaseISREnvironment, EnvironmentConfig, UAVState,
    FlightPhase, CommsStatus, GNSSStatus, ThreatLevel,
    NoFlyZone, GeofenceConfig
)
from .domain_randomizer import DomainRandomizer, RandomizationConfig
from .ontology_bridge import OntologyStateBridge


@dataclass
class DynamicNFZConfig(EnvironmentConfig):
    """Configuration for Dynamic NFZ Avoidance environment."""
    # Start and destination
    start_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.5]))
    destination: np.ndarray = field(default_factory=lambda: np.array([3000.0, 500.0, 80.0]))

    # Mission parameters
    cruise_speed: float = 6.3  # m/s
    max_mission_time: float = 600.0  # 10 minutes (generous)

    # NFZ configuration
    nfz_activation_time: float = 180.0  # T+3 minutes
    nfz_center: np.ndarray = field(default_factory=lambda: np.array([1500.0, 250.0, 80.0]))
    nfz_dimensions: Tuple[float, float] = (1000.0, 500.0)  # length x width
    nfz_height: float = 200.0  # meters
    nfz_buffer: float = 50.0  # Warning zone

    # Success criteria
    destination_accuracy: float = 5.0  # meters
    max_timeline_ratio: float = 1.2  # 120% of planned
    min_battery_at_dest: float = 20.0  # %

    # Replanning
    replan_timeout: float = 2.0  # seconds allowed


@dataclass
class FlightPath:
    """Planned flight path."""
    waypoints: List[np.ndarray]
    total_distance: float
    estimated_time: float
    is_valid: bool = True
    invalidation_reason: Optional[str] = None


class DynamicNFZAvoidanceEnv(BaseISREnvironment):
    """
    Dynamic NFZ Avoidance Training Environment.

    Implements Canonical Problem 2 from ONTOLOGY_FOUNDATION.qmd.

    Reward Structure (from ontology document):
    - NFZ entry: -1000 (terminal failure)
    - NFZ proximity warning: -50 * penetration_ratio (buffer zone)
    - Replan performance: +30 (≤500ms), +10 (≤2000ms), -20 (>2000ms)
    - Progress to destination: +0.5 per meter
    - Timeline penalty: -5 * (elapsed_ratio - 1.0) if over 100%
    """

    def __init__(self, config: Optional[DynamicNFZConfig] = None):
        if config is None:
            config = DynamicNFZConfig()

        super().__init__(config)

        self.config: DynamicNFZConfig = config

        # Path planning state
        self.current_path: Optional[FlightPath] = None
        self.planned_time: float = 0.0
        self.replan_in_progress: bool = False
        self.replan_start_time: float = 0.0
        self.last_replan_duration: float = 0.0

        # NFZ state
        self.dynamic_nfz: Optional[NoFlyZone] = None
        self.nfz_activated: bool = False

        # Progress tracking
        self.distance_to_dest: float = 0.0
        self.initial_distance: float = 0.0

    def _setup_environment(self) -> None:
        """Setup transit environment elements."""
        # Calculate planned time
        start = self.config.start_position
        dest = self.config.destination
        self.initial_distance = np.linalg.norm(dest - start)
        self.planned_time = self.initial_distance / self.config.cruise_speed

        print(f"  Transit distance: {self.initial_distance:.1f}m")
        print(f"  Planned time: {self.planned_time:.1f}s ({self.planned_time/60:.1f} min)")

        # Create dynamic NFZ (inactive initially)
        nfz_center = self.config.nfz_center
        nfz_dims = self.config.nfz_dimensions

        # Model as cylinder for simplicity (use max dimension as radius)
        nfz_radius = max(nfz_dims) / 2

        self.dynamic_nfz = NoFlyZone(
            id="aircraft_corridor_alpha",
            center=nfz_center,
            radius=nfz_radius,
            height=self.config.nfz_height,
            priority=1,  # Highest priority (aircraft corridor)
            is_dynamic=True,
            is_active=False,
            buffer_distance=self.config.nfz_buffer,
        )

        self.nfz_zones = [self.dynamic_nfz]

        # Setup geofence (generous for 3km transit)
        self.config.geofence = GeofenceConfig(
            min_x=-500.0,
            max_x=4000.0,
            min_y=-500.0,
            max_y=1500.0,
            min_z=0.0,
            max_z=200.0,
        )

        # No threat zones for this problem
        self.threat_zones = []

        # Full comms for this problem
        self.comms_zones = []

        # Initial path (direct route)
        self.current_path = self._plan_direct_path()

        # Initialize domain randomizer
        rand_config = RandomizationConfig(
            randomize_lighting=self.config.randomize_lighting,
            randomize_weather=self.config.randomize_weather,
            randomize_sensors=self.config.randomize_sensors,
        )
        self.randomizer = DomainRandomizer(rand_config)

        # Initialize ontology bridge
        self.ontology_bridge = OntologyStateBridge()

        print(f"  NFZ activation: T+{self.config.nfz_activation_time}s")
        print(f"  NFZ location: {self.config.nfz_center}")

    def _plan_direct_path(self) -> FlightPath:
        """Plan direct path from current position to destination."""
        start = self.config.start_position
        dest = self.config.destination

        # Simple direct path with intermediate waypoint
        mid = (start + dest) / 2
        mid[2] = self.config.destination[2]  # Cruise altitude

        waypoints = [start.copy(), mid, dest.copy()]
        total_dist = np.linalg.norm(mid - start) + np.linalg.norm(dest - mid)

        return FlightPath(
            waypoints=waypoints,
            total_distance=total_dist,
            estimated_time=total_dist / self.config.cruise_speed,
            is_valid=True,
        )

    def _plan_avoidance_path(self) -> FlightPath:
        """Plan path avoiding the active NFZ."""
        pos = self.uav_state.position
        dest = self.config.destination
        nfz = self.dynamic_nfz

        # Simple avoidance: go around the NFZ
        # Determine if we should go north or south

        nfz_center = nfz.center
        to_dest = dest[:2] - pos[:2]

        # Vector from NFZ to current position
        from_nfz = pos[:2] - nfz_center[:2]

        # Check which side is shorter
        # Use cross product to determine side
        cross = to_dest[0] * from_nfz[1] - to_dest[1] * from_nfz[0]

        # Avoidance offset (perpendicular to direct path)
        avoidance_dist = nfz.radius + nfz.buffer_distance + 50.0  # Extra margin

        if cross > 0:
            # Go south of NFZ
            offset = np.array([0, -avoidance_dist])
        else:
            # Go north of NFZ
            offset = np.array([0, avoidance_dist])

        # Create waypoints
        # WP1: Current position
        # WP2: Move laterally to avoid NFZ
        # WP3: Point past NFZ
        # WP4: Destination

        wp1 = pos.copy()

        # Lateral move point
        wp2 = np.array([pos[0], nfz_center[1] + offset[1], dest[2]])

        # Past NFZ point
        wp3 = np.array([nfz_center[0] + nfz.radius + 100.0, nfz_center[1] + offset[1], dest[2]])

        # Back toward destination
        wp4 = dest.copy()

        waypoints = [wp1, wp2, wp3, wp4]

        # Calculate total distance
        total_dist = 0.0
        for i in range(len(waypoints) - 1):
            total_dist += np.linalg.norm(waypoints[i+1] - waypoints[i])

        return FlightPath(
            waypoints=waypoints,
            total_distance=total_dist,
            estimated_time=total_dist / self.config.cruise_speed,
            is_valid=True,
        )

    def reset(self, seed: Optional[int] = None) -> UAVState:
        """Reset environment for new episode."""
        state = super().reset(seed)

        # Reset path planning
        self.current_path = self._plan_direct_path()
        self.replan_in_progress = False
        self.last_replan_duration = 0.0

        # Reset NFZ
        self.dynamic_nfz.is_active = False
        self.nfz_activated = False

        # Reset progress
        self.distance_to_dest = np.linalg.norm(
            self.config.destination - self.config.start_position
        )
        self.initial_distance = self.distance_to_dest

        # Set flight phase
        state.flight_phase = FlightPhase.TRANSIT

        # Randomize NFZ position if enabled
        if self.randomizer and self.config.randomize_obstacles:
            self._randomize_nfz(seed)

        return state

    def _randomize_nfz(self, seed: Optional[int] = None) -> None:
        """Randomize NFZ position and timing."""
        if seed is not None:
            np.random.seed(seed)

        # Vary NFZ position along the transit corridor
        base_center = self.config.nfz_center.copy()
        offset = np.array([
            np.random.uniform(-200, 200),  # Along path
            np.random.uniform(-100, 100),  # Lateral
            0.0,
        ])
        self.dynamic_nfz.center = base_center + offset

        # Vary activation time
        base_time = self.config.nfz_activation_time
        self.config.nfz_activation_time = base_time + np.random.uniform(-30, 60)

    def _update_state(self) -> None:
        """Update state including NFZ dynamics."""
        super()._update_state()

        # Check NFZ activation
        if (self.uav_state.mission_time >= self.config.nfz_activation_time
                and not self.nfz_activated):
            self._activate_nfz()

        # Update distance to destination
        self.distance_to_dest = np.linalg.norm(
            self.config.destination - self.uav_state.position
        )

        # Check if path invalidated by NFZ
        if self.nfz_activated and self.current_path.is_valid:
            if self._path_intersects_nfz(self.current_path):
                self._trigger_replan()

        # Check replan completion
        if self.replan_in_progress:
            elapsed = self.uav_state.mission_time - self.replan_start_time
            if elapsed >= 0.1:  # Simulated replan time
                self._complete_replan()

    def _activate_nfz(self) -> None:
        """Activate the dynamic NFZ."""
        print(f">>> NFZ ACTIVATED at T+{self.uav_state.mission_time:.1f}s")
        print(f"    Aircraft corridor active at {self.dynamic_nfz.center}")

        self.dynamic_nfz.is_active = True
        self.nfz_activated = True

    def _path_intersects_nfz(self, path: FlightPath) -> bool:
        """Check if path intersects active NFZ."""
        if not self.dynamic_nfz.is_active:
            return False

        nfz = self.dynamic_nfz
        pos = self.uav_state.position

        # Check path from current position to destination
        for wp in path.waypoints:
            # Simple point check (more sophisticated would check segments)
            dist = np.linalg.norm(wp[:2] - nfz.center[:2])
            if dist < nfz.radius:
                return True

        # Also check direct line from current position
        to_dest = self.config.destination - pos
        to_dest_norm = to_dest / (np.linalg.norm(to_dest) + 1e-6)

        # Check at intervals along path
        check_dist = np.linalg.norm(self.config.destination - pos)
        for t in np.linspace(0, check_dist, 50):
            check_point = pos + to_dest_norm * t
            dist = np.linalg.norm(check_point[:2] - nfz.center[:2])
            if dist < nfz.radius:
                return True

        return False

    def _trigger_replan(self) -> None:
        """Trigger path replanning."""
        print(f">>> Path intersects NFZ - replanning...")
        self.current_path.is_valid = False
        self.current_path.invalidation_reason = "NFZ intersection"
        self.replan_in_progress = True
        self.replan_start_time = self.uav_state.mission_time

    def _complete_replan(self) -> None:
        """Complete the replanning process."""
        self.last_replan_duration = (
            self.uav_state.mission_time - self.replan_start_time
        ) * 1000  # Convert to ms

        # Generate new path
        self.current_path = self._plan_avoidance_path()
        self.replan_in_progress = False

        print(f">>> Replan complete in {self.last_replan_duration:.0f}ms")
        print(f"    New path distance: {self.current_path.total_distance:.1f}m")

    def compute_reward(self, state: UAVState, next_state: UAVState) -> float:
        """
        Compute reward per ONTOLOGY_FOUNDATION.qmd reward structure.
        """
        reward = 0.0

        # ==========================================
        # NFZ Safety (critical)
        # ==========================================

        if next_state.in_nfz:
            return -1000.0  # Terminal failure

        # NFZ proximity warning
        if next_state.nfz_distance < self.dynamic_nfz.buffer_distance:
            penetration = 1.0 - (next_state.nfz_distance / self.dynamic_nfz.buffer_distance)
            reward -= 50.0 * penetration

        # ==========================================
        # Replan performance
        # ==========================================

        if state.mission_time < next_state.mission_time:
            # Check if replan just completed
            if (not self.replan_in_progress
                    and self.last_replan_duration > 0
                    and self.current_path.is_valid):
                if self.last_replan_duration <= 500:
                    reward += 30.0
                elif self.last_replan_duration <= 2000:
                    reward += 10.0
                else:
                    reward -= 20.0
                self.last_replan_duration = 0  # Clear after reward

        # ==========================================
        # Progress to destination
        # ==========================================

        prev_dist = np.linalg.norm(self.config.destination - state.position)
        curr_dist = np.linalg.norm(self.config.destination - next_state.position)
        progress = prev_dist - curr_dist
        reward += 0.5 * progress

        # ==========================================
        # Timeline penalty
        # ==========================================

        elapsed_ratio = next_state.mission_time / self.planned_time

        if elapsed_ratio > 1.0:
            reward -= 5.0 * (elapsed_ratio - 1.0)

        # ==========================================
        # Destination reached bonus
        # ==========================================

        if curr_dist < self.config.destination_accuracy:
            # Check timeline
            if elapsed_ratio <= self.config.max_timeline_ratio:
                reward += 200.0  # Success!
            else:
                reward += 50.0  # Arrived but late

        return reward

    def is_done(self, state: UAVState) -> bool:
        """Check if episode is complete."""
        # NFZ violation (terminal)
        if state.in_nfz:
            return True

        # Destination reached
        dist = np.linalg.norm(self.config.destination - state.position)
        if dist < self.config.destination_accuracy:
            return True

        # Mission time exceeded
        if state.mission_time >= self.config.max_mission_time:
            return True

        # Battery depleted
        if state.battery_pct < 10.0:
            return True

        # Geofence violation
        if not state.in_geofence:
            return True

        return False

    def get_info(self, state: UAVState) -> Dict[str, Any]:
        """Get additional info about current state."""
        dist = np.linalg.norm(self.config.destination - state.position)
        elapsed_ratio = state.mission_time / self.planned_time if self.planned_time > 0 else 0

        return {
            # Progress
            "distance_to_dest": dist,
            "progress_pct": 100.0 * (1 - dist / self.initial_distance),
            "elapsed_time": state.mission_time,
            "planned_time": self.planned_time,
            "elapsed_ratio": elapsed_ratio,

            # NFZ status
            "nfz_active": self.nfz_activated,
            "nfz_distance": state.nfz_distance,
            "nfz_center": self.dynamic_nfz.center.tolist() if self.dynamic_nfz else None,

            # Path status
            "path_valid": self.current_path.is_valid if self.current_path else False,
            "replan_in_progress": self.replan_in_progress,
            "last_replan_ms": self.last_replan_duration,

            # Success criteria
            "destination_reached": dist < self.config.destination_accuracy,
            "timeline_met": elapsed_ratio <= self.config.max_timeline_ratio,
            "nfz_violations": 1 if state.in_nfz else 0,

            # Mission success
            "mission_success": (
                dist < self.config.destination_accuracy and
                elapsed_ratio <= self.config.max_timeline_ratio and
                not state.in_nfz
            ),

            # Ontology state
            "ontology_state": self.ontology_bridge.get_symbolic_state(
                state, self.nfz_zones, self.threat_zones, None
            ) if self.ontology_bridge else None,
        }

    @property
    def observation_dim(self) -> int:
        """Extended observation dimension for NFZ avoidance."""
        base_dim = super().observation_dim
        # Add: destination vector (3), nfz vector (3), nfz active, replan needed
        return base_dim + 8

    def state_to_observation(self, state: UAVState) -> np.ndarray:
        """Convert state to observation with transit features."""
        base_obs = super().state_to_observation(state)

        # Vector to destination (normalized)
        to_dest = self.config.destination - state.position
        dist = np.linalg.norm(to_dest)
        to_dest_norm = to_dest / (dist + 1e-6)

        # Vector to NFZ center (normalized)
        to_nfz = self.dynamic_nfz.center - state.position
        nfz_dist = np.linalg.norm(to_nfz)
        to_nfz_norm = to_nfz / (nfz_dist + 1e-6)

        extra_features = np.array([
            to_dest_norm[0], to_dest_norm[1], to_dest_norm[2],
            to_nfz_norm[0], to_nfz_norm[1], to_nfz_norm[2],
            float(self.nfz_activated),
            float(self.replan_in_progress),
        ], dtype=np.float32)

        return np.concatenate([base_obs, extra_features])
