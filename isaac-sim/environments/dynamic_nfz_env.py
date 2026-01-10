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
class NFZRandomizationConfig:
    """Configuration for NFZ domain randomization.

    Controls how NFZs are randomized each episode to improve RL policy
    generalization. Includes position, size, timing, and count randomization.
    """
    # Enable/disable randomization
    enabled: bool = True

    # Position randomization
    position_enabled: bool = True
    position_margin: float = 200.0  # Margin from geofence edges (meters)

    # Size randomization
    size_enabled: bool = True
    size_variation: float = 0.3  # +/- 30% from base radius

    # Activation timing randomization (for dynamic NFZs)
    timing_enabled: bool = True
    spawn_time_min: float = 0.0  # Earliest spawn time (seconds)
    spawn_time_max_ratio: float = 0.5  # Latest spawn = max_mission_time * ratio
    duration_min: float = 30.0  # Minimum active duration (seconds)
    duration_max: float = 120.0  # Maximum active duration (seconds)
    immediate_spawn_probability: float = 0.3  # Probability NFZ is present from start

    # Count randomization
    count_enabled: bool = True
    min_nfz_count: int = 2  # Minimum NFZs per episode
    max_nfz_count: int = 5  # Maximum NFZs per episode

    # NFZ base parameters (for generating new NFZs)
    base_radius_min: float = 200.0  # Minimum base radius (meters)
    base_radius_max: float = 500.0  # Maximum base radius (meters)
    base_height: float = 200.0  # NFZ height (meters)
    buffer_distance: float = 50.0  # Base buffer distance (meters)
    buffer_variation_enabled: bool = True  # Randomize buffer size
    buffer_min: float = 25.0  # Minimum buffer distance (meters)
    buffer_max: float = 100.0  # Maximum buffer distance (meters)


@dataclass
class DynamicNFZ:
    """Extended NFZ with timing information for domain randomization."""
    nfz: NoFlyZone
    spawn_time: float = 0.0  # When this NFZ becomes active
    duration: float = float('inf')  # How long it stays active (inf = permanent)
    deactivation_time: float = float('inf')  # spawn_time + duration

    def should_be_active(self, mission_time: float) -> bool:
        """Check if this NFZ should be active at the given mission time."""
        return self.spawn_time <= mission_time < self.deactivation_time


@dataclass
class DynamicNFZConfig(EnvironmentConfig):
    """Configuration for Dynamic NFZ Avoidance environment."""
    # Start and destination
    start_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.5]))
    destination: np.ndarray = field(default_factory=lambda: np.array([3000.0, 500.0, 80.0]))

    # Mission parameters
    cruise_speed: float = 6.3  # m/s
    max_mission_time: float = 600.0  # 10 minutes (generous)

    # NFZ configuration (base/default - can be randomized)
    nfz_activation_time: float = 180.0  # T+3 minutes
    nfz_center: np.ndarray = field(default_factory=lambda: np.array([1500.0, 250.0, 80.0]))
    nfz_dimensions: Tuple[float, float] = (1000.0, 500.0)  # length x width
    nfz_height: float = 200.0  # meters
    nfz_buffer: float = 50.0  # Warning zone

    # NFZ domain randomization
    nfz_randomization: NFZRandomizationConfig = field(default_factory=NFZRandomizationConfig)

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

        # NFZ state - now supports multiple dynamic NFZs with timing
        # self.dynamic_nfz removed - use self.dynamic_nfzs list
        self.dynamic_nfzs: List[DynamicNFZ] = []  # All dynamic NFZs with timing
        self.nfz_activated: bool = False
        self._nfz_activation_logged: set = set()  # Track which NFZs we've logged

        # Domain randomization RNG (consistent with base class naming)
        self._dr_rng: np.random.Generator = np.random.default_rng()

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

        # Setup geofence (generous for 3km transit)
        self.config.geofence = GeofenceConfig(
            min_x=-500.0,
            max_x=4000.0,
            min_y=-500.0,
            max_y=1500.0,
            min_z=0.0,
            max_z=200.0,
        )

        # Create initial dynamic NFZ (will be randomized in reset if enabled)
        nfz_center = self.config.nfz_center
        nfz_dims = self.config.nfz_dimensions

        # Model as cylinder for simplicity (use max dimension as radius)
        nfz_radius = max(nfz_dims) / 2

        # Create DynamicNFZ with timing support (no backward-compat primary NFZ)
        dynamic_nfz = NoFlyZone(
            id="aircraft_corridor_alpha",
            center=nfz_center,
            radius=nfz_radius,
            height=self.config.nfz_height,
            priority=1,  # Highest priority (aircraft corridor)
            is_dynamic=True,
            is_active=False,
            buffer_distance=self.config.nfz_buffer,
        )
        
        # Wrap in DynamicNFZ for timing support
        self.dynamic_nfzs = [
            DynamicNFZ(
                nfz=dynamic_nfz,
                spawn_time=self.config.nfz_activation_time,
                duration=float('inf'),  # Permanent once active
                deactivation_time=float('inf'),
            )
        ]

        self.nfz_zones = [dynamic_nfz]

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

        print(f"  NFZ randomization: {'ENABLED' if self.config.nfz_randomization.enabled else 'DISABLED'}")
        print(f"  Default NFZ activation: T+{self.config.nfz_activation_time}s")
        print(f"  Default NFZ location: {self.config.nfz_center}")

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
        nfz = self._get_closest_active_nfz()

        # Simple avoidance: go around the NFZ
        # Determine if we should go north or south
        if nfz is None:
            # No active NFZ, return direct path
            return self._plan_direct_path()

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

        # Initialize RNG with seed for reproducibility
        if seed is not None:
            self._dr_rng = np.random.default_rng(seed)
        else:
            self._dr_rng = np.random.default_rng()

        # Reset path planning
        self.current_path = self._plan_direct_path()
        self.replan_in_progress = False
        self.last_replan_duration = 0.0

        # Reset NFZ activation tracking
        self._nfz_activation_logged = set()
        self.nfz_activated = False

        # Randomize NFZs if enabled (before resetting their state)
        nfz_rand = self.config.nfz_randomization
        if nfz_rand.enabled:
            self._randomize_nfzs()
        else:
            # Reset to default single NFZ
            self._reset_default_nfz()

        # Reset progress
        self.distance_to_dest = np.linalg.norm(
            self.config.destination - self.config.start_position
        )
        self.initial_distance = self.distance_to_dest

        # Set flight phase
        state.flight_phase = FlightPhase.TRANSIT

        return state

    def _reset_default_nfz(self) -> None:
        """Reset to default single NFZ configuration (no randomization)."""
        nfz_center = self.config.nfz_center.copy()
        nfz_dims = self.config.nfz_dimensions
        nfz_radius = max(nfz_dims) / 2

        dynamic_nfz = NoFlyZone(
            id="aircraft_corridor_alpha",
            center=nfz_center,
            radius=nfz_radius,
            height=self.config.nfz_height,
            priority=1,
            is_dynamic=True,
            is_active=False,
            buffer_distance=self.config.nfz_buffer,
        )

        self.dynamic_nfzs = [
            DynamicNFZ(
                nfz=dynamic_nfz,
                spawn_time=self.config.nfz_activation_time,
                duration=float('inf'),
                deactivation_time=float('inf'),
            )
        ]

        self.nfz_zones = [dynamic_nfz]

    def _randomize_nfzs(self) -> None:
        """
        Randomize NFZ configuration for domain randomization.

        Applies randomization based on NFZRandomizationConfig:
        - Position: Random positions within geofence bounds (with margin)
        - Size: Vary radius by +/- size_variation from base
        - Timing: Randomize spawn time and duration for dynamic NFZs
        - Count: Vary number of NFZs per episode
        """
        nfz_rand = self.config.nfz_randomization
        geofence = self.config.geofence

        # Determine NFZ count
        if nfz_rand.count_enabled:
            nfz_count = self._dr_rng.integers(
                nfz_rand.min_nfz_count,
                nfz_rand.max_nfz_count + 1
            )
        else:
            nfz_count = 1  # Default single NFZ

        # Clear existing NFZs
        self.dynamic_nfzs = []
        self.nfz_zones = []

        # Calculate position bounds (with margin from geofence edges)
        margin = nfz_rand.position_margin
        pos_min_x = geofence.min_x + margin
        pos_max_x = geofence.max_x - margin
        pos_min_y = geofence.min_y + margin
        pos_max_y = geofence.max_y - margin

        # Calculate spawn time bounds
        spawn_time_max = self.config.max_mission_time * nfz_rand.spawn_time_max_ratio

        for i in range(nfz_count):
            # Generate NFZ ID
            nfz_id = f"dynamic_nfz_{i:02d}"

            # Randomize position
            if nfz_rand.position_enabled:
                center_x = self._dr_rng.uniform(pos_min_x, pos_max_x)
                center_y = self._dr_rng.uniform(pos_min_y, pos_max_y)
                # Use cruise altitude as default Z, with small variation
                center_z = self.config.destination[2] + self._dr_rng.uniform(-20, 20)
            else:
                # Use default center with small offset
                center_x = self.config.nfz_center[0] + i * 200
                center_y = self.config.nfz_center[1]
                center_z = self.config.nfz_center[2]

            center = np.array([center_x, center_y, center_z])

            # Randomize size
            if nfz_rand.size_enabled:
                base_radius = self._dr_rng.uniform(
                    nfz_rand.base_radius_min,
                    nfz_rand.base_radius_max
                )
                # Apply +/- variation
                variation = self._dr_rng.uniform(
                    1.0 - nfz_rand.size_variation,
                    1.0 + nfz_rand.size_variation
                )
                radius = base_radius * variation
            else:
                radius = max(self.config.nfz_dimensions) / 2


            # Randomize buffer distance
            if nfz_rand.buffer_variation_enabled:
                buffer_dist = self._dr_rng.uniform(
                    nfz_rand.buffer_min,
                    nfz_rand.buffer_max
                )
            else:
                buffer_dist = nfz_rand.buffer_distance

            # Randomize timing
            if nfz_rand.timing_enabled:
                # Determine if this NFZ is present from start
                if self._dr_rng.random() < nfz_rand.immediate_spawn_probability:
                    spawn_time = 0.0
                    is_active_initially = True
                else:
                    spawn_time = self._dr_rng.uniform(
                        nfz_rand.spawn_time_min,
                        spawn_time_max
                    )
                    is_active_initially = False

                # Randomize duration
                duration = self._dr_rng.uniform(
                    nfz_rand.duration_min,
                    nfz_rand.duration_max
                )
                deactivation_time = spawn_time + duration
            else:
                spawn_time = self.config.nfz_activation_time
                duration = float('inf')
                deactivation_time = float('inf')
                is_active_initially = False

            # Create NoFlyZone
            nfz = NoFlyZone(
                id=nfz_id,
                center=center,
                radius=radius,
                height=nfz_rand.base_height,
                priority=1,  # Aircraft corridor priority
                is_dynamic=True,
                is_active=is_active_initially,
                buffer_distance=buffer_dist,
            )

            # Create DynamicNFZ wrapper with timing
            dynamic_nfz = DynamicNFZ(
                nfz=nfz,
                spawn_time=spawn_time,
                duration=duration,
                deactivation_time=deactivation_time,
            )

            self.dynamic_nfzs.append(dynamic_nfz)
            self.nfz_zones.append(nfz)

        # Ensure at least one NFZ exists
        if not self.dynamic_nfzs:
            # Fallback: create a default NFZ to prevent crashes
            print("  Warning: NFZ randomization generated 0 NFZs, creating default NFZ")
            self._reset_default_nfz()
            return



        # Log randomization results
        active_count = sum(1 for d in self.dynamic_nfzs if d.nfz.is_active)
        print(f"  NFZ randomization: {nfz_count} NFZs generated")
        print(f"    - Active from start: {active_count}")
        print(f"    - Appearing mid-mission: {nfz_count - active_count}")

    def _update_state(self) -> None:
        """Update state including NFZ dynamics."""
        super()._update_state()

        # Update dynamic NFZ activation states based on mission time
        self._update_nfz_activation_states()

        # Update distance to destination
        self.distance_to_dest = np.linalg.norm(
            self.config.destination - self.uav_state.position
        )

        # Check if path invalidated by any active NFZ
        if self._any_nfz_active() and self.current_path.is_valid:
            if self._path_intersects_nfz(self.current_path):
                self._trigger_replan()

        # Check replan completion
        if self.replan_in_progress:
            elapsed = self.uav_state.mission_time - self.replan_start_time
            if elapsed >= 0.1:  # Simulated replan time
                self._complete_replan()

    def _update_nfz_activation_states(self) -> None:
        """Update activation states for all dynamic NFZs based on mission time."""
        mission_time = self.uav_state.mission_time
        any_newly_activated = False

        for dynamic_nfz in self.dynamic_nfzs:
            should_be_active = dynamic_nfz.should_be_active(mission_time)
            nfz = dynamic_nfz.nfz

            # Check for state change
            if should_be_active and not nfz.is_active:
                # NFZ is activating
                nfz.is_active = True
                any_newly_activated = True

                # Log activation (only once per NFZ)
                if nfz.id not in self._nfz_activation_logged:
                    self._nfz_activation_logged.add(nfz.id)
                    print(f">>> NFZ '{nfz.id}' ACTIVATED at T+{mission_time:.1f}s")
                    print(f"    Center: {nfz.center}, Radius: {nfz.radius:.0f}m")
                    if dynamic_nfz.duration < float('inf'):
                        print(f"    Duration: {dynamic_nfz.duration:.0f}s")

            elif not should_be_active and nfz.is_active:
                # NFZ is deactivating
                nfz.is_active = False
                print(f">>> NFZ '{nfz.id}' DEACTIVATED at T+{mission_time:.1f}s")

        # Update global activated flag
        if any_newly_activated:
            self.nfz_activated = True

    def _any_nfz_active(self) -> bool:
        """Check if any NFZ is currently active."""
        return any(d.nfz.is_active for d in self.dynamic_nfzs)


    def _get_closest_active_nfz(self) -> Optional[NoFlyZone]:
        """Get the closest active NFZ to current position, or None if no NFZs active."""
        if not self.dynamic_nfzs:
            return None
        
        pos = self.uav_state.position
        active_nfzs = [d.nfz for d in self.dynamic_nfzs if d.nfz.is_active]
        
        if not active_nfzs:
            return None
        
        # Find closest
        closest = None
        min_dist = float('inf')
        for nfz in active_nfzs:
            dist = np.linalg.norm(pos[:2] - nfz.center[:2])
            if dist < min_dist:
                min_dist = dist
                closest = nfz
        
        return closest

    def _get_min_active_buffer_distance(self) -> float:
        """Get the minimum buffer distance among all active NFZs."""
        active_buffers = [
            d.nfz.buffer_distance for d in self.dynamic_nfzs if d.nfz.is_active
        ]
        if not active_buffers:
            return 0.0
        return min(active_buffers)

    def _path_intersects_nfz(self, path: FlightPath) -> bool:
        """Check if path intersects any active NFZ."""
        # Get all active NFZs
        active_nfzs = [d.nfz for d in self.dynamic_nfzs if d.nfz.is_active]
        if not active_nfzs:
            return False

        pos = self.uav_state.position

        for nfz in active_nfzs:
            # Check path waypoints
            for wp in path.waypoints:
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
        Compute reward for dynamic NFZ avoidance.

        Updated reward structure (removes arbitrary time constraint):
        - NFZ entry: -1000 (terminal failure)
        - NFZ proximity warning: -50 * penetration_ratio (buffer zone)
        - Replan performance: +30 (≤500ms), +10 (≤2000ms), -20 (>2000ms)
        - Progress to destination: +0.5 per meter
        - Small time penalty: -0.01 per step (encourages efficiency without hard cutoff)
        - Arrival bonus: +100 base + battery efficiency bonus
        """
        reward = 0.0

        # ==========================================
        # NFZ Safety (critical)
        # ==========================================

        if next_state.in_nfz:
            return -1000.0  # Terminal failure

        # NFZ proximity warning - check against all active NFZs
        min_buffer = self._get_min_active_buffer_distance()
        if min_buffer > 0 and next_state.nfz_distance < min_buffer:
            penetration = 1.0 - (next_state.nfz_distance / min_buffer)
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
        # Small time penalty (encourages efficiency without hard cutoff)
        # ==========================================

        reward -= 0.01  # Per step

        # ==========================================
        # Destination reached bonus with battery efficiency
        # ==========================================

        if curr_dist < self.config.destination_accuracy:
            # Base arrival bonus
            reward += 100.0

            # Battery efficiency bonus: +1 per % above minimum
            # Encourages finding efficient paths that conserve battery
            battery_surplus = next_state.battery_pct - self.config.min_battery_at_dest
            if battery_surplus > 0:
                reward += battery_surplus  # +1 per % above minimum

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

        # Battery depleted (can't continue safely)
        if state.battery_pct < 5.0:
            return True

        # Geofence violation
        if not state.in_geofence:
            return True

        # Note: Removed hard time limit - agent learns efficiency through
        # small time penalty and battery depletion instead

        return False

    def get_info(self, state: UAVState) -> Dict[str, Any]:
        """Get additional info about current state."""
        dist = np.linalg.norm(self.config.destination - state.position)
        elapsed_ratio = state.mission_time / self.planned_time if self.planned_time > 0 else 0
        destination_reached = dist < self.config.destination_accuracy
        battery_ok = state.battery_pct >= self.config.min_battery_at_dest

        # Collect NFZ information
        active_nfz_count = sum(1 for d in self.dynamic_nfzs if d.nfz.is_active)
        nfz_details = [
            {
                "id": d.nfz.id,
                "center": d.nfz.center.tolist(),
                "radius": d.nfz.radius,
                "is_active": d.nfz.is_active,
                "spawn_time": d.spawn_time,
                "duration": d.duration if d.duration < float('inf') else None,
            }
            for d in self.dynamic_nfzs
        ]

        return {
            # Progress
            "distance_to_dest": dist,
            "progress_pct": 100.0 * (1 - dist / self.initial_distance),
            "elapsed_time": state.mission_time,
            "planned_time": self.planned_time,
            "elapsed_ratio": elapsed_ratio,

            # NFZ status (updated for multiple NFZs)
            "nfz_active": self.nfz_activated,
            "nfz_distance": state.nfz_distance,
            "nfz_center": self.dynamic_nfzs[0].nfz.center.tolist() if self.dynamic_nfzs else None,
            "nfz_count": len(self.dynamic_nfzs),
            "active_nfz_count": active_nfz_count,
            "nfz_details": nfz_details,

            # Path status
            "path_valid": self.current_path.is_valid if self.current_path else False,
            "replan_in_progress": self.replan_in_progress,
            "last_replan_ms": self.last_replan_duration,

            # Success criteria (updated - no hard timeline)
            "destination_reached": destination_reached,
            "battery_ok": battery_ok,
            "battery_surplus": max(0, state.battery_pct - self.config.min_battery_at_dest),
            "nfz_violations": 1 if state.in_nfz else 0,

            # Mission success: arrived safely with enough battery
            "mission_success": destination_reached and battery_ok and not state.in_nfz,

            # NFZ randomization info
            "nfz_randomization_enabled": self.config.nfz_randomization.enabled,

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
        # Vector to closest active NFZ (or zero if none active)
        closest_nfz = self._get_closest_active_nfz()
        if closest_nfz is not None:
            to_nfz = closest_nfz.center - state.position
            nfz_dist = np.linalg.norm(to_nfz)
            to_nfz_norm = to_nfz / (nfz_dist + 1e-6)
        else:
            to_nfz_norm = np.zeros(3)

        extra_features = np.array([
            to_dest_norm[0], to_dest_norm[1], to_dest_norm[2],
            to_nfz_norm[0], to_nfz_norm[1], to_nfz_norm[2],
            float(self.nfz_activated),
            float(self.replan_in_progress),
        ], dtype=np.float32)

        return np.concatenate([base_obs, extra_features])
