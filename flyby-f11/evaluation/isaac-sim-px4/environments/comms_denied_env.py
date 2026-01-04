#!/usr/bin/env python3
"""
Canonical Problem 1: Comms-Denied Area Surveillance

Mission: Conduct persistent surveillance of a 500m × 500m designated area
for 15 minutes while operating without ground station communication link,
then autonomously return to launch with collected imagery.

Key Challenges:
- Comms loss triggers autonomous mode with enhanced battery reserve (30%)
- GPS degradation requires VIO for navigation
- Coverage maximization within time/battery constraints
- Safe return to launch without external guidance

Per ONTOLOGY_FOUNDATION.qmd Section 1.1
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any

from .base_isr_env import (
    BaseISREnvironment, EnvironmentConfig, UAVState,
    FlightPhase, CommsStatus, GNSSStatus, ThreatLevel,
    NoFlyZone, CommsZone, GeofenceConfig
)
from .domain_randomizer import DomainRandomizer, RandomizationConfig
from .ontology_bridge import OntologyStateBridge


@dataclass
class CommsDeniedConfig(EnvironmentConfig):
    """Configuration for Comms-Denied Surveillance environment."""
    # Skip default gridroom since we use procedural world generation
    skip_default_environment: bool = True

    # Surveillance area (centered at origin)
    surveillance_area_size: float = 500.0  # meters

    # Mission parameters
    max_mission_time: float = 900.0  # 15 minutes
    target_coverage: float = 85.0  # % required
    target_poi_count: int = 10  # POI captures required

    # Comms denial timing
    comms_denial_time: float = 30.0  # seconds after start

    # GPS degradation
    gps_hdop_threshold: float = 5.0  # HDOP > 5.0 = degraded

    # Success criteria
    return_accuracy: float = 3.0  # meters from launch
    min_battery_at_landing: float = 15.0  # %

    # POI locations (generated or fixed)
    poi_positions: List[np.ndarray] = field(default_factory=list)
    num_random_pois: int = 15  # If poi_positions empty

    # Mission tasking (operator-provided, or auto-generated)
    # If None, a default tasking is generated based on surveillance_area_size
    mission_tasking: Optional[Any] = None  # MissionTasking object


@dataclass
class POI:
    """Point of Interest for surveillance."""
    id: str
    position: np.ndarray
    captured: bool = False
    capture_time: Optional[float] = None


class CommsDeniedSurveillanceEnv(BaseISREnvironment):
    """
    Comms-Denied Area Surveillance Training Environment.

    Implements Canonical Problem 1 from ONTOLOGY_FOUNDATION.qmd.

    Reward Structure (from ontology document):
    - Safety violations: -100 (geofence), -500 (NFZ)
    - Obstacle proximity: -50 * (3.0 - distance) if < 3m
    - Battery reserve violation: -10 * (30 - battery%) if < 30% and not returning
    - Coverage gain: +2 per % coverage
    - POI capture: +10 per POI
    - Mission success bonus: +100 (coverage >= 85% AND POI >= 10)
    """

    def __init__(self, config: Optional[CommsDeniedConfig] = None):
        if config is None:
            config = CommsDeniedConfig()

        super().__init__(config)

        self.config: CommsDeniedConfig = config

        # Surveillance-specific state
        self.pois: List[POI] = []
        self.coverage_grid: Optional[np.ndarray] = None
        self.grid_resolution: float = 10.0  # meters per cell

        # Comms zone (full coverage initially, then removed)
        self.comms_operational = True

        # Return tracking
        self.returning_to_launch = False
        self.launch_position = config.spawn_position.copy()

        # Detection tracking for adaptive search learning
        # Helps agent learn that POIs cluster together
        self._detection_times: List[float] = []
        self._detection_window: float = 30.0  # seconds to consider "recent"

    def _setup_environment(self) -> None:
        """Setup surveillance environment with optional procedural world generation."""
        import random as py_random
        import sys

        # Import mission tasking system
        from .mission_tasking import (
            MissionTasking, create_comms_denied_tasking,
            generate_poi_distribution_from_tasking, TargetType
        )

        # Create or use provided mission tasking
        if self.config.mission_tasking is not None:
            self.mission_tasking = self.config.mission_tasking
        else:
            # Generate default tasking based on surveillance area
            # Create two NAIs within the surveillance area
            half_size = self.config.surveillance_area_size / 2
            primary_center = (half_size * 0.4, half_size * 0.3)  # Offset from center
            secondary_center = (-half_size * 0.3, half_size * 0.4)

            self.mission_tasking = create_comms_denied_tasking(
                surveillance_center=primary_center,
                surveillance_radius=half_size * 0.4,
                secondary_area=secondary_center,
                expected_comms_loss_time=self.config.comms_denial_time,
            )

        # Log the mission tasking
        print("\n" + "=" * 60)
        print("MISSION TASKING RECEIVED")
        print("=" * 60)
        print(self.mission_tasking.to_natural_language())
        print("=" * 60 + "\n")

        # Initialize coverage grid
        grid_size = int(self.config.surveillance_area_size / self.grid_resolution)
        self.coverage_grid = np.zeros((grid_size, grid_size), dtype=bool)

        half_size = self.config.surveillance_area_size / 2

        # World generator reference (set during Isaac Sim initialization)
        self.world_gen = None

        # Try to use world generator for realistic POI clusters
        if self.simulation_app is not None:
            try:
                # Add extension path for world generator import
                ext_path = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
                if ext_path not in sys.path:
                    sys.path.insert(0, ext_path)

                from flyby.world_generator import WorldGenerator, WorldConfig

                world_config = WorldConfig(
                    terrain_size=(self.config.surveillance_area_size,
                                  self.config.surveillance_area_size),
                    terrain_roughness=3.0,
                    randomize_lighting=self.config.randomize_lighting,
                    seed=None,  # Random each episode
                )

                self.world_gen = WorldGenerator(
                    models_path="/workspace/extensions/forest_generator/models",
                    config=world_config,
                )

                # Generate terrain and lighting
                self.world_gen.generate_terrain()
                self.world_gen.setup_lighting()

                # Register drone spawn position as exclusion zone BEFORE generating trees
                # This prevents trees from spawning on top of the drone
                from flyby.world_generator.spawners.base_spawner import BaseSpawner
                spawn_pos = self.config.spawn_position
                drone_exclusion_radius = 10.0  # 10m clearance around spawn point
                BaseSpawner.clear_position_registry()  # Clear any stale positions
                # Register at spawn (x, y) with large radius to prevent trees there
                # Note: We use a static method via the class directly
                from flyby.world_generator.spawners import base_spawner
                base_spawner._global_positions.append((spawn_pos[0], spawn_pos[1], drone_exclusion_radius))
                print(f"  [WorldGen] Registered drone exclusion zone at ({spawn_pos[0]:.1f}, {spawn_pos[1]:.1f}) radius {drone_exclusion_radius}m")

                # Generate forest/vegetation for realistic ISR environment
                # Density = trees per 100m². For 500m x 500m area:
                # density=0.3 gives 750 trees (sparse forest for testing)
                # Note: Tree type keys must be capitalized to match DEFAULT_TREES
                self.world_gen.generate_forest(
                    density=0.3,  # Sparse forest: 0.3 trees per 100m² = 750 trees total
                    proportions={"Pine": 40, "Birch": 30, "Spruce": 30},
                )

                # Generate POI distribution based on mission tasking
                # POIs are clustered near NAIs with spread based on intel confidence
                poi_distribution = generate_poi_distribution_from_tasking(
                    self.mission_tasking,
                    seed=py_random.randint(0, 999999),
                )

                print(f"  [WorldGen] Spawning {len(poi_distribution)} targets based on mission tasking")

                # Group POIs by NAI for cluster spawning
                nai_targets = {}
                unknown_targets = []
                for poi_data in poi_distribution:
                    nai_id = poi_data.get("nai_id", "UNKNOWN")
                    if nai_id == "UNKNOWN":
                        unknown_targets.append(poi_data)
                    else:
                        if nai_id not in nai_targets:
                            nai_targets[nai_id] = []
                        nai_targets[nai_id].append(poi_data)

                # Spawn targets for each NAI cluster
                self.pois = []
                import omni.isaac.core.utils.prims as prim_utils

                for nai_id, targets in nai_targets.items():
                    # Find the NAI center
                    nai = next((n for n in self.mission_tasking.nais if n.id == nai_id), None)
                    if nai is None:
                        continue

                    print(f"    NAI {nai_id}: {len(targets)} targets near ({nai.center[0]:.0f}, {nai.center[1]:.0f})")

                    # Spawn each target
                    for i, target_data in enumerate(targets):
                        pos = target_data["position"]
                        target_type = target_data["type"]

                        # Decide whether to spawn vehicle or person based on type
                        if target_type in ["MILITARY_VEHICLE", "VEHICLE", "CIVILIAN_VEHICLE"]:
                            # Spawn vehicle
                            vehicle_types = ["tank", "tank2", "sedan", "suv"] if target_type == "MILITARY_VEHICLE" else ["sedan", "suv", "taxi"]
                            vtype = py_random.choice(vehicle_types)
                            try:
                                prim_path = self.world_gen.vehicles.spawn_vehicle(
                                    vtype, position=(pos[0], pos[1])
                                )
                            except Exception as e:
                                print(f"      Failed to spawn vehicle: {e}")
                                continue
                        else:
                            # Spawn person
                            try:
                                prim_path = self.world_gen.people.spawn_person(
                                    position=(pos[0], pos[1])
                                )
                            except Exception as e:
                                print(f"      Failed to spawn person: {e}")
                                continue

                        # Get actual position from prim
                        try:
                            translate = prim_utils.get_prim_property(prim_path, "xformOp:translate")
                            if translate:
                                self.pois.append(POI(
                                    id=prim_path,
                                    position=np.array([translate[0], translate[1], 0.0]),
                                ))
                        except Exception:
                            self.pois.append(POI(
                                id=prim_path,
                                position=np.array([pos[0], pos[1], 0.0]),
                            ))

                # Spawn "surprise" targets outside NAIs
                if unknown_targets:
                    print(f"    Surprise targets: {len(unknown_targets)} in unexpected locations")
                    for target_data in unknown_targets:
                        pos = target_data["position"]
                        try:
                            # Randomly vehicle or person
                            if py_random.random() > 0.5:
                                prim_path = self.world_gen.vehicles.spawn_vehicle(
                                    py_random.choice(["sedan", "suv"]),
                                    position=(pos[0], pos[1])
                                )
                            else:
                                prim_path = self.world_gen.people.spawn_person(
                                    position=(pos[0], pos[1])
                                )
                            self.pois.append(POI(
                                id=prim_path,
                                position=np.array([pos[0], pos[1], 0.0]),
                            ))
                        except Exception:
                            pass

                print(f"  [WorldGen] Spawned {len(self.pois)} total POIs from mission tasking")

            except ImportError:
                print("  World generator not available, using random POIs")
                self._setup_fallback_pois()
            except Exception as e:
                print(f"  World generator error: {e}, using random POIs")
                self._setup_fallback_pois()
        else:
            # No simulation app - use provided or random POIs
            self._setup_fallback_pois()

        # Setup comms zone (covers entire area initially)
        self.comms_zones = [
            CommsZone(
                id="full_coverage",
                center=np.array([0.0, 0.0, 50.0]),
                radius=half_size * 1.5,  # Generous initial coverage
                status=CommsStatus.OPERATIONAL,
            )
        ]

        # Setup geofence
        self.config.geofence = GeofenceConfig(
            min_x=-half_size,
            max_x=half_size,
            min_y=-half_size,
            max_y=half_size,
            min_z=0.0,
            max_z=120.0,
        )

        # No NFZs in basic surveillance scenario
        self.nfz_zones = []

        # Initialize domain randomizer
        rand_config = RandomizationConfig(
            randomize_lighting=self.config.randomize_lighting,
            randomize_weather=self.config.randomize_weather,
            randomize_sensors=self.config.randomize_sensors,
        )
        self.randomizer = DomainRandomizer(rand_config)

        # Initialize ontology bridge
        self.ontology_bridge = OntologyStateBridge()

        print(f"  Surveillance area: {self.config.surveillance_area_size}m x {self.config.surveillance_area_size}m")
        print(f"  POIs to capture: {len(self.pois)}")
        print(f"  Target coverage: {self.config.target_coverage}%")

    def _setup_fallback_pois(self) -> None:
        """Setup POIs without world generator (fallback)."""
        if self.config.poi_positions:
            for i, pos in enumerate(self.config.poi_positions):
                self.pois.append(POI(
                    id=f"poi_{i}",
                    position=np.array(pos),
                ))
        else:
            # Generate random POIs within surveillance area
            self._generate_random_pois()

    def _generate_random_pois(self) -> None:
        """Generate random POI locations within surveillance area."""
        half_size = self.config.surveillance_area_size / 2 * 0.9  # 10% margin

        for i in range(self.config.num_random_pois):
            pos = np.array([
                np.random.uniform(-half_size, half_size),
                np.random.uniform(-half_size, half_size),
                0.0,  # Ground level
            ])
            self.pois.append(POI(id=f"poi_{i}", position=pos))

    def reset(self, seed: Optional[int] = None) -> UAVState:
        """Reset environment for new episode."""
        # Reset base
        state = super().reset(seed)

        # Reset surveillance state
        self.coverage_grid.fill(False)
        for poi in self.pois:
            poi.captured = False
            poi.capture_time = None

        # Reset comms
        self.comms_operational = True
        self.comms_zones[0].status = CommsStatus.OPERATIONAL

        # Reset mission state
        self.returning_to_launch = False
        state.coverage_pct = 0.0
        state.poi_captured = 0
        state.autonomous_mode = False
        state.comms_status = CommsStatus.OPERATIONAL

        # Reset detection tracking for adaptive search
        self._detection_times.clear()

        # Randomize POI positions if enabled
        if self.randomizer and self.config.randomize_obstacles:
            self._regenerate_pois(seed)

        return state

    def _regenerate_pois(self, seed: Optional[int] = None) -> None:
        """Regenerate POI positions with randomization."""
        if seed is not None:
            np.random.seed(seed)

        self.pois.clear()
        self._generate_random_pois()

    def _update_state(self) -> None:
        """Update state including surveillance-specific elements."""
        super()._update_state()

        # Update comms status based on mission time
        if (self.uav_state.mission_time >= self.config.comms_denial_time
                and self.comms_operational):
            self._trigger_comms_denial()

        # Update coverage
        self._update_coverage()

        # Check POI captures
        self._check_poi_captures()

        # Update mission state
        self.uav_state.coverage_pct = self._calculate_coverage_pct()
        self.uav_state.poi_captured = sum(1 for poi in self.pois if poi.captured)

        # Check if should return (battery threshold)
        if (self.uav_state.battery_pct <= self.uav_state.battery_reserve
                and not self.returning_to_launch):
            self.returning_to_launch = True
            self.uav_state.flight_phase = FlightPhase.RTL

    def _trigger_comms_denial(self) -> None:
        """Trigger communications denial."""
        print(f">>> Comms denied at T+{self.uav_state.mission_time:.1f}s")
        self.comms_operational = False
        self.comms_zones[0].status = CommsStatus.DENIED
        self.uav_state.comms_status = CommsStatus.DENIED
        self.uav_state.autonomous_mode = True
        self.uav_state.battery_reserve = 30.0  # Per ontology axiom

    def _update_coverage(self) -> None:
        """Update coverage grid based on camera footprint."""
        pos = self.uav_state.position

        # Simple coverage model: camera sees circle below
        # Footprint radius increases with altitude
        altitude = pos[2]
        if altitude < 5.0:
            return  # Too low for meaningful coverage

        # Camera FOV: 60 degrees, so footprint = 2 * altitude * tan(30)
        footprint_radius = altitude * 0.577  # tan(30 deg)

        # Convert position to grid coordinates
        half_size = self.config.surveillance_area_size / 2
        grid_size = self.coverage_grid.shape[0]

        # Mark cells in footprint as covered
        for i in range(grid_size):
            for j in range(grid_size):
                # Cell center in world coordinates
                cell_x = (i - grid_size/2) * self.grid_resolution
                cell_y = (j - grid_size/2) * self.grid_resolution

                dist = np.sqrt((pos[0] - cell_x)**2 + (pos[1] - cell_y)**2)
                if dist < footprint_radius:
                    self.coverage_grid[i, j] = True

    def _calculate_coverage_pct(self) -> float:
        """Calculate percentage of area covered."""
        return 100.0 * np.mean(self.coverage_grid)

    def _check_poi_captures(self) -> None:
        """Check if any POIs are captured by current camera view."""
        pos = self.uav_state.position
        altitude = pos[2]

        if altitude < 5.0:
            return

        # Camera footprint
        footprint_radius = altitude * 0.577

        for poi in self.pois:
            if poi.captured:
                continue

            dist = np.linalg.norm(pos[:2] - poi.position[:2])
            if dist < footprint_radius:
                poi.captured = True
                poi.capture_time = self.uav_state.mission_time
                # Track detection time for adaptive search learning
                self._detection_times.append(poi.capture_time)
                print(f">>> POI {poi.id} captured at T+{poi.capture_time:.1f}s")

    def compute_reward(self, state: UAVState, next_state: UAVState) -> float:
        """
        Compute reward per ONTOLOGY_FOUNDATION.qmd reward structure.
        """
        reward = 0.0

        # ==========================================
        # Safety (hard constraints from ontology)
        # ==========================================

        if not next_state.in_geofence:
            reward -= 100.0

        if next_state.in_nfz:
            reward -= 500.0

        # Obstacle proximity (3m minimum per ontology)
        # This would require obstacle detection which isn't in base state
        # Placeholder for when obstacle sensors are added

        # ==========================================
        # Battery reserve (30% for comms-denied)
        # ==========================================

        required_reserve = 30.0 if next_state.autonomous_mode else 25.0

        if next_state.battery_pct < required_reserve and not self.returning_to_launch:
            penalty = required_reserve - next_state.battery_pct
            reward -= 10.0 * penalty

        # ==========================================
        # Mission objectives
        # ==========================================

        # Coverage gain
        coverage_delta = next_state.coverage_pct - state.coverage_pct
        reward += 2.0 * coverage_delta

        # POI captures
        poi_delta = next_state.poi_captured - state.poi_captured
        reward += 10.0 * poi_delta

        # ==========================================
        # Mission success bonus
        # ==========================================

        if (next_state.coverage_pct >= self.config.target_coverage
                and next_state.poi_captured >= self.config.target_poi_count):
            # Only give bonus once when threshold crossed
            if (state.coverage_pct < self.config.target_coverage
                    or state.poi_captured < self.config.target_poi_count):
                reward += 100.0

        # ==========================================
        # Return to launch (when mission complete or battery low)
        # ==========================================

        if self.returning_to_launch:
            dist_to_home = np.linalg.norm(next_state.position - self.launch_position)
            prev_dist = np.linalg.norm(state.position - self.launch_position)

            # Reward for getting closer to home
            progress = prev_dist - dist_to_home
            reward += 1.0 * progress

            # Landing bonus
            if dist_to_home < self.config.return_accuracy and next_state.position[2] < 0.5:
                reward += 50.0

        return reward

    def is_done(self, state: UAVState) -> bool:
        """Check if episode is complete."""
        # Mission time exceeded
        if state.mission_time >= self.config.max_mission_time:
            return True

        # Battery depleted (emergency)
        if state.battery_pct < 10.0:
            return True

        # Safety violation (terminate episode)
        if not state.in_geofence:
            return True

        if state.in_nfz:
            return True

        # Successful landing after return
        if self.returning_to_launch:
            dist_to_home = np.linalg.norm(state.position - self.launch_position)
            if dist_to_home < self.config.return_accuracy and state.position[2] < 0.5:
                return True

        # Lost localization without fallback
        if state.gnss_status == GNSSStatus.DENIED and not state.vio_valid:
            return True

        return False

    def get_info(self, state: UAVState) -> Dict[str, Any]:
        """Get additional info about current state."""
        dist_to_home = np.linalg.norm(state.position - self.launch_position)

        # Mission success evaluation
        coverage_met = state.coverage_pct >= self.config.target_coverage
        poi_met = state.poi_captured >= self.config.target_poi_count
        safe_return = dist_to_home < self.config.return_accuracy
        battery_ok = state.battery_pct >= self.config.min_battery_at_landing

        return {
            # Mission metrics
            "coverage_pct": state.coverage_pct,
            "poi_captured": state.poi_captured,
            "poi_total": len(self.pois),
            "distance_to_home": dist_to_home,

            # Status
            "comms_status": state.comms_status.name,
            "autonomous_mode": state.autonomous_mode,
            "returning": self.returning_to_launch,

            # Success criteria
            "coverage_met": coverage_met,
            "poi_met": poi_met,
            "safe_return": safe_return if self.returning_to_launch else None,
            "battery_ok": battery_ok,

            # Mission success (all criteria met)
            "mission_success": coverage_met and poi_met and (not self.returning_to_launch or (safe_return and battery_ok)),

            # Ontology state
            "ontology_state": self.ontology_bridge.get_symbolic_state(
                state, self.nfz_zones, self.threat_zones, None
            ) if self.ontology_bridge else None,
        }

    @property
    def observation_dim(self) -> int:
        """Extended observation dimension for surveillance task."""
        base_dim = super().observation_dim
        # Add: coverage%, poi_ratio, dist_home, returning, recent_detections, time_since_detection
        return base_dim + 6

    def state_to_observation(self, state: UAVState) -> np.ndarray:
        """Convert state to observation with surveillance and cluster-aware features."""
        base_obs = super().state_to_observation(state)

        # Add surveillance-specific features
        dist_to_home = np.linalg.norm(state.position - self.launch_position)

        # Cluster-aware features for adaptive search learning
        # Count recent detections within the detection window
        current_time = state.mission_time
        recent_detections = sum(
            1 for t in self._detection_times
            if current_time - t <= self._detection_window
        )

        # Time since last detection (helps agent learn to stay in productive areas)
        if self._detection_times:
            time_since_detection = current_time - self._detection_times[-1]
        else:
            time_since_detection = current_time  # No detections yet = full mission time

        extra_features = np.array([
            state.coverage_pct / 100.0,
            state.poi_captured / max(1, len(self.pois)),  # Avoid div by zero
            np.clip(dist_to_home / 500.0, 0, 1),  # Normalized distance
            float(self.returning_to_launch),
            np.clip(recent_detections / 5.0, 0, 1),  # Normalize to ~5 max recent
            np.clip(time_since_detection / self._detection_window, 0, 1),  # Normalized
        ], dtype=np.float32)

        return np.concatenate([base_obs, extra_features])
