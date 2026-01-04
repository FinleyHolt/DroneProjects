#!/usr/bin/env python3
"""
Base ISR Environment for Flyby F-11 Training

Provides the foundational simulation environment for all three canonical problems.
Built on Isaac Sim + Pegasus Simulator with PX4 SITL integration.

Key Features:
- Modular environment architecture
- Domain randomization hooks
- Ontology state extraction
- Reward computation framework
- Safety shield integration via Vampire ATP
- Live perception integration (YOLO detection during RL training)
"""

import sys
import os
import threading
import subprocess
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from .perception_integration import PerceptionIntegration
from enum import Enum, auto
from abc import ABC, abstractmethod

# Add Pegasus to Python path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)


class FlightPhase(Enum):
    """UAV flight phases matching ontology FlightPhase class."""
    PREFLIGHT = auto()
    ARMED = auto()
    TAKEOFF = auto()
    HOVER = auto()
    TRANSIT = auto()
    LOITER = auto()
    MISSION_EXECUTION = auto()
    RTL = auto()
    LANDING = auto()
    DISARMED = auto()


class CommsStatus(Enum):
    """Communications status matching ontology CommsStatus class."""
    OPERATIONAL = auto()
    DEGRADED = auto()
    DENIED = auto()


class GNSSStatus(Enum):
    """GNSS availability status."""
    FULL = auto()
    DEGRADED = auto()
    DENIED = auto()


class ThreatLevel(Enum):
    """Threat zone classification."""
    NONE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3


@dataclass
class GeofenceConfig:
    """Geofence boundary configuration."""
    min_x: float = -150.0
    max_x: float = 150.0
    min_y: float = -150.0
    max_y: float = 150.0
    min_z: float = 0.0
    max_z: float = 120.0  # F-11 max altitude


@dataclass
class NoFlyZone:
    """No-fly zone definition."""
    id: str
    center: np.ndarray  # (x, y, z)
    radius: float  # For cylindrical NFZs
    height: float
    priority: int = 1  # 1=highest (aircraft corridor), 3=lowest
    is_dynamic: bool = False
    is_active: bool = True
    buffer_distance: float = 10.0  # Warning zone


@dataclass
class ThreatZone:
    """Threat zone definition."""
    id: str
    center: np.ndarray
    radius: float
    threat_level: ThreatLevel
    risk_rate: float = 1.0  # Penalty per second of exposure


@dataclass
class TargetOfInterest:
    """ISR target definition."""
    id: str
    position: np.ndarray
    priority: int  # 1=Critical, 2=High, 3=Medium, 4=Low
    value: float
    battery_cost: float  # Estimated % to reach and observe
    observed: bool = False
    in_high_threat: bool = False


@dataclass
class CommsZone:
    """Communications coverage zone."""
    id: str
    center: np.ndarray
    radius: float
    status: CommsStatus


@dataclass
class DomainRandomizationConfig:
    """Domain randomization configuration for sensor noise and disturbances."""
    # Master enable/disable
    enabled: bool = True

    # Battery variation (±percentage of base drain rate)
    battery_variation_enabled: bool = True
    battery_drain_rate_variation: float = 0.1  # ±10% variation

    # GNSS degradation
    gnss_degradation_enabled: bool = True
    gnss_degradation_prob: float = 0.1  # 10% chance per episode

    # VIO noise
    vio_noise_enabled: bool = True
    vio_noise_scale: float = 0.0  # Additional position drift (meters)

    # Wind disturbance (applied to PX4 SITL physics via SIM_WIND_SPD/DIR params)
    wind_enabled: bool = True
    wind_speed_range: Tuple[float, float] = (0.0, 5.0)  # m/s magnitude range


@dataclass
class UAVState:
    """Complete UAV state for RL observation."""
    # Position and velocity
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([1, 0, 0, 0]))  # quaternion
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Flight state
    flight_phase: FlightPhase = FlightPhase.PREFLIGHT
    battery_pct: float = 100.0
    battery_reserve: float = 25.0  # Required reserve (ontology: 25% normal, 30% comms-denied)

    # Sensor status
    comms_status: CommsStatus = CommsStatus.OPERATIONAL
    gnss_status: GNSSStatus = GNSSStatus.FULL
    vio_valid: bool = True
    imu_valid: bool = True

    # Mission state
    mission_time: float = 0.0
    coverage_pct: float = 0.0
    poi_captured: int = 0

    # Safety state
    in_geofence: bool = True
    in_nfz: bool = False
    current_nfz_id: Optional[str] = None
    nfz_distance: float = float('inf')

    # Threat state
    in_threat_zone: bool = False
    current_threat_level: ThreatLevel = ThreatLevel.NONE
    threat_exposure: float = 0.0
    max_threat_exposure: float = 30.0

    # Autonomy
    autonomous_mode: bool = False

    # Domain randomization state (sampled per episode)
    wind_vector: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Wind in world frame (m/s)
    vio_position_drift: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Accumulated VIO drift
    battery_drain_multiplier: float = 1.0  # Sampled battery drain rate multiplier
    gnss_degradation_triggered: bool = False  # Whether GNSS degradation has been triggered this episode
    gnss_degradation_step: int = -1  # Step at which GNSS degradation was triggered (-1 = not triggered)


@dataclass
class PerceptionConfig:
    """
    Configuration for dual-mode perception during RL training.

    DUAL-MODE ARCHITECTURE:
    - "gt" (Training Mode): Uses GroundTruthDetector with frustum math.
      No rendering required, runs at 1000+ env steps/second.
    - "full" (E2E Mode): Uses YOLODetector with real inference.
      Requires rendered images, runs at ~20 Hz for validation.

    Both modes produce identical 516-dim observation vectors.
    """
    # Enable perception in observation space
    enabled: bool = True

    # Perception mode: "gt" (training) or "full" (E2E validation)
    # Legacy values "ground_truth", "inference", "hybrid" map to "gt"
    mode: str = "gt"

    # Camera settings
    camera_resolution: Tuple[int, int] = (640, 480)
    camera_fov_degrees: float = 90.0

    # Performance
    max_encode_time_ms: float = 20.0  # Max encoding time budget
    skip_frames: int = 0  # Skip N frames between perception updates

    # Ground truth detector noise (for sim-to-real transfer)
    gt_bbox_noise_std: float = 0.02
    gt_confidence_noise_std: float = 0.05
    gt_position_noise_std: float = 0.5

    # Observation dimension (100 priority + 384 grid + 32 stats)
    observation_dim: int = 516


@dataclass
class EnvironmentConfig:
    """Environment configuration."""
    # Simulation
    headless: bool = False
    physics_dt: float = 1.0 / 250.0  # 250Hz physics
    render_dt: float = 1.0 / 60.0  # 60Hz rendering

    # World
    world_name: str = "isr_training"
    environment_usd: Optional[str] = None  # Custom USD path
    skip_default_environment: bool = False  # Skip loading default gridroom (for procedural worlds)
    geofence: GeofenceConfig = field(default_factory=GeofenceConfig)

    # Coordinates (default: Camp Pendleton)
    latitude: float = 33.3853
    longitude: float = -117.5653
    altitude: float = 100.0

    # UAV
    spawn_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.5]))  # Start slightly above ground
    spawn_orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))  # euler

    # Mission
    max_mission_time: float = 900.0  # 15 minutes

    # Domain randomization (legacy flags for visual randomization)
    randomize_lighting: bool = True
    randomize_weather: bool = True
    randomize_obstacles: bool = True
    randomize_sensors: bool = True

    # Domain randomization config (sensor noise, wind, battery variation)
    domain_randomization: DomainRandomizationConfig = field(default_factory=DomainRandomizationConfig)

    # Perception configuration for live YOLO detection during training
    perception: PerceptionConfig = field(default_factory=PerceptionConfig)


class BaseISREnvironment(ABC):
    """
    Base class for ISR training environments.

    Provides common functionality:
    - Isaac Sim + Pegasus initialization
    - PX4 SITL integration
    - State observation
    - Reward computation framework
    - Safety shield integration
    """

    def __init__(self, config: EnvironmentConfig):
        self.config = config
        self._initialized = False
        self._step_count = 0

        # Simulation objects (initialized in setup())
        self.simulation_app = None
        self.world = None
        self.timeline = None
        self.pg = None
        self.vehicle = None

        # Environment elements
        self.nfz_zones: List[NoFlyZone] = []
        self.threat_zones: List[ThreatZone] = []
        self.targets: List[TargetOfInterest] = []
        self.comms_zones: List[CommsZone] = []

        # State
        self.uav_state = UAVState()
        self._prev_state = None

        # Domain randomizer
        self.randomizer = None

        # Domain randomization state (sampled per episode)
        self._dr_rng = np.random.default_rng()  # RNG for domain randomization
        self._episode_battery_drain_multiplier = 1.0
        self._episode_gnss_degradation_step = -1  # Step at which GNSS degrades (-1 = never)
        self._episode_wind_vector = np.zeros(3)
        self._episode_vio_noise_scale = 0.0

        # Ontology bridge
        self.ontology_bridge = None

        # Perception integration for live YOLO detection
        self.perception: Optional['PerceptionIntegration'] = None
        self._camera = None  # Isaac Sim camera object
        self._perception_obs = None  # Cached perception observation

        # PX4 command path
        self._px4_cmd = "/px4/build/px4_sitl_default/bin/px4-commander"

        # Action bridge for RL-to-PX4 control
        self.action_bridge = None
        self._mav_connection = None
        self._offboard_engaged = False

        # Safety filter (Vampire theorem prover)
        self.safety_filter = None
        self._safety_filter_enabled = True
        self._safety_violations_count = 0
        self._safety_interventions_count = 0

    def setup(self) -> None:
        """Initialize the simulation environment."""
        if self._initialized:
            return

        print("=" * 60)
        print(f"Initializing {self.__class__.__name__}")
        print("=" * 60)

        # Initialize Isaac Sim
        print("\n[1/5] Initializing Isaac Sim...")
        from isaacsim import SimulationApp
        self.simulation_app = SimulationApp({"headless": self.config.headless})
        print("  Isaac Sim ready")

        # Import after Isaac Sim init
        print("\n[2/5] Loading Pegasus extension...")
        import omni.timeline
        from isaacsim.core.api.world import World
        from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
        from pegasus.simulator.logic.backends.px4_mavlink_backend import (
            PX4MavlinkBackend, PX4MavlinkBackendConfig
        )
        from pegasus.simulator.logic.vehicles.multirotor import (
            Multirotor, MultirotorConfig
        )
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
        from scipy.spatial.transform import Rotation
        print("  Pegasus loaded")

        # Initialize world
        print("\n[3/5] Setting up world...")
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Set PX4 path (container has PX4 at /px4, not ~/PX4-Autopilot)
        self.pg.set_px4_path("/px4")

        # Set global coordinates
        self.pg.set_global_coordinates(
            self.config.latitude,
            self.config.longitude,
            self.config.altitude
        )

        # Load environment
        if self.config.environment_usd:
            self.pg.load_environment(self.config.environment_usd)
        elif not self.config.skip_default_environment:
            # Use default curved gridroom (unless procedural world will be generated)
            self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        print("  Environment loaded")

        # Spawn vehicle with MANUAL PX4 launch to control timing precisely
        # Setting px4_autolaunch=False means PX4 won't start on timeline.play()
        # We'll launch it manually after we've started stepping the world
        print("\n[4/7] Spawning vehicle (PX4 delayed start)...")
        px4_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": False,  # CRITICAL: Don't auto-launch, we'll do it manually
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
            "enable_lockstep": True,
            "update_rate": 250.0,
        })
        print(f"  PX4 lockstep enabled, update_rate=250Hz, autolaunch=False")

        vehicle_config = MultirotorConfig()
        self._px4_backend = PX4MavlinkBackend(px4_config)
        vehicle_config.backends = [self._px4_backend]

        spawn_pos = self.config.spawn_position.tolist()
        spawn_rot = Rotation.from_euler(
            "XYZ", self.config.spawn_orientation.tolist(), degrees=True
        ).as_quat()

        self.vehicle = Multirotor(
            "/World/uav",
            ROBOTS["Iris"],  # TODO: Replace with F-11 model when available
            0,
            spawn_pos,
            spawn_rot,
            config=vehicle_config,
        )
        print("  Vehicle spawned (PX4 not yet started)")

        # Start timeline and physics WITHOUT PX4
        # This avoids poll timeouts because PX4 isn't waiting for data yet
        print("\n[5/7] Starting timeline and physics...", flush=True)
        self.world.reset()
        self.timeline.play()
        # Step 50 times to get physics running and stable
        for _ in range(50):
            self.world.step(render=True)
        print("  Timeline started, physics running", flush=True)

        # NOW manually launch PX4 - physics is already stepping
        # CRITICAL: We need to step the world CONTINUOUSLY during PX4 boot
        # PX4 starts waiting for sensor data within milliseconds of launch
        print("\n[6/7] Launching PX4 SITL with continuous stepping...", flush=True)
        from pegasus.simulator.logic.backends.tools.px4_launch_tool import PX4LaunchTool
        import threading
        import time

        # Create flag for stepping thread
        self._px4_stepping_active = True

        def step_world_continuously():
            """Background thread to step world while PX4 boots."""
            while self._px4_stepping_active:
                try:
                    self.world.step(render=True)
                except Exception as e:
                    print(f"  [Step thread] Error: {e}")
                    break

        # Start stepping thread BEFORE launching PX4
        step_thread = threading.Thread(target=step_world_continuously, daemon=True)
        step_thread.start()

        # Now launch PX4 - stepping is already happening
        self._px4_tool = PX4LaunchTool(
            self.pg.px4_path,
            vehicle_id=0,
            px4_model=self.pg.px4_default_airframe
        )
        self._px4_tool.launch_px4()
        # Store reference in backend so it can be stopped later
        self._px4_backend.px4_tool = self._px4_tool
        self._px4_backend.px4_autolaunch = True  # Mark as launched for cleanup

        # Let PX4 boot while stepping continues in background
        # Wait ~2 seconds (PX4 typically needs 1-2s to start)
        print("  PX4 launched, stepping in background for 2 seconds...", flush=True)
        time.sleep(2.0)

        # Stop stepping thread
        self._px4_stepping_active = False
        step_thread.join(timeout=1.0)

        # Continue with normal stepping
        print("  PX4 boot phase complete, continuing normal stepping...", flush=True)
        for _ in range(300):
            self.world.step(render=True)
        print("  PX4 initialization complete", flush=True)

        # Initialize action bridge for RL control
        # Timeline already started, just need to connect MAVLink
        print("\n[7/11] Setting up MAVLink action bridge...")
        self._setup_action_bridge()
        print("  Action bridge ready")

        # CRITICAL: After action bridge setup, timeline is playing.
        # We must continue stepping the world during all subsequent setup
        # to keep PX4 lockstep alive. Each setup phase should be quick,
        # but we step periodically to maintain sync.

        # Initialize Vampire safety filter
        print("\n[8/11] Setting up Vampire safety filter...")
        self._setup_safety_filter()
        # Step world to keep PX4 lockstep alive
        for _ in range(30):
            self.world.step(render=True)
        print("  Safety filter ready")

        # Initialize perception for live YOLO detection
        print("\n[9/11] Setting up live perception...")
        self._setup_perception()
        # Step world to keep PX4 lockstep alive
        for _ in range(30):
            self.world.step(render=True)
        print("  Perception ready")

        # Setup environment-specific elements
        print("\n[10/11] Setting up environment elements...")
        self._setup_environment()
        # Step world after environment setup
        for _ in range(30):
            self.world.step(render=True)
        print("  Environment ready")

        # CRITICAL: Wait for full simulation readiness before allowing training
        # This is the "gate" that ensures PX4 is healthy and ready
        print("\n[11/11] Waiting for full simulation readiness...")
        ready = self._wait_for_simulation_ready(max_steps=3000)
        if not ready:
            print("  WARNING: Simulation may not be fully ready!")
            print("  Training will proceed but vehicle control may be unreliable.")

        self._initialized = True
        print("\n" + "=" * 60)
        print("Simulation ready!" if ready else "Simulation setup complete (with warnings)")
        print("=" * 60)

    @abstractmethod
    def _setup_environment(self) -> None:
        """Setup environment-specific elements (NFZs, targets, etc.)."""
        pass

    @abstractmethod
    def compute_reward(self, state: UAVState, next_state: UAVState) -> float:
        """Compute reward for the state transition."""
        pass

    @abstractmethod
    def is_done(self, state: UAVState) -> bool:
        """Check if episode is complete."""
        pass

    @abstractmethod
    def get_info(self, state: UAVState) -> Dict[str, Any]:
        """Get additional info about current state."""
        pass

    def step(self, action: np.ndarray) -> Tuple[UAVState, float, bool, Dict[str, Any]]:
        """
        Execute one environment step.

        Args:
            action: Action to execute (position command or velocity command)

        Returns:
            Tuple of (next_state, reward, done, info)
        """
        if not self._initialized:
            self.setup()

        # Save previous state
        self._prev_state = self.uav_state

        # Apply action (through PX4 or direct control)
        self._apply_action(action)

        # Step simulation
        self.world.step(render=True)
        self._step_count += 1

        # Update state
        self._update_state()

        # Compute reward
        reward = self.compute_reward(self._prev_state, self.uav_state)

        # Check termination
        done = self.is_done(self.uav_state)

        # Get info
        info = self.get_info(self.uav_state)

        return self.uav_state, reward, done, info

    def reset(self, seed: Optional[int] = None) -> UAVState:
        """
        Reset the environment.

        Args:
            seed: Random seed for domain randomization

        Returns:
            Initial state
        """
        if not self._initialized:
            self.setup()

        # Apply domain randomization
        if self.randomizer and seed is not None:
            self.randomizer.randomize(seed)

        # Sample new domain randomization values for this episode
        self._sample_domain_randomization(seed)

        # Reset vehicle position without calling world.reset()
        # CRITICAL: world.reset() causes Pegasus to restart PX4 (via px4_autolaunch),
        # which breaks lockstep synchronization. Instead, we teleport the vehicle
        # back to spawn position and reset velocities directly.
        #
        # Timeline should already be playing from setup() and must stay playing
        # to keep PX4 lockstep alive.
        if not self.timeline.is_playing():
            # First reset after setup - timeline should already be playing
            # If not, start it now (shouldn't happen with current flow)
            print("  [Reset] Starting timeline (first reset)")
            self.timeline.play()

        # Teleport vehicle back to spawn position instead of world.reset()
        # This resets the vehicle without restarting PX4
        self._reset_vehicle_position()

        # Reset offboard mode flag (will re-engage on first action)
        self._offboard_engaged = False

        # Reset safety filter counters
        self._safety_violations_count = 0
        self._safety_interventions_count = 0

        # Reset state
        self._step_count = 0
        self.uav_state = UAVState(
            position=self.config.spawn_position.copy(),
            battery_pct=100.0,
            wind_vector=self._episode_wind_vector.copy(),
            battery_drain_multiplier=self._episode_battery_drain_multiplier,
            gnss_degradation_step=self._episode_gnss_degradation_step,
        )

        # Wait for PX4 to stabilize after physics reset
        # This ensures PX4 has processed the new vehicle state
        print("  [Reset] Running 100 warmup steps for PX4 sync...")
        for i in range(100):
            self.world.step(render=True)
            if i == 0:
                print("  [Reset] First step complete, PX4 lockstep synced")

        # Apply wind to PX4 SITL physics AFTER warmup
        # This ensures PX4 is ready to receive MAVLink parameters
        self._apply_wind_to_px4()

        self._update_state()
        return self.uav_state

    def _sample_domain_randomization(self, seed: Optional[int] = None) -> None:
        """
        Sample domain randomization values for a new episode.

        Args:
            seed: Optional random seed for reproducibility
        """
        dr_config = self.config.domain_randomization

        # Reset RNG with seed if provided
        if seed is not None:
            self._dr_rng = np.random.default_rng(seed)

        if not dr_config.enabled:
            # Reset to defaults when DR is disabled
            self._episode_battery_drain_multiplier = 1.0
            self._episode_gnss_degradation_step = -1
            self._episode_wind_vector = np.zeros(3)
            self._episode_vio_noise_scale = 0.0
            return

        # Battery drain rate variation: sample multiplier from uniform(1-var, 1+var)
        if dr_config.battery_variation_enabled:
            var = dr_config.battery_drain_rate_variation
            self._episode_battery_drain_multiplier = self._dr_rng.uniform(1.0 - var, 1.0 + var)
        else:
            self._episode_battery_drain_multiplier = 1.0

        # GNSS degradation: randomly select a step at which degradation occurs
        if dr_config.gnss_degradation_enabled and self._dr_rng.random() < dr_config.gnss_degradation_prob:
            # Degradation occurs at a random point in the first 80% of max mission time
            max_steps = int(self.config.max_mission_time * 60 * 0.8)  # 80% of episode at 60Hz
            self._episode_gnss_degradation_step = self._dr_rng.integers(0, max(1, max_steps))
        else:
            self._episode_gnss_degradation_step = -1  # No degradation

        # Wind disturbance: sample magnitude and direction
        if dr_config.wind_enabled:
            min_speed, max_speed = dr_config.wind_speed_range
            wind_magnitude = self._dr_rng.uniform(min_speed, max_speed)
            wind_direction = self._dr_rng.uniform(0, 2 * np.pi)  # 0-360 degrees in radians
            self._episode_wind_vector = np.array([
                wind_magnitude * np.cos(wind_direction),
                wind_magnitude * np.sin(wind_direction),
                0.0  # Horizontal wind only
            ])
        else:
            self._episode_wind_vector = np.zeros(3)

        # VIO noise scale
        if dr_config.vio_noise_enabled:
            self._episode_vio_noise_scale = dr_config.vio_noise_scale
        else:
            self._episode_vio_noise_scale = 0.0


    def _apply_wind_to_px4(self) -> None:
        """
        Apply wind parameters to PX4 SITL physics.
        
        Updates PX4 SIM_WIND_SPD and SIM_WIND_DIR parameters based on
        the episode's randomized wind vector. This ensures the drone
        actually experiences wind forces, not just observing them.
        """
        if self._mav_connection is None:
            return
        
        from pymavlink import mavutil
        import time
        
        # Convert wind vector (world frame: +X=East, +Y=North) to speed + direction
        wind_magnitude = np.linalg.norm(self._episode_wind_vector[:2])
        
        if wind_magnitude < 0.1:
            # No significant wind, skip update
            return
        
        wind_direction_rad = np.arctan2(
            self._episode_wind_vector[1],  # North component
            self._episode_wind_vector[0]   # East component
        )
        wind_direction_deg = np.degrees(wind_direction_rad)
        
        # Convert from math convention (CCW from +X=East) to aviation (CW from North)
        # In PX4: 0=North, 90=East, 180=South, 270=West
        aviation_heading = (90.0 - wind_direction_deg) % 360.0
        
        # Set PX4 wind speed parameter
        self._mav_connection.mav.param_set_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            b'SIM_WIND_SPD',
            wind_magnitude,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        
        # Set PX4 wind direction parameter
        self._mav_connection.mav.param_set_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            b'SIM_WIND_DIR',
            aviation_heading,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        # Step world to let PX4 process parameter changes
        # IMPORTANT: Use world.step() instead of time.sleep() to keep PX4 lockstep alive
        for _ in range(6):  # ~0.1s at 60Hz
            self.world.step(render=True)

    def _reset_vehicle_position(self) -> None:
        """
        Reset vehicle position and velocities without restarting PX4.

        For episode resets, we don't actually need to teleport the vehicle
        since the initial world.reset() in setup() already positioned it.
        We just need to keep stepping the world to maintain PX4 lockstep.

        Note: True position reset while keeping PX4 running is complex.
        For now, we rely on episode termination conditions to keep episodes short
        enough that position drift isn't a major issue.
        """
        # Log current position for debugging
        if hasattr(self.vehicle, 'state') and hasattr(self.vehicle.state, 'position'):
            pos = self.vehicle.state.position
            print(f"  [Reset] Current vehicle position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

        # Step world to keep PX4 lockstep alive during reset
        # Don't attempt position reset as it's unreliable without world.reset()
        print(f"  [Reset] Stepping world (100 steps) to maintain lockstep...")
        for _ in range(100):
            self.world.step(render=True)

    def close(self) -> None:
        """Cleanup and close the environment."""
        if self._initialized:
            self.timeline.stop()
            self.simulation_app.close()
            self._initialized = False

    def _setup_action_bridge(self) -> None:
        """
        Setup MAVLink connection and action bridge for RL control.

        This establishes the link between the RL agent's actions and
        PX4's offboard velocity control mode.

        NOTE: Timeline should already be started by setup() before this is called.
        We just need to connect MAVLink and wait for heartbeat.
        """
        from .action_bridge import PX4ActionBridge
        from pymavlink import mavutil

        # Ensure timeline is playing (should be started by setup())
        if not self.timeline.is_playing():
            print("  Warning: Timeline not playing, starting now...")
            self.timeline.play()
            for _ in range(200):
                self.world.step(render=True)

        # Connect to PX4 via MAVLink
        try:
            self._mav_connection = mavutil.mavlink_connection(
                'udpin:localhost:14550',
                source_system=255
            )

            # Wait for heartbeat while stepping world (keeps lockstep alive)
            print("  Waiting for MAVLink heartbeat (stepping world)...")
            heartbeat_received = False
            for i in range(500):  # Max ~8 seconds
                self.world.step(render=True)
                msg = self._mav_connection.recv_match(type='HEARTBEAT', blocking=False)
                if msg and msg.get_srcSystem() != 0:  # Ignore GCS (system 0)
                    heartbeat_received = True
                    print(f"  Got heartbeat from system {msg.get_srcSystem()} after {i} steps")
                    break

            if not heartbeat_received:
                print("  Warning: No heartbeat received after 500 steps")
                self._mav_connection = None
            else:
                # Explicitly set target to vehicle if needed
                if self._mav_connection.target_system == 0:
                    self._mav_connection.target_system = 1
                    self._mav_connection.target_component = 1
                    print(f"  MAVLink: Forcing target to system 1 (was 0)")
                print(f"  MAVLink connected to system {self._mav_connection.target_system}")

        except Exception as e:
            print(f"  Warning: MAVLink connection failed: {e}")
            print("  Actions will not control the drone (visual-only mode)")
            self._mav_connection = None

        # NOTE: Do NOT stop the timeline here!
        # Stopping timeline breaks PX4 lockstep synchronization.
        # PX4 continues to expect sensor data, and if the timeline stops,
        # its lockstep clock desyncs from the simulation clock.
        # The timeline will remain running, and reset() will use world.reset()
        # which handles the physics reset without disrupting PX4.

        # Initialize action bridge
        self.action_bridge = PX4ActionBridge(self._mav_connection)

    def _setup_safety_filter(self) -> None:
        """
        Setup Vampire-based safety filter for action shielding.

        The safety filter checks each proposed action against the ontology
        constraints before execution. If an action would violate a hard
        constraint (NFZ entry, geofence violation, etc.), it is replaced
        with a safe fallback action (hover).
        """
        from .safety_filter import VampireSafetyFilter

        # Check if Vampire is available
        import shutil
        vampire_available = shutil.which('vampire') is not None

        if not vampire_available:
            print("  Warning: Vampire theorem prover not found in PATH")
            print("  Safety filter will use fast rule-based fallback")

        # Initialize safety filter
        self.safety_filter = VampireSafetyFilter(
            ontology_path="/workspace/ontology/planning_mode",
            timeout_ms=50,  # 50ms budget per action check
            enabled=self._safety_filter_enabled
        )

        if self._safety_filter_enabled:
            print(f"  Vampire safety shielding: ENABLED")
            print(f"  Hard constraints: {', '.join(self.safety_filter.hard_constraints)}")
        else:
            print(f"  Vampire safety shielding: DISABLED (training without shield)")

    def _setup_perception(self) -> None:
        """
        Setup dual-mode perception for RL training.

        DUAL-MODE ARCHITECTURE:
        - "gt" mode: Ground truth detection using frustum math (no rendering)
        - "full" mode: Real YOLO inference on camera images

        Creates:
        1. Camera attached to the drone body (for "full" mode or visualization)
        2. DualModePerception wrapper with mode-specific detector
        3. World object registry for GT mode

        Output: 516-dimensional observation vector (identical format both modes)
        """
        if not self.config.perception.enabled:
            print("  Perception: DISABLED")
            return

        # Import dual-mode perception
        try:
            import sys
            sys.path.insert(0, '/workspace/perception')
            from perception import (
                DualModePerception, DualModeConfig, CameraParams
            )
        except ImportError:
            # Fallback to legacy perception integration
            print("  Warning: DualModePerception not available, using legacy")
            from .perception_integration import (
                PerceptionIntegration,
                PerceptionIntegrationConfig
            )
            perception_config = PerceptionIntegrationConfig(
                mode=self.config.perception.mode,
                camera_resolution=self.config.perception.camera_resolution,
                max_encode_time_ms=self.config.perception.max_encode_time_ms,
                skip_frames=self.config.perception.skip_frames,
                observation_dim=self.config.perception.observation_dim,
            )
            self.perception = PerceptionIntegration(perception_config)
            self.perception.initialize(None)
            print(f"  Mode: {self.config.perception.mode} (legacy)")
            return

        # Normalize mode string
        mode = self.config.perception.mode.lower()
        if mode in ["ground_truth", "hybrid", "training"]:
            mode = "gt"
        elif mode in ["inference", "e2e", "yolo"]:
            mode = "full"

        # Create camera only if needed (E2E mode or for visualization)
        if mode == "full":
            try:
                from isaacsim.sensors.camera import Camera
                from scipy.spatial.transform import Rotation

                camera_path = "/World/uav/body/perception_camera"
                res = self.config.perception.camera_resolution
                self._camera = Camera(
                    prim_path=camera_path,
                    frequency=30,
                    resolution=res,
                )

                # Camera pointing down at 45 degrees for ISR
                self._camera.set_local_pose(
                    np.array([0.1, 0.0, -0.05]),  # Forward and below
                    Rotation.from_euler("ZYX", [0.0, 45.0, 0.0], degrees=True).as_quat()
                )
                self._camera.initialize()
                self._camera.set_focal_length(4.5)  # Wide angle
                self._camera.set_clipping_range(0.1, 500.0)

                print(f"  Camera: {res[0]}x{res[1]} @ 30Hz")
            except Exception as e:
                print(f"  Warning: Could not create camera: {e}")
                print(f"  Falling back to GT mode (no rendering)")
                mode = "gt"
                self._camera = None
        else:
            print(f"  Camera: Not needed for GT mode (no rendering)")
            self._camera = None

        # Initialize dual-mode perception
        perception_config = DualModeConfig(
            mode=mode,
            camera_params=CameraParams(
                width=self.config.perception.camera_resolution[0],
                height=self.config.perception.camera_resolution[1],
                fov_horizontal=self.config.perception.camera_fov_degrees,
            ),
            gt_bbox_noise_std=self.config.perception.gt_bbox_noise_std,
            gt_confidence_noise_std=self.config.perception.gt_confidence_noise_std,
            gt_position_noise_std=self.config.perception.gt_position_noise_std,
            skip_frames=self.config.perception.skip_frames,
            max_encode_time_ms=self.config.perception.max_encode_time_ms,
        )

        self.perception = DualModePerception(perception_config)

        print(f"  Mode: {mode} ({'ground truth' if mode == 'gt' else 'full inference'})")
        print(f"  Observation dim: {self.perception.OUTPUT_DIM}")

    def _check_px4_mode(self) -> Tuple[Optional[bool], Optional[int]]:
        """Check PX4 armed state and flight mode from heartbeat."""
        from pymavlink import mavutil
        msg = self._mav_connection.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.get_srcSystem() != 0:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            custom_mode = msg.custom_mode
            return armed, custom_mode
        return None, None

    def _check_px4_health(self) -> Tuple[bool, List[str]]:
        """
        Check PX4 health status from SYS_STATUS message.

        Returns:
            Tuple of (healthy, list_of_failed_checks)
        """
        if self._mav_connection is None:
            return False, ["No MAVLink connection"]

        from pymavlink import mavutil

        # Request SYS_STATUS message
        msg = self._mav_connection.recv_match(type='SYS_STATUS', blocking=False)
        if msg is None:
            return False, ["No SYS_STATUS received"]

        failed_checks = []

        # Check onboard_control_sensors_health bits
        # Bit positions for key health flags (from MAVLink spec)
        SENSOR_3D_GYRO = 1 << 0
        SENSOR_3D_ACCEL = 1 << 1
        SENSOR_3D_MAG = 1 << 2
        SENSOR_GPS = 1 << 5
        SENSOR_RC_RECEIVER = 1 << 13
        SENSOR_AHRS = 1 << 21
        SENSOR_BATTERY = 1 << 26

        health = msg.onboard_control_sensors_health
        present = msg.onboard_control_sensors_present

        # Only check sensors that are present
        if (present & SENSOR_3D_GYRO) and not (health & SENSOR_3D_GYRO):
            failed_checks.append("gyro")
        if (present & SENSOR_3D_ACCEL) and not (health & SENSOR_3D_ACCEL):
            failed_checks.append("accel")
        if (present & SENSOR_AHRS) and not (health & SENSOR_AHRS):
            failed_checks.append("AHRS/EKF")

        return len(failed_checks) == 0, failed_checks

    def _wait_for_simulation_ready(self, max_steps: int = 2000) -> bool:
        """
        Wait for simulation to be fully ready before training begins.

        This is the initialization "gate" that ensures:
        1. Isaac Sim physics is running
        2. PX4 heartbeat is received
        3. PX4 EKF2 health checks pass
        4. Vehicle position is stable (not falling)

        Only returns True when all conditions are met.

        Args:
            max_steps: Maximum simulation steps to wait

        Returns:
            True if simulation is ready, False if timed out
        """
        if self._mav_connection is None:
            print("  [READY] No MAVLink connection - cannot verify readiness")
            return False

        print("\n" + "=" * 50)
        print(" WAITING FOR SIMULATION READY")
        print("=" * 50)

        from pymavlink import mavutil

        # Phase 1: Wait for stable heartbeat
        print("\n  [1/4] Waiting for stable PX4 heartbeat...")
        heartbeat_count = 0
        required_heartbeats = 5

        for i in range(max_steps // 4):
            self.world.step(render=True)
            msg = self._mav_connection.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.get_srcSystem() != 0:
                heartbeat_count += 1
                if heartbeat_count >= required_heartbeats:
                    print(f"    ✓ Got {heartbeat_count} heartbeats after {i} steps")
                    break

            if i % 100 == 99:
                print(f"    ... step {i+1}, heartbeats: {heartbeat_count}")

        if heartbeat_count < required_heartbeats:
            print(f"    ✗ Only got {heartbeat_count} heartbeats")
            return False

        # Phase 2: Wait for EKF2 health
        print("\n  [2/4] Waiting for EKF2 health checks to pass...")
        health_ok_count = 0
        required_health_ok = 10  # Need sustained health

        for i in range(max_steps // 2):
            self.world.step(render=True)
            healthy, failed = self._check_px4_health()

            if healthy:
                health_ok_count += 1
                if health_ok_count >= required_health_ok:
                    print(f"    ✓ Health checks passed {health_ok_count} times after {i} steps")
                    break
            else:
                health_ok_count = 0  # Reset - need consecutive

            if i % 100 == 99:
                if failed:
                    print(f"    ... step {i+1}, failed: {failed}")
                else:
                    print(f"    ... step {i+1}, health_ok_streak: {health_ok_count}")

        if health_ok_count < required_health_ok:
            print(f"    ✗ Health checks not stable (streak: {health_ok_count})")
            # Continue anyway - EKF may still be usable

        # Phase 3: Check vehicle isn't falling
        print("\n  [3/4] Verifying vehicle position is stable...")
        initial_z = self.vehicle.state.position[2] if hasattr(self.vehicle, 'state') else 0

        for i in range(100):
            self.world.step(render=True)

        final_z = self.vehicle.state.position[2] if hasattr(self.vehicle, 'state') else 0
        z_change = final_z - initial_z

        if z_change < -1.0:
            print(f"    ✗ Vehicle falling! z changed by {z_change:.2f}m")
            print(f"      Initial: {initial_z:.2f}m, Final: {final_z:.2f}m")
            return False
        else:
            print(f"    ✓ Vehicle stable (z change: {z_change:.2f}m)")

        # Phase 4: Prime setpoints to prepare for OFFBOARD
        print("\n  [4/4] Priming setpoints for OFFBOARD mode...")
        for i in range(200):
            self.action_bridge.hover()
            self.world.step(render=True)
        print(f"    ✓ Sent 200 hover setpoints")

        # Final status
        armed, mode = self._check_px4_mode()
        print("\n" + "=" * 50)
        print(f" SIMULATION READY")
        print(f"   Armed: {armed}, Mode: {mode}")
        print(f"   Vehicle pos: {self.vehicle.state.position}")
        print("=" * 50 + "\n")

        return True

    def _engage_offboard_mode(self) -> bool:
        """
        Engage PX4 offboard mode for velocity control.

        NOTE: EKF2 warmup and setpoint priming is now done in _wait_for_simulation_ready()
        during setup(), so we can proceed directly to mode/arm commands here.

        Steps the world while sending commands to keep lockstep alive.
        Verifies mode transitions before proceeding.
        Returns True if successful.
        """
        if self._mav_connection is None:
            print("  [OFFBOARD] No MAVLink connection - cannot arm")
            return False

        if self._offboard_engaged:
            return True

        from pymavlink import mavutil

        mav = self._mav_connection
        print(f"  [OFFBOARD] Engaging offboard mode (target system {mav.target_system})")

        # Check current mode
        armed, mode = self._check_px4_mode()
        print(f"  [OFFBOARD] Current state: armed={armed}, custom_mode={mode}")

        # Set OFFBOARD mode with retry (keep stepping and sending setpoints)
        print("  [OFFBOARD] Requesting OFFBOARD mode...")
        for attempt in range(5):
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, 0, 0, 0, 0, 0  # 6 = OFFBOARD
            )
            # Keep sending setpoints while waiting (critical for OFFBOARD)
            for _ in range(100):
                self.action_bridge.hover()
                self.world.step(render=True)

            armed, mode = self._check_px4_mode()
            print(f"  [OFFBOARD] Attempt {attempt+1}: armed={armed}, custom_mode={mode}")
            if mode == 6:  # OFFBOARD mode
                break

        # Arm with retry
        print("  [OFFBOARD] Arming vehicle...")
        for attempt in range(5):
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0
            )
            # Keep sending setpoints
            for _ in range(100):
                self.action_bridge.hover()
                self.world.step(render=True)

            armed, mode = self._check_px4_mode()
            print(f"  [OFFBOARD] Arm attempt {attempt+1}: armed={armed}, custom_mode={mode}")
            if armed:
                break

        # Set OFFBOARD again after arming (sometimes needed)
        if mode != 6:
            print("  [OFFBOARD] Re-requesting OFFBOARD mode after arming...")
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, 0, 0, 0, 0, 0
            )
            for _ in range(50):
                self.action_bridge.hover()
                self.world.step(render=True)

        # Final verification
        armed, mode = self._check_px4_mode()
        print(f"  [OFFBOARD] Final state: armed={armed}, custom_mode={mode}")

        if armed and mode == 6:
            self._offboard_engaged = True
            print("  [OFFBOARD] SUCCESS - Vehicle armed in OFFBOARD mode!")
            return True
        else:
            print(f"  [OFFBOARD] WARNING - Mode engagement incomplete (armed={armed}, mode={mode})")
            # Still mark as engaged to allow commands, but warn
            self._offboard_engaged = True
            return False

    def _apply_action(self, action: np.ndarray) -> None:
        """
        Apply RL action to the vehicle via PX4 offboard velocity control.

        Action format: [vx, vy, vz, yaw_rate] normalized to [-1, 1]
        Converted to velocity commands via action_bridge.

        Safety shielding: If enabled, the action is first checked against
        the Vampire theorem prover. If the action would violate a hard
        constraint, it is replaced with a safe fallback (hover).

        If MAVLink is not connected (visual-only mode), action is ignored.
        """
        if self.action_bridge is None:
            return

        # Engage offboard mode on first action
        if not self._offboard_engaged:
            self._engage_offboard_mode()

        # Apply safety filter if enabled
        safe_action = action
        if self.safety_filter is not None and self._safety_filter_enabled:
            # Build current state dict for safety filter
            current_state = {
                'x': self.uav_state.position[0],
                'y': self.uav_state.position[1],
                'z': self.uav_state.position[2],
                'battery': self.uav_state.battery_pct,
                'in_geofence': self.uav_state.in_geofence,
                'in_nfz': self.uav_state.in_nfz,
            }

            # Convert normalized action to velocity for prediction
            vx, vy, vz, _ = self.action_bridge.action_to_velocity(action)
            velocity_action = np.array([vx, vy, vz, 0.0])

            # Filter action through Vampire safety check
            safe_action, was_modified, violation = self.safety_filter.filter_action(
                current_state,
                velocity_action,
                dt=1.0 / 60.0  # Assume 60Hz control rate
            )

            if was_modified:
                self._safety_interventions_count += 1
                if violation:
                    self._safety_violations_count += 1
                # Convert safe velocity back to normalized action
                safe_action = self.action_bridge.velocity_to_action(
                    safe_action[0], safe_action[1], safe_action[2], 0.0
                )

        # Convert normalized action to velocity and send
        self.action_bridge.execute_action(safe_action)

    def _update_state(self) -> None:
        """Update UAV state from simulation."""
        if self.vehicle is None:
            return

        # Get vehicle state from Pegasus
        state = self.vehicle.state

        # Update position and velocity
        self.uav_state.position = np.array(state.position)
        self.uav_state.velocity = np.array(state.linear_velocity)
        self.uav_state.orientation = np.array(state.attitude)
        self.uav_state.angular_velocity = np.array(state.angular_velocity)

        # Update timing
        self.uav_state.mission_time = self._step_count / 60.0  # Assuming 60Hz

        # Apply domain randomization effects
        self._apply_domain_randomization_effects()

        # Check geofence
        self.uav_state.in_geofence = self._check_geofence()

        # Check NFZ proximity
        self._update_nfz_state()

        # Check threat zones
        self._update_threat_state()

        # Update comms status
        self._update_comms_state()

        # Update autonomy mode
        if self.uav_state.comms_status == CommsStatus.DENIED:
            self.uav_state.autonomous_mode = True

        # Update battery reserve dynamically based on distance to home
        self.uav_state.battery_reserve = self.compute_required_battery_reserve()

    def _apply_domain_randomization_effects(self) -> None:
        """
        Apply domain randomization effects during state update.

        This method applies:
        - Battery drain rate variation
        - GNSS degradation triggering
        - VIO position drift accumulation
        - Wind vector updates
        """
        dr_config = self.config.domain_randomization

        if not dr_config.enabled:
            return

        dt = 1.0 / 60.0  # Assuming 60Hz simulation

        # Battery drain with variation
        # Base drain rate: ~0.1% per second at cruise (F-11 spec)
        base_drain_rate = 0.1  # % per second
        actual_drain_rate = base_drain_rate * self._episode_battery_drain_multiplier
        # Note: Assumes _update_state() called once per simulation step
        self.uav_state.battery_pct -= actual_drain_rate * dt
        self.uav_state.battery_pct = max(0.0, self.uav_state.battery_pct)

        # GNSS degradation trigger
        if (self._episode_gnss_degradation_step >= 0 and
                self._step_count >= self._episode_gnss_degradation_step and
                not self.uav_state.gnss_degradation_triggered):
            self.uav_state.gnss_status = GNSSStatus.DEGRADED
            self.uav_state.gnss_degradation_triggered = True
            self.uav_state.gnss_degradation_step = self._step_count

        # VIO position drift accumulation (random walk)
        if self._episode_vio_noise_scale > 0.0:
            # Accumulate drift as a random walk
            drift_increment = self._dr_rng.normal(0, self._episode_vio_noise_scale * dt, 3)
            self.uav_state.vio_position_drift += drift_increment

            # Cap VIO drift at realistic bounds (real VIO has loop closure)
            max_drift = 10.0  # meters
            drift_magnitude = np.linalg.norm(self.uav_state.vio_position_drift)
            if drift_magnitude > max_drift:
                self.uav_state.vio_position_drift *= (max_drift / drift_magnitude)

        # Update wind vector in state (constant for episode, but exposed for observation)
        self.uav_state.wind_vector = self._episode_wind_vector.copy()

    def _check_geofence(self) -> bool:
        """Check if UAV is within geofence."""
        pos = self.uav_state.position
        gf = self.config.geofence
        return (
            gf.min_x <= pos[0] <= gf.max_x and
            gf.min_y <= pos[1] <= gf.max_y and
            gf.min_z <= pos[2] <= gf.max_z
        )

    def _update_nfz_state(self) -> None:
        """Update NFZ-related state."""
        pos = self.uav_state.position
        min_dist = float('inf')
        in_nfz = False
        current_nfz = None

        for nfz in self.nfz_zones:
            if not nfz.is_active:
                continue

            # Calculate distance to NFZ cylinder
            horizontal_dist = np.linalg.norm(pos[:2] - nfz.center[:2])
            dist = horizontal_dist - nfz.radius

            if dist < min_dist:
                min_dist = dist

            # Check if inside NFZ
            if horizontal_dist < nfz.radius:
                z_min = nfz.center[2] - nfz.height / 2
                z_max = nfz.center[2] + nfz.height / 2
                if z_min <= pos[2] <= z_max:
                    in_nfz = True
                    current_nfz = nfz.id

        self.uav_state.in_nfz = in_nfz
        self.uav_state.current_nfz_id = current_nfz
        self.uav_state.nfz_distance = max(0, min_dist)

    def _update_threat_state(self) -> None:
        """Update threat zone state."""
        pos = self.uav_state.position
        max_threat = ThreatLevel.NONE
        in_threat = False

        for zone in self.threat_zones:
            dist = np.linalg.norm(pos[:2] - zone.center[:2])
            if dist < zone.radius:
                in_threat = True
                if zone.threat_level.value > max_threat.value:
                    max_threat = zone.threat_level

                    # Accumulate exposure for medium threats
                    if zone.threat_level == ThreatLevel.MEDIUM:
                        dt = 1.0 / 60.0  # Assuming 60Hz
                        self.uav_state.threat_exposure += zone.risk_rate * dt

        self.uav_state.in_threat_zone = in_threat
        self.uav_state.current_threat_level = max_threat

    def _update_comms_state(self) -> None:
        """Update communications status based on zones."""
        pos = self.uav_state.position

        # Default to operational if no zones defined
        if not self.comms_zones:
            return

        # Check if in any comms zone
        for zone in self.comms_zones:
            dist = np.linalg.norm(pos[:2] - zone.center[:2])
            if dist < zone.radius:
                self.uav_state.comms_status = zone.status
                return

        # Outside all zones = denied
        self.uav_state.comms_status = CommsStatus.DENIED

    def compute_required_battery_reserve(self) -> float:
        """
        Compute required battery reserve based on distance to home.

        Uses F-11 flight characteristics to calculate minimum battery needed
        to safely return to launch point. This replaces hardcoded flat reserves.

        Formula: reserve = (distance / cruise_speed) * power_rate * safety_factor

        F-11 parameters:
        - Cruise speed: 6.3 m/s
        - Power consumption: ~0.1% per second at cruise
        - Safety factor: 1.5 (accounts for wind, altitude changes, maneuvering)
        - Minimum reserve: 10% (hard floor for landing buffer)

        Returns:
            Required battery reserve percentage
        """
        # Calculate 3D distance to home (spawn position)
        distance_to_home = np.linalg.norm(
            self.uav_state.position - self.config.spawn_position
        )

        # F-11 flight parameters
        cruise_speed = 6.3   # m/s (F-11 cruise speed)
        power_rate = 0.1     # % per second at cruise
        safety_factor = 1.5  # Wind, altitude changes, maneuvering buffer
        min_reserve = 10.0   # % hard floor for landing

        # Time to fly home at cruise speed
        time_to_home = distance_to_home / cruise_speed

        # Battery needed to return
        required = time_to_home * power_rate * safety_factor
        # Apply domain randomization battery drain multiplier
        # If drain is faster, we need more reserve to get home
        required *= self.uav_state.battery_drain_multiplier


        # Apply comms-denied multiplier (more conservative when autonomous)
        if self.uav_state.comms_status == CommsStatus.DENIED:
            required *= 1.2  # 20% extra buffer when comms-denied

        return max(min_reserve, required)

    def px4_cmd_async(self, cmd_args: str, callback: Optional[Callable] = None) -> None:
        """Execute PX4 commander command in background thread."""
        def run():
            try:
                result = subprocess.run(
                    [self._px4_cmd] + cmd_args.split(),
                    capture_output=True,
                    text=True,
                    timeout=10,
                    cwd="/px4"
                )
                if callback:
                    callback(result.returncode == 0)
            except Exception as e:
                print(f"  PX4 cmd error: {e}")
                if callback:
                    callback(False)

        thread = threading.Thread(target=run, daemon=True)
        thread.start()

    def arm(self) -> None:
        """Arm the vehicle."""
        self.px4_cmd_async("arm")

    def disarm(self) -> None:
        """Disarm the vehicle."""
        self.px4_cmd_async("disarm")

    def takeoff(self) -> None:
        """Command takeoff."""
        self.px4_cmd_async("takeoff")

    def land(self) -> None:
        """Command landing."""
        self.px4_cmd_async("land")

    def rtl(self) -> None:
        """Return to launch."""
        self.px4_cmd_async("mode rtl")

    # ==========================================
    # Safety Shield Interface
    # ==========================================

    def check_action_safety(self, action: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Check if action is safe using Vampire theorem prover.

        Returns:
            Tuple of (is_safe, safe_alternative)
        """
        if self.ontology_bridge is None:
            return True, None

        return self.ontology_bridge.verify_action_safety(
            action, self.uav_state, self.nfz_zones, self.threat_zones
        )

    # ==========================================
    # State Conversion for RL
    # ==========================================

    def state_to_observation(self, state: UAVState) -> np.ndarray:
        """
        Convert UAV state to RL observation vector.

        Returns:
            Flat numpy array suitable for neural network input.
            If perception is enabled, includes 516-dim perception observation.

        DUAL-MODE PERCEPTION:
        - GT mode: Uses world_objects from _get_world_objects() (no rendering)
        - Full mode: Uses camera image from _camera
        """
        # Normalize wind vector (max 10 m/s for normalization)
        wind_normalized = state.wind_vector / 10.0

        # Base state observation
        # Clip nfz_distance to avoid infinity (default is inf when no NFZ nearby)
        nfz_distance_clipped = min(state.nfz_distance, 500.0)  # Cap at 500m

        state_obs = np.concatenate([
            state.position,
            state.velocity,
            state.orientation,
            wind_normalized,  # Wind vector for planning (3 values)
            [
                state.battery_pct / 100.0,
                state.mission_time / self.config.max_mission_time,
                state.coverage_pct / 100.0,
                float(state.in_geofence),
                float(state.in_nfz),
                nfz_distance_clipped / 100.0,  # Use clipped value (0-5 range)
                float(state.in_threat_zone),
                state.current_threat_level.value / 3.0,
                state.threat_exposure / state.max_threat_exposure,
                float(state.autonomous_mode),
                state.comms_status.value / 2.0,
                state.gnss_status.value / 2.0,
                float(state.vio_valid),
                float(state.gnss_degradation_triggered),  # GNSS degradation flag
                state.battery_drain_multiplier,  # Battery drain rate (for awareness)
            ]
        ])

        # Add perception observation if enabled
        if self.config.perception.enabled and self.perception is not None:
            # Check if we're using DualModePerception (new) or PerceptionIntegration (legacy)
            if hasattr(self.perception, 'get_observation'):
                # New DualModePerception
                perception_obs = self._get_perception_observation_dual_mode(state)
            else:
                # Legacy PerceptionIntegration
                image = None
                if self._camera is not None and hasattr(self.perception, 'capture_image'):
                    image = self.perception.capture_image()

                perception_obs = self.perception.encode(
                    image=image,
                    uav_position=state.position,
                    uav_orientation=state.orientation,
                )

            # Cache for info dict
            self._perception_obs = perception_obs

            # Combine state + perception
            return np.concatenate([state_obs, perception_obs]).astype(np.float32)

        return state_obs.astype(np.float32)

    def _get_perception_observation_dual_mode(self, state: UAVState) -> np.ndarray:
        """
        Get perception observation using DualModePerception.

        Handles both GT mode (world objects) and full mode (camera image).
        """
        mode = self.perception.mode if hasattr(self.perception, 'mode') else 'gt'

        if mode == 'gt':
            # Get world objects from simulation for ground truth detection
            world_objects = self._get_world_objects_for_perception()
            return self.perception.get_observation_gt(
                world_objects=world_objects,
                uav_position=state.position,
                uav_orientation=state.orientation,
            )
        else:
            # Full mode: get camera image
            image = None
            if self._camera is not None:
                try:
                    rgba = self._camera.get_rgba()
                    if rgba is not None and rgba.max() > 5:
                        image = rgba[:, :, :3]  # RGB only
                except Exception:
                    pass

            if image is None:
                # Return zeros if no image available
                return np.zeros(self.perception.OUTPUT_DIM, dtype=np.float32)

            return self.perception.get_observation_full(
                image=image,
                uav_position=state.position,
            )

    def _get_world_objects_for_perception(self) -> List:
        """
        Get world objects from simulation for ground truth perception.

        Override this in subclasses to provide simulation-specific object extraction.

        Returns:
            List of WorldObject instances representing detectable objects
        """
        # Import WorldObject type
        try:
            import sys
            sys.path.insert(0, '/workspace/perception')
            from perception import WorldObject
        except ImportError:
            return []

        world_objects = []

        # Convert targets to world objects
        for target in self.targets:
            world_objects.append(WorldObject(
                id=target.id,
                class_id=4,  # POI target
                class_name='poi_target',
                position=target.position,
                ontology_class='TargetOfInterest',
                priority=target.priority,
            ))

        # Subclasses should override this to add more objects
        # (people, vehicles, buildings from procedural generation, etc.)

        return world_objects

    @property
    def observation_dim(self) -> int:
        """Dimension of observation space."""
        # pos(3) + vel(3) + quat(4) + wind(3) + scalars(15) = 28
        base_dim = 3 + 3 + 4 + 3 + 15

        # Add perception dimension if enabled
        if self.config.perception.enabled:
            return base_dim + self.config.perception.observation_dim

        return base_dim

    def get_camera_image(self) -> Optional[np.ndarray]:
        """
        Get RGB image from drone camera for rendering.

        Returns:
            RGB image as numpy array (H, W, 3) or None
        """
        if self._camera is None:
            return None
        if self.perception is not None:
            return self.perception.capture_image()
        return None

    def get_perception_stats(self) -> Dict[str, Any]:
        """Get perception statistics for logging."""
        if self.perception is not None:
            return self.perception.get_stats()
        return {"perception_enabled": False}

    @property
    def action_dim(self) -> int:
        """Dimension of action space."""
        return 4  # x, y, z velocity + yaw rate (or position setpoint)

    def get_safety_stats(self) -> Dict[str, Any]:
        """
        Get safety filter statistics for logging/info dict.

        Returns:
            Dict with safety intervention counts and rates
        """
        total_steps = max(1, self._step_count)
        return {
            'safety_filter_enabled': self._safety_filter_enabled,
            'safety_violations': self._safety_violations_count,
            'safety_interventions': self._safety_interventions_count,
            'intervention_rate': self._safety_interventions_count / total_steps,
        }

    def get_domain_randomization_stats(self) -> Dict[str, Any]:
        """
        Get domain randomization statistics for logging/info dict.

        Returns:
            Dict with current episode's DR parameters
        """
        wind_magnitude = np.linalg.norm(self._episode_wind_vector)
        wind_direction_deg = np.degrees(np.arctan2(
            self._episode_wind_vector[1],
            self._episode_wind_vector[0]
        )) if wind_magnitude > 0 else 0.0

        return {
            'dr_enabled': self.config.domain_randomization.enabled,
            'battery_drain_multiplier': self._episode_battery_drain_multiplier,
            'gnss_degradation_step': self._episode_gnss_degradation_step,
            'gnss_degraded': self.uav_state.gnss_degradation_triggered,
            'wind_magnitude_mps': wind_magnitude,
            'wind_direction_deg': wind_direction_deg,
            'vio_noise_scale': self._episode_vio_noise_scale,
            'vio_drift_magnitude': np.linalg.norm(self.uav_state.vio_position_drift),
        }
