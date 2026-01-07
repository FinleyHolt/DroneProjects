#!/usr/bin/env python3
"""
Base ISR Environment for Flyby F-11 Training

Provides the foundational simulation environment for all three canonical problems.
Built on Isaac Sim + Pegasus Simulator with PX4 SITL integration.

Architecture:
    BaseISREnvironment (orchestrator) composes:
    - SimulationCore: Isaac Sim + Pegasus + PX4 initialization
    - ZoneManager: NFZ, threat, comms zone tracking
    - BatteryManager: Battery state and reserve calculations
    - DomainRandomizationManager: Episode DR sampling and effects
    - PerceptionManager: Camera and detection integration
    - SafetyFilter: Vampire ATP action shielding
    - ActionBridge: RL-to-PX4 control

Key Features:
- Modular environment architecture
- Domain randomization hooks
- Ontology state extraction
- Reward computation framework
- Safety shield integration via Vampire ATP
- Live perception integration (YOLO detection during RL training)
"""

import sys
import subprocess
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Callable, TYPE_CHECKING
from enum import Enum, auto
from abc import ABC, abstractmethod

if TYPE_CHECKING:
    from .perception_manager import PerceptionManager

# Add Pegasus to Python path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)


# =============================================================================
# Enums and Data Classes
# =============================================================================

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
    max_z: float = 120.0


@dataclass
class NoFlyZone:
    """No-fly zone definition."""
    id: str
    center: np.ndarray
    radius: float
    height: float
    priority: int = 1
    is_dynamic: bool = False
    is_active: bool = True
    buffer_distance: float = 10.0


@dataclass
class ThreatZone:
    """Threat zone definition."""
    id: str
    center: np.ndarray
    radius: float
    threat_level: ThreatLevel
    risk_rate: float = 1.0


@dataclass
class TargetOfInterest:
    """ISR target definition."""
    id: str
    position: np.ndarray
    priority: int
    value: float
    battery_cost: float
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
class UAVState:
    """Complete UAV state for RL observation."""
    # Position and velocity
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([1, 0, 0, 0]))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Flight state
    flight_phase: FlightPhase = FlightPhase.PREFLIGHT
    battery_pct: float = 100.0
    battery_reserve: float = 25.0

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

    # Domain randomization state
    wind_vector: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vio_position_drift: np.ndarray = field(default_factory=lambda: np.zeros(3))
    battery_drain_multiplier: float = 1.0
    gnss_degradation_triggered: bool = False
    gnss_degradation_step: int = -1


@dataclass
class DomainRandomizationConfig:
    """Domain randomization configuration."""
    enabled: bool = True
    battery_variation_enabled: bool = True
    battery_drain_rate_variation: float = 0.1
    gnss_degradation_enabled: bool = True
    gnss_degradation_prob: float = 0.1
    vio_noise_enabled: bool = True
    vio_noise_scale: float = 0.0
    wind_enabled: bool = True
    wind_speed_range: Tuple[float, float] = (0.0, 5.0)


@dataclass
class PerceptionConfig:
    """Perception configuration."""
    enabled: bool = True
    mode: str = "gt"
    camera_resolution: Tuple[int, int] = (640, 480)
    camera_fov_degrees: float = 90.0
    max_encode_time_ms: float = 20.0
    skip_frames: int = 0
    gt_bbox_noise_std: float = 0.02
    gt_confidence_noise_std: float = 0.05
    gt_position_noise_std: float = 0.5
    observation_dim: int = 516

    # Resolution presets for fast training
    resolution_preset: str = "full"  # "training", "evaluation", "full"

    @property
    def effective_resolution(self) -> Tuple[int, int]:
        """Get resolution based on preset, falling back to camera_resolution if custom."""
        presets = {
            "training": (128, 128),
            "evaluation": (320, 240),
            "full": (640, 480),
        }
        if self.resolution_preset in presets:
            return presets[self.resolution_preset]
        return self.camera_resolution


@dataclass
class EnvironmentConfig:
    """Environment configuration."""
    headless: bool = False
    physics_dt: float = 1.0 / 250.0
    render_dt: float = 1.0 / 60.0
    world_name: str = "isr_training"
    environment_usd: Optional[str] = None
    skip_default_environment: bool = False
    geofence: GeofenceConfig = field(default_factory=GeofenceConfig)
    latitude: float = 33.3853
    longitude: float = -117.5653
    altitude: float = 100.0
    spawn_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.5]))
    spawn_orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    max_mission_time: float = 900.0
    randomize_lighting: bool = True
    randomize_weather: bool = True
    randomize_obstacles: bool = True
    randomize_sensors: bool = True
    domain_randomization: DomainRandomizationConfig = field(default_factory=DomainRandomizationConfig)
    perception: PerceptionConfig = field(default_factory=PerceptionConfig)

    # Fast training: render frame skipping
    render_interval: int = 1  # Render every Nth frame (1 = every frame, 4 = skip 3 frames)


# =============================================================================
# BaseISREnvironment
# =============================================================================

class BaseISREnvironment(ABC):
    """
    Base class for ISR training environments.

    Composes specialized managers for different concerns:
    - Zone tracking (NFZ, threat, comms)
    - Battery management
    - Domain randomization
    - Perception
    - Safety filtering
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
        self._dr_rng = np.random.default_rng()
        self._episode_battery_drain_multiplier = 1.0
        self._episode_gnss_degradation_step = -1
        self._episode_wind_vector = np.zeros(3)
        self._episode_vio_noise_scale = 0.0

        # Ontology bridge
        self.ontology_bridge = None

        # Perception
        self.perception: Optional['PerceptionManager'] = None
        self._camera = None
        self._perception_obs = None

        # PX4 command path
        self._px4_cmd = "/px4/build/px4_sitl_default/bin/px4-commander"

        # Action bridge
        self.action_bridge = None
        self._mav_connection = None
        self._offboard_engaged = False

        # Safety filter
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

        # Phase 1: Isaac Sim
        print("\n[1/5] Initializing Isaac Sim...")
        self._init_isaac_sim()

        # Phase 2: Pegasus + Vehicle
        print("\n[2/5] Setting up Pegasus and vehicle...")
        self._init_pegasus_and_vehicle()

        # Phase 3: PX4
        print("\n[3/5] Launching PX4 SITL...")
        self._launch_px4()

        # Phase 4: Managers
        print("\n[4/5] Initializing managers...")
        self._init_managers()

        # Phase 5: Environment
        print("\n[5/5] Setting up environment...")
        self._setup_environment()

        # Ready check
        ready = self._wait_for_simulation_ready(max_steps=3000)

        self._initialized = True
        print("\n" + "=" * 60)
        print("Simulation ready!" if ready else "Simulation ready (with warnings)")
        print("=" * 60)

    def _init_isaac_sim(self) -> None:
        """Initialize Isaac Sim application."""
        from isaacsim import SimulationApp
        self.simulation_app = SimulationApp({"headless": self.config.headless})
        print("  Isaac Sim ready")

    def _init_pegasus_and_vehicle(self) -> None:
        """Initialize Pegasus interface and spawn vehicle."""
        import omni.timeline
        from isaacsim.core.api.world import World
        from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
        from pegasus.simulator.logic.backends.px4_mavlink_backend import (
            PX4MavlinkBackend, PX4MavlinkBackendConfig
        )
        from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
        from scipy.spatial.transform import Rotation

        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.set_px4_path("/px4")
        self.pg.set_global_coordinates(
            self.config.latitude, self.config.longitude, self.config.altitude
        )

        # Load environment
        if self.config.environment_usd:
            self.pg.load_environment(self.config.environment_usd)
        elif not self.config.skip_default_environment:
            self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Spawn vehicle
        px4_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": False,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
            "enable_lockstep": True,
            "update_rate": 250.0,
        })

        vehicle_config = MultirotorConfig()
        self._px4_backend = PX4MavlinkBackend(px4_config)
        vehicle_config.backends = [self._px4_backend]

        spawn_rot = Rotation.from_euler(
            "XYZ", self.config.spawn_orientation.tolist(), degrees=True
        ).as_quat()

        self.vehicle = Multirotor(
            "/World/uav",
            ROBOTS["Iris"],
            0,
            self.config.spawn_position.tolist(),
            spawn_rot,
            config=vehicle_config,
        )
        print("  Pegasus and vehicle ready")

    def _launch_px4(self) -> None:
        """Launch PX4 SITL with continuous stepping."""
        from pegasus.simulator.logic.backends.tools.px4_launch_tool import PX4LaunchTool
        import time

        # Start timeline
        self.world.reset()
        self.timeline.play()
        for _ in range(50):
            self.world.step(render=True)

        # Continuous stepping during PX4 boot
        self._px4_stepping_active = True

        def step_continuously():
            while self._px4_stepping_active:
                try:
                    self.world.step(render=True)
                except Exception:
                    break

        step_thread = threading.Thread(target=step_continuously, daemon=True)
        step_thread.start()

        # Launch PX4
        self._px4_tool = PX4LaunchTool(
            self.pg.px4_path, vehicle_id=0, px4_model=self.pg.px4_default_airframe
        )
        self._px4_tool.launch_px4()
        self._px4_backend.px4_tool = self._px4_tool
        self._px4_backend.px4_autolaunch = True

        time.sleep(2.0)
        self._px4_stepping_active = False
        step_thread.join(timeout=1.0)

        for _ in range(300):
            self.world.step(render=True)
        print("  PX4 SITL ready")

    def _init_managers(self) -> None:
        """Initialize component managers."""
        # Action bridge
        self._setup_action_bridge()

        # Safety filter
        self._setup_safety_filter()

        # Perception
        self._setup_perception()

    def _setup_action_bridge(self) -> None:
        """Setup MAVLink action bridge."""
        from .action_bridge import PX4ActionBridge
        from pymavlink import mavutil

        if not self.timeline.is_playing():
            self.timeline.play()
            for _ in range(200):
                self.world.step(render=True)

        try:
            self._mav_connection = mavutil.mavlink_connection(
                'udpin:localhost:14550', source_system=255
            )

            heartbeat_received = False
            for i in range(500):
                self.world.step(render=True)
                msg = self._mav_connection.recv_match(type='HEARTBEAT', blocking=False)
                if msg and msg.get_srcSystem() != 0:
                    heartbeat_received = True
                    break

            if heartbeat_received:
                if self._mav_connection.target_system == 0:
                    self._mav_connection.target_system = 1
                    self._mav_connection.target_component = 1
            else:
                self._mav_connection = None

        except Exception as e:
            print(f"  Warning: MAVLink connection failed: {e}")
            self._mav_connection = None

        self.action_bridge = PX4ActionBridge(self._mav_connection)
        print("  Action bridge ready")

    def _setup_safety_filter(self) -> None:
        """Setup Vampire safety filter."""
        from .safety_filter import VampireSafetyFilter
        import shutil

        vampire_available = shutil.which('vampire') is not None

        self.safety_filter = VampireSafetyFilter(
            ontology_path="/workspace/ontology/planning_mode",
            timeout_ms=50,
            enabled=self._safety_filter_enabled
        )
        print("  Safety filter ready")

    def _setup_perception(self) -> None:
        """Setup perception pipeline."""
        if not self.config.perception.enabled:
            print("  Perception: DISABLED")
            return

        from .perception_manager import PerceptionManager, PerceptionConfig as PMConfig

        pm_config = PMConfig(
            enabled=self.config.perception.enabled,
            mode=self.config.perception.mode,
            camera_resolution=self.config.perception.camera_resolution,
            camera_fov_degrees=self.config.perception.camera_fov_degrees,
            max_encode_time_ms=self.config.perception.max_encode_time_ms,
            skip_frames=self.config.perception.skip_frames,
            gt_bbox_noise_std=self.config.perception.gt_bbox_noise_std,
            gt_confidence_noise_std=self.config.perception.gt_confidence_noise_std,
            gt_position_noise_std=self.config.perception.gt_position_noise_std,
            observation_dim=self.config.perception.observation_dim,
        )

        self.perception = PerceptionManager(pm_config)
        self.perception.initialize(self.world)
        print(f"  Perception ready (mode: {self.perception.mode})")

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
        """Execute one environment step with optional render skipping for fast training."""
        if not self._initialized:
            self.setup()

        self._prev_state = self.uav_state
        self._apply_action(action)

        # Render skipping: only render every render_interval frames
        self._step_count += 1
        need_render = (self._step_count % self.config.render_interval == 0)
        self.world.step(render=need_render)

        self._update_state()

        reward = self.compute_reward(self._prev_state, self.uav_state)
        done = self.is_done(self.uav_state)
        info = self.get_info(self.uav_state) or {}

        # Add render info for debugging/monitoring
        info['rendered'] = need_render
        info['render_interval'] = self.config.render_interval

        return self.uav_state, reward, done, info

    def reset(self, seed: Optional[int] = None) -> UAVState:
        """Reset the environment."""
        if not self._initialized:
            self.setup()

        if self.randomizer and seed is not None:
            self.randomizer.randomize(seed)

        self._sample_domain_randomization(seed)

        if not self.timeline.is_playing():
            self.timeline.play()

        self._reset_vehicle_position()
        self._offboard_engaged = False
        self._safety_violations_count = 0
        self._safety_interventions_count = 0
        self._step_count = 0

        self.uav_state = UAVState(
            position=self.config.spawn_position.copy(),
            battery_pct=100.0,
            wind_vector=self._episode_wind_vector.copy(),
            battery_drain_multiplier=self._episode_battery_drain_multiplier,
            gnss_degradation_step=self._episode_gnss_degradation_step,
        )

        for _ in range(100):
            self.world.step(render=True)

        self._apply_wind_to_px4()
        self._update_state()
        return self.uav_state

    def _sample_domain_randomization(self, seed: Optional[int] = None) -> None:
        """Sample domain randomization values for episode."""
        dr_config = self.config.domain_randomization

        if seed is not None:
            self._dr_rng = np.random.default_rng(seed)

        if not dr_config.enabled:
            self._episode_battery_drain_multiplier = 1.0
            self._episode_gnss_degradation_step = -1
            self._episode_wind_vector = np.zeros(3)
            self._episode_vio_noise_scale = 0.0
            return

        if dr_config.battery_variation_enabled:
            var = dr_config.battery_drain_rate_variation
            self._episode_battery_drain_multiplier = self._dr_rng.uniform(1.0 - var, 1.0 + var)
        else:
            self._episode_battery_drain_multiplier = 1.0

        if dr_config.gnss_degradation_enabled and self._dr_rng.random() < dr_config.gnss_degradation_prob:
            max_steps = int(self.config.max_mission_time * 60 * 0.8)
            self._episode_gnss_degradation_step = self._dr_rng.integers(0, max(1, max_steps))
        else:
            self._episode_gnss_degradation_step = -1

        if dr_config.wind_enabled:
            min_speed, max_speed = dr_config.wind_speed_range
            wind_magnitude = self._dr_rng.uniform(min_speed, max_speed)
            wind_direction = self._dr_rng.uniform(0, 2 * np.pi)
            self._episode_wind_vector = np.array([
                wind_magnitude * np.cos(wind_direction),
                wind_magnitude * np.sin(wind_direction),
                0.0
            ])
        else:
            self._episode_wind_vector = np.zeros(3)

        self._episode_vio_noise_scale = dr_config.vio_noise_scale if dr_config.vio_noise_enabled else 0.0

    def _apply_wind_to_px4(self) -> None:
        """Apply wind parameters to PX4 SITL."""
        if self._mav_connection is None:
            return

        from pymavlink import mavutil

        wind_magnitude = np.linalg.norm(self._episode_wind_vector[:2])
        if wind_magnitude < 0.1:
            return

        wind_direction_rad = np.arctan2(
            self._episode_wind_vector[1], self._episode_wind_vector[0]
        )
        aviation_heading = (90.0 - np.degrees(wind_direction_rad)) % 360.0

        self._mav_connection.mav.param_set_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            b'SIM_WIND_SPD', wind_magnitude,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        self._mav_connection.mav.param_set_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            b'SIM_WIND_DIR', aviation_heading,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        for _ in range(6):
            self.world.step(render=True)

    def _reset_vehicle_position(self) -> None:
        """Reset vehicle position without restarting PX4."""
        for _ in range(100):
            self.world.step(render=True)

    def _wait_for_simulation_ready(self, max_steps: int = 2000) -> bool:
        """Wait for simulation to be fully ready."""
        if self._mav_connection is None:
            return False

        # Heartbeat check
        heartbeat_count = 0
        for i in range(max_steps // 4):
            self.world.step(render=True)
            msg = self._mav_connection.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.get_srcSystem() != 0:
                heartbeat_count += 1
                if heartbeat_count >= 5:
                    break

        if heartbeat_count < 5:
            return False

        # Stability check
        initial_z = self.vehicle.state.position[2] if hasattr(self.vehicle, 'state') else 0
        for _ in range(100):
            self.world.step(render=True)
        final_z = self.vehicle.state.position[2] if hasattr(self.vehicle, 'state') else 0

        if final_z - initial_z < -1.0:
            return False

        # Prime setpoints
        for _ in range(200):
            self.action_bridge.hover()
            self.world.step(render=True)

        return True

    def _engage_offboard_mode(self) -> bool:
        """Engage PX4 offboard mode."""
        if self._mav_connection is None or self._offboard_engaged:
            return self._offboard_engaged

        from pymavlink import mavutil
        mav = self._mav_connection

        # Set OFFBOARD mode
        for _ in range(5):
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, 0, 0, 0, 0, 0
            )
            for _ in range(100):
                self.action_bridge.hover()
                self.world.step(render=True)

        # Arm
        for _ in range(5):
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0
            )
            for _ in range(100):
                self.action_bridge.hover()
                self.world.step(render=True)

        self._offboard_engaged = True
        return True

    def _apply_action(self, action: np.ndarray) -> None:
        """Apply RL action via PX4 offboard control."""
        if self.action_bridge is None:
            return

        if not self._offboard_engaged:
            self._engage_offboard_mode()

        # Safety filter
        safe_action = action
        if self.safety_filter is not None and self._safety_filter_enabled:
            current_state = {
                'x': self.uav_state.position[0],
                'y': self.uav_state.position[1],
                'z': self.uav_state.position[2],
                'battery': self.uav_state.battery_pct,
                'in_geofence': self.uav_state.in_geofence,
                'in_nfz': self.uav_state.in_nfz,
            }

            vx, vy, vz, _ = self.action_bridge.action_to_velocity(action)
            velocity_action = np.array([vx, vy, vz, 0.0])

            safe_action, was_modified, violation = self.safety_filter.filter_action(
                current_state, velocity_action, dt=1.0 / 60.0
            )

            if was_modified:
                self._safety_interventions_count += 1
                if violation:
                    self._safety_violations_count += 1
                safe_action = self.action_bridge.velocity_to_action(
                    safe_action[0], safe_action[1], safe_action[2], 0.0
                )

        self.action_bridge.execute_action(safe_action)

    def _update_state(self) -> None:
        """Update UAV state from simulation."""
        if self.vehicle is None:
            return

        state = self.vehicle.state
        self.uav_state.position = np.array(state.position)
        self.uav_state.velocity = np.array(state.linear_velocity)
        self.uav_state.orientation = np.array(state.attitude)
        self.uav_state.angular_velocity = np.array(state.angular_velocity)
        self.uav_state.mission_time = self._step_count / 60.0

        self._apply_domain_randomization_effects()
        self.uav_state.in_geofence = self._check_geofence()
        self._update_nfz_state()
        self._update_threat_state()
        self._update_comms_state()

        if self.uav_state.comms_status == CommsStatus.DENIED:
            self.uav_state.autonomous_mode = True

        self.uav_state.battery_reserve = self.compute_required_battery_reserve()

    def _apply_domain_randomization_effects(self) -> None:
        """Apply DR effects during state update."""
        dr_config = self.config.domain_randomization
        if not dr_config.enabled:
            return

        dt = 1.0 / 60.0

        # Battery drain
        base_drain_rate = 0.1
        actual_drain_rate = base_drain_rate * self._episode_battery_drain_multiplier
        self.uav_state.battery_pct -= actual_drain_rate * dt
        self.uav_state.battery_pct = max(0.0, self.uav_state.battery_pct)

        # GNSS degradation
        if (self._episode_gnss_degradation_step >= 0 and
                self._step_count >= self._episode_gnss_degradation_step and
                not self.uav_state.gnss_degradation_triggered):
            self.uav_state.gnss_status = GNSSStatus.DEGRADED
            self.uav_state.gnss_degradation_triggered = True
            self.uav_state.gnss_degradation_step = self._step_count

        # VIO drift
        if self._episode_vio_noise_scale > 0.0:
            drift_increment = self._dr_rng.normal(0, self._episode_vio_noise_scale * dt, 3)
            self.uav_state.vio_position_drift += drift_increment
            max_drift = 10.0
            drift_magnitude = np.linalg.norm(self.uav_state.vio_position_drift)
            if drift_magnitude > max_drift:
                self.uav_state.vio_position_drift *= (max_drift / drift_magnitude)

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

            horizontal_dist = np.linalg.norm(pos[:2] - nfz.center[:2])
            dist = horizontal_dist - nfz.radius

            if dist < min_dist:
                min_dist = dist

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
                    if zone.threat_level == ThreatLevel.MEDIUM:
                        dt = 1.0 / 60.0
                        self.uav_state.threat_exposure += zone.risk_rate * dt

        self.uav_state.in_threat_zone = in_threat
        self.uav_state.current_threat_level = max_threat

    def _update_comms_state(self) -> None:
        """Update communications status."""
        pos = self.uav_state.position

        if not self.comms_zones:
            return

        for zone in self.comms_zones:
            dist = np.linalg.norm(pos[:2] - zone.center[:2])
            if dist < zone.radius:
                self.uav_state.comms_status = zone.status
                return

        self.uav_state.comms_status = CommsStatus.DENIED

    def compute_required_battery_reserve(self) -> float:
        """Compute required battery reserve based on distance to home."""
        distance_to_home = np.linalg.norm(
            self.uav_state.position - self.config.spawn_position
        )

        cruise_speed = 6.3
        power_rate = 0.1
        safety_factor = 1.5
        min_reserve = 10.0

        time_to_home = distance_to_home / cruise_speed
        required = time_to_home * power_rate * safety_factor
        required *= self.uav_state.battery_drain_multiplier

        if self.uav_state.comms_status == CommsStatus.DENIED:
            required *= 1.2

        return max(min_reserve, required)

    def close(self) -> None:
        """Cleanup and close the environment."""
        if self._initialized:
            self.timeline.stop()
            self.simulation_app.close()
            self._initialized = False

    # PX4 Commands
    def px4_cmd_async(self, cmd_args: str, callback: Optional[Callable] = None) -> None:
        """Execute PX4 commander command in background thread."""
        def run():
            try:
                result = subprocess.run(
                    [self._px4_cmd] + cmd_args.split(),
                    capture_output=True, text=True, timeout=10, cwd="/px4"
                )
                if callback:
                    callback(result.returncode == 0)
            except Exception as e:
                if callback:
                    callback(False)

        threading.Thread(target=run, daemon=True).start()

    def arm(self) -> None:
        self.px4_cmd_async("arm")

    def disarm(self) -> None:
        self.px4_cmd_async("disarm")

    def takeoff(self) -> None:
        self.px4_cmd_async("takeoff")

    def land(self) -> None:
        self.px4_cmd_async("land")

    def rtl(self) -> None:
        self.px4_cmd_async("mode rtl")

    # Observation
    def state_to_observation(self, state: UAVState, include_perception: bool = True) -> np.ndarray:
        """
        Convert UAV state to RL observation vector.

        Args:
            state: Current UAV state
            include_perception: If False, return state-only observation (for skipped render frames)
        """
        wind_normalized = state.wind_vector / 10.0
        nfz_distance_clipped = min(state.nfz_distance, 500.0)

        state_obs = np.concatenate([
            state.position,
            state.velocity,
            state.orientation,
            wind_normalized,
            [
                state.battery_pct / 100.0,
                state.mission_time / self.config.max_mission_time,
                state.coverage_pct / 100.0,
                float(state.in_geofence),
                float(state.in_nfz),
                nfz_distance_clipped / 100.0,
                float(state.in_threat_zone),
                state.current_threat_level.value / 3.0,
                state.threat_exposure / state.max_threat_exposure,
                float(state.autonomous_mode),
                state.comms_status.value / 2.0,
                state.gnss_status.value / 2.0,
                float(state.vio_valid),
                float(state.gnss_degradation_triggered),
                state.battery_drain_multiplier,
            ]
        ])

        if self.config.perception.enabled and self.perception is not None and include_perception:
            world_objects = self._get_world_objects_for_perception()
            perception_obs = self.perception.get_observation(
                uav_position=state.position,
                uav_orientation=state.orientation,
                world_objects=world_objects,
            )
            self._perception_obs = perception_obs
            return np.concatenate([state_obs, perception_obs]).astype(np.float32)

        # For state-only obs, pad with zeros to maintain consistent observation dimension
        if self.config.perception.enabled:
            perception_zeros = np.zeros(self.config.perception.observation_dim, dtype=np.float32)
            return np.concatenate([state_obs, perception_zeros]).astype(np.float32)

        return state_obs.astype(np.float32)

    def get_state_only_observation(self, state: UAVState) -> np.ndarray:
        """Get observation without perception (for skipped render frames)."""
        return self.state_to_observation(state, include_perception=False)

    def _get_world_objects_for_perception(self) -> List:
        """Get world objects for ground truth perception."""
        try:
            import sys
            sys.path.insert(0, '/workspace/perception')
            from perception import WorldObject
        except ImportError:
            return []

        world_objects = []
        for target in self.targets:
            world_objects.append(WorldObject(
                id=target.id,
                class_id=4,
                class_name='poi_target',
                position=target.position,
                ontology_class='TargetOfInterest',
                priority=target.priority,
            ))

        return world_objects

    @property
    def observation_dim(self) -> int:
        """Dimension of observation space."""
        base_dim = 3 + 3 + 4 + 3 + 15  # 28
        if self.config.perception.enabled:
            return base_dim + self.config.perception.observation_dim
        return base_dim

    @property
    def action_dim(self) -> int:
        """Dimension of action space."""
        return 4

    def get_camera_image(self) -> Optional[np.ndarray]:
        """Get RGB image from drone camera."""
        if self.perception is not None:
            return self.perception.get_camera_image()
        return None

    def get_perception_stats(self) -> Dict[str, Any]:
        """Get perception statistics."""
        if self.perception is not None:
            return self.perception.get_stats()
        return {"perception_enabled": False}

    def get_safety_stats(self) -> Dict[str, Any]:
        """Get safety filter statistics."""
        total_steps = max(1, self._step_count)
        return {
            'safety_filter_enabled': self._safety_filter_enabled,
            'safety_violations': self._safety_violations_count,
            'safety_interventions': self._safety_interventions_count,
            'intervention_rate': self._safety_interventions_count / total_steps,
        }

    def get_domain_randomization_stats(self) -> Dict[str, Any]:
        """Get domain randomization statistics."""
        wind_magnitude = np.linalg.norm(self._episode_wind_vector)
        wind_direction_deg = np.degrees(np.arctan2(
            self._episode_wind_vector[1], self._episode_wind_vector[0]
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
