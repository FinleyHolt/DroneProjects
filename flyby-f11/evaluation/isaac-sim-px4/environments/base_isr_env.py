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
"""

import sys
import os
import threading
import subprocess
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Callable
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
    geofence: GeofenceConfig = field(default_factory=GeofenceConfig)

    # Coordinates (default: Camp Pendleton)
    latitude: float = 33.3853
    longitude: float = -117.5653
    altitude: float = 100.0

    # UAV
    spawn_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.5]))
    spawn_orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))  # euler

    # Mission
    max_mission_time: float = 900.0  # 15 minutes

    # Domain randomization
    randomize_lighting: bool = True
    randomize_weather: bool = True
    randomize_obstacles: bool = True
    randomize_sensors: bool = True


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

        # Ontology bridge
        self.ontology_bridge = None

        # PX4 command path
        self._px4_cmd = "/px4/build/px4_sitl_default/bin/px4-commander"

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

        # Set global coordinates
        self.pg.set_new_global_coordinates(
            self.config.latitude,
            self.config.longitude,
            self.config.altitude
        )

        # Load environment
        if self.config.environment_usd:
            self.pg.load_environment(self.config.environment_usd)
        else:
            # Use default curved gridroom
            self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        print("  Environment loaded")

        # Spawn vehicle
        print("\n[4/5] Spawning vehicle with PX4...")
        px4_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
        })

        vehicle_config = MultirotorConfig()
        vehicle_config.backends = [PX4MavlinkBackend(px4_config)]

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
        print("  Vehicle spawned")

        # Setup environment-specific elements
        print("\n[5/5] Setting up environment elements...")
        self._setup_environment()
        print("  Environment ready")

        # Reset and start
        self.world.reset()
        self.timeline.play()

        self._initialized = True
        print("\n" + "=" * 60)
        print("Simulation ready!")
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

        # Reset simulation
        self.timeline.stop()
        self.world.reset()
        self.timeline.play()

        # Reset state
        self._step_count = 0
        self.uav_state = UAVState(
            position=self.config.spawn_position.copy(),
            battery_pct=100.0,
        )

        # Wait for initialization
        for _ in range(300):  # ~5 seconds at 60Hz
            self.world.step(render=True)

        self._update_state()
        return self.uav_state

    def close(self) -> None:
        """Cleanup and close the environment."""
        if self._initialized:
            self.timeline.stop()
            self.simulation_app.close()
            self._initialized = False

    def _apply_action(self, action: np.ndarray) -> None:
        """
        Apply action to the vehicle.
        Override in subclass for specific control modes.
        """
        # Default: position setpoint
        # This would interface with PX4 offboard mode
        pass

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
            self.uav_state.battery_reserve = 30.0  # Increased reserve per ontology

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
            Flat numpy array suitable for neural network input
        """
        obs = np.concatenate([
            state.position,
            state.velocity,
            state.orientation,
            [
                state.battery_pct / 100.0,
                state.mission_time / self.config.max_mission_time,
                state.coverage_pct / 100.0,
                float(state.in_geofence),
                float(state.in_nfz),
                state.nfz_distance / 100.0,
                float(state.in_threat_zone),
                state.current_threat_level.value / 3.0,
                state.threat_exposure / state.max_threat_exposure,
                float(state.autonomous_mode),
                state.comms_status.value / 2.0,
                state.gnss_status.value / 2.0,
                float(state.vio_valid),
            ]
        ])
        return obs.astype(np.float32)

    @property
    def observation_dim(self) -> int:
        """Dimension of observation space."""
        return 3 + 3 + 4 + 13  # pos + vel + quat + scalars

    @property
    def action_dim(self) -> int:
        """Dimension of action space."""
        return 4  # x, y, z velocity + yaw rate (or position setpoint)
