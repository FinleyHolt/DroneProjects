"""
Simulation Core - Isaac Sim + Pegasus + PX4 initialization.

Single responsibility: Initialize and manage the simulation stack.
"""

import sys
import threading
import time
from typing import Optional

import numpy as np

# Add Pegasus to Python path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)


class SimulationCore:
    """
    Manages Isaac Sim, Pegasus, and PX4 SITL initialization.

    Responsibilities:
    - Isaac Sim application lifecycle
    - Pegasus interface setup
    - PX4 SITL launch with lockstep
    - Vehicle spawning
    - MAVLink connection management
    """

    def __init__(self, headless: bool = False):
        """
        Initialize SimulationCore.

        Args:
            headless: Run without rendering
        """
        self.headless = headless
        self._initialized = False

        # Simulation objects
        self.simulation_app = None
        self.world = None
        self.timeline = None
        self.pg = None
        self.vehicle = None

        # PX4 objects
        self._px4_backend = None
        self._px4_tool = None
        self._mav_connection = None

    def initialize(
        self,
        latitude: float = 33.3853,
        longitude: float = -117.5653,
        altitude: float = 100.0,
        spawn_position: np.ndarray = None,
        spawn_orientation: np.ndarray = None,
        environment_usd: Optional[str] = None,
        skip_default_environment: bool = False,
    ) -> None:
        """
        Initialize the full simulation stack.

        Args:
            latitude: Global latitude
            longitude: Global longitude
            altitude: Global altitude
            spawn_position: Vehicle spawn position
            spawn_orientation: Vehicle spawn euler angles (degrees)
            environment_usd: Custom USD path
            skip_default_environment: Skip loading default gridroom
        """
        if self._initialized:
            return

        spawn_position = spawn_position if spawn_position is not None else np.array([0.0, 0.0, 0.5])
        spawn_orientation = spawn_orientation if spawn_orientation is not None else np.array([0.0, 0.0, 0.0])

        print("=" * 60)
        print("Initializing Simulation Core")
        print("=" * 60)

        # Phase 1: Isaac Sim
        print("\n[1/7] Initializing Isaac Sim...")
        self._init_isaac_sim()

        # Phase 2: Pegasus
        print("\n[2/7] Loading Pegasus extension...")
        self._init_pegasus(latitude, longitude, altitude, environment_usd, skip_default_environment)

        # Phase 3: Vehicle
        print("\n[3/7] Spawning vehicle...")
        self._spawn_vehicle(spawn_position, spawn_orientation)

        # Phase 4: Timeline
        print("\n[4/7] Starting timeline and physics...")
        self._start_timeline()

        # Phase 5: PX4
        print("\n[5/7] Launching PX4 SITL...")
        self._launch_px4()

        # Phase 6: MAVLink
        print("\n[6/7] Connecting MAVLink...")
        self._connect_mavlink()

        # Phase 7: Ready check
        print("\n[7/7] Waiting for simulation ready...")
        ready = self._wait_for_ready()

        self._initialized = True
        print("\n" + "=" * 60)
        print("Simulation Core ready!" if ready else "Simulation Core ready (with warnings)")
        print("=" * 60)

    def _init_isaac_sim(self) -> None:
        """Initialize Isaac Sim application."""
        from isaacsim import SimulationApp
        self.simulation_app = SimulationApp({"headless": self.headless})
        print("  Isaac Sim ready")

    def _init_pegasus(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
        environment_usd: Optional[str],
        skip_default_environment: bool,
    ) -> None:
        """Initialize Pegasus interface and world."""
        import omni.timeline
        from isaacsim.core.api.world import World
        from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Set PX4 path
        self.pg.set_px4_path("/px4")

        # Set global coordinates
        self.pg.set_global_coordinates(latitude, longitude, altitude)

        # Load environment
        if environment_usd:
            self.pg.load_environment(environment_usd)
        elif not skip_default_environment:
            self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        print("  Pegasus loaded")

    def _spawn_vehicle(
        self,
        spawn_position: np.ndarray,
        spawn_orientation: np.ndarray,
    ) -> None:
        """Spawn the UAV with PX4 backend."""
        from pegasus.simulator.params import ROBOTS
        from pegasus.simulator.logic.backends.px4_mavlink_backend import (
            PX4MavlinkBackend, PX4MavlinkBackendConfig
        )
        from pegasus.simulator.logic.vehicles.multirotor import (
            Multirotor, MultirotorConfig
        )
        from scipy.spatial.transform import Rotation

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
            "XYZ", spawn_orientation.tolist(), degrees=True
        ).as_quat()

        self.vehicle = Multirotor(
            "/World/uav",
            ROBOTS["Iris"],
            0,
            spawn_position.tolist(),
            spawn_rot,
            config=vehicle_config,
        )
        print("  Vehicle spawned")

    def _start_timeline(self) -> None:
        """Start timeline and physics."""
        self.world.reset()
        self.timeline.play()

        # Warmup steps
        for _ in range(50):
            self.world.step(render=True)

        print("  Timeline started")

    def _launch_px4(self) -> None:
        """Launch PX4 SITL with continuous stepping."""
        from pegasus.simulator.logic.backends.tools.px4_launch_tool import PX4LaunchTool

        # Flag for stepping thread
        self._px4_stepping_active = True

        def step_continuously():
            while self._px4_stepping_active:
                try:
                    self.world.step(render=True)
                except Exception as e:
                    print(f"  [Step thread] Error: {e}")
                    break

        # Start stepping before PX4 launch
        step_thread = threading.Thread(target=step_continuously, daemon=True)
        step_thread.start()

        # Launch PX4
        self._px4_tool = PX4LaunchTool(
            self.pg.px4_path,
            vehicle_id=0,
            px4_model=self.pg.px4_default_airframe
        )
        self._px4_tool.launch_px4()
        self._px4_backend.px4_tool = self._px4_tool
        self._px4_backend.px4_autolaunch = True

        # Wait for PX4 boot
        time.sleep(2.0)

        # Stop stepping thread
        self._px4_stepping_active = False
        step_thread.join(timeout=1.0)

        # Additional warmup
        for _ in range(300):
            self.world.step(render=True)

        print("  PX4 SITL launched")

    def _connect_mavlink(self) -> None:
        """Connect to PX4 via MAVLink."""
        from pymavlink import mavutil

        try:
            self._mav_connection = mavutil.mavlink_connection(
                'udpin:localhost:14550',
                source_system=255
            )

            # Wait for heartbeat
            heartbeat_received = False
            for i in range(500):
                self.world.step(render=True)
                msg = self._mav_connection.recv_match(type='HEARTBEAT', blocking=False)
                if msg and msg.get_srcSystem() != 0:
                    heartbeat_received = True
                    print(f"  Got heartbeat from system {msg.get_srcSystem()}")
                    break

            if not heartbeat_received:
                print("  Warning: No heartbeat received")
                self._mav_connection = None
            else:
                if self._mav_connection.target_system == 0:
                    self._mav_connection.target_system = 1
                    self._mav_connection.target_component = 1

        except Exception as e:
            print(f"  Warning: MAVLink connection failed: {e}")
            self._mav_connection = None

    def _wait_for_ready(self, max_steps: int = 2000) -> bool:
        """Wait for simulation to be fully ready."""
        if self._mav_connection is None:
            return False

        from pymavlink import mavutil

        # Phase 1: Stable heartbeat
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

        # Phase 2: EKF health
        health_ok_count = 0
        for i in range(max_steps // 2):
            self.world.step(render=True)
            msg = self._mav_connection.recv_match(type='SYS_STATUS', blocking=False)
            if msg:
                health_ok_count += 1
                if health_ok_count >= 10:
                    break

        # Phase 3: Vehicle stability
        initial_z = self.vehicle.state.position[2] if hasattr(self.vehicle, 'state') else 0
        for _ in range(100):
            self.world.step(render=True)
        final_z = self.vehicle.state.position[2] if hasattr(self.vehicle, 'state') else 0

        if final_z - initial_z < -1.0:
            return False

        print("  Simulation ready")
        return True

    def step(self, render: bool = True) -> None:
        """Step the simulation."""
        self.world.step(render=render)

    def close(self) -> None:
        """Cleanup simulation."""
        if self._initialized:
            self.timeline.stop()
            self.simulation_app.close()
            self._initialized = False

    @property
    def mav_connection(self):
        """MAVLink connection."""
        return self._mav_connection

    @property
    def is_initialized(self) -> bool:
        """Whether simulation is initialized."""
        return self._initialized
