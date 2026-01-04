#!/usr/bin/env python3
"""
PX4 Simulation Core - Minimal, working PX4+Isaac Sim integration.

This module provides a clean, simple interface for PX4 SITL with Isaac Sim.
It follows the EXACT pattern from the working test script (test_takeoff3.py).

Design principles:
1. Do ONE thing well: Get PX4 flying in Isaac Sim
2. Linear initialization - no complex state machines
3. Step the world continuously - never leave gaps
4. Everything else (perception, safety, RL) layers on top

Usage:
    core = PX4SimulationCore(headless=False)
    core.setup()

    # Fly!
    for _ in range(1000):
        core.send_velocity(0, 0, -1.0, 0)  # Ascend
        core.step()

    core.close()
"""

import sys
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass

PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)


@dataclass
class VehicleState:
    """Simple vehicle state."""
    position: np.ndarray  # [x, y, z] in meters
    velocity: np.ndarray  # [vx, vy, vz] in m/s
    attitude: np.ndarray  # quaternion [qx, qy, qz, qw]
    armed: bool
    mode: int  # PX4 custom mode (6 = OFFBOARD)


class PX4SimulationCore:
    """
    Minimal PX4 + Isaac Sim integration.

    Provides:
    - Vehicle spawn with PX4 SITL
    - MAVLink connection
    - OFFBOARD mode engagement
    - Velocity commands
    - State reading

    Does NOT provide (these layer on top):
    - Perception / YOLO
    - Safety filter / Vampire
    - RL environment interface
    - Domain randomization
    """

    def __init__(
        self,
        headless: bool = True,
        px4_path: str = "/px4",
        spawn_position: Tuple[float, float, float] = (0.0, 0.0, 0.07),
    ):
        self.headless = headless
        self.px4_path = px4_path
        self.spawn_position = list(spawn_position)

        # Will be set during setup
        self.simulation_app = None
        self.world = None
        self.timeline = None
        self.vehicle = None
        self.mav = None

        self._initialized = False
        self._offboard_engaged = False

    def setup(self) -> None:
        """
        Initialize simulation - follows test_takeoff3.py pattern EXACTLY.
        """
        if self._initialized:
            return

        print("=" * 60)
        print("PX4SimulationCore - Initializing")
        print("=" * 60)

        # 1. Start Isaac Sim
        print("\n[1/5] Starting Isaac Sim...")
        from isaacsim import SimulationApp
        self.simulation_app = SimulationApp({"headless": self.headless})

        # 2. Import Pegasus (after Isaac Sim init)
        print("[2/5] Loading Pegasus...")
        import omni.timeline
        from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
        from pegasus.simulator.logic.backends.px4_mavlink_backend import (
            PX4MavlinkBackend, PX4MavlinkBackendConfig
        )
        from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
        from scipy.spatial.transform import Rotation

        # 3. Create world
        print("[3/5] Creating world...")
        self.timeline = omni.timeline.get_timeline_interface()
        pg = PegasusInterface()
        from omni.isaac.core.world import World
        pg._world = World(**pg._world_settings)
        self.world = pg.world
        pg.set_px4_path(self.px4_path)
        pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # 4. Spawn vehicle with PX4 backend
        print("[4/5] Spawning vehicle with PX4...")
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": pg.px4_path,
            "px4_vehicle_model": pg.px4_default_airframe,
            "enable_lockstep": True,
            "update_rate": 250.0,
        })
        config = MultirotorConfig()
        config.backends = [PX4MavlinkBackend(mavlink_config)]

        self.vehicle = Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            self.spawn_position,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )

        # 5. Start simulation - CRITICAL: reset, play, step immediately
        print("[5/5] Starting simulation...")
        self.world.reset()
        self.timeline.play()

        # Step immediately - no delays!
        for i in range(200):
            self.world.step(render=True)

        print(f"  Initial altitude: {self.vehicle.state.position[2]:.3f}m")

        # Connect MAVLink
        self._connect_mavlink()

        self._initialized = True
        print("\n" + "=" * 60)
        print("PX4SimulationCore - Ready")
        print("=" * 60)

    def _connect_mavlink(self) -> None:
        """Connect to PX4 via MAVLink."""
        from pymavlink import mavutil

        print("  Connecting MAVLink...")
        self.mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

        # Wait for heartbeat while stepping
        for i in range(500):
            self.world.step(render=True)
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.get_srcSystem() != 0:
                print(f"  Got heartbeat after {i} steps")
                break

        if self.mav.target_system == 0:
            self.mav.target_system = 1
            self.mav.target_component = 1

        print(f"  MAVLink connected to system {self.mav.target_system}")

    def step(self) -> None:
        """Step the simulation one physics tick."""
        if not self._initialized:
            raise RuntimeError("Call setup() first")
        self.world.step(render=True)

    def get_state(self) -> VehicleState:
        """Get current vehicle state."""
        if not self._initialized:
            raise RuntimeError("Call setup() first")

        state = self.vehicle.state
        armed, mode = self._check_mode()

        return VehicleState(
            position=np.array(state.position),
            velocity=np.array(state.linear_velocity),
            attitude=np.array(state.attitude),
            armed=armed if armed is not None else False,
            mode=mode if mode is not None else 0,
        )

    def _check_mode(self) -> Tuple[Optional[bool], Optional[int]]:
        """Check PX4 armed state and mode from heartbeat."""
        from pymavlink import mavutil
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.get_srcSystem() != 0:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            return armed, msg.custom_mode
        return None, None

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """
        Send velocity command to PX4.

        Args:
            vx: Forward velocity (m/s, body frame)
            vy: Right velocity (m/s, body frame)
            vz: Vertical velocity (m/s, NED: negative = up)
            yaw_rate: Yaw rate (rad/s)
        """
        if not self._initialized:
            raise RuntimeError("Call setup() first")

        # Engage OFFBOARD on first command
        if not self._offboard_engaged:
            self._engage_offboard()

        from pymavlink import mavutil
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # velocity only
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate
        )

    def hover(self) -> None:
        """Command hover (zero velocity)."""
        self.send_velocity(0, 0, 0, 0)

    def _engage_offboard(self) -> bool:
        """Engage OFFBOARD mode and arm - follows test script pattern."""
        from pymavlink import mavutil

        print("  Engaging OFFBOARD mode...")

        # Prime with setpoints
        for _ in range(200):
            self.hover()
            self.step()

        # Request OFFBOARD (mode 6)
        for attempt in range(3):
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, 0, 0, 0, 0, 0
            )
            for _ in range(100):
                self.hover()
                self.step()

            armed, mode = self._check_mode()
            if mode == 6:
                break

        # Arm
        print("  Arming...")
        for attempt in range(3):
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0
            )
            for _ in range(100):
                self.hover()
                self.step()

            armed, mode = self._check_mode()
            if armed:
                break

        # Ensure OFFBOARD after arming
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6, 0, 0, 0, 0, 0
        )
        for _ in range(50):
            self.hover()
            self.step()

        armed, mode = self._check_mode()
        self._offboard_engaged = True

        if armed and mode == 6:
            print(f"  OFFBOARD engaged, vehicle armed!")
            return True
        else:
            print(f"  Warning: armed={armed}, mode={mode}")
            return False

    def close(self) -> None:
        """Shutdown simulation."""
        if self._initialized:
            self.timeline.stop()
            self.simulation_app.close()
            self._initialized = False


# Simple test
if __name__ == "__main__":
    core = PX4SimulationCore(headless=False)
    core.setup()

    print("\nFlying up for 5 seconds...")
    for i in range(300):  # ~5 seconds at 60Hz
        core.send_velocity(0, 0, -2.0, 0)  # Ascend at 2 m/s
        core.step()

        if i % 60 == 0:
            state = core.get_state()
            print(f"  Step {i}: alt={state.position[2]:.2f}m, armed={state.armed}")

    state = core.get_state()
    print(f"\nFinal altitude: {state.position[2]:.2f}m")

    if state.position[2] > 1.0:
        print("SUCCESS!")
    else:
        print("FAILED")

    core.close()
