#!/usr/bin/env python3
"""
Spawn F-11 drone in Isaac Sim via Pegasus Simulator API.

This script demonstrates the minimal viable example of:
1. Connecting to Isaac Sim
2. Loading an outdoor world
3. Spawning a quadcopter with camera
4. Configuring ArduPilot SITL backend

Usage:
    python3 spawn_drone.py

Prerequisites:
    - Isaac Sim running (headless or GUI)
    - Pegasus extension loaded
    - ArduPilot SITL running (optional, can start automatically)
"""

import asyncio
import sys
from pathlib import Path

# Pegasus imports (available when running inside Isaac Sim)
try:
    from pegasus.simulator.params import ROBOTS
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
    from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
    from pegasus.simulator.logic.graphs import ROS2Camera
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
except ImportError:
    print("Error: Pegasus Simulator not found.")
    print("This script must be run from within Isaac Sim with Pegasus extension loaded.")
    print("")
    print("Try: ./isaac-sim.sh --enable pegasus.simulator scripts/spawn_drone.py")
    sys.exit(1)

# Isaac Sim imports
from omni.isaac.kit import SimulationApp

# Configuration
WORLD_USD_PATH = "/workspace/worlds/outdoor_training.usd"
DRONE_USD_PATH = "/workspace/models/f11_isr/f11_isr.usd"
SPAWN_POSITION = [0.0, 0.0, 0.5]  # x, y, z in meters
SPAWN_ROTATION = [0.0, 0.0, 0.0, 1.0]  # quaternion (x, y, z, w)


class F11DroneConfig:
    """Configuration for the Flyby F-11 ISR drone."""

    def __init__(self):
        # Use default quadrotor if custom model not available
        self.usd_file = DRONE_USD_PATH if Path(DRONE_USD_PATH).exists() else ROBOTS["Iris"]

        # Vehicle configuration
        self.vehicle_name = "f11_isr"
        self.position = SPAWN_POSITION
        self.rotation = SPAWN_ROTATION

        # ArduPilot backend configuration
        self.ardupilot_config = MavlinkBackendConfig({
            "vehicle_id": 0,
            "connection_type": "udpin",
            "connection_ip": "127.0.0.1",
            "connection_port": 5760,
            "enable_lockstep": True,
            "num_rotors": 4,
        })

        # Camera configuration
        self.camera_config = {
            "camera_prim_path": f"/World/{self.vehicle_name}/camera_link/isr_camera",
            "ros2_topic": "/f11/camera/image_raw",
            "resolution": (1280, 720),
            "update_rate": 30.0,
        }


async def main():
    """Main entry point for drone spawning."""

    print("=" * 50)
    print("  Flyby F-11 Drone Spawn Script")
    print("=" * 50)
    print("")

    # Initialize Pegasus interface
    print("[1/4] Initializing Pegasus interface...")
    pegasus = PegasusInterface()

    # Load world
    print("[2/4] Loading outdoor training world...")
    if Path(WORLD_USD_PATH).exists():
        pegasus.load_environment(WORLD_USD_PATH)
    else:
        print(f"  Warning: {WORLD_USD_PATH} not found, using default environment")
        # Load a default Pegasus environment
        pegasus.load_environment_by_name("Default Environment")

    # Configure drone
    print("[3/4] Configuring F-11 drone...")
    config = F11DroneConfig()

    # Create multirotor vehicle
    multirotor_config = MultirotorConfig()

    # Add ArduPilot MAVLink backend
    mavlink_backend = MavlinkBackend(config.ardupilot_config)

    # Create the drone
    drone = Multirotor(
        "/World/" + config.vehicle_name,
        config.usd_file,
        config.vehicle_name,
        config.position,
        config.rotation,
        multirotor_config,
        backends=[mavlink_backend],
    )

    # Add camera sensor with ROS2 publisher
    print("[4/4] Configuring camera sensor...")
    camera = ROS2Camera(
        camera_prim_path=config.camera_config["camera_prim_path"],
        topic_name=config.camera_config["ros2_topic"],
        resolution=config.camera_config["resolution"],
    )

    print("")
    print("=" * 50)
    print("  Drone spawned successfully!")
    print("=" * 50)
    print(f"  Vehicle: {config.vehicle_name}")
    print(f"  Position: {config.position}")
    print(f"  Camera topic: {config.camera_config['ros2_topic']}")
    print(f"  MAVLink: udp://127.0.0.1:5760")
    print("")
    print("Connect with: mavproxy.py --master=udp:127.0.0.1:14550")
    print("")

    # Start simulation
    pegasus.run()


if __name__ == "__main__":
    asyncio.run(main())
