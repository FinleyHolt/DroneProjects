#!/usr/bin/env python3
"""
Test script for ArduPilot + Pegasus Simulator integration in Isaac Sim.

This script spawns an Iris quadcopter in the Lisbon world and connects it
to ArduPilot SITL via the JSON MAVLink backend.

Usage:
    # Start ArduPilot SITL in a separate terminal/container first:
    # podman-compose --profile sitl up ardupilot-sitl

    # Then run this script inside Isaac Sim:
    # /isaac-sim/python.sh /workspace/scripts/test_ardupilot_pegasus.py

    # For headless mode (CI/testing):
    # HEADLESS=1 /isaac-sim/python.sh /workspace/scripts/test_ardupilot_pegasus.py

Author: Finley Holt
"""

# Add Pegasus extension to path BEFORE starting SimulationApp
import sys
import os
sys.path.insert(0, "/workspace/PegasusSimulator/extensions/pegasus.simulator")

print("=" * 60, flush=True)
print("ArduPilot + Pegasus Test Script Starting...", flush=True)
print("=" * 60, flush=True)
print(f"Python path: {sys.path[:3]}", flush=True)
print(f"HEADLESS env: {os.environ.get('HEADLESS', 'not set')}", flush=True)

# Check if we should run headless
headless = os.environ.get("HEADLESS", "0") == "1"
print(f"Running in {'HEADLESS' if headless else 'GUI'} mode", flush=True)

import carb
from isaacsim import SimulationApp

# Start Isaac Sim (must be first!)
print("Creating SimulationApp...", flush=True)
simulation_app = SimulationApp({"headless": headless})
print("SimulationApp created successfully!", flush=True)

# Now we can import the rest
print("Importing Isaac Sim modules...", flush=True)
import omni.timeline
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation
print("Isaac Sim modules imported successfully!", flush=True)

# Pegasus imports
print("Importing Pegasus modules...", flush=True)
try:
    from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
    print(f"  - Available robots: {list(ROBOTS.keys())}", flush=True)
    print(f"  - Available environments: {list(SIMULATION_ENVIRONMENTS.keys())}", flush=True)
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
    from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (
        ArduPilotMavlinkBackend,
        ArduPilotMavlinkBackendConfig
    )
    print("Pegasus modules imported successfully!", flush=True)
except ImportError as e:
    print(f"ERROR: Failed to import Pegasus modules: {e}", flush=True)
    print("Make sure PegasusSimulator is in /workspace/PegasusSimulator", flush=True)
    simulation_app.close()
    sys.exit(1)


class ArduPilotTestApp:
    """
    Test application for ArduPilot + Pegasus integration.
    """

    def __init__(self):
        """Initialize the simulation environment."""

        # Get the timeline interface
        self.timeline = omni.timeline.get_timeline_interface()

        # Initialize Pegasus interface
        self.pg = PegasusInterface()

        # Create the World
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load a test environment (Curved Gridroom - good for indoor testing)
        carb.log_info("Loading Curved Gridroom environment...")
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Configure ArduPilot MAVLink backend
        # This connects to ArduPilot SITL via UDP JSON interface
        ardupilot_config = ArduPilotMavlinkBackendConfig({
            "vehicle_id": 0,
            "connection_type": "udpin",
            "connection_ip": "127.0.0.1",
            "connection_baseport": 14550,
            # Disable auto-launch - we run SITL externally in a container
            "ardupilot_autolaunch": False,
            # Enable lockstep for synchronized simulation
            "enable_lockstep": True,
            "num_rotors": 4,
            "update_rate": 400.0,
        })

        # Configure the multirotor
        config = MultirotorConfig()
        config.backends = [ArduPilotMavlinkBackend(ardupilot_config)]

        # Spawn the Iris quadcopter
        carb.log_info("Spawning Iris quadcopter...")
        initial_position = [0.0, 0.0, 0.5]  # 50cm above ground
        initial_orientation = Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat()

        self.vehicle = Multirotor(
            "/World/Iris",
            ROBOTS['Iris'],
            0,
            initial_position,
            initial_orientation,
            config=config,
        )

        # Reset the world to initialize physics
        carb.log_info("Initializing physics...")
        self.world.reset()

        carb.log_info("ArduPilot test environment ready!")
        carb.log_info("Connect MAVProxy or QGroundControl to UDP:127.0.0.1:14550")

    def run(self):
        """Main simulation loop."""

        carb.log_info("Starting simulation...")
        self.timeline.play()

        frame_count = 0

        while simulation_app.is_running():
            # Step the simulation
            self.world.step(render=True)

            frame_count += 1

            # Log status every 250 frames (~1 second at 250Hz)
            if frame_count % 250 == 0:
                carb.log_info(f"Simulation running... frame {frame_count}")

        # Cleanup
        carb.log_warn("Shutting down simulation...")
        self.timeline.stop()
        simulation_app.close()


def main():
    """Entry point."""
    carb.log_info("=" * 50)
    carb.log_info("ArduPilot + Pegasus Simulator Test")
    carb.log_info("=" * 50)

    app = ArduPilotTestApp()
    app.run()


if __name__ == "__main__":
    main()
