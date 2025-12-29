#!/usr/bin/env python3
"""
Basic Isaac Sim + Pegasus test - No flight controller required.
Tests that the simulation environment loads and a vehicle can be spawned.
"""

import argparse
import sys

# Parse args before importing anything else
parser = argparse.ArgumentParser(description="Basic Isaac Sim + Pegasus Test")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
parser.add_argument("--duration", type=int, default=10, help="Simulation duration in seconds")
args = parser.parse_args()

# Start Isaac Sim
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})

print("[TEST] Isaac Sim started successfully")

# Now import the rest
import omni.timeline
from isaacsim.core.api.world import World

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation
import numpy as np

class BasicSimTest:
    """Test that Isaac Sim + Pegasus can spawn a vehicle and run physics."""

    def __init__(self, duration: int = 10):
        self.duration = duration
        self.timeline = omni.timeline.get_timeline_interface()

        # Initialize Pegasus
        print("[TEST] Initializing Pegasus interface...")
        self.pg = PegasusInterface()

        # Explicitly create the World (required for standalone apps)
        print("[TEST] Creating simulation world...")
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load environment
        print("[TEST] Loading environment...")
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Create vehicle WITHOUT flight controller backend
        print("[TEST] Spawning multirotor (no flight controller)...")
        config = MultirotorConfig()
        config.backends = []  # No backends - just physics

        self.vehicle = Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.5],  # Start slightly above ground
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )

        # Reset world to initialize
        print("[TEST] Resetting world...")
        self.world.reset()

        print("[TEST] Setup complete!")

    def run(self):
        """Run the simulation for the specified duration."""
        print(f"[TEST] Starting simulation for {self.duration} seconds...")
        self.timeline.play()

        step_count = 0
        # 60 Hz update rate
        max_steps = self.duration * 60

        while simulation_app.is_running() and step_count < max_steps:
            self.world.step(render=True)
            step_count += 1

            # Report status every second
            if step_count % 60 == 0:
                elapsed = step_count / 60
                # Get vehicle state
                state = self.vehicle.state
                pos = state.position if hasattr(state, 'position') else [0, 0, 0]
                print(f"[TEST] t={elapsed:.1f}s - Vehicle position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

        print(f"[TEST] Simulation completed after {step_count} steps")
        self.timeline.stop()

        return True

    def cleanup(self):
        """Clean up resources."""
        print("[TEST] Shutting down...")
        simulation_app.close()


def main():
    test = None
    try:
        test = BasicSimTest(duration=args.duration)
        success = test.run()

        if success:
            print("\n" + "="*60)
            print("[SUCCESS] Basic Isaac Sim + Pegasus test PASSED!")
            print("  - Isaac Sim initialized correctly")
            print("  - Pegasus extension loaded")
            print("  - Environment loaded")
            print("  - Vehicle spawned and physics simulated")
            print("="*60)
            return 0
        else:
            print("[FAILED] Test did not complete")
            return 1

    except Exception as e:
        print(f"[ERROR] Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if test:
            test.cleanup()


if __name__ == "__main__":
    sys.exit(main())
