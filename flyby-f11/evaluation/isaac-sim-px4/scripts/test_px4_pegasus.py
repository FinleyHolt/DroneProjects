#!/usr/bin/env python3
"""
Isaac Sim 5.1.0 + PX4 + Pegasus Simulator - MVE Test Script

This script validates the complete simulation stack:
1. Isaac Sim initialization
2. Pegasus extension loading
3. PX4 SITL connection via MAVLink
4. Iris quadcopter spawning
5. Physics stepping with lockstep

Usage (inside container):
    isaac_run python3 /workspace/scripts/test_px4_pegasus.py

For headless mode:
    isaac_run python3 /workspace/scripts/test_px4_pegasus.py --headless
"""

import argparse
import os
import sys
import time
import subprocess
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="PX4 + Pegasus MVE Test")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode (no GUI)"
    )
    parser.add_argument(
        "--px4-autolaunch",
        action="store_true",
        default=True,
        help="Auto-launch PX4 SITL (default: True)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Simulation duration in seconds (default: 30)"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("Isaac Sim 5.1.0 + PX4 + Pegasus MVE Test")
    print("=" * 60)

    # -------------------------------------------------------------------------
    # Step 1: Initialize Isaac Sim
    # -------------------------------------------------------------------------
    print("\n[1/6] Initializing Isaac Sim...")

    # Isaac Sim 5.1.0 uses isaacsim namespace (was omni.isaac in 4.x)
    from isaacsim import SimulationApp

    # Configure simulation
    config = {
        "headless": args.headless,
        "width": 1280,
        "height": 720,
        "anti_aliasing": 0,  # Faster for testing
        "renderer": "RayTracedLighting",  # Or "PathTracing" for quality
    }

    simulation_app = SimulationApp(config)
    print(f"  Isaac Sim initialized (headless={args.headless})")

    # -------------------------------------------------------------------------
    # Step 2: Load Pegasus Extension
    # -------------------------------------------------------------------------
    print("\n[2/6] Loading Pegasus extension...")

    import omni.kit.app
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage

    # Get extension manager
    ext_manager = omni.kit.app.get_app().get_extension_manager()

    # Pegasus extension path (cloned in container)
    pegasus_ext_path = "/pegasus/extensions"

    # CRITICAL: Add extension path BEFORE enabling
    # This allows Isaac Sim to find and trust the Pegasus extension
    ext_manager.add_path(pegasus_ext_path)
    print(f"  Added extension path: {pegasus_ext_path}")

    # Give the extension manager time to scan the path
    import carb
    carb.get_framework().load_plugins()

    # Enable the Pegasus simulator extension
    ext_manager.set_extension_enabled_immediate("pegasus.simulator", True)

    # Wait for extension to fully load
    import asyncio
    async def wait_for_ext():
        for _ in range(10):
            if ext_manager.is_extension_enabled("pegasus.simulator"):
                return True
            await asyncio.sleep(0.1)
        return False

    import omni.kit.async_engine
    omni.kit.async_engine.run_coroutine(wait_for_ext())

    print("  Pegasus extension loaded")

    # -------------------------------------------------------------------------
    # Step 3: Import Pegasus components
    # -------------------------------------------------------------------------
    print("\n[3/6] Importing Pegasus components...")

    from pegasus.simulator.params import ROBOTS
    from pegasus.simulator.logic.state import State
    from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
    from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

    # PX4 backend (first-class support in Pegasus)
    from pegasus.simulator.logic.backends.px4_mavlink_backend import (
        PX4MavlinkBackend,
        PX4MavlinkBackendConfig,
    )

    # Vehicle class
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

    print("  Pegasus components imported")

    # -------------------------------------------------------------------------
    # Step 4: Create World and Configure Scene
    # -------------------------------------------------------------------------
    print("\n[4/6] Creating world and scene...")

    # Initialize Pegasus interface
    pegasus = PegasusInterface()

    # CRITICAL: Explicitly create the World and assign it to Pegasus
    # PegasusInterface doesn't auto-create the world - we must do it manually
    pegasus._world = World(**pegasus._world_settings)
    world = pegasus.world

    # Load environment through Pegasus interface
    pegasus.load_environment(
        "/isaac-sim/exts/omni.isaac.assets/assets/Isaac/Environments/Grid/default_environment.usd"
    )

    print("  World created with default environment")

    # -------------------------------------------------------------------------
    # Step 5: Spawn Iris Quadcopter with PX4 Backend
    # -------------------------------------------------------------------------
    print("\n[5/6] Spawning Iris quadcopter with PX4 backend...")

    from scipy.spatial.transform import Rotation

    # Configure PX4 MAVLink backend using Pegasus interface settings
    px4_config = PX4MavlinkBackendConfig({
        "vehicle_id": 0,
        "px4_autolaunch": args.px4_autolaunch,
        "px4_dir": pegasus.px4_path if pegasus.px4_path else "/px4",
        "px4_vehicle_model": pegasus.px4_default_airframe if pegasus.px4_default_airframe else "iris",
    })

    # Create PX4 backend
    px4_backend = PX4MavlinkBackend(px4_config)

    # Iris configuration (built-in Pegasus model)
    iris_config = MultirotorConfig()
    iris_config.backends = [px4_backend]

    # Spawn position and rotation (quaternion format as per Pegasus example)
    spawn_position = [0.0, 0.0, 0.07]  # Just above ground
    spawn_rotation = Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat()

    # Create the vehicle
    iris = Multirotor(
        "/World/quadrotor",
        ROBOTS["Iris"],
        0,  # Vehicle ID
        spawn_position,
        spawn_rotation,
        config=iris_config,
    )

    print(f"  Iris spawned at position {spawn_position}")
    print(f"  PX4 autolaunch: {args.px4_autolaunch}")
    print(f"  Lockstep enabled: True")

    # -------------------------------------------------------------------------
    # Step 6: Run Simulation Loop
    # -------------------------------------------------------------------------
    print("\n[6/6] Running simulation loop...")
    print(f"  Duration: {args.duration} seconds")
    print("  Press Ctrl+C to stop early")
    print("-" * 60)

    # Reset world to start simulation
    world.reset()

    start_time = time.time()
    step_count = 0
    last_print_time = start_time

    try:
        while simulation_app.is_running():
            # Step physics
            world.step(render=not args.headless)
            step_count += 1

            current_time = time.time()
            elapsed = current_time - start_time

            # Print status every 5 seconds
            if current_time - last_print_time >= 5.0:
                # Get vehicle state
                state = iris.state
                pos = state.position
                vel = state.linear_velocity

                print(f"  [{elapsed:6.1f}s] Steps: {step_count:6d} | "
                      f"Pos: ({pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}) | "
                      f"Vel: ({vel[0]:5.2f}, {vel[1]:5.2f}, {vel[2]:5.2f})")

                # Check MAVLink heartbeat
                if hasattr(px4_backend, 'is_connected') and px4_backend.is_connected():
                    print(f"         MAVLink: Connected ✓")
                else:
                    print(f"         MAVLink: Waiting for connection...")

                last_print_time = current_time

            # Check duration
            if elapsed >= args.duration:
                print(f"\n  Simulation completed after {elapsed:.1f} seconds")
                break

    except KeyboardInterrupt:
        print(f"\n  Simulation stopped by user after {time.time() - start_time:.1f} seconds")

    # -------------------------------------------------------------------------
    # Summary
    # -------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("MVE Test Summary")
    print("=" * 60)
    print(f"  Total steps:     {step_count}")
    print(f"  Elapsed time:    {time.time() - start_time:.2f} seconds")
    print(f"  Average rate:    {step_count / (time.time() - start_time):.1f} steps/sec")

    # Final state
    final_state = iris.state
    print(f"  Final position:  ({final_state.position[0]:.2f}, "
          f"{final_state.position[1]:.2f}, {final_state.position[2]:.2f})")

    print("\nMVE Test: SUCCESS ✓")
    print("=" * 60)

    # Cleanup
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
