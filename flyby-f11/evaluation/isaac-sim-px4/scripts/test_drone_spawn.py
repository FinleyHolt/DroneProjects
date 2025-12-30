#!/usr/bin/env python3
"""
Test script for F-11 drone spawning in Isaac Sim.

This script demonstrates:
1. Spawning a single F-11 ISR drone
2. Spawning multiple drones in formation
3. Basic physics simulation with PX4 integration

Usage:
    # From Isaac Sim container
    /isaac-sim/python.sh scripts/test_drone_spawn.py

    # With specific variant
    /isaac-sim/python.sh scripts/test_drone_spawn.py --variant lidar

    # Multi-drone formation
    /isaac-sim/python.sh scripts/test_drone_spawn.py --count 4 --formation wedge
"""

import argparse
import sys
import time
import numpy as np

# Isaac Sim imports
from isaacsim import SimulationApp

# Parse args before SimulationApp initialization
parser = argparse.ArgumentParser(description="F-11 Drone Spawn Test")
parser.add_argument(
    "--variant",
    type=str,
    default="isr_camera",
    choices=["base", "isr_camera", "lidar", "multispectral"],
    help="Drone variant to spawn",
)
parser.add_argument(
    "--count",
    type=int,
    default=1,
    help="Number of drones to spawn (default: 1)",
)
parser.add_argument(
    "--formation",
    type=str,
    default="line",
    choices=["line", "wedge", "box", "circle"],
    help="Formation type for multi-drone spawn",
)
parser.add_argument(
    "--altitude",
    type=float,
    default=10.0,
    help="Spawn altitude in meters",
)
parser.add_argument(
    "--headless",
    action="store_true",
    help="Run in headless mode (no GUI)",
)
parser.add_argument(
    "--px4",
    action="store_true",
    help="Enable PX4 SITL connection",
)
parser.add_argument(
    "--duration",
    type=float,
    default=30.0,
    help="Simulation duration in seconds (0 for infinite)",
)

args = parser.parse_args()

# Launch Isaac Sim
config = {
    "headless": args.headless,
    "renderer": "RayTracedLighting" if not args.headless else "PathTracing",
    "width": 1920,
    "height": 1080,
}
simulation_app = SimulationApp(config)

# Now import Isaac Sim modules
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_current_stage
import omni.isaac.core.utils.prims as prim_utils

# Import our extension modules
sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")
from flyby.world_generator import WorldGenerator, WorldConfig
from flyby.world_generator.spawners import DroneSpawner, DroneConfig, F11_VARIANTS
from flyby.world_generator.drones import PX4Bridge, ThrustModel, QuadrotorDynamics


def setup_world():
    """Setup Isaac Sim world with physics."""
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Set physics timestep
    world.set_simulation_dt(physics_dt=1.0/250.0, rendering_dt=1.0/60.0)

    return world


def spawn_drone_standalone(variant: str, position: tuple, heading: float = 0.0) -> str:
    """
    Spawn a single drone without WorldGenerator.

    Useful for isolated testing.
    """
    stage = get_current_stage()
    spawn_config = SpawnConfig(area_size=(200.0, 200.0))

    spawner = DroneSpawner(stage, config=spawn_config)
    drone_path = spawner.spawn(variant=variant, position=position, heading=heading)

    print(f"Spawned drone at {drone_path}")
    print(f"  Variant: {variant}")
    print(f"  Position: {position}")

    return drone_path


def spawn_with_world_generator(
    variant: str,
    count: int,
    formation: str,
    altitude: float,
) -> list:
    """
    Spawn drones using WorldGenerator (recommended approach).
    """
    # Create world generator
    config = WorldConfig(
        terrain_size=(200.0, 200.0),
        terrain_roughness=2.0,
        tree_density=0.5,  # Light vegetation for visibility
    )

    world_gen = WorldGenerator(
        models_path="/workspace/extensions/forest_generator/models",
        config=config,
    )

    # Generate basic environment
    print("Generating terrain...")
    world_gen.generate_terrain()

    print("Setting up lighting...")
    world_gen.setup_lighting()

    print("Generating sparse forest...")
    world_gen.generate_forest()

    # Spawn drones
    print(f"\nSpawning {count} F-11 {variant} drone(s) in {formation} formation...")

    if count == 1:
        # Single drone at origin
        drone_path = world_gen.spawn_drone(
            variant=variant,
            position=(0, 0, altitude),
            heading=0.0,
        )
        drone_paths = [drone_path]
    else:
        # Multi-drone formation
        drone_paths = world_gen.spawn_drone_formation(
            variant=variant,
            center=(0, 0, altitude),
            count=count,
            spacing=5.0,
            formation=formation,
        )

    # Print stats
    stats = world_gen.get_stats()
    print(f"\nWorld statistics:")
    for key, value in stats.items():
        print(f"  {key}: {value}")

    return drone_paths


def run_simulation_loop(world: World, drone_paths: list, duration: float, enable_px4: bool):
    """
    Run simulation loop with optional PX4 integration.
    """
    print(f"\nStarting simulation (duration: {duration}s)...")

    # Setup PX4 bridges if enabled
    px4_bridges = []
    if enable_px4:
        for i, path in enumerate(drone_paths):
            bridge = PX4Bridge(instance=i)
            if bridge.connect():
                bridge.start_async_receive()
                px4_bridges.append(bridge)
                print(f"  Connected PX4 bridge for drone {i}")
            else:
                print(f"  Warning: Failed to connect PX4 for drone {i}")

    # Create dynamics models for each drone
    dynamics = []
    for path in drone_paths:
        # Get variant from path (e.g., "/World/Drones/F11_isr_camera_0000")
        variant_name = path.split("/")[-1].split("_")[1]
        if variant_name in F11_VARIANTS:
            cfg = F11_VARIANTS[variant_name]
            mass = cfg.mass
            inertia = cfg.inertia
        else:
            mass = 6.3
            inertia = (0.115, 0.115, 0.175)

        dyn = QuadrotorDynamics(mass=mass, inertia=inertia)
        dyn.reset(position=np.array([0, 0, -10]))  # NED: Z down
        dynamics.append(dyn)

    # Simulation loop
    world.reset()
    start_time = time.time()
    frame_count = 0

    try:
        while True:
            elapsed = time.time() - start_time

            if duration > 0 and elapsed >= duration:
                print(f"\nSimulation complete ({duration}s)")
                break

            # Step simulation
            world.step(render=True)
            frame_count += 1

            # Update PX4 if enabled
            for i, bridge in enumerate(px4_bridges):
                if i < len(dynamics):
                    dyn = dynamics[i]

                    # Get actuator commands from PX4
                    cmd = bridge.get_last_actuators()
                    throttle = cmd.motor if cmd.armed else np.zeros(4)

                    # Step dynamics (simplified - real implementation reads from USD)
                    state = dyn.step(throttle, dt=1.0/250.0)

            # Print progress every 5 seconds
            if frame_count % 300 == 0:
                print(f"  Elapsed: {elapsed:.1f}s, Frames: {frame_count}")

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")

    finally:
        # Cleanup
        for bridge in px4_bridges:
            bridge.disconnect()

    print(f"\nTotal frames: {frame_count}")
    print(f"Average FPS: {frame_count / elapsed:.1f}")


def main():
    """Main entry point."""
    print("=" * 60)
    print("F-11 Drone Spawn Test")
    print("=" * 60)
    print(f"Variant: {args.variant}")
    print(f"Count: {args.count}")
    print(f"Formation: {args.formation}")
    print(f"Altitude: {args.altitude}m")
    print(f"PX4: {'Enabled' if args.px4 else 'Disabled'}")
    print("=" * 60)

    # Setup world
    world = setup_world()

    # Spawn drones
    drone_paths = spawn_with_world_generator(
        variant=args.variant,
        count=args.count,
        formation=args.formation,
        altitude=args.altitude,
    )

    print(f"\nSpawned {len(drone_paths)} drone(s):")
    for path in drone_paths:
        print(f"  {path}")

    # Run simulation
    run_simulation_loop(world, drone_paths, args.duration, args.px4)

    # Shutdown
    simulation_app.close()
    print("\nDone!")


if __name__ == "__main__":
    main()
