#!/usr/bin/env python3
"""
Scene inspection script - spawns procedurally generated environment.
Keeps GUI open for visual inspection without training.

Usage:
    /isaac-sim/python.sh /workspace/scripts/inspect_scene.py --problem multi_objective
"""

import argparse
import sys
import time
import random
from pathlib import Path

# Add project paths
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "environments"))

# Pegasus path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)


def main():
    parser = argparse.ArgumentParser(description="Inspect simulation scene")
    parser.add_argument(
        "--problem",
        type=str,
        choices=['comms_denied', 'dynamic_nfz', 'multi_objective'],
        default='multi_objective',
        help="Problem environment to load"
    )
    parser.add_argument(
        "--terrain-size",
        type=float,
        default=500.0,
        help="Terrain size in meters"
    )
    parser.add_argument(
        "--num-clusters",
        type=int,
        default=5,
        help="Number of POI clusters to spawn"
    )
    args = parser.parse_args()

    print("=" * 60)
    print(f"Scene Inspection: {args.problem}")
    print(f"Terrain size: {args.terrain_size}m x {args.terrain_size}m")
    print(f"POI clusters: {args.num_clusters}")
    print("=" * 60)

    # Import Isaac Sim modules
    from isaacsim import SimulationApp

    # Launch with GUI - RayTraced for best visuals
    simulation_app = SimulationApp({
        "headless": False,
        "width": 1920,
        "height": 1080,
        "window_width": 1920,
        "window_height": 1080,
        "anti_aliasing": 1,
        "renderer": "RayTracedLighting",
    })

    print("\n[1/5] Isaac Sim GUI started")

    # Import after SimulationApp is created
    import omni
    import numpy as np
    from pxr import UsdGeom, Gf
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    import omni.isaac.core.utils.prims as prim_utils
    from pegasus.simulator.params import ROBOTS
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

    print("\n[2/5] Creating world...")

    # Initialize Pegasus (without loading default environment)
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world

    # Set coordinates (Camp Pendleton)
    pg.set_global_coordinates(33.3853, -117.5653, 100.0)

    print("\n[3/5] Generating procedural terrain...")

    # Try to use world generator
    try:
        # Add extension path
        ext_path = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
        if ext_path not in sys.path:
            sys.path.insert(0, ext_path)

        from flyby.world_generator import WorldGenerator, WorldConfig

        world_config = WorldConfig(
            terrain_size=(args.terrain_size, args.terrain_size),
            terrain_roughness=3.0,
            randomize_lighting=True,
            seed=random.randint(0, 10000),
        )

        world_gen = WorldGenerator(
            models_path="/workspace/extensions/forest_generator/models",
            config=world_config,
        )

        # Generate terrain
        print("  Generating terrain...")
        world_gen.generate_terrain()

        # Setup lighting (day/dusk/dawn)
        print("  Setting up lighting...")
        world_gen.setup_lighting()

        # Spawn vegetation (forest)
        print("  Generating forest...")
        world_gen.generate_forest(
            density=0.5,  # Trees per 100m²
            include_undergrowth=True,
        )

        # Spawn POI clusters (vehicles and people)
        print(f"  Spawning {args.num_clusters} POI clusters...")
        clusters = world_gen.spawn_poi_clusters(
            num_clusters=args.num_clusters,
            targets_per_cluster=(3, 8),
            cluster_radius=30.0,
            include_vehicles=True,
            include_people=True,
        )

        total_targets = sum(c['num_targets'] for c in clusters)
        print(f"  Created {len(clusters)} clusters with {total_targets} total targets")

        for i, cluster in enumerate(clusters):
            print(f"    Cluster {i+1}: {cluster['num_targets']} targets at ({cluster['center'][0]:.0f}, {cluster['center'][1]:.0f})")

    except ImportError as e:
        print(f"  World generator not available: {e}")
        print("  Loading default environment instead...")
        from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
        pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    except Exception as e:
        print(f"  World generator error: {e}")
        import traceback
        traceback.print_exc()
        print("  Loading default environment instead...")
        from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
        pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

    print("\n[4/5] Spawning F-11 drone (no PX4 - visual only)...")

    # Spawn drone WITHOUT PX4 backend (just for visual inspection)
    # This avoids PX4 path issues and keeps the scene clean
    config = MultirotorConfig()
    config.backends = []  # No backends - just visual model

    drone = Multirotor(
        "/World/f11_isr",
        ROBOTS["Iris"],
        0,
        [0.0, 0.0, 50.0],  # Start at 50m altitude for overview
        config=config,
    )

    # Reset world to initialize physics
    print("\n[5/5] Initializing physics...")
    world.reset()

    # Set camera to bird's eye view
    print("  Setting camera position...")
    try:
        viewport = omni.kit.viewport_legacy.get_viewport_interface()
        if viewport:
            camera = viewport.get_viewport_window().get_active_camera()
            # Move camera up and back for overview
            prim_utils.set_prim_property(
                camera,
                "xformOp:translate",
                Gf.Vec3d(0.0, -200.0, 150.0)
            )
    except Exception:
        pass  # Camera adjustment is optional

    print("\n" + "=" * 60)
    print("Scene loaded successfully!")
    print("=" * 60)
    print("\nControls:")
    print("  - Right-click + drag to orbit camera")
    print("  - Scroll to zoom")
    print("  - Middle-click + drag to pan")
    print("  - Press SPACEBAR to start/stop simulation")
    print("  - Close window to exit")
    print("\nScene contents:")
    print(f"  - Terrain: {args.terrain_size}m x {args.terrain_size}m")
    print(f"  - POI clusters: {args.num_clusters}")
    print(f"  - Drone at: (0, 0, 50)")
    print("\nKeeping GUI open for inspection...")

    # Keep GUI running
    try:
        while simulation_app.is_running():
            world.step(render=True)
            time.sleep(0.016)  # ~60 FPS
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
