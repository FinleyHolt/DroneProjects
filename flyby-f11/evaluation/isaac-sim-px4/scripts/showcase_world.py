#!/usr/bin/env python3
"""
Showcase World - Full procedural environment demonstrating all generator features.

Creates a rich environment with:
- Procedural terrain with Perlin noise
- Forest vegetation (trees, undergrowth)
- Vehicle clusters (civilian and military)
- People groups
- Time-of-day lighting variations
- Weather/atmosphere effects

Run with GUI to visually inspect the complete world generation system.

Usage:
    /isaac-sim/python.sh /workspace/scripts/showcase_world.py

Author: Finley Holt
"""

import sys
import os

print("=" * 70)
print("SHOWCASE WORLD - Full Procedural Environment")
print("=" * 70)

# Force GUI mode for visual inspection
HEADLESS = False
print(f"[Config] Running in GUI mode for visual inspection", flush=True)

# Import SimulationApp and start it
print("[Init] Importing SimulationApp...", flush=True)
from isaacsim import SimulationApp

simulation_config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 0,
    "width": 1920,
    "height": 1080,
}
print(f"[Init] Creating SimulationApp with config: {simulation_config}", flush=True)
simulation_app = SimulationApp(simulation_config)
print("[Init] SimulationApp created successfully", flush=True)

import omni
import omni.timeline
import numpy as np
import time
import random
from pxr import UsdGeom, Gf, UsdPhysics, Sdf

print("[Init] Core imports completed", flush=True)

# Add paths
sys.path.insert(0, "/workspace")

EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

from isaacsim.core.utils.stage import get_current_stage
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.world import World

from flyby.world_generator.world_generator import WorldGenerator, WorldConfig

print("[Init] All imports completed successfully", flush=True)


# ============================================================================
# Configuration
# ============================================================================

MODELS_PATH = "/workspace/extensions/forest_generator/models"

# World settings
TERRAIN_SIZE = (300.0, 300.0)  # 300m x 300m terrain
TERRAIN_ROUGHNESS = 5.0        # Moderate height variation

# Vegetation
TREE_DENSITY = 1.0             # Trees per 100m²
INCLUDE_UNDERGROWTH = False    # Skip undergrowth for performance

# Targets
NUM_VEHICLE_CLUSTERS = 5       # Number of vehicle clusters
VEHICLES_PER_CLUSTER = (3, 6)  # Min/max vehicles per cluster
NUM_PEOPLE_GROUPS = 4          # Number of people groups
PEOPLE_PER_GROUP = (4, 8)      # Min/max people per group

# Lighting - pick a random time of day
TIME_OF_DAY_OPTIONS = ["dawn", "morning", "noon", "afternoon", "dusk", "overcast"]

# Weather - mostly clear with some variety
WEATHER_OPTIONS = ["clear", "clear", "clear", "light_haze", "moderate_haze"]


# ============================================================================
# Main Function
# ============================================================================

def run_showcase_world():
    """Create and display a full procedural world."""

    print("\n" + "=" * 70)
    print("GENERATING SHOWCASE WORLD")
    print("=" * 70)

    # Pick random time of day and weather
    time_of_day = random.choice(TIME_OF_DAY_OPTIONS)
    weather = random.choice(WEATHER_OPTIONS)

    print(f"\nWorld Configuration:")
    print(f"  Terrain: {TERRAIN_SIZE[0]}m x {TERRAIN_SIZE[1]}m")
    print(f"  Roughness: {TERRAIN_ROUGHNESS}m")
    print(f"  Tree density: {TREE_DENSITY} per 100m²")
    print(f"  Time of day: {time_of_day}")
    print(f"  Weather: {weather}")
    print()

    # Initialize world
    timeline = omni.timeline.get_timeline_interface()
    world = World()
    stage = get_current_stage()

    # Ensure physics scene exists
    physics_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(physics_path):
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_path))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

    # Configure world generator
    world_config = WorldConfig(
        terrain_size=TERRAIN_SIZE,
        terrain_roughness=TERRAIN_ROUGHNESS,
        tree_density=TREE_DENSITY,
        time_of_day=time_of_day,
        weather=weather,
        randomize_lighting=False,  # We set it explicitly
        randomize_weather=False,   # We set it explicitly
        randomize_terrain=True,    # Random terrain seed
        seed=None,                 # Random seed for variety
    )

    print("[WorldGen] Initializing WorldGenerator...", flush=True)
    world_gen = WorldGenerator(MODELS_PATH, world_config)

    # Generate terrain
    print("[WorldGen] Generating terrain...", flush=True)
    terrain_path = world_gen.generate_terrain()
    print(f"  Terrain created at: {terrain_path}")

    # Setup lighting
    print("[WorldGen] Setting up lighting...", flush=True)
    world_gen.setup_lighting()
    print(f"  Time of day: {world_gen.time_of_day}")
    print(f"  Weather: {world_gen.current_weather}")
    print(f"  Visibility: {world_gen.visibility_range}m")

    # SPAWN ORDER: Vehicles first, then people, then vegetation
    # This ensures important objects (vehicles, people) get priority placement
    # before trees fill up the remaining space.

    # Spawn vehicle clusters FIRST
    print("\n[WorldGen] Spawning vehicle clusters...", flush=True)
    all_vehicles = []
    half_size = min(TERRAIN_SIZE) / 2 * 0.7  # Stay within 70% of terrain

    # Get available vehicle types
    vehicle_types = list(world_gen.vehicles.vehicle_configs.keys())
    print(f"  Available vehicles: {vehicle_types}")

    for i in range(NUM_VEHICLE_CLUSTERS):
        # Random cluster center
        center_x = random.uniform(-half_size, half_size)
        center_y = random.uniform(-half_size, half_size)
        center = (center_x, center_y)

        # Random number of vehicles
        count = random.randint(*VEHICLES_PER_CLUSTER)

        # Pick random vehicle types for this cluster
        cluster_types = random.sample(vehicle_types, min(4, len(vehicle_types)))

        try:
            paths = world_gen.vehicles.spawn_vehicle_group(
                vehicle_types=cluster_types,
                count=count,
                clustering=0.7,
                center=center,
            )
            all_vehicles.extend(paths)
            print(f"  Cluster {i+1}: {count} vehicles at ({center_x:.1f}, {center_y:.1f})")
        except Exception as e:
            print(f"  Warning: Failed to spawn cluster {i+1}: {e}")

    print(f"  Total vehicles: {len(all_vehicles)}")

    # Spawn people groups
    print("\n[WorldGen] Spawning people groups...", flush=True)
    all_people = []

    # Get available person types
    person_types = list(world_gen.people.person_configs.keys())
    print(f"  Available people: {person_types}")

    if person_types:
        for i in range(NUM_PEOPLE_GROUPS):
            # Random group center
            center_x = random.uniform(-half_size, half_size)
            center_y = random.uniform(-half_size, half_size)
            center = (center_x, center_y)

            # Random number of people
            count = random.randint(*PEOPLE_PER_GROUP)

            try:
                paths = world_gen.people.spawn_crowd(
                    person_types=person_types,
                    count=count,
                    center=center,
                    radius=10.0,
                )
                all_people.extend(paths)
                print(f"  Group {i+1}: {count} people at ({center_x:.1f}, {center_y:.1f})")
            except Exception as e:
                print(f"  Warning: Failed to spawn group {i+1}: {e}")

        print(f"  Total people: {len(all_people)}")
    else:
        print("  No person models available")

    # Generate forest LAST (after vehicles and people have claimed their spots)
    print("\n[WorldGen] Generating forest...", flush=True)
    forest_result = world_gen.generate_forest(
        density=TREE_DENSITY,
        include_undergrowth=INCLUDE_UNDERGROWTH,
    )
    print(f"  Trees: {len(forest_result['trees'])}")
    print(f"  Bushes: {len(forest_result['bushes'])}")

    # Get final stats
    stats = world_gen.get_stats()

    print("\n" + "=" * 70)
    print("WORLD GENERATION COMPLETE")
    print("=" * 70)
    print(f"\nFinal Statistics:")
    print(f"  Trees: {stats['trees']}")
    print(f"  Bushes: {stats['bushes']}")
    print(f"  Vehicles: {stats['vehicles']}")
    print(f"  People: {stats['people']}")
    print(f"  Time of day: {stats['time_of_day']}")
    print(f"  Weather: {stats['weather']}")

    # Start simulation for rendering
    print("\n[Simulation] Starting physics and rendering...", flush=True)
    world.reset()
    timeline.play()

    # Run initial frames
    print("[Simulation] Running initial frames...", flush=True)
    for _ in range(120):
        world.step(render=True)
        simulation_app.update()

    # Print navigation instructions
    print("\n" + "=" * 70)
    print("SHOWCASE WORLD READY FOR EXPLORATION")
    print("=" * 70)
    print()
    print("Camera Controls:")
    print("  - Right-click + drag: Orbit camera")
    print("  - Middle-click + drag: Pan camera")
    print("  - Scroll wheel: Zoom in/out")
    print("  - Press 'F': Frame selection")
    print("  - Press 'H': Reset view to home")
    print()
    print("Tips:")
    print("  - Use the viewport's camera controls to fly around")
    print("  - Check the Stage panel to see the scene hierarchy")
    print("  - Vehicles are under /World/Vehicles")
    print("  - People are under /World/People")
    print("  - Trees are under /World/Vegetation/Trees")
    print()
    print("Press Ctrl+C to exit when done exploring.")
    print()

    # Keep running for visual inspection
    try:
        frame = 0
        while True:
            world.step(render=True)
            simulation_app.update()
            time.sleep(1/60)  # ~60 FPS

            # Periodic status update
            frame += 1
            if frame % 600 == 0:  # Every 10 seconds
                print(f"[Running] Frame {frame}, simulation time: {frame/60:.1f}s")

    except KeyboardInterrupt:
        print("\n[Exit] Shutting down...")

    # Cleanup
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    run_showcase_world()
