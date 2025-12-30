#!/usr/bin/env python3
"""
Complete World Generator Test - ISR Training Environment

Tests all world generation features:
- Terrain with forest floor PBR material
- Forest HDRI skybox (forest_slope_2k.hdr) for realistic lighting
- Procedural trees and undergrowth
- Procedural vehicles (civilian and military)
- Reallusion character models (Worker, Debra)

Run with GUI:
  podman exec px4-sim /isaac-sim/python.sh /workspace/scripts/test_world_generator.py

Run headless:
  podman exec px4-sim /isaac-sim/python.sh /workspace/scripts/test_world_generator.py --headless
"""

import argparse
import sys
import time

# Force unbuffered output
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# Parse args before importing Isaac Sim
parser = argparse.ArgumentParser(description="Test World Generator")
parser.add_argument("--headless", action="store_true", help="Run headless (no GUI)")
parser.add_argument("--size", type=int, default=200, help="World size in meters")
parser.add_argument("--duration", type=int, default=60, help="Test duration in seconds")
args = parser.parse_args()

print("=" * 60)
print("Flyby World Generator Test")
print("=" * 60)
print(f"\nWorld Size: {args.size}m x {args.size}m")
print(f"Headless: {args.headless}")
print(f"Duration: {args.duration}s")

# Initialize Isaac Sim
print("\n[1/5] Initializing Isaac Sim...")
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "width": 1920,
    "height": 1080,
    "window_width": 1920,
    "window_height": 1080,
    "anti_aliasing": 0,  # Disable for stability
    "renderer": "RayTracedLighting",  # More stable than PathTracing
    "fast_shutdown": False,  # Clean shutdown
    "sync_loads": True,  # Wait for assets to load
    "multi_gpu": False,  # Single GPU for stability
})
print("  Isaac Sim ready")

# Import after Isaac Sim initialization
print("\n[2/5] Loading extensions...")
import omni.timeline
import omni.usd
from pxr import UsdPhysics, Sdf, Gf
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import get_current_stage

# Add extension path
sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")

from flyby.world_generator import WorldGenerator, WorldConfig
print("  Extensions loaded")

# Setup world
print("\n[3/5] Setting up simulation world...")
timeline = omni.timeline.get_timeline_interface()
world = World()
stage = get_current_stage()

# Add physics scene
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)
print("  World ready")

# Create world generator with randomization
print("\n[4/5] Generating procedural environment...")
import random
import time

# Use time-based seed for true randomization each run
random_seed = int(time.time() * 1000) % 100000
random.seed(random_seed)
print(f"  Random seed: {random_seed}")

# Randomize tree density between 1.0 and 2.5
tree_density = random.uniform(1.0, 2.5)

config = WorldConfig(
    terrain_size=(float(args.size), float(args.size)),
    terrain_roughness=random.uniform(2.0, 5.0),  # Randomize terrain
    terrain_material="forest_floor",
    tree_density=tree_density,
    tree_proportions={"Birch": 30, "Spruce": 40, "Pine": 30},
    undergrowth_density=0.0,  # Disabled - trees only, no bushes
    randomize_lighting=True,
    seed=None,  # No fixed seed - full randomization
)

world_gen = WorldGenerator(
    models_path="/workspace/extensions/forest_generator/models",
    config=config,
)

# Generate terrain
print("  Generating terrain...")
terrain_path = world_gen.generate_terrain()
print(f"    Created: {terrain_path}")

# Setup lighting
print("  Setting up HDRI lighting...")
world_gen.setup_lighting()
print("    Lighting configured")

# Generate forest
print("  Generating forest...")
forest = world_gen.generate_forest()
print(f"    Trees: {len(forest['trees'])}")
print(f"    Bushes: {len(forest['bushes'])}")

# Spawn vehicles with randomization
print("  Spawning vehicles...")
vehicles_spawned = []

# Random number of civilian vehicles (3-8)
civilian_types = ["sedan", "sedan2", "suv", "sports_car", "sports_car2", "taxi", "police"]
num_civilian = random.randint(3, 8)
for i in range(num_civilian):
    vtype = random.choice(civilian_types)
    # Random position within terrain bounds
    x = random.uniform(-args.size * 0.4, args.size * 0.4)
    y = random.uniform(-args.size * 0.4, args.size * 0.4)
    try:
        path = world_gen.vehicles.spawn_vehicle(vtype, position=(x, y))
        vehicles_spawned.append(path)
        print(f"    {vtype} at ({x:.1f}, {y:.1f})")
    except Exception as e:
        print(f"    Failed to spawn {vtype}: {e}")

# Random number of tanks (1-4)
tank_types = ["tank", "tank2", "tank3", "tank4"]
num_tanks = random.randint(1, 4)
for i in range(num_tanks):
    ttype = random.choice(tank_types)
    x = random.uniform(-args.size * 0.4, args.size * 0.4)
    y = random.uniform(-args.size * 0.4, args.size * 0.4)
    try:
        path = world_gen.vehicles.spawn_vehicle(ttype, position=(x, y))
        vehicles_spawned.append(path)
        print(f"    {ttype} at ({x:.1f}, {y:.1f})")
    except Exception as e:
        print(f"    Failed to spawn {ttype}: {e}")

# Spawn people with randomization
print("  Spawning people...")
people_spawned = []

if world_gen.people.person_configs:
    available = list(world_gen.people.person_configs.keys())
    print(f"    Available characters: {available}")

    # Random number of individual people (5-15)
    num_people = random.randint(5, 15)
    for i in range(num_people):
        ptype = random.choice(available)
        x = random.uniform(-args.size * 0.4, args.size * 0.4)
        y = random.uniform(-args.size * 0.4, args.size * 0.4)
        try:
            path = world_gen.people.spawn_person(ptype, position=(x, y))
            people_spawned.append(path)
            print(f"    {ptype} at ({x:.1f}, {y:.1f})")
        except Exception as e:
            print(f"    Failed to spawn {ptype}: {e}")

    # Random crowds (0-2 crowds)
    num_crowds = random.randint(0, 2)
    for c in range(num_crowds):
        crowd_size = random.randint(3, 8)
        center_x = random.uniform(-args.size * 0.3, args.size * 0.3)
        center_y = random.uniform(-args.size * 0.3, args.size * 0.3)
        try:
            crowd = world_gen.people.spawn_crowd(
                person_types=available,
                count=crowd_size,
                center=(center_x, center_y),
                radius=random.uniform(3.0, 8.0),
            )
            people_spawned.extend(crowd)
            print(f"    Crowd of {crowd_size} at ({center_x:.1f}, {center_y:.1f})")
        except Exception as e:
            print(f"    Failed to spawn crowd: {e}")
else:
    print("    No character models found (check models/characters/ path)")

# Get final stats
stats = world_gen.get_stats()
print("\n  Generation complete!")
print(f"    Total trees: {stats['trees']}")
print(f"    Total bushes: {stats['bushes']}")
print(f"    Vehicles: {stats['vehicles']}")
print(f"    People: {stats['people']}")

# Run simulation
print("\n[5/5] Running simulation...")
print("=" * 60)
print("Environment Test Running")
print()
print("Features tested:")
print("  - Procedural terrain with Perlin noise")
print("  - Forest floor PBR material with UV mapping")
print("  - Forest HDRI skybox (forest_slope_2k.hdr)")
print("  - Tree and bush spawning")
print("  - Procedural vehicles (civilian + military)")
print("  - Reallusion character models")
print()
print("Press Ctrl+C to stop early.")
print("=" * 60)

world.reset()
timeline.play()

start_time = time.time()
step_count = 0

try:
    while simulation_app.is_running():
        world.step(render=True)
        step_count += 1

        elapsed = time.time() - start_time

        # Status update every 10 seconds
        if step_count % 600 == 0:
            print(f"t={elapsed:6.1f}s | Steps: {step_count}")

        # Only exit on duration if --duration was explicitly set (not default 60)
        if args.duration != 60 and elapsed >= args.duration:
            print(f"\nTest completed after {elapsed:.1f}s")
            break

except KeyboardInterrupt:
    print("\n\nStopping simulation...")

# Cleanup - wrap in try/except to avoid segfault on shutdown
try:
    timeline.stop()
    simulation_app.close()
except:
    pass  # Ignore shutdown errors

print("\n" + "=" * 60)
print("World Generator Test Complete!")
print("=" * 60)
print(f"  Terrain: {terrain_path}")
print(f"  Trees: {stats['trees']}")
print(f"  Bushes: {stats['bushes']}")
print(f"  Vehicles: {stats['vehicles']}")
print(f"  People: {stats['people']}")
print("=" * 60)
