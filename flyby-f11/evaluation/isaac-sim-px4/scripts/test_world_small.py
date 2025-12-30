#!/usr/bin/env python3
"""
Small World Test - Minimal terrain with few trees for quick visualization.
"""

import sys
sys.stdout.reconfigure(line_buffering=True)

print("=" * 50)
print("Small World Test (30m x 30m, ~10 trees)")
print("=" * 50)

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})
print("Isaac Sim ready")

import omni.timeline
from pxr import UsdPhysics, Sdf, Gf
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import get_current_stage

sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")
from flyby.world_generator import WorldGenerator, WorldConfig

# Setup
world = World()
stage = get_current_stage()
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)

# Small world config
config = WorldConfig(
    terrain_size=(30.0, 30.0),  # Small 30m x 30m
    terrain_roughness=1.0,
    tree_density=1.0,  # Very sparse - ~9 trees total
    undergrowth_density=2.0,
    seed=42,
)

print("Generating world...")
world_gen = WorldGenerator(
    models_path="/workspace/extensions/forest_generator/models",
    config=config,
)

# Generate
world_gen.generate_terrain()
print("  Terrain done")

world_gen.setup_lighting()
print("  Lighting done")

forest = world_gen.generate_forest(include_undergrowth=False)
print(f"  Forest done: {len(forest['trees'])} trees")

# Run
print("\nWorld generated! Running simulation...")
print("Use mouse to navigate viewport. Press Ctrl+C to exit.")
print("=" * 50)

world.reset()
timeline = omni.timeline.get_timeline_interface()
timeline.play()

try:
    while simulation_app.is_running():
        world.step(render=True)
except KeyboardInterrupt:
    pass

timeline.stop()
simulation_app.close()
print("Done!")
