#!/usr/bin/env python3
"""
Shader warmup script - compiles Vulkan shaders to avoid runtime compilation.

Run this script once after container startup if shaders weren't pre-compiled
during build. The compiled shaders will be cached for subsequent runs.

Usage (inside container):
    /isaac-sim/python.sh /workspace/scripts/warmup_shaders.py

This script:
1. Starts Isaac Sim in headless mode with RTX rendering
2. Creates a simple scene to trigger shader compilation
3. Runs rendering for 60 seconds to compile all shaders
4. Reports shader cache sizes after completion
"""

import sys
import time
import os

print("=" * 60)
print("SHADER WARMUP")
print("=" * 60)
print()
print("This script pre-compiles Vulkan shaders to avoid runtime")
print("compilation overhead. It will take 60-90 seconds.")
print()

# Start Isaac Sim in headless mode
print("[1/4] Starting Isaac Sim...")
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": True,
    "width": 1280,
    "height": 720,
    "anti_aliasing": 0,
    "renderer": "RayTracedLighting",  # Force RTX to compile RTX shaders
})

print("      Isaac Sim started")

# Import after SimulationApp is created
import omni
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicSphere, GroundPlane
from omni.isaac.core.prims import XFormPrim
import numpy as np

# Create world with physics
print("[2/4] Loading scene...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add objects to trigger material/shader compilation
for i in range(5):
    sphere = world.scene.add(
        DynamicSphere(
            prim_path=f"/World/Sphere_{i}",
            name=f"sphere_{i}",
            position=np.array([i * 2.0, 0, 1.0]),
            radius=0.5,
            color=np.array([1.0, 0.0, 0.0]),
        )
    )

print("      Scene loaded")

# Reset world to start simulation
world.reset()

# Run simulation to compile all shaders
print("[3/4] Warming up shaders...")
start_time = time.time()
warmup_duration = 60  # seconds
step_count = 0

while time.time() - start_time < warmup_duration:
    world.step(render=True)
    step_count += 1
    if step_count % 100 == 0:
        elapsed = time.time() - start_time
        remaining = warmup_duration - elapsed
        print(f"      Progress: {elapsed:.0f}s / {warmup_duration}s ({remaining:.0f}s remaining)")

print(f"      Warmup complete ({step_count} render steps)")

# Clean shutdown
print("[4/4] Shutting down...")
simulation_app.close()
print("      Isaac Sim closed")

# Report shader cache sizes
print()
print("=" * 60)
print("SHADER CACHE STATUS")
print("=" * 60)

cache_paths = [
    "/isaac-sim/kit/cache/shadercache",
    "/isaac-sim/kit/cache/nv_shadercache",
]

# Find RTX shader cache (path includes version number)
import glob
rtx_caches = glob.glob("/isaac-sim/extscache/omni.hydra.rtx.shadercache.vulkan-*/cache/shadercache")
cache_paths.extend(rtx_caches)

total_size = 0
for path in cache_paths:
    if os.path.exists(path):
        size = sum(
            os.path.getsize(os.path.join(dirpath, filename))
            for dirpath, dirnames, filenames in os.walk(path)
            for filename in filenames
        )
        total_size += size
        print(f"  {path}: {size / 1024 / 1024:.1f} MB")
    else:
        print(f"  {path}: NOT FOUND")

print()
print(f"  Total shader cache: {total_size / 1024 / 1024:.1f} MB")
print()
print("Shader warmup complete! Subsequent Isaac Sim starts will be faster.")
print("=" * 60)
