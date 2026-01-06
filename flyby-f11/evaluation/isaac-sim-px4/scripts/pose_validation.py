#!/usr/bin/env python3
"""
Pose Validation Script - Visual grid of all figure types at discrete yaw angles.

Creates a flat ground plane with a visual grid showing:
- Each figure type (vehicles, people) as a row
- Each column showing the figure at a discrete yaw angle (0, 15, 30, ... 345 degrees)

This allows visual inspection to verify that:
1. All figures spawn upright (no pitch/roll variation)
2. Only yaw rotation is applied (around world Z-axis)
3. All 24 discrete orientations look correct

Usage:
    /isaac-sim/python.sh /workspace/scripts/pose_validation.py

Author: Finley Holt
"""

import sys
import os

print("=" * 70)
print("POSE VALIDATION - Visual Grid Test")
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
from pxr import UsdGeom, Gf, UsdPhysics, Sdf, UsdShade, UsdLux

print("[Init] Core imports completed", flush=True)

# Add paths
sys.path.insert(0, "/workspace")

EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

from isaacsim.core.utils.stage import get_current_stage
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.world import World

from flyby.world_generator.spawners.vehicle_spawner import VehicleSpawner, DEFAULT_VEHICLES
from flyby.world_generator.spawners.people_spawner import PeopleSpawner
from flyby.world_generator.spawners.base_spawner import SpawnConfig

print("[Init] All imports completed successfully", flush=True)


# ============================================================================
# Configuration
# ============================================================================

# Discrete yaw angles (24 total, 15-degree increments)
DISCRETE_YAW_ANGLES = [i * 15.0 for i in range(24)]  # 0, 15, 30, ... 345

# Grid spacing - increased for better visibility of larger vehicles/tanks
VEHICLE_COLUMN_SPACING = 15.0   # Space between yaw angles (columns) for vehicles
VEHICLE_ROW_SPACING = 18.0      # Space between vehicle types (rows)
PEOPLE_COLUMN_SPACING = 5.0     # Space between yaw angles for people (smaller)
PEOPLE_ROW_SPACING = 6.0        # Space between person types

# Models path
MODELS_PATH = "/workspace/extensions/forest_generator/models"

# Ground plane size (should fit all figures) - increased for larger spacing
GROUND_SIZE = (450.0, 350.0)


# ============================================================================
# Helper Functions
# ============================================================================

def create_flat_ground_plane(stage, size=(200.0, 200.0)):
    """Create a simple flat ground plane."""
    ground_path = "/World/Ground"

    # Create as mesh (simple quad)
    vertices = np.array([
        [-size[0]/2, -size[1]/2, 0],
        [size[0]/2, -size[1]/2, 0],
        [size[0]/2, size[1]/2, 0],
        [-size[0]/2, size[1]/2, 0],
    ], dtype=np.float32)

    triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)

    ground_mesh = stage.DefinePrim(ground_path, "Mesh")
    ground_mesh.GetAttribute("points").Set(vertices)
    ground_mesh.GetAttribute("faceVertexIndices").Set(triangles.flatten())
    ground_mesh.GetAttribute("faceVertexCounts").Set(np.array([3, 3], dtype=np.int32))

    # Add collision
    prim = stage.GetPrimAtPath(ground_path)
    UsdPhysics.CollisionAPI.Apply(prim)

    # Apply gray material
    mat_path = "/World/Looks/ground_mat"
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.3, 0.3, 0.3)  # Medium gray
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(material)

    print(f"[Ground] Created flat ground plane at {ground_path}")
    return ground_path


def setup_lighting(stage):
    """Setup simple lighting for visual inspection."""
    # Create lighting parent
    if not stage.GetPrimAtPath("/World/Lighting"):
        prim_utils.create_prim("/World/Lighting", "Xform")

    # Dome light for ambient
    dome_path = "/World/Lighting/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_path)
    dome_light.CreateIntensityAttr(1000.0)
    dome_light.CreateColorAttr(Gf.Vec3f(0.9, 0.92, 1.0))  # Slightly blue for sky

    # Sun light for shadows
    sun_path = "/World/Lighting/SunLight"
    sun_light = UsdLux.DistantLight.Define(stage, sun_path)
    sun_light.CreateIntensityAttr(3000.0)
    sun_light.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.9))
    sun_light.CreateAngleAttr(0.53)

    # Position sun for good shadow visibility
    sun_xform = UsdGeom.Xformable(sun_light.GetPrim())
    sun_xform.AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 30.0, 0.0))

    print("[Lighting] Created dome and sun lights")


def create_row_label(stage, text, position):
    """Create a text label for a row (using a simple prim for now)."""
    # Note: Isaac Sim doesn't have easy text prims, so we skip visual labels
    # The console output will indicate which row is which
    pass


def create_angle_markers(stage, num_angles, column_spacing, start_y):
    """Create visual markers showing angle values."""
    # Create small cubes at each angle position to help with orientation
    for i, angle in enumerate(DISCRETE_YAW_ANGLES):
        x = i * column_spacing
        marker_path = f"/World/Markers/angle_{int(angle)}"

        # Create a small flat marker
        marker = UsdGeom.Cube.Define(stage, marker_path)
        marker_xform = UsdGeom.Xformable(marker.GetPrim())
        marker_xform.AddTranslateOp().Set(Gf.Vec3d(x, start_y - 3, 0.02))
        marker_xform.AddScaleOp().Set(Gf.Vec3d(0.3, 0.3, 0.02))

        # Color based on angle (gradient)
        mat_path = f"/World/Looks/marker_{int(angle)}_mat"
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")

        # Color gradient from red (0) to blue (360)
        hue = angle / 360.0
        r = max(0, 1 - 2*hue) if hue < 0.5 else 0
        b = max(0, 2*hue - 1) if hue > 0.5 else 0
        g = 1 - abs(2*hue - 1)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(r, g, b)
        )
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(marker.GetPrim()).Bind(material)


# ============================================================================
# Main Test Function
# ============================================================================

def run_pose_validation():
    """Main function to create pose validation grid."""

    print("\n" + "=" * 70)
    print("CREATING POSE VALIDATION GRID")
    print("=" * 70)
    print(f"Yaw angles: {len(DISCRETE_YAW_ANGLES)} positions (0 to 345 deg, 15 deg increments)")
    print(f"Vehicle spacing: {VEHICLE_COLUMN_SPACING}m x {VEHICLE_ROW_SPACING}m")
    print(f"People spacing: {PEOPLE_COLUMN_SPACING}m x {PEOPLE_ROW_SPACING}m")
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

    # Create world structure
    for path in ["/World", "/World/Vehicles", "/World/People", "/World/Markers"]:
        if not stage.GetPrimAtPath(path):
            prim_utils.create_prim(path, "Xform")

    # Create flat ground and lighting
    create_flat_ground_plane(stage, GROUND_SIZE)
    setup_lighting(stage)

    # Initialize spawners with large area to prevent position adjustments
    spawn_config = SpawnConfig(
        area_size=(500.0, 500.0),  # Large area
        min_spacing=0.0,  # No minimum spacing
    )

    vehicle_spawner = VehicleSpawner(stage, MODELS_PATH, spawn_config)
    people_spawner = PeopleSpawner(stage, MODELS_PATH, spawn_config)

    # Get available figure types
    vehicle_types = list(DEFAULT_VEHICLES.keys())
    person_types = list(people_spawner.person_configs.keys())

    print(f"\nAvailable vehicle types ({len(vehicle_types)}): {vehicle_types}")
    print(f"Available person types ({len(person_types)}): {person_types}")

    # Calculate grid layout with separate spacing for vehicles and people
    vehicle_grid_width = len(DISCRETE_YAW_ANGLES) * VEHICLE_COLUMN_SPACING
    vehicle_grid_height = len(vehicle_types) * VEHICLE_ROW_SPACING
    people_grid_width = len(DISCRETE_YAW_ANGLES) * PEOPLE_COLUMN_SPACING
    people_grid_height = len(person_types) * PEOPLE_ROW_SPACING

    print(f"\nVehicle grid: {vehicle_grid_width:.1f}m x {vehicle_grid_height:.1f}m")
    print(f"People grid: {people_grid_width:.1f}m x {people_grid_height:.1f}m")
    print(f"Total figures to spawn: {(len(vehicle_types) + len(person_types)) * len(DISCRETE_YAW_ANGLES)}")

    # Calculate starting position for vehicles (top section)
    vehicle_start_x = -vehicle_grid_width / 2 + VEHICLE_COLUMN_SPACING / 2
    vehicle_start_y = vehicle_grid_height / 2 - VEHICLE_ROW_SPACING / 2

    # People grid will be placed below vehicles with a gap
    gap_between_sections = 20.0
    people_start_x = -people_grid_width / 2 + PEOPLE_COLUMN_SPACING / 2
    people_start_y = -vehicle_grid_height / 2 - gap_between_sections - PEOPLE_ROW_SPACING / 2

    # Create angle markers for vehicle section
    if not stage.GetPrimAtPath("/World/Markers"):
        prim_utils.create_prim("/World/Markers", "Xform")
    create_angle_markers(stage, len(DISCRETE_YAW_ANGLES), VEHICLE_COLUMN_SPACING, vehicle_start_y + VEHICLE_ROW_SPACING)

    row_idx = 0
    spawned_count = 0

    # Spawn vehicles
    print("\n--- Spawning Vehicles ---")
    for vehicle_type in vehicle_types:
        y_pos = vehicle_start_y - row_idx * VEHICLE_ROW_SPACING
        print(f"Row {row_idx}: {vehicle_type} at y={y_pos:.1f}m")

        for col_idx, angle in enumerate(DISCRETE_YAW_ANGLES):
            x_pos = vehicle_start_x + col_idx * VEHICLE_COLUMN_SPACING

            try:
                path = vehicle_spawner.spawn_vehicle(
                    vehicle_type=vehicle_type,
                    position=(x_pos, y_pos),
                    heading=angle,  # Discrete yaw angle
                )
                spawned_count += 1
            except Exception as e:
                print(f"  Warning: Failed to spawn {vehicle_type} at {angle}deg: {e}")

        row_idx += 1

    # Spawn people (separate grid below vehicles)
    print("\n--- Spawning People ---")
    person_row_idx = 0
    for person_type in person_types:
        y_pos = people_start_y - person_row_idx * PEOPLE_ROW_SPACING
        print(f"Row {row_idx}: {person_type} at y={y_pos:.1f}m")

        for col_idx, angle in enumerate(DISCRETE_YAW_ANGLES):
            x_pos = people_start_x + col_idx * PEOPLE_COLUMN_SPACING

            try:
                path = people_spawner.spawn_person(
                    person_type=person_type,
                    position=(x_pos, y_pos),
                    facing=angle,  # Discrete yaw angle
                )
                spawned_count += 1
            except Exception as e:
                print(f"  Warning: Failed to spawn {person_type} at {angle}deg: {e}")

        row_idx += 1
        person_row_idx += 1

    print(f"\n[Complete] Spawned {spawned_count} figures")

    # Start simulation for rendering
    world.reset()
    timeline.play()

    # Run a few frames to let everything settle
    print("\n[Rendering] Running initial frames...")
    for _ in range(60):
        world.step(render=True)
        simulation_app.update()

    # Print viewing instructions
    print("\n" + "=" * 70)
    print("POSE VALIDATION GRID READY FOR INSPECTION")
    print("=" * 70)
    print()
    print("Grid Layout:")
    print("  - Each ROW = one figure type (vehicles first, then people)")
    print("  - Each COLUMN = one yaw angle (0, 15, 30, ... 345 degrees)")
    print("  - Colored markers at the top indicate angle progression")
    print("    (red=0deg, green=180deg, blue=360deg)")
    print()
    print("What to verify:")
    print("  1. All figures are UPRIGHT (no tilted/flipped models)")
    print("  2. Rotation is ONLY around Z-axis (yaw)")
    print("  3. Figures progress smoothly through orientations")
    print()
    print("Camera Controls:")
    print("  - Use mouse to orbit/pan/zoom")
    print("  - Press 'F' to frame selection")
    print("  - Press 'H' to reset view")
    print()
    print("Grid center is at (0, 0). Vehicles at top, people at bottom.")
    print()

    # Print row legend
    print("Row Legend (Vehicles):")
    for i, vtype in enumerate(vehicle_types):
        y = vehicle_start_y - i * VEHICLE_ROW_SPACING
        print(f"  Row {i}: {vtype:20s} (y = {y:+.1f}m)")
    print("\nRow Legend (People):")
    for i, ptype in enumerate(person_types):
        y = people_start_y - i * PEOPLE_ROW_SPACING
        print(f"  Row {len(vehicle_types) + i}: {ptype:20s} (y = {y:+.1f}m)")

    print()
    print("Press Ctrl+C to exit when done inspecting.")
    print()

    # Keep running for visual inspection
    try:
        while True:
            world.step(render=True)
            simulation_app.update()
            time.sleep(1/60)  # ~60 FPS
    except KeyboardInterrupt:
        print("\n[Exit] Shutting down...")

    # Cleanup
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    run_pose_validation()
