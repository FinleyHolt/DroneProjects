#!/usr/bin/env python3
"""
Camera Orientation Grid Test

Creates a grid of cameras with different orientations to find the correct
euler angles for a forward-looking, downward-tilted camera.

Each camera captures the same scene and the results are composited into
a single grid image for easy comparison.

Author: Finley Holt
"""

from isaacsim import SimulationApp

simulation_config = {
    "headless": True,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 0,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(simulation_config)

import omni
import omni.replicator.core as rep
from omni.isaac.core.world import World
from pxr import UsdGeom, UsdLux, UsdShade, Sdf, Gf
from isaacsim.sensors.camera import Camera
from isaacsim.core.utils.stage import get_current_stage
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation
from datetime import datetime

print("=" * 70)
print("Camera Orientation Grid Test")
print("=" * 70)

# Output directory
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_DIR = f"/workspace/output/camera_orientation_test_{timestamp}"
os.makedirs(OUTPUT_DIR, exist_ok=True)
print(f"Output: {OUTPUT_DIR}")

# Grid configuration - test various orientations
# Format: (description, method, params)
# Methods: "euler" uses rot_utils, "matrix" uses rotation matrix approach, "scipy" uses scipy euler
CAMERA_CONFIGS = [
    # Row 1: Variations around [0,0,0] which looked good
    ("euler [0,0,0]", "euler", [0, 0, 0]),
    ("euler [0,15,0]", "euler", [0, 15, 0]),
    ("euler [0,25,0]", "euler", [0, 25, 0]),
    ("euler [0,30,0]", "euler", [0, 30, 0]),

    # Row 2: Negative pitch (tilt down)
    ("euler [0,-15,0]", "euler", [0, -15, 0]),
    ("euler [0,-25,0]", "euler", [0, -25, 0]),
    ("euler [0,-30,0]", "euler", [0, -30, 0]),
    ("euler [0,-45,0]", "euler", [0, -45, 0]),

    # Row 3: Small variations in roll and yaw
    ("euler [5,25,0]", "euler", [5, 25, 0]),
    ("euler [-5,25,0]", "euler", [-5, 25, 0]),
    ("euler [0,25,5]", "euler", [0, 25, 5]),
    ("euler [0,25,-5]", "euler", [0, 25, -5]),

    # Row 4: Negative pitch with small adjustments
    ("euler [5,-25,0]", "euler", [5, -25, 0]),
    ("euler [-5,-25,0]", "euler", [-5, -25, 0]),
    ("euler [0,-25,5]", "euler", [0, -25, 5]),
    ("euler [0,-25,-5]", "euler", [0, -25, -5]),
]

GRID_COLS = 4
GRID_ROWS = 4
CAMERA_RES = (320, 240)  # Small resolution for grid


def create_orientation_euler(angles_deg):
    """Create orientation using Isaac Sim's euler_angles_to_quats."""
    import isaacsim.core.utils.numpy.rotations as rot_utils
    return rot_utils.euler_angles_to_quats(np.array(angles_deg), degrees=True)


def create_orientation_matrix(tilt_degrees):
    """Create orientation using rotation matrix approach (from GimbalController)."""
    tilt_rad = np.radians(tilt_degrees)
    cos_tilt = np.cos(tilt_rad)
    sin_tilt = np.sin(tilt_rad)

    # Look direction: forward (+X) with tilt
    look_dir = np.array([cos_tilt, 0.0, sin_tilt])
    look_dir = look_dir / np.linalg.norm(look_dir)

    # Build camera coordinate frame
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(look_dir, world_up)
    right_norm = np.linalg.norm(right)
    if right_norm < 1e-6:
        right = np.array([0.0, 1.0, 0.0])
    else:
        right = right / right_norm
    up = np.cross(right, look_dir)
    up = up / np.linalg.norm(up)

    # Camera convention: looks down -Z axis
    R_cam_to_world = np.column_stack([right, up, -look_dir])
    world_rot = Rotation.from_matrix(R_cam_to_world)
    quat_xyzw = world_rot.as_quat()

    # Convert to wxyz for Isaac Sim
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])


def create_orientation_scipy(angles_deg, convention):
    """Create orientation using scipy Rotation with specified convention."""
    rot = Rotation.from_euler(convention, angles_deg, degrees=True)
    quat_xyzw = rot.as_quat()
    # Convert to wxyz for Isaac Sim
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])


def setup_scene_with_color(stage):
    """Setup scene with proper HDRI lighting and colored materials."""
    print("[Scene] Setting up with HDRI and colored materials...")

    # Create lighting parent
    stage.DefinePrim("/World/Lighting", "Xform")

    # Dome light with HDRI texture for sky
    dome_path = "/World/Lighting/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_path)
    dome_light.CreateIntensityAttr(1000.0)

    # Try to use HDRI texture if available
    hdri_paths = [
        "/workspace/extensions/forest_generator/models/textures/sky_2k.hdr",
        "/isaac-sim/exts/omni.replicator.core/omni/replicator/core/data/dome_lights/studio_small_08_4k.hdr",
    ]
    hdri_found = False
    for hdri_path in hdri_paths:
        if os.path.exists(hdri_path):
            dome_light.CreateTextureFileAttr().Set(hdri_path)
            dome_light.CreateTextureFormatAttr().Set("latlong")
            print(f"  Using HDRI: {hdri_path}")
            hdri_found = True
            break

    if not hdri_found:
        # Fallback to blue sky color
        dome_light.CreateColorAttr(Gf.Vec3f(0.4, 0.6, 1.0))
        print("  Using fallback blue sky color")

    dome_light.CreateSpecularAttr(1.0)

    # Sun light
    sun_path = "/World/Lighting/SunLight"
    sun_light = UsdLux.DistantLight.Define(stage, sun_path)
    sun_light.CreateIntensityAttr(3000.0)
    sun_light.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.85))  # Warm sunlight
    sun_light.CreateAngleAttr(0.53)
    sun_xform = UsdGeom.Xformable(sun_light.GetPrim())
    sun_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))

    # Create materials container
    stage.DefinePrim("/World/Looks", "Scope")

    # Green ground material
    create_pbr_material(stage, "ground_green", Gf.Vec3f(0.1, 0.5, 0.1), roughness=0.8)

    # Gray obstacle material
    create_pbr_material(stage, "obstacle_gray", Gf.Vec3f(0.5, 0.5, 0.55), roughness=0.6)

    # Red marker material
    create_pbr_material(stage, "marker_red", Gf.Vec3f(0.8, 0.1, 0.1), roughness=0.5)

    print("[Scene] Lighting and materials configured")


def create_pbr_material(stage, name, color, roughness=0.5):
    """Create a PBR material with proper settings for headless rendering."""
    mat_path = f"/World/Looks/{name}_mat"
    material = UsdShade.Material.Define(stage, mat_path)

    shader_path = f"{mat_path}/Shader"
    shader = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.04, 0.04, 0.04))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def apply_material(stage, prim_path, material_name):
    """Apply material to a prim."""
    mat_path = f"/World/Looks/{material_name}_mat"
    material = UsdShade.Material.Get(stage, mat_path)
    prim = stage.GetPrimAtPath(prim_path)
    if material and prim.IsValid():
        UsdShade.MaterialBindingAPI(prim).Bind(material)


def create_test_scene(stage):
    """Create test scene with ground, pillars, and orientation markers."""
    import omni.isaac.core.utils.prims as prim_utils

    print("[Scene] Creating test objects...")

    # Large ground plane
    ground_path = "/World/Ground"
    prim_utils.create_prim(
        prim_path=ground_path,
        prim_type="Cube",
        position=np.array([0.0, 0.0, -0.5]),
        scale=np.array([500.0, 500.0, 1.0]),
    )
    apply_material(stage, ground_path, "ground_green")

    # Vertical pillars at various distances
    pillar_positions = [
        (30, 0, 45),    # Close center
        (60, -20, 45),  # Medium left
        (60, 20, 45),   # Medium right
        (100, 0, 45),   # Far center
    ]

    for i, (x, y, h) in enumerate(pillar_positions):
        pillar_path = f"/World/Pillar_{i}"
        prim_utils.create_prim(
            prim_path=pillar_path,
            prim_type="Cylinder",
            position=np.array([x, y, h/2]),
            scale=np.array([5.0, 5.0, h/2]),  # radius=5m, height=h
        )
        apply_material(stage, pillar_path, "obstacle_gray")

    # Forward marker (red cube at +X direction)
    marker_path = "/World/ForwardMarker"
    prim_utils.create_prim(
        prim_path=marker_path,
        prim_type="Cube",
        position=np.array([50.0, 0.0, 30.0]),
        scale=np.array([3.0, 3.0, 3.0]),
    )
    apply_material(stage, marker_path, "marker_red")

    print(f"  Created ground, {len(pillar_positions)} pillars, forward marker")


def create_cameras(stage):
    """Create all test cameras with different orientations."""
    cameras = []
    render_products = []
    rgb_annotators = []

    # Camera position (drone at 30m altitude)
    camera_pos = np.array([0.0, 0.0, 30.0])

    print(f"\n[Cameras] Creating {len(CAMERA_CONFIGS)} cameras...")

    for i, (desc, method, params) in enumerate(CAMERA_CONFIGS):
        camera_path = f"/World/Camera_{i}"

        # Create orientation based on method
        if method == "euler":
            orientation = create_orientation_euler(params)
        elif method == "matrix":
            orientation = create_orientation_matrix(params)
        elif method.startswith("scipy_"):
            convention = method.split("_")[1].upper()
            orientation = create_orientation_scipy(params, convention)
        else:
            orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Identity

        # Create camera
        camera = Camera(
            prim_path=camera_path,
            position=camera_pos.copy(),
            frequency=30,
            resolution=CAMERA_RES,
            orientation=orientation,
        )
        camera.initialize()
        simulation_app.update()
        camera.initialize()
        simulation_app.update()

        # Set up render product and annotator
        rp = rep.create.render_product(camera_path, CAMERA_RES)
        rgb_ann = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_ann.attach([rp])

        cameras.append(camera)
        render_products.append(rp)
        rgb_annotators.append(rgb_ann)

        print(f"  [{i}] {desc}")

    return cameras, render_products, rgb_annotators


def capture_grid(rgb_annotators, descriptions):
    """Capture frames from all cameras and create grid image."""
    print("\n[Capture] Capturing from all cameras...")

    # Trigger render
    rep.orchestrator.step(rt_subframes=8)

    # Collect frames
    frames = []
    for i, ann in enumerate(rgb_annotators):
        data = ann.get_data()
        if data is not None and data.size > 0:
            # Convert RGBA to BGR
            if len(data.shape) == 3:
                if data.shape[-1] == 4:
                    frame = cv2.cvtColor(data[:, :, :3], cv2.COLOR_RGB2BGR)
                else:
                    frame = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            else:
                frame = np.zeros((CAMERA_RES[1], CAMERA_RES[0], 3), dtype=np.uint8)
            frames.append(frame)
        else:
            # Black frame if capture failed
            frames.append(np.zeros((CAMERA_RES[1], CAMERA_RES[0], 3), dtype=np.uint8))

    # Create grid
    grid_h = GRID_ROWS * CAMERA_RES[1]
    grid_w = GRID_COLS * CAMERA_RES[0]
    grid = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)

    for i, (frame, (desc, _, _)) in enumerate(zip(frames, CAMERA_CONFIGS)):
        row = i // GRID_COLS
        col = i % GRID_COLS

        y_start = row * CAMERA_RES[1]
        x_start = col * CAMERA_RES[0]

        # Add frame to grid
        grid[y_start:y_start+CAMERA_RES[1], x_start:x_start+CAMERA_RES[0]] = frame

        # Add label with index and description
        label = f"[{i}] {desc}"
        cv2.putText(grid, label, (x_start + 5, y_start + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # Add border
        cv2.rectangle(grid, (x_start, y_start),
                      (x_start + CAMERA_RES[0] - 1, y_start + CAMERA_RES[1] - 1),
                      (255, 255, 255), 1)

    return grid, frames


# Main execution
print("\n[Init] Creating world...")
world = World(stage_units_in_meters=1.0)
stage = get_current_stage()

# Setup scene with proper lighting
setup_scene_with_color(stage)

# Create test objects
create_test_scene(stage)

# Create cameras
cameras, render_products, rgb_annotators = create_cameras(stage)

# Reset world
print("\n[Init] world.reset()...")
world.reset()

# Start timeline
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Warmup
print("\n[Warmup] Running 100 frames...")
for i in range(100):
    world.step(render=True)
    if i % 20 == 0:
        rep.orchestrator.step(rt_subframes=4)

# Capture grid
descriptions = [cfg[0] for cfg in CAMERA_CONFIGS]
grid, frames = capture_grid(rgb_annotators, descriptions)

# Save grid image
grid_path = f"{OUTPUT_DIR}/orientation_grid.jpg"
cv2.imwrite(grid_path, grid)
print(f"\n[Output] Saved grid: {grid_path}")

# Save individual frames
for i, frame in enumerate(frames):
    frame_path = f"{OUTPUT_DIR}/frame_{i:02d}.jpg"
    cv2.imwrite(frame_path, frame)

print(f"[Output] Saved {len(frames)} individual frames")

# Print summary
print("\n" + "=" * 70)
print("CAMERA ORIENTATION TEST COMPLETE")
print("=" * 70)
print(f"\nGrid image: {grid_path}")
print(f"\nCamera configurations tested:")
for i, (desc, method, params) in enumerate(CAMERA_CONFIGS):
    print(f"  [{i}] {desc}")
print("\nLook for:")
print("  - Pillars should be VERTICAL (not tilted/horizontal)")
print("  - Ground should be at BOTTOM of frame")
print("  - Sky should be at TOP of frame")
print("  - Red cube marker should be visible (forward direction)")
print("=" * 70)

# Cleanup
timeline.stop()
simulation_app.close()
