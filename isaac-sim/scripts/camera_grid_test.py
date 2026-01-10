#!/usr/bin/env python3
"""
Camera Orientation Grid Test - Simplified
Creates multiple camera snapshots to find correct forward-looking orientation.
"""

import numpy as np
import sys
import os

# Isaac Sim startup
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni
from pxr import Gf, UsdGeom
from isaacsim.core.api import World
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.sensors.camera import Camera
import omni.replicator.core as rep
from PIL import Image, ImageDraw

# Output directory
OUTPUT_DIR = "/workspace/output/camera_grid_test"
os.makedirs(OUTPUT_DIR, exist_ok=True)

print("=" * 70)
print("CAMERA ORIENTATION GRID TEST")
print("=" * 70)

# Create world
world = World(stage_units_in_meters=1.0)
stage = get_current_stage()

# Ground plane
ground_path = "/World/Ground"
ground_prim = stage.DefinePrim(ground_path, "Cube")
xform = UsdGeom.Xformable(ground_prim)
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
xform.AddScaleOp().Set(Gf.Vec3d(200, 200, 1))
# Make ground green-ish
ground_prim.GetAttribute("primvars:displayColor").Set([(0.2, 0.5, 0.2)])

# Create colored pillars at known positions for orientation reference
pillar_configs = [
    (40, 0, "RED - FRONT"),      # Directly ahead
    (40, 20, "BLUE - LEFT"),     # Left of center
    (40, -20, "YELLOW - RIGHT"), # Right of center
    (60, 0, "WHITE - FAR"),      # Far ahead
]

colors = [(0.8, 0.1, 0.1), (0.1, 0.1, 0.8), (0.8, 0.8, 0.1), (0.9, 0.9, 0.9)]

for i, ((x, y, name), color) in enumerate(zip(pillar_configs, colors)):
    pillar_path = f"/World/Pillar_{i}"
    pillar_prim = stage.DefinePrim(pillar_path, "Cylinder")
    pxform = UsdGeom.Xformable(pillar_prim)
    pxform.AddTranslateOp().Set(Gf.Vec3d(x, y, 25))
    pxform.AddScaleOp().Set(Gf.Vec3d(3, 3, 50))
    pillar_prim.GetAttribute("primvars:displayColor").Set([color])
    print(f"  Pillar {i}: {name} at ({x}, {y})")

# Sky/dome light
light_path = "/World/DomeLight"
light_prim = stage.DefinePrim(light_path, "DomeLight")
light_prim.GetAttribute("inputs:intensity").Set(1500.0)

print("[Scene] Created ground and reference pillars")

# Drone position: at origin, 30m up, facing +X
drone_pos = np.array([0.0, 0.0, 30.0])
camera_offset = np.array([0.5, 0.0, -0.2])

# Initialize
world.reset()
for _ in range(50):
    simulation_app.update()

print("[Render] Warming up...")
for _ in range(100):
    simulation_app.update()

# Test orientations - focusing on what might be "forward looking with slight down pitch"
import isaacsim.core.utils.numpy.rotations as rot_utils

# Format: (roll, pitch, yaw, description)
# Isaac Sim uses right-handed coordinate system
# +X is forward, +Y is left, +Z is up
orientations = [
    # Row 1: Pitch variations (looking up/down)
    (0, -30, 0, "pitch=-30 (down)"),
    (0, -15, 0, "pitch=-15 (slight down)"),
    (0, 0, 0, "pitch=0 (level)"),
    (0, 15, 0, "pitch=+15 (slight up)"),

    # Row 2: More pitch
    (0, 30, 0, "pitch=+30 (up)"),
    (0, 45, 0, "pitch=+45 (more up)"),
    (0, -45, 0, "pitch=-45 (more down)"),
    (0, -60, 0, "pitch=-60 (looking at ground)"),

    # Row 3: Yaw variations (turning left/right)
    (0, 0, -30, "yaw=-30 (turn right)"),
    (0, 0, 30, "yaw=+30 (turn left)"),
    (0, 15, 0, "pitch=+15 BASELINE"),
    (0, -15, 0, "pitch=-15 BASELINE"),

    # Row 4: Combined and roll
    (0, 15, 15, "P+15 Y+15"),
    (0, 15, -15, "P+15 Y-15"),
    (15, 0, 0, "roll=+15"),
    (-15, 0, 0, "roll=-15"),
]

print(f"\n[Test] Capturing {len(orientations)} camera orientations...")

captured_images = []

for idx, (roll, pitch, yaw, desc) in enumerate(orientations):
    camera_path = f"/World/Camera_{idx}"
    camera_pos = drone_pos + camera_offset

    euler = np.array([roll, pitch, yaw])
    orient_quat = rot_utils.euler_angles_to_quats(euler, degrees=True)

    print(f"[{idx:2d}] {desc}")

    # Create camera
    camera = Camera(
        prim_path=camera_path,
        position=camera_pos,
        frequency=30,
        resolution=(640, 480),
        orientation=orient_quat,
    )

    camera.initialize()
    simulation_app.update()
    camera.initialize()
    simulation_app.update()

    # Wide FOV
    camera.set_focal_length(4.0)
    camera.set_clipping_range(0.1, 500.0)

    camera_prim = stage.GetPrimAtPath(camera_path)
    if camera_prim.IsValid():
        usd_camera = UsdGeom.Camera(camera_prim)
        if usd_camera:
            usd_camera.GetHorizontalApertureAttr().Set(50.0)

    # Render product
    rp = rep.create.render_product(camera_path, (640, 480))
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([rp])

    # Render
    for _ in range(20):
        simulation_app.update()
    rep.orchestrator.step(rt_subframes=4)

    # Capture
    rgb_data = rgb_annot.get_data()
    if rgb_data is not None:
        if isinstance(rgb_data, dict):
            rgb_data = rgb_data.get('data', rgb_data)
        rgb_array = np.array(rgb_data)
        if len(rgb_array.shape) == 3 and rgb_array.shape[2] >= 3:
            rgb_array = rgb_array[:, :, :3]

            img = Image.fromarray(rgb_array.astype(np.uint8))
            draw = ImageDraw.Draw(img)

            # Draw info
            text = f"[{idx}] R={roll} P={pitch} Y={yaw}"
            text2 = desc

            # Black outline
            for dx, dy in [(-1,-1), (-1,1), (1,-1), (1,1), (0,-1), (0,1), (-1,0), (1,0)]:
                draw.text((10+dx, 10+dy), text, fill=(0,0,0))
                draw.text((10+dx, 30+dy), text2, fill=(0,0,0))
            draw.text((10, 10), text, fill=(255, 255, 0))
            draw.text((10, 30), text2, fill=(0, 255, 255))

            # What we expect to see
            draw.text((10, 450), "Expect: RED pillar center, BLUE left, YELLOW right", fill=(255,255,255))

            img_path = f"{OUTPUT_DIR}/cam_{idx:02d}.png"
            img.save(img_path)
            captured_images.append(img)
            print(f"     Saved: cam_{idx:02d}.png")

    # Cleanup
    rgb_annot.detach([rp])
    rp.destroy()

# Create grid
print("\n[Grid] Creating 4x4 composite...")
if len(captured_images) >= 16:
    grid = Image.new('RGB', (640*4, 480*4))
    for i, img in enumerate(captured_images[:16]):
        row, col = i // 4, i % 4
        grid.paste(img, (col*640, row*480))
    grid.save(f"{OUTPUT_DIR}/grid.png")
    print(f"[Grid] Saved: grid.png")

print("\n" + "=" * 70)
print("DONE - Check output/camera_grid_test/grid.png")
print("=" * 70)

# Don't call close() - just let it exit to avoid the crash
