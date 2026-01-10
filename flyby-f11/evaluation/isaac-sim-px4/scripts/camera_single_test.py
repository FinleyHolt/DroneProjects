#!/usr/bin/env python3
"""
Single Camera Test - Just verify camera capture works
"""

import numpy as np
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True, "width": 640, "height": 480})

from pxr import Gf, UsdGeom
from isaacsim.core.api import World
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.sensors.camera import Camera
import omni.replicator.core as rep
from PIL import Image, ImageDraw
import isaacsim.core.utils.numpy.rotations as rot_utils

OUTPUT_DIR = "/workspace/output/camera_grid_test"
os.makedirs(OUTPUT_DIR, exist_ok=True)

print("=" * 60)
print("SINGLE CAMERA TEST")
print("=" * 60)

world = World(stage_units_in_meters=1.0)
stage = get_current_stage()

# Ground
ground = stage.DefinePrim("/World/Ground", "Cube")
UsdGeom.Xformable(ground).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
UsdGeom.Xformable(ground).AddScaleOp().Set(Gf.Vec3d(200, 200, 1))
ground.GetAttribute("primvars:displayColor").Set([(0.2, 0.5, 0.2)])

# Pillars
for i, (x, y, color) in enumerate([
    (40, 0, (0.8, 0.1, 0.1)),    # RED center
    (40, 20, (0.1, 0.1, 0.8)),   # BLUE left
    (40, -20, (0.8, 0.8, 0.1)),  # YELLOW right
]):
    p = stage.DefinePrim(f"/World/Pillar_{i}", "Cylinder")
    UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(x, y, 25))
    UsdGeom.Xformable(p).AddScaleOp().Set(Gf.Vec3d(3, 3, 50))
    p.GetAttribute("primvars:displayColor").Set([color])

# Light
light = stage.DefinePrim("/World/Light", "DomeLight")
light.GetAttribute("inputs:intensity").Set(1500.0)

print("[Scene] Created")

world.reset()
for _ in range(100):
    simulation_app.update()

print("[Render] Warmed up")

# Test orientations
tests = [
    (0, 0, 0, "level"),
    (0, 15, 0, "pitch+15"),
    (0, -15, 0, "pitch-15"),
    (0, 30, 0, "pitch+30"),
]

for roll, pitch, yaw, name in tests:
    cam_path = f"/World/Cam_{name}"
    euler = np.array([roll, pitch, yaw])
    quat = rot_utils.euler_angles_to_quats(euler, degrees=True)

    cam = Camera(
        prim_path=cam_path,
        position=np.array([0.5, 0.0, 29.8]),
        resolution=(640, 480),
        orientation=quat,
    )
    cam.initialize()
    simulation_app.update()
    cam.initialize()
    simulation_app.update()

    cam.set_focal_length(4.0)
    cp = stage.GetPrimAtPath(cam_path)
    if cp.IsValid():
        UsdGeom.Camera(cp).GetHorizontalApertureAttr().Set(50.0)

    rp = rep.create.render_product(cam_path, (640, 480))
    ann = rep.AnnotatorRegistry.get_annotator("rgb")
    ann.attach([rp])

    for _ in range(30):
        simulation_app.update()
    rep.orchestrator.step(rt_subframes=4)

    data = ann.get_data()
    if data is not None:
        if isinstance(data, dict):
            data = data.get('data', data)
        arr = np.array(data)[:, :, :3]
        img = Image.fromarray(arr.astype(np.uint8))
        draw = ImageDraw.Draw(img)
        txt = f"R={roll} P={pitch} Y={yaw} ({name})"
        for dx, dy in [(-1,-1),(-1,1),(1,-1),(1,1)]:
            draw.text((10+dx, 10+dy), txt, fill=(0,0,0))
        draw.text((10, 10), txt, fill=(255, 255, 0))
        draw.text((10, 450), "RED=front, BLUE=left, YELLOW=right", fill=(255,255,255))

        img.save(f"{OUTPUT_DIR}/{name}.png")
        print(f"[OK] Saved {name}.png")

    ann.detach([rp])
    rp.destroy()

print("=" * 60)
print("DONE - Check output/camera_grid_test/")
print("=" * 60)
