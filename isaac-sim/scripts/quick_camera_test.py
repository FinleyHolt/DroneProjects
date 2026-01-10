#!/usr/bin/env python3
"""Quick camera test - using explicit Replicator API for headless capture.

The Camera class internal methods (get_rgba, get_current_frame) return None in headless mode.
We need to use the explicit omni.replicator API with render products and annotators.
"""

import numpy as np

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
from isaacsim.sensors.camera import Camera
from pxr import UsdGeom, UsdLux, Gf
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.utils.stage import get_current_stage
import omni.isaac.core.utils.prims as prim_utils

print("\n[1] Creating world...", flush=True)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Pre-warmup (10 frames) - required for renderer initialization
print("[2] Pre-warmup (10 frames)...", flush=True)
for _ in range(10):
    simulation_app.update()

# Add visible objects
print("[3] Creating scene objects...", flush=True)
prim_utils.create_prim(
    prim_path="/World/Cube",
    prim_type="Cube",
    position=np.array([5.0, 0.0, 1.0]),
    scale=np.array([2.0, 2.0, 2.0]),
)

# Setup lighting (REQUIRED for RGB in headless mode)
print("[3b] Setting up lighting...", flush=True)
stage = get_current_stage()

# Dome light - ambient sky illumination
dome_light = UsdLux.DomeLight.Define(stage, "/World/Lighting/DomeLight")
dome_light.CreateIntensityAttr(1000.0)
dome_light.CreateColorAttr(Gf.Vec3f(0.9, 0.95, 1.0))
print("  Created DomeLight", flush=True)

# Distant light (sun) - primary directional light
sun_light = UsdLux.DistantLight.Define(stage, "/World/Lighting/SunLight")
sun_light.CreateIntensityAttr(3000.0)
sun_light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
sun_xform = UsdGeom.Xformable(sun_light.GetPrim())
sun_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))
print("  Created DistantLight (sun)", flush=True)

# Create camera BEFORE world.reset()
print("[4] Creating camera BEFORE world.reset()...", flush=True)
CAMERA_PATH = "/World/isr_camera"
RESOLUTION = (640, 480)

camera = Camera(
    prim_path=CAMERA_PATH,
    position=np.array([0.0, 0.0, 5.0]),
    frequency=30,
    resolution=RESOLUTION,
    orientation=rot_utils.euler_angles_to_quats(np.array([90, 0, 0]), degrees=True),
)

# Double initialization (CRITICAL for Isaac Sim 5.x)
print("  Double initialization...", flush=True)
camera.initialize()
simulation_app.update()
camera.initialize()
simulation_app.update()

# Set horizontal aperture for wider FOV
camera_prim = stage.GetPrimAtPath(CAMERA_PATH)
if camera_prim.IsValid():
    usd_camera = UsdGeom.Camera(camera_prim)
    if usd_camera:
        usd_camera.GetHorizontalApertureAttr().Set(42.0)
        print("  Set horizontal aperture to 42.0mm", flush=True)

# EXPLICIT REPLICATOR API - this is what works in headless mode
print("[4b] Setting up explicit Replicator render product and annotators...", flush=True)
try:
    # Disable automatic capture on play - we want manual control
    rep.orchestrator.set_capture_on_play(False)

    # Create render product from camera prim path
    render_product = rep.create.render_product(CAMERA_PATH, RESOLUTION)
    print(f"  Created render product for {CAMERA_PATH}", flush=True)

    # Create and attach RGB annotator
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach([render_product])
    print("  Attached RGB annotator", flush=True)

    # Create and attach depth annotator
    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth_annotator.attach([render_product])
    print("  Attached depth annotator (distance_to_image_plane)", flush=True)

except Exception as e:
    print(f"  Replicator setup error: {e}", flush=True)
    import traceback
    traceback.print_exc()
    render_product = None
    rgb_annotator = None
    depth_annotator = None

# NOW call world.reset() - AFTER camera is set up
print("[5] world.reset() (AFTER camera setup)...", flush=True)
world.reset()

# timeline.play() comes last
print("[6] timeline.play()...", flush=True)
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Extended warmup (200 frames) - cameras need substantial warmup in headless
print("[7] Extended warmup (200 frames)...", flush=True)
for i in range(200):
    world.step(render=True)
    if i % 50 == 0:
        print(f"  Frame {i}...", flush=True)

print("\n[8] Testing camera capture with Replicator API...", flush=True)
success_count = 0
depth_success_count = 0


def capture_with_replicator():
    """Capture RGB and depth using Replicator annotators."""
    rgb_data = None
    depth_data = None

    if rgb_annotator is not None:
        try:
            # Step the orchestrator to ensure annotators are updated
            rep.orchestrator.step(rt_subframes=0, pause_timeline=False)

            rgb_data = rgb_annotator.get_data()
        except Exception as e:
            print(f"  RGB annotator error: {e}", flush=True)

    if depth_annotator is not None:
        try:
            depth_data = depth_annotator.get_data()
        except Exception as e:
            print(f"  Depth annotator error: {e}", flush=True)

    return rgb_data, depth_data


for i in range(50):
    world.step(render=True)

    if i % 10 == 0:
        print(f"\n  Step {i}:", flush=True)

        # Get data via Replicator
        rgb, depth = capture_with_replicator()

        # Check RGB
        if rgb is not None and rgb.size > 0:
            print(f"    RGB: shape={rgb.shape}, dtype={rgb.dtype}, max={rgb.max()}, min={rgb.min()}", flush=True)
            if rgb.max() > 0:
                print(f"    RGB SUCCESS!", flush=True)
                success_count += 1
            else:
                print(f"    RGB: max=0 (black frame)", flush=True)
        else:
            print(f"    RGB: None or empty (shape={rgb.shape if rgb is not None else 'None'})", flush=True)

        # Check Depth
        if depth is not None and depth.size > 0:
            # Handle shape: may be (H, W) or (H, W, 1)
            if len(depth.shape) == 3 and depth.shape[-1] == 1:
                depth = depth.squeeze(-1)

            valid_mask = (depth > 0.1) & (depth < 500.0)
            if valid_mask.sum() > 0:
                print(f"    DEPTH: shape={depth.shape}, min={depth[valid_mask].min():.2f}m, max={depth[valid_mask].max():.2f}m", flush=True)
                print(f"    DEPTH SUCCESS!", flush=True)
                depth_success_count += 1
            else:
                print(f"    DEPTH: shape={depth.shape}, no valid values in range [0.1, 500]", flush=True)
        else:
            print(f"    DEPTH: None or empty", flush=True)

print(f"\n[9] Results: RGB {success_count}/5 successful, Depth {depth_success_count}/5 successful", flush=True)
if success_count >= 4:
    print("CAMERA TEST PASSED!", flush=True)
else:
    print("CAMERA TEST FAILED - Replicator API may need additional configuration", flush=True)

# Cleanup
print("\n[10] Cleanup...", flush=True)
try:
    if rgb_annotator:
        rgb_annotator.detach()
    if depth_annotator:
        depth_annotator.detach()
    if render_product:
        render_product.destroy()
    rep.orchestrator.wait_until_complete()
except Exception as e:
    print(f"  Cleanup warning: {e}", flush=True)

timeline.stop()

try:
    simulation_app.close()
except:
    pass
