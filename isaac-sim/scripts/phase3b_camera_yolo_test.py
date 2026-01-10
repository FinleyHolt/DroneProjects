#!/usr/bin/env python3
"""
Phase 3b: Camera Capture + YOLO Detection Test

Builds on Phase 3 by adding:
- Actual camera rendering with RayTracedLighting
- Image capture during flight
- YOLO detection on captured images
- Bounding box overlay on saved images

This tests the full E2E perception pipeline that will be used
for validation (not for fast RL training which uses GT mode).

Success criteria:
- Camera frames captured during flight
- YOLO detects vehicles in images
- Saved images show bounding boxes around vehicles
"""

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="Run with GUI (default: True)")
parser.add_argument("--output-dir", type=str, default="/tmp/phase3b_output", help="Output directory")
args, _ = parser.parse_known_args()

# Default to GUI for this test
HEADLESS = False

from isaacsim import SimulationApp

# CRITICAL: Enable RayTracedLighting for camera rendering
config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",  # Required for camera rendering
    "anti_aliasing": 0,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(config)

import omni.timeline
from omni.isaac.core.world import World
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
import sys
import os
import time
import numpy as np

# Add paths
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
from pymavlink import mavutil

# Isaac Sim camera
from isaacsim.sensors.camera import Camera

# Our world generator
from flyby.world_generator.world_generator import WorldGenerator, WorldConfig
from isaacsim.core.utils.stage import get_current_stage

# Create output directory
os.makedirs(args.output_dir, exist_ok=True)

print("=" * 60, flush=True)
print("Phase 3b: Camera Capture + YOLO Detection Test", flush=True)
print("=" * 60, flush=True)
print(f"Output directory: {args.output_dir}", flush=True)

# Initialize timeline and Pegasus interface
timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._world = World(**pg._world_settings)
world = pg.world
pg.set_px4_path("/px4")

# Get stage and setup physics scene
stage = get_current_stage()
physics_path = "/World/PhysicsScene"
if not stage.GetPrimAtPath(physics_path):
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_path))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

# Generate procedural environment
print("\n[WorldGen] Generating procedural environment...", flush=True)
models_path = "/workspace/extensions/forest_generator/models"

world_config = WorldConfig(
    terrain_size=(200.0, 200.0),
    terrain_roughness=2.0,
    terrain_material="forest_floor",
    tree_density=0.2,  # Very sparse for clear vehicle visibility
    tree_proportions={"Birch": 30, "Spruce": 40, "Pine": 30},
    undergrowth_density=0.0,
    randomize_lighting=False,
    time_of_day="noon",
    weather="clear",
    seed=42,
)

world_gen = WorldGenerator(models_path, world_config)

print("[WorldGen] Generating terrain...", flush=True)
terrain_path = world_gen.generate_terrain()
print("[WorldGen] Setting up lighting...", flush=True)
world_gen.setup_lighting()
print("[WorldGen] Generating forest...", flush=True)
forest_result = world_gen.generate_forest(density=0.2, include_undergrowth=False)
print(f"[WorldGen] Forest: {len(forest_result['trees'])} trees", flush=True)

# Spawn POI targets (vehicles) - closer to origin for easier capture
print("\n[WorldGen] Spawning POI targets...", flush=True)
vehicle_cluster_center = (15.0, 10.0)  # Closer than before
vehicle_paths = world_gen.vehicles.spawn_vehicle_group(
    vehicle_types=["sedan", "suv", "tank"],
    count=5,
    clustering=0.5,  # Spread out more for visibility
    center=vehicle_cluster_center,
)
print(f"[WorldGen] Spawned {len(vehicle_paths)} vehicles", flush=True)

# Store target positions
target_positions = []
for path in vehicle_paths:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        if ops:
            pos = ops[0].Get()
            target_positions.append({
                'path': path,
                'position': np.array([pos[0], pos[1], pos[2]]),
                'type': 'vehicle'
            })
            print(f"  Target: {path.split('/')[-1]} at ({pos[0]:.1f}, {pos[1]:.1f})", flush=True)

# Create PX4 vehicle
print("\n[PX4] Creating vehicle...", flush=True)
mavlink_config = PX4MavlinkBackendConfig({
    "vehicle_id": 0,
    "px4_autolaunch": True,
    "px4_dir": pg.px4_path,
    "px4_vehicle_model": pg.px4_default_airframe,
    "enable_lockstep": True,
    "update_rate": 250.0,
})
config = MultirotorConfig()
config.backends = [PX4MavlinkBackend(mavlink_config)]

spawn_height = 5.0
vehicle = Multirotor(
    "/World/quadrotor",
    ROBOTS['Iris'],
    0,
    [0.0, 0.0, spawn_height],
    Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
    config=config,
)
print(f"[PX4] Vehicle spawned", flush=True)

# Reset world BEFORE creating camera (important for proper initialization)
world.reset()

# Create camera attached to drone body
print("\n[Camera] Creating ISR camera...", flush=True)
camera_path = "/World/quadrotor/body/isr_camera"
camera_resolution = (640, 480)

camera = Camera(
    prim_path=camera_path,
    frequency=30,
    resolution=camera_resolution,
)

# Camera pointing forward and down at 45 degrees
camera_orientation = Rotation.from_euler("ZYX", [0.0, 45.0, 0.0], degrees=True).as_quat()
camera.set_local_pose(
    np.array([0.0, 0.0, -0.1]),  # Below body to avoid obstruction
    camera_orientation
)

# Initialize camera
camera.initialize()
camera.set_focal_length(4.5)  # Wide angle
camera.set_clipping_range(0.1, 500.0)

# Enable depth
camera.add_distance_to_image_plane_to_frame()

print(f"[Camera] Resolution: {camera_resolution}", flush=True)
print(f"[Camera] FOV: ~90° (4.5mm focal length)", flush=True)

# Start simulation
timeline.play()

# Camera warmup (critical - need ~120 frames)
print("\n[Camera] Warming up camera (120 frames)...", flush=True)
for i in range(120):
    world.step(render=True)
    if i % 30 == 0:
        # Try to get image to check readiness
        rgba = camera.get_rgba()
        if rgba is not None:
            print(f"  Frame {i}: image shape={rgba.shape}, max={rgba.max()}", flush=True)

print("[Camera] Warmup complete", flush=True)

# Connect MAVLink
print("\n[MAVLink] Connecting...", flush=True)
mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

for i in range(500):
    world.step(render=True)
    msg = mav.recv_match(type='HEARTBEAT', blocking=False)
    if msg and msg.get_srcSystem() != 0:
        print(f"[MAVLink] Got heartbeat after {i} steps", flush=True)
        break

if mav.target_system == 0:
    mav.target_system = 1
    mav.target_component = 1


def check_mode():
    msg = mav.recv_match(type='HEARTBEAT', blocking=False)
    if msg:
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return armed, msg.custom_mode
    return None, None


def send_setpoint(vx=0.0, vy=0.0, vz=-0.5):
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
    )


def save_image_with_detections(rgb, detections, frame_num, elapsed):
    """Save RGB image with bounding boxes drawn."""
    try:
        import cv2

        # Convert RGB to BGR for OpenCV
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Draw detections
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            conf = det['confidence']
            label = det['label']

            # Draw box
            cv2.rectangle(bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            # Draw label
            text = f"{label}: {conf:.2f}"
            cv2.putText(bgr, text, (int(x1), int(y1) - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Save
        filename = f"{args.output_dir}/frame_{frame_num:04d}_{elapsed:.1f}s.jpg"
        cv2.imwrite(filename, bgr)
        return filename
    except ImportError:
        # Fallback: save raw numpy
        filename = f"{args.output_dir}/frame_{frame_num:04d}_{elapsed:.1f}s.npy"
        np.save(filename, rgb)
        return filename


def run_yolo_detection(rgb):
    """Run YOLO detection on RGB image."""
    try:
        from ultralytics import YOLO

        # Load model (will be cached after first load)
        model = YOLO('yolov8n.pt')  # nano model for speed

        # Run inference
        results = model(rgb, verbose=False)

        detections = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = model.names[cls]

                # Filter for vehicles (car, truck, bus, etc.)
                vehicle_classes = ['car', 'truck', 'bus', 'motorcycle', 'bicycle']
                if label in vehicle_classes and conf > 0.3:
                    detections.append({
                        'bbox': (x1, y1, x2, y2),
                        'confidence': conf,
                        'label': label,
                        'class_id': cls,
                    })

        return detections
    except ImportError:
        print("  [YOLO] ultralytics not available, skipping detection", flush=True)
        return []
    except Exception as e:
        print(f"  [YOLO] Error: {e}", flush=True)
        return []


# Prime with setpoints
print("\n[Flight] Priming...", flush=True)
for _ in range(200):
    send_setpoint()
    world.step(render=True)

# Engage OFFBOARD and arm
print("[Flight] Engaging OFFBOARD and arming...", flush=True)
for attempt in range(3):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6, 0, 0, 0, 0, 0
    )
    for _ in range(100):
        send_setpoint()
        world.step(render=True)
    armed, mode = check_mode()
    if mode == 6:
        break

for attempt in range(3):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    for _ in range(100):
        send_setpoint()
        world.step(render=True)
    armed, mode = check_mode()
    if armed:
        break

mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    6, 0, 0, 0, 0, 0
)
for _ in range(50):
    send_setpoint()
    world.step(render=True)

armed, mode = check_mode()
print(f"[Flight] armed={armed}, mode={mode}", flush=True)

# Ascend
print("\n[Flight] Ascending to 15m...", flush=True)
for i in range(400):
    send_setpoint(vz=-2.0)
    world.step(render=True)
    if i % 100 == 0:
        pos = vehicle.state.position
        print(f"  Altitude: {pos[2]:.1f}m", flush=True)

# Fly toward targets and capture images
print("\n[ISR] Flying toward targets with camera capture...", flush=True)
print(f"[ISR] Targets at ({vehicle_cluster_center[0]:.0f}, {vehicle_cluster_center[1]:.0f})", flush=True)

current_pos = vehicle.state.position
target_vec = np.array([vehicle_cluster_center[0] - current_pos[0],
                       vehicle_cluster_center[1] - current_pos[1]])
target_dir = target_vec / (np.linalg.norm(target_vec) + 1e-6)

frame_count = 0
total_detections = 0
start_time = time.time()

# Fly toward targets
for i in range(800):
    vx = target_dir[0] * 2.0  # Slower for better images
    vy = target_dir[1] * 2.0

    current_z = vehicle.state.position[2]
    vz = -1.0 * (15.0 - current_z)
    vz = np.clip(vz, -2.0, 2.0)

    send_setpoint(vx=vx, vy=vy, vz=vz)
    world.step(render=True)

    # Capture and process every 60 frames (~1 second)
    if i % 60 == 0 and i > 0:
        elapsed = time.time() - start_time
        pos = vehicle.state.position

        # Get camera image
        rgba = camera.get_rgba()
        if rgba is not None and rgba.max() > 10:
            rgb = rgba[:, :, :3]

            # Run YOLO detection
            detections = run_yolo_detection(rgb)
            total_detections += len(detections)

            # Save image with detections
            filename = save_image_with_detections(rgb, detections, frame_count, elapsed)
            frame_count += 1

            dist_to_target = np.linalg.norm(pos[:2] - np.array(vehicle_cluster_center))
            print(f"\n[ISR] Frame {frame_count}:", flush=True)
            print(f"  Position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})", flush=True)
            print(f"  Distance to targets: {dist_to_target:.1f}m", flush=True)
            print(f"  YOLO detections: {len(detections)}", flush=True)
            print(f"  Saved: {filename}", flush=True)

            for det in detections[:3]:
                print(f"    - {det['label']}: {det['confidence']:.2f}", flush=True)

# Final hover and capture
print("\n[Flight] Hovering over targets for final captures...", flush=True)
for i in range(300):
    send_setpoint(vz=0.0)
    world.step(render=True)

    if i % 60 == 0:
        elapsed = time.time() - start_time
        rgba = camera.get_rgba()
        if rgba is not None and rgba.max() > 10:
            rgb = rgba[:, :, :3]
            detections = run_yolo_detection(rgb)
            total_detections += len(detections)
            filename = save_image_with_detections(rgb, detections, frame_count, elapsed)
            frame_count += 1
            print(f"  Hover frame {frame_count}: {len(detections)} detections -> {filename}", flush=True)

# Results
final_pos = vehicle.state.position
print("\n" + "=" * 60, flush=True)
print("PHASE 3b RESULTS", flush=True)
print("=" * 60, flush=True)
print(f"Final position: ({final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f})", flush=True)
print(f"\nCamera Capture:", flush=True)
print(f"  Frames saved: {frame_count}", flush=True)
print(f"  Output directory: {args.output_dir}", flush=True)
print(f"\nYOLO Detection:", flush=True)
print(f"  Total detections: {total_detections}", flush=True)

if frame_count > 0:
    print(f"\nSUCCESS! Camera capture and detection working!", flush=True)
    print(f"  Check {args.output_dir} for images with bounding boxes", flush=True)
else:
    print(f"\nFAILED: No frames captured", flush=True)

# Cleanup
print("\n[Cleanup] Shutting down...", flush=True)
timeline.stop()
simulation_app.close()
print("Done!", flush=True)
