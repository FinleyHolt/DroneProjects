#!/usr/bin/env python3
"""
Phase 3c: Live YOLO Inference in Simulation Loop

This test demonstrates the full perception pipeline with real YOLO inference
running on viewport-captured images during drone flight.

Architecture:
1. ViewportCamera captures images from drone's perspective
2. YOLODetector runs inference on each frame
3. Detections are logged to JSONL for post-processing
4. Perception observations (516-dim) are generated for RL

This is the E2E validation path - proving that YOLO inference works
in the simulation loop before full training runs.

Success criteria:
- YOLO detections on vehicles in the procedural forest
- Detection logging to JSONL with frame timestamps
- Perception observation vectors generated
- Optional: Frame recording for post-processing visualization
"""

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", default=True, help="Run with GUI")
parser.add_argument("--output-dir", type=str, default="/tmp/phase3c_yolo", help="Output directory")
parser.add_argument("--save-frames", action="store_true", help="Save all frames to disk")
parser.add_argument("--yolo-model", type=str, default="yolo11n.pt", help="YOLO model path")
args, _ = parser.parse_known_args()

HEADLESS = not args.gui

from isaacsim import SimulationApp

config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 0,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(config)

import omni
import omni.timeline
import omni.kit.viewport.utility as viewport_utils
from omni.isaac.core.world import World
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
import sys
import os
import time
import numpy as np
import json

# Add paths
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

PERCEPTION_PATH = "/workspace/perception"
if PERCEPTION_PATH not in sys.path:
    sys.path.insert(0, PERCEPTION_PATH)

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
from pymavlink import mavutil

from flyby.world_generator.world_generator import WorldGenerator, WorldConfig
from isaacsim.core.utils.stage import get_current_stage

# Import perception pipeline
try:
    from perception.viewport_camera import ViewportCamera, ViewportCameraConfig, DetectionLogger
    from perception.detector import YOLODetector, Detection
    from perception.perception_encoder import PerceptionEncoder, PerceptionConfig
    PERCEPTION_AVAILABLE = True
    print("[YOLO] Perception modules loaded successfully")
except ImportError as e:
    print(f"[YOLO] Warning: Perception modules not fully available: {e}")
    PERCEPTION_AVAILABLE = False

# Create output directory
os.makedirs(args.output_dir, exist_ok=True)

print("=" * 60, flush=True)
print("Phase 3c: Live YOLO Inference Test", flush=True)
print("=" * 60, flush=True)
print(f"Output directory: {args.output_dir}", flush=True)
print(f"YOLO model: {args.yolo_model}", flush=True)
print(f"Save frames: {args.save_frames}", flush=True)

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

# Generate procedural environment with vehicles
print("\n[WorldGen] Generating procedural environment...", flush=True)
models_path = "/workspace/extensions/forest_generator/models"

world_config = WorldConfig(
    terrain_size=(200.0, 200.0),
    terrain_roughness=2.0,
    terrain_material="forest_floor",
    tree_density=0.15,  # Sparser trees so vehicles are visible
    tree_proportions={"Birch": 30, "Spruce": 40, "Pine": 30},
    undergrowth_density=0.0,
    randomize_lighting=False,
    time_of_day="noon",
    weather="clear",
    seed=42,
)

world_gen = WorldGenerator(models_path, world_config)
world_gen.generate_terrain()
world_gen.setup_lighting()
forest_result = world_gen.generate_forest(density=0.15, include_undergrowth=False)
print(f"[WorldGen] Forest: {len(forest_result['trees'])} trees", flush=True)

# Spawn vehicles as detection targets
print("\n[WorldGen] Spawning vehicle targets...", flush=True)
vehicle_cluster_centers = [
    (15.0, 10.0),
    (-10.0, 20.0),
    (25.0, -15.0),
]
all_vehicle_paths = []
all_vehicle_positions = []

for i, center in enumerate(vehicle_cluster_centers):
    paths = world_gen.vehicles.spawn_vehicle_group(
        vehicle_types=["sedan", "suv", "tank"],
        count=3,
        clustering=0.4,
        center=center,
    )
    all_vehicle_paths.extend(paths)
    # Record approximate positions
    for j in range(3):
        all_vehicle_positions.append(np.array([center[0] + np.random.uniform(-5, 5),
                                                center[1] + np.random.uniform(-5, 5),
                                                0.5]))

print(f"[WorldGen] Spawned {len(all_vehicle_paths)} vehicles at 3 clusters", flush=True)

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

# Reset world
world.reset()

# Initialize ViewportCamera
print("\n[Perception] Setting up viewport camera...", flush=True)
camera_prim_path = "/World/ISRCamera"
camera_config = ViewportCameraConfig(
    resolution=(640, 480),
    mount_offset=(0.0, 0.0, -0.1),
    pitch_down_degrees=45.0,
    capture_to_file=args.save_frames,
    output_dir=args.output_dir,
    log_detections=True,
    detection_log_path=f"{args.output_dir}/detections.jsonl",
)

# Create camera using USD (not Isaac Camera API to avoid bugs)
camera_prim = UsdGeom.Camera.Define(stage, camera_prim_path)
camera_prim.CreateFocalLengthAttr(4.5)
camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.1, 500.0))
print(f"[Perception] Created camera at {camera_prim_path}", flush=True)

# Set viewport to use our camera
try:
    viewport_api = viewport_utils.get_active_viewport()
    if viewport_api:
        viewport_api.set_active_camera(camera_prim_path)
        print("[Perception] Set viewport camera", flush=True)
except Exception as e:
    print(f"[Perception] Warning: Could not set viewport camera: {e}", flush=True)

# Initialize YOLO detector
yolo_detector = None
perception_encoder = None

if PERCEPTION_AVAILABLE:
    try:
        yolo_detector = YOLODetector(mode="inference")
        perception_encoder = PerceptionEncoder(PerceptionConfig())
        print(f"[YOLO] Detector initialized (model: {args.yolo_model})", flush=True)
        print(f"[YOLO] Perception encoder ready (output: {perception_encoder.output_dim} dims)", flush=True)
    except Exception as e:
        print(f"[YOLO] Warning: Could not initialize detector: {e}", flush=True)

# Initialize detection logger
detection_logger = DetectionLogger(
    log_dir=args.output_dir,
    episode_id="phase3c_test"
)
print(f"[Logging] Detection logger initialized", flush=True)

# Start simulation
timeline.play()

# Warmup
print("\n[Sim] Warming up (100 frames)...", flush=True)
for i in range(100):
    world.step(render=True)

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


def update_camera_pose(drone_pos, drone_orient):
    """Update the ISR camera to follow the drone."""
    camera_xform = UsdGeom.Xformable(stage.GetPrimAtPath(camera_prim_path))
    camera_xform.ClearXformOpOrder()

    camera_xform.AddTranslateOp().Set(Gf.Vec3d(drone_pos[0], drone_pos[1], drone_pos[2] - 0.1))

    drone_rot = Rotation.from_quat(drone_orient)
    drone_euler = drone_rot.as_euler('ZYX', degrees=True)
    drone_yaw = drone_euler[0]

    # Camera looks forward and down at 45 degrees
    camera_rot = Rotation.from_euler("ZX", [drone_yaw, 45.0], degrees=True)
    quat = camera_rot.as_quat()
    camera_xform.AddOrientOp().Set(Gf.Quatf(quat[3], quat[0], quat[1], quat[2]))


def capture_and_detect(frame_num, uav_pos, uav_orient, uav_vel):
    """Capture viewport image and run YOLO detection."""
    try:
        # Capture viewport to temp file
        filename = f"/tmp/yolo_capture_{frame_num % 5}.png"
        viewport_utils.capture_viewport_to_file(viewport_api, filename)

        # Wait for file to be written
        for _ in range(5):
            world.step(render=True)

        if not os.path.exists(filename):
            return None, []

        # Load image
        from PIL import Image
        img = Image.open(filename)
        image = np.array(img)[:, :, :3]

        # Run YOLO if available
        detections = []
        if yolo_detector is not None:
            detections = yolo_detector.detect(image, uav_position=uav_pos)

        # Log detections
        detection_logger.log_frame(
            frame_id=frame_num,
            timestamp=time.time(),
            uav_position=uav_pos,
            uav_orientation=uav_orient,
            uav_velocity=uav_vel,
            detections=detections,
            perception_mode="full"
        )

        return image, detections

    except Exception as e:
        print(f"  [YOLO] Capture/detect error: {e}", flush=True)
        return None, []


# Prime and arm
print("\n[Flight] Priming and arming...", flush=True)
for _ in range(200):
    send_setpoint()
    world.step(render=True)

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
print("\n[Flight] Ascending to 20m...", flush=True)
for i in range(500):
    send_setpoint(vz=-2.0)
    world.step(render=True)

    pos = vehicle.state.position
    orient = vehicle.state.attitude
    update_camera_pose(pos, orient)

    if i % 100 == 0:
        print(f"  Altitude: {pos[2]:.1f}m", flush=True)

# ISR flight with live YOLO
print("\n" + "=" * 60, flush=True)
print("ISR FLIGHT WITH LIVE YOLO INFERENCE", flush=True)
print("=" * 60, flush=True)

start_time = time.time()
frame_count = 0
total_detections = 0
yolo_times = []

# Visit each vehicle cluster
for cluster_idx, target_center in enumerate(vehicle_cluster_centers):
    print(f"\n[ISR] Flying to cluster {cluster_idx + 1}: ({target_center[0]:.0f}, {target_center[1]:.0f})", flush=True)

    # Fly toward target
    for i in range(300):
        pos = vehicle.state.position
        orient = vehicle.state.attitude
        vel = vehicle.state.linear_velocity

        # Compute direction to target
        target_vec = np.array([target_center[0] - pos[0], target_center[1] - pos[1]])
        dist_to_target = np.linalg.norm(target_vec)
        target_dir = target_vec / (dist_to_target + 1e-6)

        vx = target_dir[0] * 3.0
        vy = target_dir[1] * 3.0

        # Maintain altitude
        vz = -1.0 * (20.0 - pos[2])
        vz = np.clip(vz, -2.0, 2.0)

        send_setpoint(vx=vx, vy=vy, vz=vz)
        world.step(render=True)

        # Update camera
        update_camera_pose(pos, orient)

        # Run YOLO every 50 frames (~0.8 seconds)
        if i % 50 == 0:
            yolo_start = time.perf_counter()
            image, detections = capture_and_detect(frame_count, pos, orient, vel)
            yolo_elapsed = (time.perf_counter() - yolo_start) * 1000

            if image is not None:
                frame_count += 1
                total_detections += len(detections)
                yolo_times.append(yolo_elapsed)

                if detections:
                    print(f"\n  [YOLO] Frame {frame_count}: {len(detections)} detections ({yolo_elapsed:.1f}ms)", flush=True)
                    for det in detections[:3]:  # Show top 3
                        print(f"    - {det.class_name}: conf={det.confidence:.2f}, dist={det.distance:.1f}m", flush=True)
                else:
                    print(f"  [YOLO] Frame {frame_count}: No detections ({yolo_elapsed:.1f}ms)", flush=True)

        # Check if close enough
        if dist_to_target < 5.0:
            print(f"  Reached cluster {cluster_idx + 1}", flush=True)
            break

    # Hover over cluster and capture more frames
    print(f"  Hovering over cluster {cluster_idx + 1}...", flush=True)
    for i in range(100):
        send_setpoint(vz=0.0)
        world.step(render=True)

        pos = vehicle.state.position
        orient = vehicle.state.attitude
        vel = vehicle.state.linear_velocity
        update_camera_pose(pos, orient)

        if i % 30 == 0:
            image, detections = capture_and_detect(frame_count, pos, orient, vel)
            if image is not None:
                frame_count += 1
                total_detections += len(detections)
                if detections:
                    print(f"    [Hover] {len(detections)} detections", flush=True)

# Final results
elapsed_time = time.time() - start_time
final_pos = vehicle.state.position

# Close detection logger
detection_logger.log_episode_end(
    total_reward=0.0,
    episode_length=frame_count,
    success=True,
    info={
        "total_detections": total_detections,
        "vehicle_clusters_visited": len(vehicle_cluster_centers),
    }
)
detection_logger.close()

print("\n" + "=" * 60, flush=True)
print("PHASE 3c RESULTS", flush=True)
print("=" * 60, flush=True)
print(f"Final position: ({final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f})", flush=True)
print(f"\nYOLO Inference:", flush=True)
print(f"  Frames processed: {frame_count}", flush=True)
print(f"  Total detections: {total_detections}", flush=True)
if yolo_times:
    print(f"  Avg YOLO time: {np.mean(yolo_times):.1f}ms", flush=True)
    print(f"  Max YOLO time: {np.max(yolo_times):.1f}ms", flush=True)
print(f"\nLogging:", flush=True)
print(f"  Detection log: {args.output_dir}/detections.jsonl", flush=True)
print(f"  Logger stats: {detection_logger.get_stats()}", flush=True)

if total_detections > 0:
    print(f"\nSUCCESS! Live YOLO inference working in simulation loop", flush=True)
    print(f"  Detections can be replayed with bounding boxes from log", flush=True)
else:
    print(f"\nPARTIAL: YOLO ran but no detections (check model/targets)", flush=True)

# Cleanup
print("\n[Cleanup] Shutting down...", flush=True)
timeline.stop()
simulation_app.close()
print("Done!", flush=True)
