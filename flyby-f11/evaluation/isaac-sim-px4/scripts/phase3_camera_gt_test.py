#!/usr/bin/env python3
"""
Phase 3: Ground Truth Perception Test (No Camera Rendering)

Builds on Phase 2 (procedural world + PX4 flight) by adding:
- POI targets (vehicles) spawned in the world
- Ground truth detection using frustum math (no actual camera/rendering)
- This is the "GT mode" that will be used for fast RL training

NOTE: We skip the Isaac Sim Camera API because it triggers the replicator
extension which has compatibility issues in Isaac Sim 5.1. For actual
camera images during E2E validation, we'll use a different approach.

Success criteria:
- Targets spawned in world with known positions
- Ground truth detections computed correctly during flight
- Detections printed when targets would be in camera FOV
"""

from isaacsim import SimulationApp

# Launch with GUI
simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
import sys
import time
import numpy as np

# Add Pegasus to path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

# Add our extension to path
EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
from pymavlink import mavutil

# Import our world generator
from flyby.world_generator.world_generator import WorldGenerator, WorldConfig
from isaacsim.core.utils.stage import get_current_stage

print("=" * 60, flush=True)
print("Phase 3: Ground Truth Perception Test", flush=True)
print("=" * 60, flush=True)

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
    print("Physics scene created", flush=True)

# Generate procedural forest environment
print("\n[WorldGen] Generating procedural environment...", flush=True)
models_path = "/workspace/extensions/forest_generator/models"

world_config = WorldConfig(
    terrain_size=(200.0, 200.0),
    terrain_roughness=2.0,
    terrain_material="forest_floor",
    tree_density=0.3,  # Sparse trees for better target visibility
    tree_proportions={"Birch": 30, "Spruce": 40, "Pine": 30},
    undergrowth_density=0.0,
    randomize_lighting=False,
    time_of_day="noon",
    weather="clear",
    seed=42,
)

world_gen = WorldGenerator(models_path, world_config)

# Generate terrain and lighting
print("[WorldGen] Generating terrain...", flush=True)
terrain_path = world_gen.generate_terrain()
print(f"[WorldGen] Terrain created at: {terrain_path}", flush=True)

print("[WorldGen] Setting up lighting...", flush=True)
world_gen.setup_lighting()

# Generate sparse forest
print("[WorldGen] Generating forest...", flush=True)
forest_result = world_gen.generate_forest(density=0.3, include_undergrowth=False)
print(f"[WorldGen] Forest generated: {len(forest_result['trees'])} trees", flush=True)

# Spawn POI targets (vehicles)
print("\n[WorldGen] Spawning POI targets...", flush=True)

# Spawn vehicle cluster at known position
vehicle_cluster_center = (30.0, 20.0)  # 30m east, 20m north of origin
vehicle_paths = world_gen.vehicles.spawn_vehicle_group(
    vehicle_types=["sedan", "suv", "tank"],
    count=5,
    clustering=0.7,
    center=vehicle_cluster_center,
)
print(f"[WorldGen] Spawned {len(vehicle_paths)} vehicles at cluster center {vehicle_cluster_center}", flush=True)

# Store target positions for ground truth
target_positions = []
for path in vehicle_paths:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        xform = UsdGeom.Xformable(prim)
        translate_op = xform.GetOrderedXformOps()[0] if xform.GetOrderedXformOps() else None
        if translate_op:
            pos = translate_op.Get()
            target_positions.append({
                'path': path,
                'position': np.array([pos[0], pos[1], pos[2]]),
                'type': 'vehicle'
            })
            print(f"  Target: {path.split('/')[-1]} at ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})", flush=True)

print(f"[WorldGen] Total POI targets: {len(target_positions)}", flush=True)

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

# Spawn drone above terrain at origin
spawn_height = 5.0
vehicle = Multirotor(
    "/World/quadrotor",
    ROBOTS['Iris'],
    0,
    [0.0, 0.0, spawn_height],
    Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
    config=config,
)
print(f"[PX4] Vehicle spawned at height {spawn_height}m", flush=True)

# Ground truth perception parameters (simulated camera FOV)
CAMERA_FOV_DEG = 90.0
CAMERA_PITCH_DEG = 45.0  # Looking down at 45 degrees
MAX_DETECTION_RANGE = 100.0


def compute_ground_truth_detections(uav_pos, uav_orient, targets):
    """
    Compute ground truth detections using frustum math.

    Simulates what a camera with given FOV would see without
    actually rendering any images. This is the "GT mode" for
    fast RL training.

    Returns list of targets visible in simulated camera frustum.
    """
    detections = []

    # Get camera forward vector (body +X, pitched down)
    r = Rotation.from_quat(uav_orient)
    camera_pitch = Rotation.from_euler('Y', CAMERA_PITCH_DEG, degrees=True)
    camera_rotation = r * camera_pitch

    camera_forward = camera_rotation.apply([1, 0, 0])
    camera_right = camera_rotation.apply([0, 1, 0])
    camera_down = camera_rotation.apply([0, 0, 1])

    fov_half = np.radians(CAMERA_FOV_DEG / 2)

    for target in targets:
        to_target = target['position'] - uav_pos
        distance = np.linalg.norm(to_target)

        if distance > MAX_DETECTION_RANGE or distance < 1.0:
            continue

        to_target_norm = to_target / distance
        cos_angle = np.dot(to_target_norm, camera_forward)
        angle = np.arccos(np.clip(cos_angle, -1, 1))

        # Check if within FOV cone
        if angle < fov_half:
            forward_dist = np.dot(to_target, camera_forward)
            if forward_dist < 1.0:
                continue

            right_offset = np.dot(to_target, camera_right)
            down_offset = np.dot(to_target, camera_down)

            # Normalized screen coordinates (-1 to 1)
            screen_x = right_offset / (forward_dist * np.tan(fov_half))
            screen_y = down_offset / (forward_dist * np.tan(fov_half))

            # Approximate bounding box size
            object_size = 4.0  # meters (vehicle size)
            angular_size = 2 * np.arctan(object_size / (2 * distance))
            bbox_size = angular_size / fov_half

            detections.append({
                'target': target,
                'distance': distance,
                'angle_deg': np.degrees(angle),
                'screen_x': screen_x,
                'screen_y': screen_y,
                'bbox_size': bbox_size,
                'confidence': 1.0,  # Ground truth = 100% confidence
            })

    return detections


# Reset world and start simulation
world.reset()
timeline.play()

# Wait for PX4 to initialize
print("\n[PX4] Running 200 steps to let PX4 initialize...", flush=True)
for i in range(200):
    world.step(render=True)

initial_pos = vehicle.state.position
print(f"[PX4] Initial position: ({initial_pos[0]:.2f}, {initial_pos[1]:.2f}, {initial_pos[2]:.2f})", flush=True)

# Connect MAVLink
print("\n[MAVLink] Connecting...", flush=True)
mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

# Wait for heartbeat while stepping
for i in range(500):
    world.step(render=True)
    msg = mav.recv_match(type='HEARTBEAT', blocking=False)
    if msg and msg.get_srcSystem() != 0:
        print(f"[MAVLink] Got heartbeat from system {msg.get_srcSystem()} after {i} steps", flush=True)
        break

if mav.target_system == 0:
    mav.target_system = 1
    mav.target_component = 1
print(f"[MAVLink] Target: system {mav.target_system}, component {mav.target_component}", flush=True)


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


# Prime with setpoints
print("\n[Flight] Priming with 200 setpoints...", flush=True)
for _ in range(200):
    send_setpoint()
    world.step(render=True)

# Engage OFFBOARD mode and arm
print("\n[Flight] Engaging OFFBOARD mode and arming...", flush=True)
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
print(f"[Flight] Final state: armed={armed}, mode={mode}", flush=True)

# Fly up to observation altitude
print("\n[Flight] Ascending to observation altitude (20m)...", flush=True)
for i in range(500):
    send_setpoint(vz=-2.0)
    world.step(render=True)
    if i % 100 == 0:
        pos = vehicle.state.position
        print(f"[Flight] Step {i}: altitude = {pos[2]:.1f}m", flush=True)

# Now fly toward the vehicle cluster while running ground truth detection
print("\n[ISR] Flying toward vehicle cluster with ground truth perception...", flush=True)
print(f"[ISR] Target cluster at ({vehicle_cluster_center[0]:.0f}, {vehicle_cluster_center[1]:.0f})", flush=True)
print(f"[ISR] Simulated camera: FOV={CAMERA_FOV_DEG}°, pitch={CAMERA_PITCH_DEG}°", flush=True)

detections_total = 0
detection_frames = 0

# Calculate direction to target cluster
current_pos = vehicle.state.position
target_vec = np.array([vehicle_cluster_center[0] - current_pos[0],
                       vehicle_cluster_center[1] - current_pos[1], 0])
target_dir = target_vec[:2] / (np.linalg.norm(target_vec[:2]) + 1e-6)

# Fly toward target for 1000 steps
for i in range(1000):
    # Velocity toward target cluster
    vx = target_dir[0] * 3.0
    vy = target_dir[1] * 3.0

    # Maintain altitude at 20m
    current_z = vehicle.state.position[2]
    vz = -1.0 * (20.0 - current_z)
    vz = np.clip(vz, -2.0, 2.0)

    send_setpoint(vx=vx, vy=vy, vz=vz)
    world.step(render=True)

    # Run ground truth detection every 30 steps
    if i % 30 == 0:
        pos = vehicle.state.position
        orient = vehicle.state.attitude

        detections = compute_ground_truth_detections(pos, orient, target_positions)
        if detections:
            detection_frames += 1
            detections_total += len(detections)

        if i % 150 == 0:
            dist_to_cluster = np.linalg.norm(pos[:2] - np.array(vehicle_cluster_center))
            print(f"\n[ISR] Step {i}:", flush=True)
            print(f"  Position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})", flush=True)
            print(f"  Distance to cluster: {dist_to_cluster:.1f}m", flush=True)
            print(f"  GT Detections: {len(detections)}", flush=True)

            for det in detections[:3]:
                t = det['target']
                print(f"    - {t['path'].split('/')[-1]}: "
                      f"dist={det['distance']:.1f}m, "
                      f"angle={det['angle_deg']:.1f}°, "
                      f"screen=({det['screen_x']:.2f}, {det['screen_y']:.2f})", flush=True)

# Final hover
print("\n[Flight] Hovering for final observations...", flush=True)
final_detections = []
for i in range(300):
    send_setpoint(vz=0.0)
    world.step(render=True)

    if i % 50 == 0:
        pos = vehicle.state.position
        orient = vehicle.state.attitude
        detections = compute_ground_truth_detections(pos, orient, target_positions)
        if detections:
            final_detections = detections
            detection_frames += 1
            detections_total += len(detections)

# Report results
final_pos = vehicle.state.position
print("\n" + "=" * 60, flush=True)
print("PHASE 3 RESULTS", flush=True)
print("=" * 60, flush=True)
print(f"Final position: ({final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f})", flush=True)
print(f"World stats: {world_gen.get_stats()}", flush=True)
print(f"\nGround Truth Perception:", flush=True)
print(f"  POI targets in world: {len(target_positions)}", flush=True)
print(f"  Frames with detections: {detection_frames}", flush=True)
print(f"  Total detections logged: {detections_total}", flush=True)
print(f"  Simulated camera FOV: {CAMERA_FOV_DEG}°", flush=True)

if final_detections:
    print(f"\nFinal frame detections ({len(final_detections)}):", flush=True)
    for det in final_detections:
        t = det['target']
        print(f"  - {t['path'].split('/')[-1]}: "
              f"dist={det['distance']:.1f}m, "
              f"screen=({det['screen_x']:.2f}, {det['screen_y']:.2f})", flush=True)

if detection_frames > 0 and detections_total > 0:
    print("\nSUCCESS! Ground truth perception working!", flush=True)
    print("  Targets detected using frustum math (no camera rendering)", flush=True)
    print("  This is the 'GT mode' for fast RL training", flush=True)
else:
    print("\nFAILED: No ground truth detections", flush=True)
    print("  Check target positions and camera FOV parameters", flush=True)

# Cleanup
print("\n[Cleanup] Shutting down...", flush=True)
timeline.stop()
simulation_app.close()
print("Done!", flush=True)
