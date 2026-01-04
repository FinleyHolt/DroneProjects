#!/usr/bin/env python3
"""
Phase 2: Test PX4 flight in procedurally generated forest environment.

This script combines:
- The working PX4/MAVLink flight code from test_takeoff3.py
- Our procedural world generator for realistic forest terrain

Success criteria:
- Drone takes off and hovers in proc-gen forest world
- No poll timeouts or MAVLink issues
- GUI visible showing forest terrain
"""

from isaacsim import SimulationApp

# Launch with GUI
simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World
from pxr import UsdPhysics, Sdf, Gf
import sys
import time

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
print("Phase 2: PX4 Flight in Procedural Forest Environment", flush=True)
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
print("\n[WorldGen] Generating procedural forest environment...", flush=True)
models_path = "/workspace/extensions/forest_generator/models"

world_config = WorldConfig(
    terrain_size=(200.0, 200.0),  # Standard size
    terrain_roughness=2.0,  # Gentle hills
    terrain_material="forest_floor",
    tree_density=0.5,  # Lower density for performance during testing
    tree_proportions={"Birch": 30, "Spruce": 40, "Pine": 30},
    undergrowth_density=0.0,  # Disable for performance
    randomize_lighting=False,  # Use consistent lighting for testing
    time_of_day="noon",  # Consistent bright lighting
    weather="clear",
    seed=42,  # Reproducible
)

world_gen = WorldGenerator(models_path, world_config)

# Generate terrain
print("[WorldGen] Generating terrain...", flush=True)
terrain_path = world_gen.generate_terrain()
print(f"[WorldGen] Terrain created at: {terrain_path}", flush=True)

# Setup lighting
print("[WorldGen] Setting up lighting...", flush=True)
world_gen.setup_lighting()
print("[WorldGen] Lighting configured", flush=True)

# Generate forest (fewer trees for initial test)
print("[WorldGen] Generating forest...", flush=True)
forest_result = world_gen.generate_forest(density=0.5, include_undergrowth=False)
print(f"[WorldGen] Forest generated: {len(forest_result['trees'])} trees", flush=True)

# Spawn vehicles to test orientation
print("[WorldGen] Spawning vehicles...", flush=True)
from flyby.world_generator.spawners.vehicle_spawner import VehicleSpawner, SpawnConfig

spawn_config = SpawnConfig(
    spawn_area=(-80.0, 80.0, -80.0, 80.0),  # Within 200x200 terrain
    min_spacing=15.0,
)
vehicle_spawner = VehicleSpawner(stage, models_path, spawn_config)

# Spawn a variety of vehicles (2 of each = 20 vehicles)
vehicle_types = ["sedan", "sedan2", "suv", "sports_car", "taxi", "police", "tank", "tank2", "tank3", "tank4"]
spawned_vehicles = []
for vtype in vehicle_types:
    for _ in range(2):  # 2 of each type = 20 vehicles
        try:
            path = vehicle_spawner.spawn_vehicle(vtype)
            spawned_vehicles.append(path)
        except Exception as e:
            print(f"  Warning: Failed to spawn {vtype}: {e}", flush=True)

print(f"[WorldGen] Spawned {len(spawned_vehicles)} vehicles", flush=True)

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
spawn_height = 5.0  # Start above ground
vehicle = Multirotor(
    "/World/quadrotor",
    ROBOTS['Iris'],
    0,
    [0.0, 0.0, spawn_height],
    Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
    config=config,
)
print(f"[PX4] Vehicle spawned at height {spawn_height}m", flush=True)

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
    """Check current flight mode."""
    msg = mav.recv_match(type='HEARTBEAT', blocking=False)
    if msg:
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        custom_mode = msg.custom_mode
        return armed, custom_mode
    return None, None


def send_setpoint(vz=-0.5):
    """Send velocity setpoint (negative Z = up in NED)."""
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,
        0, 0, vz,
        0, 0, 0,
        0, 0
    )


# Prime with setpoints
print("\n[Flight] Priming with 200 setpoints...", flush=True)
for _ in range(200):
    send_setpoint()
    world.step(render=True)

armed, mode = check_mode()
print(f"[Flight] After priming: armed={armed}, mode={mode}", flush=True)

# Request OFFBOARD mode
print("\n[Flight] Requesting OFFBOARD mode...", flush=True)
for attempt in range(3):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6, 0, 0, 0, 0, 0  # Mode 6 = OFFBOARD
    )
    for _ in range(100):
        send_setpoint()
        world.step(render=True)

    armed, mode = check_mode()
    print(f"[Flight] Attempt {attempt+1}: armed={armed}, mode={mode}", flush=True)
    if mode == 6:
        break

# Arm
print("\n[Flight] Arming...", flush=True)
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
    print(f"[Flight] Attempt {attempt+1}: armed={armed}, mode={mode}", flush=True)
    if armed:
        break

# Ensure OFFBOARD after arming
print("\n[Flight] Ensuring OFFBOARD mode...", flush=True)
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

# Fly up
print("\n[Flight] Ascending for 1000 steps...", flush=True)
for i in range(1000):
    send_setpoint(vz=-2.0)  # 2 m/s ascent
    world.step(render=True)

    if i % 200 == 0:
        pos = vehicle.state.position
        armed, mode = check_mode()
        print(f"[Flight] Step {i}: pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), armed={armed}, mode={mode}", flush=True)

# Report results
final_pos = vehicle.state.position
print("\n" + "=" * 60, flush=True)
print("RESULTS", flush=True)
print("=" * 60, flush=True)
print(f"Final position: ({final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f})", flush=True)
print(f"World stats: {world_gen.get_stats()}", flush=True)

if final_pos[2] > spawn_height + 5.0:
    print("\nSUCCESS! Drone took off in procedural forest!", flush=True)
else:
    print("\nFAILED: Drone did not achieve sufficient altitude", flush=True)

# Hover for a bit so user can see the result
print("\n[Flight] Hovering for 500 steps (observe the forest!)...", flush=True)
for i in range(500):
    send_setpoint(vz=0.0)  # Hover
    world.step(render=True)

# Cleanup
print("\n[Cleanup] Shutting down...", flush=True)
timeline.stop()
simulation_app.close()
print("Done!", flush=True)
