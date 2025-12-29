#!/usr/bin/env python3
"""
Test Forest Environment with Procedural Generation

Generates a procedural forest terrain and spawns a drone for ISR training.
Based on the Nvidia-Isaac-Sim-Procedual-Forest-Generator extension.

Run with GUI:
  podman exec px4-sim /isaac-sim/python.sh /workspace/scripts/test_forest_env.py
"""

import argparse
import sys
import random as r
import numpy as np

# Force unbuffered output
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# Add Pegasus to Python path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

# Parse args before importing Isaac Sim
parser = argparse.ArgumentParser(description="Test Forest Environment")
parser.add_argument("--headless", action="store_true", help="Run headless (no GUI)")
parser.add_argument("--size", type=int, default=200, help="Forest size in meters")
parser.add_argument("--density", type=int, default=3, help="Trees per 10x10m area")
parser.add_argument("--duration", type=int, default=120, help="Test duration in seconds")
args = parser.parse_args()

# Initialize Isaac Sim
print("=" * 60)
print("Flyby F-11 Forest Environment Test")
print("=" * 60)

print(f"\nForest Size: {args.size}m x {args.size}m")
print(f"Tree Density: {args.density} per 100m²")
print(f"Headless: {args.headless}")
print(f"Duration: {args.duration}s")

print("\n[1/7] Initializing Isaac Sim...")
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "width": 1920,
    "height": 1080,
    "anti_aliasing": 0,
})
print("  Isaac Sim ready")

# Now import the rest
print("\n[2/7] Loading extensions...")
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics, Sdf, PhysxSchema
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
import isaacsim.core.utils.prims as prim_utils
from isaacsim.core.prims import XFormPrim
from scipy import interpolate

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation
print("  Extensions loaded")

# Model paths (relative to workspace mount)
MODELS_PATH = "/workspace/extensions/forest_generator/models"
BIRCH_USD = f"{MODELS_PATH}/Birch_obj/Birch.usd"
SPRUCE_USD = f"{MODELS_PATH}/Spruce_obj/Spruce.usd"
PINE_USD = f"{MODELS_PATH}/Pine_obj/Pine.usd"
ROCK_USD = f"{MODELS_PATH}/Rock_obj/Rock.usd"
BUSH_USD = f"{MODELS_PATH}/Bush_obj/Bush.usd"

# Tree tracking
trees = []
rocks = []


def convert_rotation(roll, pitch, yaw):
    """Convert euler angles (degrees) to quaternion."""
    roll = roll * (np.pi / 180)
    pitch = pitch * (np.pi / 180)
    yaw = yaw * (np.pi / 180)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qw, qx, qy, qz


def create_tree(tree_type, area_x, area_y, tree_age, stage):
    """Create a single tree at a random position."""
    area_x = (area_x / 2) - 1.0
    area_y = (area_y / 2) - 1.0

    usd_paths = {
        "Birch": BIRCH_USD,
        "Spruce": SPRUCE_USD,
        "Pine": PINE_USD,
    }

    # Scale factor: models are very small, need 1000x scale
    # Then apply age variation for natural look (5-15m tall trees)
    base_scale = 50.0  # 1000x bigger than before (0.05 * 1000)
    tree_scale = base_scale * r.uniform(0.8, 1.2) * (0.5 + tree_age * 0.5)

    tree_path = f'/World/Forest/Tree_{str(len(trees)).rjust(4, "0")}'
    prim_utils.create_prim(tree_path, "Xform")

    add_reference_to_stage(usd_path=usd_paths.get(tree_type, PINE_USD), prim_path=tree_path)

    # Random yaw rotation around Z axis, plus 90 deg X and 180 deg Y to stand upright
    # (models are Y-up, Isaac Sim is Z-up, and need Y flip)
    yaw = r.randrange(-180, 180)
    qw, qx, qy, qz = convert_rotation(90, 180, yaw)  # 90 X + 180 Y to stand correctly

    pos_x = r.uniform(-area_x, area_x)
    pos_y = r.uniform(-area_y, area_y)

    prim_utils.set_prim_property(tree_path, "xformOp:orient", Gf.Quatd(qw, qx, qy, qz))
    prim_utils.set_prim_property(tree_path, "xformOp:translate", Gf.Vec3d(pos_x, pos_y, 0.0))
    prim_utils.set_prim_property(tree_path, "xformOp:scale", Gf.Vec3d(tree_scale, tree_scale, tree_scale))

    trees.append(tree_path)


def generate_forest(stage, density, area_x, area_y):
    """Generate a procedural forest."""
    total_trees = int(density * (area_x * area_y) / 100.0)

    # Distribution: 30% birch, 40% spruce, 30% pine
    n_birch = int(total_trees * 0.30)
    n_spruce = int(total_trees * 0.40)
    n_pine = total_trees - n_birch - n_spruce

    print(f"  Generating {total_trees} trees ({n_birch} birch, {n_spruce} spruce, {n_pine} pine)...")

    for _ in range(n_birch):
        age = r.uniform(0.5, 1.0)
        create_tree("Birch", area_x, area_y, age, stage)

    for _ in range(n_spruce):
        age = r.uniform(0.5, 1.0)
        create_tree("Spruce", area_x, area_y, age, stage)

    for _ in range(n_pine):
        age = r.uniform(0.5, 1.0)
        create_tree("Pine", area_x, area_y, age, stage)

    print(f"  Created {len(trees)} trees")


def generate_terrain(stage, width, length, roughness=2.0):
    """Generate procedural terrain mesh using Perlin noise."""
    try:
        import noise
    except ImportError:
        print("  Warning: 'noise' module not available, using flat terrain")
        # Create flat terrain
        vertices = np.array([
            [-width/2, -length/2, 0],
            [width/2, -length/2, 0],
            [width/2, length/2, 0],
            [-width/2, length/2, 0],
        ], dtype=np.float32)
        triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)

        terrain_mesh = stage.DefinePrim("/World/Terrain", "Mesh")
        terrain_mesh.GetAttribute("points").Set(vertices)
        terrain_mesh.GetAttribute("faceVertexIndices").Set(triangles.flatten())
        terrain_mesh.GetAttribute("faceVertexCounts").Set(np.array([3, 3]))

        # Add collision
        UsdPhysics.CollisionAPI.Apply(terrain_mesh)

        print("  Created flat terrain (noise module not available)")
        return

    # Generate heightfield with Perlin noise
    horizontal_scale = 1.0
    vertical_scale = 0.1
    num_rows = int(width / horizontal_scale)
    num_cols = int(length / horizontal_scale)

    seed = r.randint(0, 1000)
    heightfield = np.zeros((num_rows, num_cols), dtype=np.float32)

    for i in range(num_rows):
        for j in range(num_cols):
            x = i / num_rows * 4
            y = j / num_cols * 4
            heightfield[i, j] = noise.pnoise2(x, y, octaves=4, base=seed) * roughness

    # Convert heightfield to mesh
    y_vals = np.linspace(-length/2, length/2, num_cols)
    x_vals = np.linspace(-width/2, width/2, num_rows)
    yy, xx = np.meshgrid(y_vals, x_vals)

    vertices = np.zeros((num_rows * num_cols, 3), dtype=np.float32)
    vertices[:, 0] = xx.flatten()
    vertices[:, 1] = yy.flatten()
    vertices[:, 2] = heightfield.flatten()

    # Generate triangles
    triangles = []
    for i in range(num_rows - 1):
        for j in range(num_cols - 1):
            idx = i * num_cols + j
            triangles.append([idx, idx + num_cols + 1, idx + 1])
            triangles.append([idx, idx + num_cols, idx + num_cols + 1])

    triangles = np.array(triangles, dtype=np.uint32)

    terrain_mesh = stage.DefinePrim("/World/Terrain", "Mesh")
    terrain_mesh.GetAttribute("points").Set(vertices)
    terrain_mesh.GetAttribute("faceVertexIndices").Set(triangles.flatten())
    terrain_mesh.GetAttribute("faceVertexCounts").Set(np.array([3] * len(triangles)))

    # Add collision
    terrain_prim = stage.GetPrimAtPath("/World/Terrain")
    UsdPhysics.CollisionAPI.Apply(terrain_prim)

    print(f"  Created terrain: {width}m x {length}m with {len(vertices)} vertices")


# Initialize simulation
print("\n[3/7] Setting up world...")
timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._world = World(**pg._world_settings)
world = pg.world

# Set coordinates (Camp Pendleton)
pg.set_global_coordinates(33.3853, -117.5653, 100.0)

# Add physics scene
stage = get_current_stage()
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)

# Create forest parent prim
prim_utils.create_prim("/World/Forest", "Xform")

print("  World setup complete")

# Add lighting
print("\n[4/8] Adding lighting...")
try:
    from pxr import UsdLux

    # Create a dome light (sky)
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(1000)
    dome_light.CreateColorAttr(Gf.Vec3f(0.8, 0.9, 1.0))  # Slightly blue sky

    # Create a distant light (sun)
    sun_light = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun_light.CreateIntensityAttr(3000)
    sun_light.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.8))  # Warm sunlight
    sun_light.CreateAngleAttr(0.53)  # Sun angular diameter

    # Rotate sun to cast shadows
    sun_xform = UsdGeom.Xformable(sun_light.GetPrim())
    sun_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))

    print("  Added dome light (sky) and distant light (sun)")
except Exception as e:
    print(f"  Warning: Could not add lighting: {e}")

# Generate terrain
print("\n[5/8] Generating terrain...")
generate_terrain(stage, args.size, args.size, roughness=3.0)

# Generate forest
print("\n[6/8] Generating forest...")
generate_forest(stage, args.density, args.size, args.size)

# Spawn drone
print("\n[6/7] Spawning Iris with PX4...")
print(f"  PX4 path: {pg.px4_path}")

px4_config = PX4MavlinkBackendConfig({
    "vehicle_id": 0,
    "px4_autolaunch": True,
    "px4_dir": pg.px4_path,
    "px4_vehicle_model": pg.px4_default_airframe,
})

iris_config = MultirotorConfig()
iris_config.backends = [PX4MavlinkBackend(px4_config)]

# Spawn above the terrain
spawn_pos = [0.0, 0.0, 5.0]  # Start at 5m altitude
spawn_rot = Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat()

iris = Multirotor(
    "/World/Drone",
    ROBOTS["Iris"],
    0,
    spawn_pos,
    spawn_rot,
    config=iris_config,
)
print("  Drone spawned at 5m altitude")

# Reset and start
world.reset()
timeline.play()

# Flight control
import threading
import subprocess

PX4_CMD = "/px4/build/px4_sitl_default/bin/px4-commander"


def px4_cmd_async(cmd_args, callback=None):
    """Execute PX4 commander command in background thread."""
    def run():
        try:
            result = subprocess.run(
                [PX4_CMD] + cmd_args.split(),
                capture_output=True,
                text=True,
                timeout=10,
                cwd="/px4"
            )
            if callback:
                callback(result.returncode == 0)
        except Exception as e:
            print(f"  PX4 cmd error: {e}")
            if callback:
                callback(False)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()


# State machine
class FlightState:
    INIT = 0
    WAIT_READY = 1
    ARM = 2
    TAKEOFF = 3
    HOVER = 4
    LAND = 5
    DONE = 6


state = FlightState.INIT
step_count = 0
phase_start_step = 0
command_sent = False

STEPS_INIT = 300
STEPS_WAIT_READY = 180
STEPS_ARM_WAIT = 120
STEPS_TAKEOFF = 480
STEPS_HOVER = args.duration * 60
STEPS_LAND = 600

print("\n[7/7] Starting simulation loop...")
print("=" * 60)
print("Forest Environment Test Running")
print()
print("Environment:")
print(f"  - Procedural terrain: {args.size}m x {args.size}m")
print(f"  - Trees: {len(trees)}")
print()
print("Flight Plan:")
print("  1. Initialize and wait for PX4")
print("  2. Arm vehicle")
print("  3. Takeoff to 20m altitude")
print("  4. Hover and observe environment")
print("  5. Land")
print()
print("Press Ctrl+C to stop early.")
print("=" * 60)

try:
    while simulation_app.is_running():
        world.step(render=True)
        step_count += 1

        if state == FlightState.INIT:
            if step_count >= STEPS_INIT:
                state = FlightState.WAIT_READY
                phase_start_step = step_count
                print("\n>>> Waiting for PX4 ready...")

        elif state == FlightState.WAIT_READY:
            if step_count - phase_start_step >= STEPS_WAIT_READY:
                state = FlightState.ARM
                phase_start_step = step_count
                command_sent = False
                print(">>> Sending arm command...")

        elif state == FlightState.ARM:
            if not command_sent:
                px4_cmd_async("arm")
                command_sent = True
            if step_count - phase_start_step >= STEPS_ARM_WAIT:
                state = FlightState.TAKEOFF
                phase_start_step = step_count
                command_sent = False
                print(">>> Sending takeoff command...")

        elif state == FlightState.TAKEOFF:
            if not command_sent:
                px4_cmd_async("takeoff")
                command_sent = True
            if step_count - phase_start_step >= STEPS_TAKEOFF:
                state = FlightState.HOVER
                phase_start_step = step_count
                print(">>> Hovering over forest...")

        elif state == FlightState.HOVER:
            if step_count - phase_start_step >= STEPS_HOVER:
                state = FlightState.LAND
                phase_start_step = step_count
                command_sent = False
                print(">>> Landing...")

        elif state == FlightState.LAND:
            if not command_sent:
                px4_cmd_async("land")
                command_sent = True
            if step_count - phase_start_step >= STEPS_LAND:
                state = FlightState.DONE
                print(">>> Flight complete!")

        # Log position every 2 seconds
        if step_count % 120 == 0:
            pos = iris.state.position
            state_names = ["INIT", "WAIT_READY", "ARM", "TAKEOFF", "HOVER", "LAND", "DONE"]
            t = step_count / 60.0
            print(f"t={t:6.1f}s | State: {state_names[state]:10s} | Pos: [{pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:7.2f}]")

except KeyboardInterrupt:
    print("\n\nStopping simulation...")

# Cleanup
timeline.stop()
simulation_app.close()
print("\nForest test complete!")
