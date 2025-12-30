#!/usr/bin/env python3
"""
Flight test: Fly drone to different positions and capture photos.
Camera mounted further from body to avoid obstruction.
"""

import argparse
import time
import os

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="Run with GUI")
args, _ = parser.parse_known_args()

HEADLESS = not args.gui

from isaacsim import SimulationApp
config = {"headless": HEADLESS, "renderer": "RayTracedLighting", "anti_aliasing": 0}
simulation_app = SimulationApp(config)

import sys
sys.path.insert(0, "/pegasus/extensions/pegasus.simulator")
sys.path.insert(0, "/workspace/extensions/forest_generator/exts/flyby.world_generator")

import numpy as np
import omni.timeline
from omni.isaac.core.world import World
from isaacsim.sensors.camera import Camera
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.dynamics import LinearDrag
from scipy.spatial.transform import Rotation
from flyby.world_generator import WorldGenerator, WorldConfig
from pxr import UsdGeom, Gf

OUTPUT_DIR = "/tmp/flight_photos"
os.makedirs(OUTPUT_DIR, exist_ok=True)

F11_ISR_SPECS = {"mass": 6.3, "arm_length": 0.295, "drag_coeff": [0.35, 0.35, 0.1]}

class F11ThrustCurve(QuadraticThrustCurve):
    def __init__(self):
        super().__init__()
        self._rotor_constant = [4.1e-5] * 4
        self._rolling_moment_coefficient = [1e-6] * 4
        arm = F11_ISR_SPECS["arm_length"]
        self._rotor_positions = [
            [arm * 0.707, -arm * 0.707, 0.0], [-arm * 0.707, arm * 0.707, 0.0],
            [arm * 0.707, arm * 0.707, 0.0], [-arm * 0.707, -arm * 0.707, 0.0],
        ]

print("=" * 60)
print("Flight Photo Test - 25m Altitude Survey")
print("=" * 60)

# Setup
timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._px4_path = "/px4"
pg._world = World(**pg._world_settings)
world = pg.world

# Generate larger terrain with more trees
print("Generating world with dense forest...")
world_config = WorldConfig(terrain_size=(300.0, 300.0), terrain_roughness=3.0, tree_density=0.6)
world_gen = WorldGenerator(models_path="/workspace/extensions/forest_generator/models", config=world_config)
world_gen.generate_terrain()
world_gen.setup_lighting()
world_gen.generate_forest()
print(f"  Stats: {world_gen.get_stats()}")

# Spawn drone at 25m
config = MultirotorConfig()
config.thrust_curve = F11ThrustCurve()
config.drag = LinearDrag(F11_ISR_SPECS["drag_coeff"])
config.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
config.graphical_sensors = []
mavlink_config = PX4MavlinkBackendConfig({"vehicle_id": 0, "px4_autolaunch": True, "px4_dir": "/px4", "px4_vehicle_model": "none_iris"})
config.backends = [PX4MavlinkBackend(mavlink_config)]

ALTITUDE = 25.0
print(f"Spawning drone at {ALTITUDE}m altitude...")
drone = Multirotor("/World/F11_ISR", ROBOTS['Iris'], 0, [0.0, 0.0, ALTITUDE],
                   Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(), config=config)
world.reset()

# Create camera - mounted BELOW the drone body to avoid obstruction
# Position: 0.3m below center, pitched down 60 degrees
print("Creating ISR camera (belly-mounted, clear FOV)...")
camera = Camera(prim_path="/World/F11_ISR/body/isr_camera", frequency=30, resolution=(640, 480))
# Mount below body: z=-0.15 (15cm below), pitched down 60 degrees
camera.set_local_pose(
    np.array([0.0, 0.0, -0.15]),  # Directly below body
    Rotation.from_euler("ZYX", [0.0, 60.0, 0.0], degrees=True).as_quat()  # 60° down
)
camera.initialize()
camera.set_focal_length(4.5)
camera.set_clipping_range(0.1, 500.0)
camera.add_distance_to_image_plane_to_frame()

def move_drone(x, y, z, yaw_deg=0):
    """Teleport drone to position."""
    stage = world.stage
    prim = stage.GetPrimAtPath("/World/F11_ISR")
    if prim:
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        rot = Rotation.from_euler("Z", yaw_deg, degrees=True)
        quat = rot.as_quat()  # [x, y, z, w]
        xform.AddOrientOp().Set(Gf.Quatd(quat[3], quat[0], quat[1], quat[2]))
    print(f"  Moved to: ({x}, {y}, {z}), yaw={yaw_deg}°")

def capture(name):
    """Capture RGB and depth."""
    # Let scene render
    for _ in range(60):
        world.step(render=True)
    
    rgba = camera.get_rgba()
    depth = camera.get_depth()
    
    if rgba is not None and rgba.max() > 5:
        np.save(f"{OUTPUT_DIR}/{name}_rgb.npy", rgba[:,:,:3])
        print(f"  RGB: max={rgba.max()}, mean={rgba.mean():.1f}")
    else:
        print(f"  RGB: too dark (max={rgba.max() if rgba is not None else 'None'})")
        
    if depth is not None:
        np.save(f"{OUTPUT_DIR}/{name}_depth.npy", depth)
        valid = depth[(depth > 0) & (depth < 500)]
        if len(valid) > 0:
            print(f"  Depth: {valid.min():.1f}-{valid.max():.1f}m (mean={valid.mean():.1f}m)")

# Start simulation
timeline.play()
print("\nWarming up...")
for _ in range(180):
    world.step(render=True)

# Survey positions at 25m altitude
positions = [
    ("center", 0, 0, ALTITUDE, 0),
    ("north", 0, 50, ALTITUDE, 180),      # Looking south
    ("south", 0, -50, ALTITUDE, 0),       # Looking north
    ("east", 50, 0, ALTITUDE, 270),       # Looking west
    ("west", -50, 0, ALTITUDE, 90),       # Looking east
    ("corner_ne", 40, 40, ALTITUDE, 225), # Looking SW
]

print("\n" + "=" * 60)
print("Capturing survey photos at 25m altitude")
print("=" * 60)

for name, x, y, z, yaw in positions:
    print(f"\n[{name}]")
    move_drone(x, y, z, yaw)
    capture(name)

# Also test straight down view
print(f"\n[nadir] - Straight down view")
# Reposition camera to look straight down
camera.set_local_pose(
    np.array([0.0, 0.0, -0.15]),
    Rotation.from_euler("ZYX", [0.0, 90.0, 0.0], degrees=True).as_quat()  # 90° = straight down
)
for _ in range(60):
    world.step(render=True)
capture("nadir")

print("\n" + "=" * 60)
print(f"Done! Photos saved to: {OUTPUT_DIR}")
print("=" * 60)

timeline.stop()
simulation_app.close()
