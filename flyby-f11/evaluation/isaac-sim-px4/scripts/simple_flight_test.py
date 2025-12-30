#!/usr/bin/env python3
"""Simple flight test with belly-mounted camera - fixed version."""

import argparse
import time
import os

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
args, _ = parser.parse_known_args()

HEADLESS = not args.gui

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS, "renderer": "RayTracedLighting"})

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

OUTPUT = "/tmp/flight_photos"
os.makedirs(OUTPUT, exist_ok=True)

class F11Thrust(QuadraticThrustCurve):
    def __init__(self):
        super().__init__()
        self._rotor_constant = [4.1e-5] * 4
        self._rolling_moment_coefficient = [1e-6] * 4
        arm = 0.295
        self._rotor_positions = [[arm*0.707,-arm*0.707,0],[-arm*0.707,arm*0.707,0],[arm*0.707,arm*0.707,0],[-arm*0.707,-arm*0.707,0]]

print("="*60)
print("Simple Flight Test - Belly Camera (Fixed)")
print("="*60)

# Setup
timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._px4_path = "/px4"
pg._world = World(**pg._world_settings)
world = pg.world

# World
print("Generating world...")
wc = WorldConfig(terrain_size=(200,200), terrain_roughness=2.0, tree_density=0.5)
wg = WorldGenerator(models_path="/workspace/extensions/forest_generator/models", config=wc)
wg.generate_terrain()
wg.setup_lighting()
wg.generate_forest()

# Drone at 25m
cfg = MultirotorConfig()
cfg.thrust_curve = F11Thrust()
cfg.drag = LinearDrag([0.35,0.35,0.1])
cfg.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
cfg.graphical_sensors = []
cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({"vehicle_id":0,"px4_autolaunch":True,"px4_dir":"/px4","px4_vehicle_model":"none_iris"}))]

print("Spawning drone at 25m...")
drone = Multirotor("/World/F11", ROBOTS['Iris'], 0, [0,0,25], Rotation.from_euler("XYZ",[0,0,0],degrees=True).as_quat(), config=cfg)
world.reset()

# Belly camera - mounted below, looking 60deg down
print("Creating belly-mounted camera...")
cam = Camera(prim_path="/World/F11/body/belly_cam", frequency=30, resolution=(640,480))
cam.set_local_pose(np.array([0,0,-0.2]), Rotation.from_euler("ZYX",[0,60,0],degrees=True).as_quat())
cam.initialize()
cam.set_focal_length(4.5)
cam.set_clipping_range(0.1,500)
cam.add_distance_to_image_plane_to_frame()

# Run
timeline.play()
print("Warming up (5s for camera)...")
for _ in range(300): world.step(render=True)  # 5 seconds at 60fps

# Capture at different angles
angles = [
    ("forward_60deg", 60),   # 60 deg down - oblique
    ("forward_45deg", 45),   # 45 deg down
    ("nadir_90deg", 90),     # straight down
]

print("\nCapturing at different pitch angles:")
for name, pitch in angles:
    print(f"\n[{name}]")
    cam.set_local_pose(np.array([0,0,-0.2]), Rotation.from_euler("ZYX",[0,pitch,0],degrees=True).as_quat())
    # Extra warmup for each angle
    for _ in range(90): world.step(render=True)
    
    rgba = cam.get_rgba()
    depth = cam.get_depth()
    
    # Check RGBA validity
    if rgba is not None:
        if len(rgba.shape) == 3 and rgba.shape[2] >= 3:
            rgb = rgba[:,:,:3]
            if rgb.max() > 5:
                np.save(f"{OUTPUT}/{name}_rgb.npy", rgb)
                print(f"  RGB: shape={rgb.shape}, max={rgb.max()}, mean={rgb.mean():.1f}")
            else:
                print(f"  RGB: too dark (max={rgb.max()})")
        else:
            print(f"  RGB: invalid shape {rgba.shape}")
    else:
        print(f"  RGB: None")
        
    # Check depth
    if depth is not None and len(depth.shape) == 2:
        np.save(f"{OUTPUT}/{name}_depth.npy", depth)
        v = depth[(depth>0)&(depth<500)]
        if len(v)>0: 
            print(f"  Depth: {v.min():.1f}-{v.max():.1f}m")
    else:
        print(f"  Depth: invalid")

print(f"\nDone! Photos in {OUTPUT}")
timeline.stop()
simulation_app.close()
