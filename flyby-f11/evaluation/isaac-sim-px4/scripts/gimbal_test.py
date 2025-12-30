#!/usr/bin/env python3
"""Quick gimbal rotation and depth capture test."""

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

OUTPUT_DIR = "/tmp/gimbal_test_output"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# F-11 specs
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
print("Gimbal Rotation & Depth Capture Test")
print("=" * 60)

# Setup world
timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._px4_path = "/px4"
pg._world = World(**pg._world_settings)
world = pg.world

# Generate terrain
print("Generating world...")
world_config = WorldConfig(terrain_size=(200.0, 200.0), terrain_roughness=2.0, tree_density=0.4)
world_gen = WorldGenerator(models_path="/workspace/extensions/forest_generator/models", config=world_config)
world_gen.generate_terrain()
world_gen.setup_lighting()
world_gen.generate_forest()

# Spawn drone
config = MultirotorConfig()
config.thrust_curve = F11ThrustCurve()
config.drag = LinearDrag(F11_ISR_SPECS["drag_coeff"])
config.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
config.graphical_sensors = []
mavlink_config = PX4MavlinkBackendConfig({"vehicle_id": 0, "px4_autolaunch": True, "px4_dir": "/px4", "px4_vehicle_model": "none_iris"})
config.backends = [PX4MavlinkBackend(mavlink_config)]

print("Spawning drone at 15m altitude...")
drone = Multirotor("/World/F11_ISR", ROBOTS['Iris'], 0, [0.0, 0.0, 15.0],
                   Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(), config=config)
world.reset()

# Create camera
print("Creating camera...")
camera = Camera(prim_path="/World/F11_ISR/body/isr_camera", frequency=30, resolution=(640, 480))
camera.set_local_pose(np.array([0.1, 0.0, -0.05]), Rotation.from_euler("ZYX", [0.0, 45.0, 0.0], degrees=True).as_quat())
camera.initialize()
camera.set_focal_length(4.5)
camera.set_clipping_range(0.1, 500.0)
camera.add_distance_to_image_plane_to_frame()

def set_gimbal(pitch, yaw):
    """Set gimbal angles (degrees)."""
    rot = Rotation.from_euler("ZYX", [yaw, pitch, 0.0], degrees=True)
    camera.set_local_pose(np.array([0.1, 0.0, -0.05]), rot.as_quat())
    print(f"  Gimbal: pitch={pitch}°, yaw={yaw}°")

def capture(name):
    """Capture RGB and depth."""
    rgba = camera.get_rgba()
    depth = camera.get_depth()
    if rgba is not None and rgba.max() > 5:
        np.save(f"{OUTPUT_DIR}/{name}_rgb.npy", rgba[:,:,:3])
        print(f"  RGB: shape={rgba.shape}, max={rgba.max()}")
    if depth is not None:
        np.save(f"{OUTPUT_DIR}/{name}_depth.npy", depth)
        valid = depth[(depth > 0) & (depth < 1000)]
        if len(valid) > 0:
            print(f"  Depth: range={valid.min():.2f}-{valid.max():.2f}m")

# Run simulation
timeline.play()
print("\nWarming up (3 seconds)...")

# Warmup
for _ in range(180):  # 3 seconds at 60fps
    world.step(render=True)

# Test different gimbal positions
gimbal_tests = [
    ("look_forward", 0, 0),      # Looking straight ahead
    ("look_down_45", 45, 0),     # Looking down 45°
    ("look_down_90", 90, 0),     # Looking straight down
    ("look_left_30", 45, -30),   # Down 45°, left 30°
    ("look_right_30", 45, 30),   # Down 45°, right 30°
]

print("\n" + "=" * 60)
print("Capturing at different gimbal angles")
print("=" * 60)

for name, pitch, yaw in gimbal_tests:
    print(f"\n[{name}]")
    set_gimbal(pitch, yaw)
    # Let scene settle
    for _ in range(30):
        world.step(render=True)
    capture(name)

print("\n" + "=" * 60)
print(f"Done! Output saved to: {OUTPUT_DIR}")
print("=" * 60)

timeline.stop()
simulation_app.close()
