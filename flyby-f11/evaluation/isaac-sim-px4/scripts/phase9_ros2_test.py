#!/usr/bin/env python3
"""
Phase 9 ROS2: Comms-Denied Integration Test with Manual ROS 2 Publishing

Due to Isaac Sim 5.0+ headless regression, the OmniGraph ROS2CameraHelper
doesn't work in headless mode. This script uses manual rclpy publishing:
1. Uses isaacsim.sensors.camera.Camera class to capture images
2. Manually publishes to ROS 2 using rclpy

Architecture:
    Isaac Sim (Camera.get_rgba()) → Manual rclpy publish → /camera/image_raw
                                                                ↓
                                              External ROS 2 nodes:
                                              - yolo_detector_node → /perception/detections
                                              - ontology_controller_node → /autonomy/behavior_command

IMPORTANT: DDS discovery takes ~20-30 seconds. Wait before checking topics.

Usage:
    # Run this test (publishes to ROS 2 topics)
    /isaac-sim/python.sh scripts/phase9_ros2_test.py --headless

    # Wait 30 seconds, then in separate terminals run ROS 2 perception nodes:
    source /opt/ros/jazzy/setup.bash
    ros2 topic list
    ros2 topic hz /camera/image_raw
    ros2 run flyby_perception yolo_detector_node
    ros2 run flyby_autonomy ontology_controller_node

Author: Finley Holt
"""

from isaacsim import SimulationApp

import sys
HEADLESS = "--headless" in sys.argv

simulation_config = {
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",
    "anti_aliasing": 0,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(simulation_config)

import omni
import omni.timeline
import numpy as np
import time
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
from pxr import UsdGeom, Gf

# Add paths
sys.path.insert(0, "/workspace")

PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)

EXTENSION_PATH = "/workspace/extensions/forest_generator/exts/flyby.world_generator"
if EXTENSION_PATH not in sys.path:
    sys.path.insert(0, EXTENSION_PATH)

from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils import extensions
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.camera import Camera

# Enable ROS 2 bridge extension BEFORE importing rclpy
print("[ROS2] Enabling isaacsim.ros2.bridge extension...", flush=True)
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
print("[ROS2] Extension enabled", flush=True)

# ROS 2 imports (AFTER extension is enabled)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

# Pegasus imports (after extensions enabled)
from omni.isaac.core.world import World
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from flyby.world_generator.world_generator import WorldGenerator, WorldConfig

# Camera settings
CAMERA_PATH = "/World/isr_camera"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


class CameraPublisherNode(Node):
    """ROS 2 node that manually publishes camera images."""

    def __init__(self):
        super().__init__('isaac_camera_publisher')
        self.rgb_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.frame_id = "isr_camera"
        self.seq = 0
        self.get_logger().info('Camera publisher node initialized')

    def publish_image(self, rgba_data: np.ndarray, timestamp: float):
        """Publish RGBA image as RGB."""
        if rgba_data is None or rgba_data.size == 0:
            return

        # Convert RGBA to RGB
        rgb_data = rgba_data[:, :, :3].copy()

        # Create header
        header = Header()
        header.frame_id = self.frame_id
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        header.stamp = Time(sec=sec, nanosec=nanosec)

        # Create Image message
        img_msg = Image()
        img_msg.header = header
        img_msg.height = rgb_data.shape[0]
        img_msg.width = rgb_data.shape[1]
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = False
        img_msg.step = rgb_data.shape[1] * 3
        img_msg.data = rgb_data.tobytes()

        # Publish
        self.rgb_pub.publish(img_msg)

        # Also publish camera info
        info_msg = CameraInfo()
        info_msg.header = header
        info_msg.height = rgb_data.shape[0]
        info_msg.width = rgb_data.shape[1]
        info_msg.distortion_model = "plumb_bob"
        # Simple pinhole camera model for ISR camera
        fx = fy = 500.0  # focal length
        cx = rgb_data.shape[1] / 2.0
        cy = rgb_data.shape[0] / 2.0
        info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.info_pub.publish(info_msg)
        self.seq += 1


def run_phase9_ros2_test():
    """Main test function using manual ROS 2 publishing for headless mode."""
    print("=" * 60)
    print("Phase 9 ROS2: Manual Publishing for Headless Mode")
    print("=" * 60)
    print(f"Mode: {'Headless' if HEADLESS else 'GUI'}")
    print()

    # Check numpy version
    print(f"[Info] NumPy version: {np.__version__}")
    if np.__version__.startswith("2."):
        print("[WARNING] NumPy 2.x detected - may cause dtype errors!")
        print("[WARNING] Run: pip install --break-system-packages numpy==1.26.4")
    print()

    # Initialize ROS 2
    print("[ROS2] Initializing rclpy...", flush=True)
    rclpy.init()
    ros_node = CameraPublisherNode()
    print("[ROS2] Node created, publishing to /camera/image_raw and /camera/camera_info", flush=True)

    # Initialize timeline and Pegasus interface
    print("\n[World] Initializing Pegasus and simulation world...", flush=True)
    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.set_px4_path("/px4")
    stage = get_current_stage()

    # Create forest world
    models_path = "/workspace/extensions/forest_generator/models"
    world_config = WorldConfig(
        terrain_size=(200.0, 200.0),
        terrain_roughness=2.0,
        terrain_material="forest_floor",
        tree_density=0.3,
        tree_proportions={"Birch": 30, "Spruce": 40, "Pine": 30},
        undergrowth_density=0.0,
        randomize_lighting=False,
        time_of_day="noon",
        weather="clear",
        seed=42,
    )
    world_gen = WorldGenerator(models_path, world_config)
    terrain_path = world_gen.generate_terrain()
    world_gen.setup_lighting()
    forest_result = world_gen.generate_forest(density=0.3, include_undergrowth=False)
    print(f"[WorldGen] Forest generated: {len(forest_result['trees'])} trees", flush=True)

    # Spawn POI targets
    print("\n[WorldGen] Spawning POI targets...", flush=True)
    poi_clusters = [
        (30.0, 20.0),
        (-20.0, 30.0),
        (40.0, -25.0),
        (-35.0, -30.0),
    ]

    all_target_paths = []
    for center in poi_clusters:
        vehicle_paths = world_gen.vehicles.spawn_vehicle_group(
            vehicle_types=["sedan", "suv", "tank"],
            count=3,
            clustering=0.5,
            center=center,
        )
        all_target_paths.extend(vehicle_paths)

    print(f"[WorldGen] Spawned {len(all_target_paths)} targets", flush=True)

    # Multiple updates to ensure stage is ready
    for _ in range(10):
        simulation_app.update()

    # Create camera using Camera class (NOT USD prim - this works reliably)
    print("\n[Camera] Creating camera using Camera class...", flush=True)
    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array([0.0, 0.0, 25.0]),  # Start high above terrain
        frequency=30,
        resolution=(CAMERA_WIDTH, CAMERA_HEIGHT),
        orientation=rot_utils.euler_angles_to_quats(np.array([90, 0, 0]), degrees=True),  # Looking down
    )

    # Initialize camera (called TWICE as per official example)
    print("[Camera] Initializing camera (double init)...", flush=True)
    camera.initialize()
    simulation_app.update()
    camera.initialize()
    simulation_app.update()

    print(f"[Camera] Camera created at {CAMERA_PATH}", flush=True)
    print(f"[Camera] Resolution: {CAMERA_WIDTH}x{CAMERA_HEIGHT}", flush=True)

    # Create vehicle
    print("\n[PX4] Creating Iris quadrotor with PX4 backend...", flush=True)
    mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": 0,
        "px4_autolaunch": True,
        "px4_dir": "/px4",
        "px4_vehicle_model": pg.px4_default_airframe,
    })

    config = MultirotorConfig()
    config.backends = [PX4MavlinkBackend(mavlink_config)]

    vehicle = Multirotor(
        "/World/quadrotor",
        ROBOTS['Iris'],
        0,
        [0.0, 0.0, 0.5],
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=config,
    )

    world.reset()

    # Start simulation
    timeline.play()

    # Extended warmup frames for headless mode
    warmup_frames = 50
    print(f"\n[Sim] Running {warmup_frames} warmup frames...", flush=True)
    for i in range(warmup_frames):
        world.step(render=True)

    # Verify camera is working
    print("\n[Camera] Verifying camera capture...", flush=True)
    rgba = camera.get_rgba()
    if rgba is not None and rgba.size > 0 and rgba.max() > 0:
        print(f"[Camera] SUCCESS: Capturing {rgba.shape} images, max={rgba.max()}, mean={rgba.mean():.1f}", flush=True)
    else:
        print("[Camera] WARNING: Camera may not be capturing properly", flush=True)

    # Camera warmup
    print("\n[Camera] Warming up (120 frames)...", flush=True)
    for i in range(120):
        world.step(render=True)

    # PX4 initialization
    print("\n[PX4] Running initialization...", flush=True)
    for i in range(200):
        world.step(render=True)

    # MAVLink connection
    print("\n[MAVLink] Connecting...", flush=True)
    mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)

    for i in range(500):
        world.step(render=True)
        msg = mav.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.get_srcSystem() != 0:
            print(f"[MAVLink] Connected to system {msg.get_srcSystem()}", flush=True)
            break

        # Spin ROS 2 during MAVLink wait
        rclpy.spin_once(ros_node, timeout_sec=0)

    mav.target_system = 1
    mav.target_component = 1

    # DDS discovery wait
    print("\n[ROS2] Waiting 5 seconds for DDS discovery...", flush=True)
    for i in range(50):
        world.step(render=True)
        rclpy.spin_once(ros_node, timeout_sec=0.1)
    print("[ROS2] DDS discovery period complete.", flush=True)

    # Main test loop
    print("\n[Test] Starting main loop (5000 steps)...", flush=True)
    print("[Test] Camera images publishing to /camera/image_raw", flush=True)
    print("[Test] Check ROS 2 topics in another terminal:", flush=True)
    print("       source /opt/ros/jazzy/setup.bash", flush=True)
    print("       ros2 topic list", flush=True)
    print("       ros2 topic hz /camera/image_raw", flush=True)
    print("       ros2 run flyby_perception yolo_detector_node", flush=True)
    print("       ros2 topic echo /perception/detections", flush=True)
    print()

    max_steps = 5000

    for step in range(max_steps):
        world.step(render=True)

        # Publish camera image every frame
        try:
            rgba = camera.get_rgba()
            if rgba is not None and rgba.size > 0:
                ros_node.publish_image(rgba, time.time())
        except Exception as e:
            if step % 500 == 0:
                print(f"[Camera] Error getting/publishing image: {e}", flush=True)

        # Spin ROS 2 (non-blocking)
        rclpy.spin_once(ros_node, timeout_sec=0)

        # Progress logging
        if step % 500 == 0:
            print(f"[Progress] Step {step}/{max_steps}, published {ros_node.seq} images", flush=True)

    print(f"\n[Test] Complete! Published {ros_node.seq} images total.", flush=True)

    # Cleanup
    timeline.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    run_phase9_ros2_test()
