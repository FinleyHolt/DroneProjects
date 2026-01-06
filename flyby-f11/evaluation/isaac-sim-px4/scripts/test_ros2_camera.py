#!/usr/bin/env python3
"""
ROS 2 Camera Test - Headless Mode with Manual Publishing Workaround

Author: Finley Holt

=============================================================================
CRITICAL: Isaac Sim 5.0+ Headless Mode ROS 2 Camera Workaround
=============================================================================

PROBLEM:
    Isaac Sim 5.0+ has a regression where the built-in OmniGraph ROS2CameraHelper
    node does NOT work in headless mode. Topics are created but no images are
    published. This affects all Isaac Sim 5.x versions (tested on 5.0.0 and 5.1.0).

SYMPTOMS (if you accidentally break this):
    - ROS 2 topics exist (/camera/rgb, /camera/camera_info) but never publish
    - `ros2 topic hz /camera/rgb` shows 0 Hz
    - `ros2 topic echo /camera/rgb` hangs forever
    - GUI mode works fine, only headless mode is broken

SOLUTION (implemented in this script):
    1. Use isaacsim.sensors.camera.Camera class with get_rgba() method
    2. Manually publish to ROS 2 topics using rclpy
    3. Initialize camera TWICE (required per official Isaac Sim examples)
    4. Enable ROS 2 extension BEFORE importing rclpy
    5. Use NumPy 1.26.4 (not 2.x - causes dtype errors with Isaac Sim cameras)

WHY THESE PATTERNS ARE REQUIRED:
    - Camera class: The only reliable way to get image data in headless mode
    - Double initialization: Isaac Sim camera requires two init calls to work
    - Extension order: rclpy imports fail if extension not enabled first
    - NumPy 1.26.4: Isaac Sim's internal camera system expects NumPy 1.x dtypes

IMPORTANT: DDS discovery takes ~20-30 seconds. Wait before checking topics.

Usage:
    /isaac-sim/python.sh scripts/test_ros2_camera.py --headless

    # Wait 30 seconds for DDS discovery, then in another terminal:
    source /opt/ros/jazzy/setup.bash
    ros2 topic list
    ros2 topic hz /camera/rgb

See Also:
    - Containerfile: NumPy version pinning explanation
    - phase9_ros2_test.py: Full integration test with same workaround
"""

from isaacsim import SimulationApp

import sys
HEADLESS = "--headless" in sys.argv

simulation_app = SimulationApp({
    "headless": HEADLESS,
    "renderer": "RayTracedLighting",
    "width": 1280,
    "height": 720,
})

import numpy as np
import omni
import threading
import time

from isaacsim.core.api import World
from isaacsim.core.utils import extensions, stage
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path

# =============================================================================
# CRITICAL: Enable ROS 2 extension BEFORE importing rclpy
# =============================================================================
# The isaacsim.ros2.bridge extension sets up the ROS 2 environment, including
# LD_LIBRARY_PATH for ROS 2 libraries. If you import rclpy before enabling
# this extension, the import will fail with missing library errors.
#
# DO NOT MOVE THIS SECTION - rclpy imports MUST come after enable_extension()
# =============================================================================
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ROS 2 imports (MUST be AFTER extension is enabled - see warning above)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


CAMERA_PATH = "/World/Camera"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


class CameraPublisherNode(Node):
    """ROS 2 node that publishes camera images."""

    def __init__(self):
        super().__init__('isaac_camera_publisher')
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.frame_id = "camera"
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
        # Simple pinhole camera model
        fx = fy = 500.0  # focal length
        cx = rgb_data.shape[1] / 2.0
        cy = rgb_data.shape[0] / 2.0
        info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.info_pub.publish(info_msg)
        self.seq += 1


def run_test():
    print("=" * 60)
    print("ROS 2 Camera Test - Manual Publishing Workaround")
    print("=" * 60)
    print(f"Mode: {'Headless' if HEADLESS else 'GUI'}")
    print()

    # Check numpy version
    try:
        print(f"[Info] NumPy version: {np.__version__}")
        if np.__version__.startswith("2."):
            print("[WARNING] NumPy 2.x detected - may cause dtype errors!")
    except Exception as e:
        print(f"[Warning] Could not check numpy version: {e}")
    print()

    # Initialize ROS 2 (extension already enabled at top of file)
    print("[ROS2] Initializing rclpy...", flush=True)
    rclpy.init()
    ros_node = CameraPublisherNode()
    print("[ROS2] Node created, publishing to /camera/rgb and /camera/camera_info", flush=True)

    # Create world
    print("\n[World] Creating world...", flush=True)
    world = World(stage_units_in_meters=1.0)

    # Load a simple environment
    assets_root_path = get_assets_root_path()
    if assets_root_path:
        env_usd = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        print(f"[World] Loading environment: {env_usd}", flush=True)
        stage.add_reference_to_stage(env_usd, "/World/environment")
    else:
        print("[World] No assets path, using empty scene", flush=True)

    # Multiple updates to ensure stage is ready
    for _ in range(10):
        simulation_app.update()

    # =========================================================================
    # CRITICAL: Use Camera class, NOT USD prim or OmniGraph ROS2CameraHelper
    # =========================================================================
    # The OmniGraph ROS2CameraHelper does NOT work in headless mode (Isaac Sim
    # 5.0+ regression). We must use the Camera class directly and manually
    # publish images to ROS 2. Do NOT refactor this to use OmniGraph - it will
    # break headless operation.
    # =========================================================================
    print("\n[Camera] Creating camera using Camera class...", flush=True)
    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array([0.0, -5.0, 2.0]),
        frequency=30,
        resolution=(CAMERA_WIDTH, CAMERA_HEIGHT),
        orientation=rot_utils.euler_angles_to_quats(np.array([70, 0, 0]), degrees=True),
    )

    # =========================================================================
    # CRITICAL: Initialize camera TWICE
    # =========================================================================
    # Per official Isaac Sim examples, the camera requires double initialization
    # to work reliably. The first call creates internal resources, the second
    # finalizes them. Removing either call may cause get_rgba() to return None
    # or empty arrays.
    #
    # Reference: Isaac Sim standalone_examples/api/isaacsim.sensors.camera
    # =========================================================================
    print("[Camera] Initializing camera...", flush=True)
    camera.initialize()
    simulation_app.update()
    camera.initialize()  # Second init required - do not remove
    simulation_app.update()

    print(f"[Camera] Camera created at {CAMERA_PATH}", flush=True)
    print(f"[Camera] Resolution: {CAMERA_WIDTH}x{CAMERA_HEIGHT}", flush=True)

    # Reset world
    print("\n[World] Resetting world...", flush=True)
    world.reset()

    # Extended warmup frames for headless mode
    warmup_frames = 50
    print(f"[Sim] Running {warmup_frames} warmup frames...", flush=True)
    for i in range(warmup_frames):
        world.step(render=True)

    # Verify camera is working
    print("\n[Camera] Verifying camera capture...", flush=True)
    rgba = camera.get_rgba()
    if rgba is not None and rgba.size > 0 and rgba.max() > 0:
        print(f"[Camera] SUCCESS: Capturing {rgba.shape} images, max={rgba.max()}, mean={rgba.mean():.1f}", flush=True)
    else:
        print("[Camera] WARNING: Camera may not be capturing properly", flush=True)

    # DDS discovery wait
    print("\n[ROS2] Waiting 5 seconds for DDS discovery...", flush=True)
    for i in range(50):
        rclpy.spin_once(ros_node, timeout_sec=0.1)
    print("[ROS2] DDS discovery period complete.", flush=True)

    # Run simulation loop
    print("\n[Test] Starting main loop (3000 steps, ~100 seconds)...", flush=True)
    print("[Test] Check ROS 2 topics in another terminal:", flush=True)
    print("       source /opt/ros/jazzy/setup.bash", flush=True)
    print("       ros2 topic list", flush=True)
    print("       ros2 topic hz /camera/rgb", flush=True)
    print()

    max_steps = 3000

    for step in range(max_steps):
        world.step(render=True)

        # Publish every frame for maximum rate
        try:
            rgba = camera.get_rgba()
            if rgba is not None and rgba.size > 0:
                ros_node.publish_image(rgba, time.time())
        except Exception as e:
            if step % 200 == 0:
                print(f"[Camera] Error getting/publishing image: {e}", flush=True)

        # Spin ROS 2 (non-blocking)
        rclpy.spin_once(ros_node, timeout_sec=0)

        if step % 200 == 0:
            print(f"[Progress] Step {step}/{max_steps}, published {ros_node.seq} images", flush=True)

    print(f"\n[Test] Complete! Published {ros_node.seq} images total.", flush=True)

    # Cleanup
    world.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    run_test()
