"""
ROS 2 Camera Bridge for Isaac Sim.

This module provides utilities for publishing Isaac Sim camera images
to ROS 2 topics. Use this instead of the direct Camera API to achieve
simulation-to-deployment parity.

Usage in Isaac Sim script:
    from environments.ros2_camera_bridge import ROS2CameraBridge

    # Initialize ROS 2 bridge
    bridge = ROS2CameraBridge(camera_prim_path="/World/quadrotor/body/isr_camera")
    bridge.initialize()

    # In simulation loop
    bridge.publish_frame()

The bridge publishes to:
    /camera/image_raw (sensor_msgs/Image) - RGB image
    /camera/camera_info (sensor_msgs/CameraInfo) - Camera calibration
"""

import numpy as np
from typing import Optional, Tuple


class ROS2CameraBridge:
    """
    Bridge between Isaac Sim camera and ROS 2 topics.

    This class wraps Isaac Sim's camera and publishes images via ROS 2,
    providing the same interface as a real camera on deployment hardware.
    """

    def __init__(
        self,
        camera_prim_path: str,
        image_topic: str = "/camera/image_raw",
        camera_info_topic: str = "/camera/camera_info",
        frame_id: str = "camera_link",
        resolution: Tuple[int, int] = (640, 480),
        frequency: int = 30,
    ):
        """
        Initialize the camera bridge.

        Args:
            camera_prim_path: USD path to the camera prim in Isaac Sim
            image_topic: ROS 2 topic for RGB images
            camera_info_topic: ROS 2 topic for camera info
            frame_id: TF frame ID for the camera
            resolution: (width, height) of camera images
            frequency: Camera capture frequency (Hz)
        """
        self.camera_prim_path = camera_prim_path
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.frame_id = frame_id
        self.resolution = resolution
        self.frequency = frequency

        # Will be initialized later
        self.camera = None
        self.ros_node = None
        self.image_pub = None
        self.camera_info_pub = None
        self.bridge = None

        self._initialized = False
        self._frame_count = 0

    def initialize(self):
        """
        Initialize the camera and ROS 2 publishers.

        Call this after Isaac Sim world is loaded but before simulation starts.
        """
        if self._initialized:
            return

        # Import Isaac Sim camera
        try:
            from omni.isaac.sensor import Camera
            from omni.isaac.core.utils.rotations import euler_angles_to_quat
        except ImportError:
            print("[ROS2CameraBridge] Warning: Isaac Sim not available, using mock mode")
            self._initialized = True
            return

        # Import ROS 2
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import Image, CameraInfo
            from cv_bridge import CvBridge
        except ImportError:
            print("[ROS2CameraBridge] Warning: ROS 2 not available")
            self._initialized = True
            return

        # Create camera
        self.camera = Camera(
            prim_path=self.camera_prim_path,
            frequency=self.frequency,
            resolution=self.resolution,
        )

        # Configure camera orientation (pointing down for ISR)
        # Gimbal angle: -45 degrees (nadir-facing)
        camera_orientation = euler_angles_to_quat(
            np.array([-45, 0, 0]) * np.pi / 180  # Roll=-45 for downward tilt
        )
        self.camera.set_local_pose(
            translation=np.array([0.0, 0.0, -0.1]),  # Below body
            orientation=camera_orientation
        )

        self.camera.initialize()
        self.camera.set_focal_length(4.5)  # ~90 degree FOV
        self.camera.set_clipping_range(0.1, 500.0)

        # Initialize ROS 2 if not already running
        if not rclpy.ok():
            rclpy.init()

        # Create ROS 2 node
        self.ros_node = rclpy.create_node('isaac_sim_camera_bridge')

        # Create publishers
        self.image_pub = self.ros_node.create_publisher(Image, self.image_topic, 10)
        self.camera_info_pub = self.ros_node.create_publisher(
            CameraInfo, self.camera_info_topic, 10
        )

        # CV bridge for image conversion
        self.bridge = CvBridge()

        # Create camera info message (static)
        self._camera_info_msg = self._create_camera_info()

        self._initialized = True
        print(f"[ROS2CameraBridge] Initialized camera at {self.camera_prim_path}")
        print(f"[ROS2CameraBridge] Publishing to {self.image_topic}")

    def _create_camera_info(self):
        """Create CameraInfo message with camera parameters."""
        from sensor_msgs.msg import CameraInfo

        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width = self.resolution[0]
        msg.height = self.resolution[1]

        # Approximate camera intrinsics for 90 degree FOV
        # fx = fy = width / (2 * tan(fov/2))
        fov_rad = np.pi / 2  # 90 degrees
        fx = self.resolution[0] / (2 * np.tan(fov_rad / 2))
        fy = fx  # Square pixels
        cx = self.resolution[0] / 2
        cy = self.resolution[1] / 2

        # Camera matrix K
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        # No distortion
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.distortion_model = 'plumb_bob'

        # Rectification matrix (identity for monocular)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix P
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return msg

    def publish_frame(self) -> Optional[np.ndarray]:
        """
        Capture frame from Isaac Sim camera and publish to ROS 2.

        Returns:
            RGB image as numpy array (H, W, 3) or None if not ready
        """
        if not self._initialized:
            return None

        if self.camera is None:
            return None

        # Get RGBA image from Isaac Sim
        rgba = self.camera.get_rgba()
        if rgba is None or rgba.size == 0:
            return None

        # Convert RGBA to RGB
        rgb = rgba[:, :, :3]

        # Publish to ROS 2
        if self.image_pub is not None:
            try:
                from builtin_interfaces.msg import Time

                # Create Image message
                img_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
                img_msg.header.frame_id = self.frame_id

                # Set timestamp
                now = self.ros_node.get_clock().now()
                img_msg.header.stamp = now.to_msg()

                # Publish image
                self.image_pub.publish(img_msg)

                # Publish camera info with matching timestamp
                self._camera_info_msg.header.stamp = img_msg.header.stamp
                self.camera_info_pub.publish(self._camera_info_msg)

                self._frame_count += 1

            except Exception as e:
                print(f"[ROS2CameraBridge] Publish error: {e}")

        return rgb

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get current frame without publishing.

        Returns:
            RGB image as numpy array (H, W, 3) or None if not ready
        """
        if not self._initialized or self.camera is None:
            return None

        rgba = self.camera.get_rgba()
        if rgba is None or rgba.size == 0:
            return None

        return rgba[:, :, :3]

    def spin_once(self, timeout_sec: float = 0.001):
        """Process pending ROS 2 callbacks."""
        if self.ros_node is not None:
            import rclpy
            rclpy.spin_once(self.ros_node, timeout_sec=timeout_sec)

    def shutdown(self):
        """Clean up ROS 2 resources."""
        if self.ros_node is not None:
            self.ros_node.destroy_node()
            self.ros_node = None

    @property
    def frame_count(self) -> int:
        """Number of frames published."""
        return self._frame_count

    @property
    def is_ready(self) -> bool:
        """Check if bridge is ready to publish."""
        return self._initialized and self.camera is not None


class MockCameraBridge:
    """
    Mock camera bridge for testing without Isaac Sim or ROS 2.

    Generates random images for testing the perception pipeline.
    """

    def __init__(
        self,
        image_topic: str = "/camera/image_raw",
        resolution: Tuple[int, int] = (640, 480),
    ):
        self.image_topic = image_topic
        self.resolution = resolution
        self._frame_count = 0
        self._initialized = True

    def initialize(self):
        pass

    def publish_frame(self) -> np.ndarray:
        """Generate random test image."""
        img = np.random.randint(0, 255, (*self.resolution[::-1], 3), dtype=np.uint8)
        self._frame_count += 1
        return img

    def get_frame(self) -> np.ndarray:
        return self.publish_frame()

    def spin_once(self, timeout_sec: float = 0.001):
        pass

    def shutdown(self):
        pass

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def is_ready(self) -> bool:
        return True
