#!/usr/bin/env python3
"""
ISR Camera interface node for F-11 Sony A7R-class camera payload.

Provides camera state monitoring and image republishing from Gazebo simulation.
Camera specifications from ontology (isr_extensions.kif):
  - Resolution: 1920x1080
  - Frame rate: 30 Hz
  - Horizontal FOV: 1.047 rad (60 degrees)
  - Range: 0.1m - 1000m

Topics:
  Subscriptions:
    /f11_isr/camera/image_raw (sensor_msgs/Image) - From Gazebo bridge

  Publishers:
    /f11/camera/image_raw (sensor_msgs/Image) - Republished for ROS 2 nodes
    /f11/camera/state (flyby_f11_sensors/msg/ISRCameraState)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class ISRCameraNode(Node):
    """ROS 2 node for ISR camera interface."""

    # Camera specifications from ontology (isr_extensions.kif)
    IMAGE_WIDTH = 1920
    IMAGE_HEIGHT = 1080
    FRAME_RATE = 30.0
    HORIZONTAL_FOV = 1.047  # radians (60 degrees)
    CLIP_NEAR = 0.1
    CLIP_FAR = 1000.0

    def __init__(self):
        super().__init__('isr_camera')

        # Declare parameters
        self.declare_parameter('republish_images', True)
        self.declare_parameter('state_publish_rate', 10.0)  # Hz

        # Get parameters
        self.republish = self.get_parameter('republish_images').value
        state_rate = self.get_parameter('state_publish_rate').value

        # Camera state
        self.camera_active = False
        self.mode = 'rgb'
        self.zoom_level = 1.0
        self.exposure = 1.0
        self.operational = False
        self.last_image_time = None
        self.image_count = 0

        # QoS for camera images (best effort for high-bandwidth sensor data)
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # QoS for state (reliable)
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscribe to Gazebo camera images
        self.image_sub = self.create_subscription(
            Image,
            '/f11_isr/camera/image_raw',
            self.image_callback,
            image_qos
        )

        # Republish images on standard ROS 2 topic
        if self.republish:
            self.image_pub = self.create_publisher(
                Image,
                '/f11/camera/image_raw',
                image_qos
            )

        # Publish camera state
        try:
            from flyby_f11_sensors.msg import ISRCameraState
            self._msg_available = True

            self.state_pub = self.create_publisher(
                ISRCameraState,
                '/f11/camera/state',
                state_qos
            )
        except ImportError:
            self._msg_available = False
            self.get_logger().warn(
                'Custom messages not yet built. '
                'Run colcon build to generate flyby_f11_sensors messages.'
            )

        # State publishing timer
        timer_period = 1.0 / state_rate
        self.state_timer = self.create_timer(timer_period, self.publish_state)

        # Camera timeout timer (check if we're receiving images)
        self.timeout_timer = self.create_timer(1.0, self.check_camera_health)

        self.get_logger().info('ISR camera node initialized')
        self.get_logger().info(
            f'Camera specs: {self.IMAGE_WIDTH}x{self.IMAGE_HEIGHT} @ {self.FRAME_RATE}Hz'
        )

    def image_callback(self, msg: Image) -> None:
        """Handle incoming camera image."""
        self.last_image_time = self.get_clock().now()
        self.image_count += 1
        self.camera_active = True
        self.operational = True

        # Republish on ROS 2 topic
        if self.republish:
            # Update frame_id to use F-11 frame
            msg.header.frame_id = 'camera_link'
            self.image_pub.publish(msg)

        if self.image_count == 1:
            self.get_logger().info('First camera image received')
        elif self.image_count % 300 == 0:  # Log every 10 seconds at 30 fps
            self.get_logger().debug(f'Camera images received: {self.image_count}')

    def check_camera_health(self) -> None:
        """Check if camera is still publishing images."""
        if self.last_image_time is None:
            return

        time_since_last = (
            self.get_clock().now() - self.last_image_time
        ).nanoseconds / 1e9

        if time_since_last > 2.0:  # 2 second timeout
            if self.operational:
                self.get_logger().warn(
                    f'Camera timeout: no images for {time_since_last:.1f}s'
                )
                self.operational = False
                self.camera_active = False

    def publish_state(self) -> None:
        """Publish current camera state."""
        if not self._msg_available:
            return

        try:
            from flyby_f11_sensors.msg import ISRCameraState

            state = ISRCameraState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = 'camera_link'

            state.camera_active = self.camera_active
            state.mode = self.mode
            state.zoom_level = self.zoom_level
            state.exposure = self.exposure
            state.operational = self.operational
            state.image_width = self.IMAGE_WIDTH
            state.image_height = self.IMAGE_HEIGHT
            state.frame_rate = self.FRAME_RATE

            self.state_pub.publish(state)
        except Exception as e:
            self.get_logger().error(f'Failed to publish state: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = ISRCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
