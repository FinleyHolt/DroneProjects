#!/usr/bin/env python3
"""
3-axis gimbal controller for F-11 ISR camera payload.

Interfaces with Gazebo joint controllers in simulation.
Enforces gimbal angle limits from the UAV ontology (isr_extensions.kif).

Gimbal Limits (from ontology):
  - Yaw:   ±6.02 rad  (±345 degrees)
  - Pitch: ±2.09 rad  (±120 degrees)
  - Roll:  ±0.785 rad (±45 degrees)

Topics:
  Subscriptions:
    /f11/gimbal/command (flyby_f11_sensors/msg/GimbalCommand)

  Publishers:
    /f11/gimbal/state (flyby_f11_sensors/msg/GimbalState)
    /f11_isr/gimbal/yaw (std_msgs/Float64) - Gazebo joint controller
    /f11_isr/gimbal/pitch (std_msgs/Float64) - Gazebo joint controller
    /f11_isr/gimbal/roll (std_msgs/Float64) - Gazebo joint controller
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


class GimbalControllerNode(Node):
    """ROS 2 node for controlling the F-11 ISR 3-axis gimbal."""

    # Gimbal limits from ontology (isr_extensions.kif)
    # These values match the Gazebo joint limits in f11_isr_camera/model.sdf
    YAW_LIMITS = (-6.02, 6.02)      # ±345 degrees in radians
    PITCH_LIMITS = (-2.09, 2.09)    # ±120 degrees in radians
    ROLL_LIMITS = (-0.785, 0.785)   # ±45 degrees in radians

    def __init__(self):
        super().__init__('gimbal_controller')

        # Declare parameters
        self.declare_parameter('state_publish_rate', 50.0)  # Hz
        self.declare_parameter('enable_limit_warnings', True)

        # Get parameters
        state_rate = self.get_parameter('state_publish_rate').value
        self.enable_warnings = self.get_parameter('enable_limit_warnings').value

        # Current gimbal state
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.operational = True

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Import message types (generated at build time)
        # For now, use a compatible approach that works before messages are built
        try:
            from flyby_f11_sensors.msg import GimbalCommand, GimbalState
            self._msg_available = True

            # Subscribe to gimbal commands
            self.cmd_sub = self.create_subscription(
                GimbalCommand,
                '/f11/gimbal/command',
                self.command_callback,
                sensor_qos
            )

            # Publish gimbal state
            self.state_pub = self.create_publisher(
                GimbalState,
                '/f11/gimbal/state',
                sensor_qos
            )
        except ImportError:
            self._msg_available = False
            self.get_logger().warn(
                'Custom messages not yet built. '
                'Run colcon build to generate flyby_f11_sensors messages.'
            )

        # Publishers to Gazebo joint controllers
        self.yaw_pub = self.create_publisher(
            Float64, '/f11_isr/gimbal/yaw', 10
        )
        self.pitch_pub = self.create_publisher(
            Float64, '/f11_isr/gimbal/pitch', 10
        )
        self.roll_pub = self.create_publisher(
            Float64, '/f11_isr/gimbal/roll', 10
        )

        # State publishing timer
        timer_period = 1.0 / state_rate
        self.timer = self.create_timer(timer_period, self.publish_state)

        self.get_logger().info('Gimbal controller initialized')
        self.get_logger().info(
            f'Limits: yaw={self.YAW_LIMITS}, '
            f'pitch={self.PITCH_LIMITS}, '
            f'roll={self.ROLL_LIMITS}'
        )

    def clamp(self, value: float, limits: tuple) -> float:
        """Clamp value to specified limits."""
        return max(limits[0], min(limits[1], value))

    def command_callback(self, msg) -> None:
        """Handle incoming gimbal command."""
        # Clamp commanded angles to ontology-defined limits
        yaw = self.clamp(msg.yaw, self.YAW_LIMITS)
        pitch = self.clamp(msg.pitch, self.PITCH_LIMITS)
        roll = self.clamp(msg.roll, self.ROLL_LIMITS)

        # Warn if command was clamped
        if self.enable_warnings:
            if yaw != msg.yaw:
                self.get_logger().warn(
                    f'Yaw command {msg.yaw:.3f} clamped to {yaw:.3f} '
                    f'(limits: {self.YAW_LIMITS})'
                )
            if pitch != msg.pitch:
                self.get_logger().warn(
                    f'Pitch command {msg.pitch:.3f} clamped to {pitch:.3f} '
                    f'(limits: {self.PITCH_LIMITS})'
                )
            if roll != msg.roll:
                self.get_logger().warn(
                    f'Roll command {msg.roll:.3f} clamped to {roll:.3f} '
                    f'(limits: {self.ROLL_LIMITS})'
                )

        # Publish to Gazebo joint controllers
        yaw_msg = Float64()
        yaw_msg.data = yaw
        self.yaw_pub.publish(yaw_msg)

        pitch_msg = Float64()
        pitch_msg.data = pitch
        self.pitch_pub.publish(pitch_msg)

        roll_msg = Float64()
        roll_msg.data = roll
        self.roll_pub.publish(roll_msg)

        # Update internal state
        self.current_yaw = yaw
        self.current_pitch = pitch
        self.current_roll = roll

        self.get_logger().debug(
            f'Gimbal command: yaw={yaw:.3f}, pitch={pitch:.3f}, roll={roll:.3f}'
        )

    def publish_state(self) -> None:
        """Publish current gimbal state."""
        if not self._msg_available:
            return

        try:
            from flyby_f11_sensors.msg import GimbalState

            state = GimbalState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = 'gimbal_mount_link'

            state.yaw = self.current_yaw
            state.pitch = self.current_pitch
            state.roll = self.current_roll

            state.yaw_limits = list(self.YAW_LIMITS)
            state.pitch_limits = list(self.PITCH_LIMITS)
            state.roll_limits = list(self.ROLL_LIMITS)

            state.operational = self.operational

            self.state_pub.publish(state)
        except Exception as e:
            self.get_logger().error(f'Failed to publish state: {e}')

    def set_angles(self, yaw: float, pitch: float, roll: float) -> None:
        """
        Programmatic interface to set gimbal angles.

        Args:
            yaw: Yaw angle in radians
            pitch: Pitch angle in radians
            roll: Roll angle in radians
        """
        # Create a mock command for the callback
        class MockCommand:
            pass

        cmd = MockCommand()
        cmd.yaw = yaw
        cmd.pitch = pitch
        cmd.roll = roll

        self.command_callback(cmd)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = GimbalControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
