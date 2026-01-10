"""
ROS 2 State Estimator Node for Flyby F-11 UAV.

Aggregates sensor data and publishes unified UAV state.
Works with both Isaac Sim (via simulation bridge) and real hardware.

Topics:
    Subscribed:
        /mavros/state (mavros_msgs/State) - Flight controller state
        /mavros/local_position/pose (geometry_msgs/PoseStamped) - Position
        /mavros/local_position/velocity (geometry_msgs/TwistStamped) - Velocity
        /mavros/battery (sensor_msgs/BatteryState) - Battery
        /simulation/state (custom) - Isaac Sim state (simulation only)

    Published:
        /uav/state (flyby_msgs/UAVState) - Unified UAV state
"""

import numpy as np
from typing import Optional
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, Point, Vector3
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Bool

from flyby_msgs.msg import UAVState, FlightPhase


def quaternion_to_euler(q: Quaternion):
    """Convert quaternion to roll, pitch, yaw."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class StateEstimatorNode(Node):
    """ROS 2 node for aggregating sensor data into UAV state."""

    def __init__(self):
        super().__init__('state_estimator_node')

        # Parameters
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('use_simulation', True)
        self.declare_parameter('home_lat', 0.0)
        self.declare_parameter('home_lon', 0.0)
        self.declare_parameter('home_alt', 0.0)

        publish_rate = self.get_parameter('publish_rate').value
        self.use_simulation = self.get_parameter('use_simulation').value

        # State variables
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.angular_velocity = np.zeros(3)
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        self.battery_percent = 100.0
        self.battery_voltage = 22.2  # 6S typical
        self.battery_current = 0.0

        self.armed = False
        self.in_air = False
        self.comms_ok = True
        self.localization_ok = True
        self.comms_denied_time = 0.0

        self.home_position = np.zeros(3)
        self.home_set = False

        self.flight_phase = FlightPhase.PREFLIGHT

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers - MAVROS topics for real hardware
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            sensor_qos
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._velocity_callback,
            sensor_qos
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery_callback,
            sensor_qos
        )

        # Simulation-specific subscribers
        if self.use_simulation:
            # For Isaac Sim, we receive state differently
            # This will be populated by the simulation bridge
            pass

        # Comms status (can be toggled externally)
        self.comms_sub = self.create_subscription(
            Bool,
            '/comms/status',
            self._comms_callback,
            10
        )

        # Publisher
        self.state_pub = self.create_publisher(UAVState, '/uav/state', 10)

        # Timer
        self.state_timer = self.create_timer(1.0 / publish_rate, self._publish_state)

        self.get_logger().info(
            f'State Estimator Node initialized:\n'
            f'  Publish rate: {publish_rate}Hz\n'
            f'  Simulation mode: {self.use_simulation}'
        )

    def _pose_callback(self, msg: PoseStamped):
        """Update position and orientation from pose."""
        self.position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.orientation = msg.pose.orientation
        self.roll, self.pitch, self.yaw = quaternion_to_euler(self.orientation)

        # Set home on first valid position
        if not self.home_set and np.linalg.norm(self.position) > 0.1:
            self.home_position = self.position.copy()
            self.home_position[2] = 0.0  # Home altitude at ground
            self.home_set = True
            self.get_logger().info(f'Home position set: {self.home_position}')

    def _velocity_callback(self, msg: TwistStamped):
        """Update velocity."""
        self.velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
        self.angular_velocity = np.array([
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        ])

    def _battery_callback(self, msg: BatteryState):
        """Update battery state."""
        self.battery_percent = msg.percentage * 100.0  # Convert 0-1 to 0-100
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current

    def _comms_callback(self, msg: Bool):
        """Update comms status."""
        was_ok = self.comms_ok
        self.comms_ok = msg.data

        if was_ok and not self.comms_ok:
            self.comms_denied_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().warn('Communications DENIED')
        elif not was_ok and self.comms_ok:
            self.comms_denied_time = 0.0
            self.get_logger().info('Communications RESTORED')

    def _publish_state(self):
        """Publish aggregated UAV state."""
        msg = UAVState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Position
        msg.position.x = self.position[0]
        msg.position.y = self.position[1]
        msg.position.z = self.position[2]
        msg.altitude = self.position[2]  # Z is up in ENU/Isaac Sim
        msg.altitude_agl = self.position[2]  # Simplified - assumes flat ground

        # Velocity
        msg.velocity.x = self.velocity[0]
        msg.velocity.y = self.velocity[1]
        msg.velocity.z = self.velocity[2]
        msg.ground_speed = np.linalg.norm(self.velocity[:2])

        # Attitude
        msg.orientation = self.orientation
        msg.roll = self.roll
        msg.pitch = self.pitch
        msg.yaw = self.yaw

        # Angular velocity
        msg.angular_velocity.x = self.angular_velocity[0]
        msg.angular_velocity.y = self.angular_velocity[1]
        msg.angular_velocity.z = self.angular_velocity[2]

        # Battery
        msg.battery_percent = self.battery_percent
        msg.battery_voltage = self.battery_voltage
        msg.battery_current = self.battery_current
        # Estimate remaining time (rough approximation)
        if self.battery_current > 0.1:
            capacity_remaining = (self.battery_percent / 100.0) * 5000  # mAh (assume 5Ah)
            msg.estimated_remaining_sec = (capacity_remaining / self.battery_current) * 3.6
        else:
            msg.estimated_remaining_sec = 0.0

        # Flight phase
        msg.flight_phase.stamp = msg.header.stamp
        msg.flight_phase.phase = self.flight_phase

        # Navigation
        if self.home_set:
            to_home = self.home_position - self.position
            msg.distance_to_home = np.linalg.norm(to_home[:2])  # 2D distance
            msg.heading_to_home = np.arctan2(to_home[1], to_home[0])
        else:
            msg.distance_to_home = 0.0
            msg.heading_to_home = 0.0

        # Comms state
        msg.comms_ok = self.comms_ok
        if not self.comms_ok:
            now = self.get_clock().now().nanoseconds / 1e9
            msg.comms_denied_duration_sec = now - self.comms_denied_time
        else:
            msg.comms_denied_duration_sec = 0.0

        # Localization
        msg.localization_ok = self.localization_ok
        msg.position_uncertainty = 0.5  # Placeholder

        # Flags
        msg.armed = self.armed
        msg.in_air = self.position[2] > 0.5  # Simple in-air detection
        msg.home_position_valid = self.home_set

        self.state_pub.publish(msg)

    # Methods for simulation to directly set state
    def set_state_from_simulation(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        orientation: np.ndarray,  # quaternion [x,y,z,w]
        battery_percent: float,
        armed: bool = True
    ):
        """Set state directly from simulation (Isaac Sim)."""
        self.position = position
        self.velocity = velocity
        self.orientation.x = orientation[0]
        self.orientation.y = orientation[1]
        self.orientation.z = orientation[2]
        self.orientation.w = orientation[3]
        self.roll, self.pitch, self.yaw = quaternion_to_euler(self.orientation)
        self.battery_percent = battery_percent
        self.armed = armed

        if not self.home_set and np.linalg.norm(position) > 0.1:
            self.home_position = position.copy()
            self.home_position[2] = 0.0
            self.home_set = True


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
