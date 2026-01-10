#!/usr/bin/env python3
"""
state_grounding_node.py - ROS 2 node for state-to-TPTP grounding

Subscribes to MAVLink/MAVROS sensor topics and converts ROS messages
to TPTP facts stored in the HotFactBuffer. This implements the
perception-to-reasoning bridge described in ADR-002.

Subscribed Topics:
    /mavros/battery (sensor_msgs/BatteryState)
    /mavros/local_position/pose (geometry_msgs/PoseStamped)
    /mavros/state (mavros_msgs/State)

The node updates the HotFactBuffer which is shared with VampireNode
for fact injection during query construction.

Usage:
    ros2 run vampire_bridge state_grounding_node

Parameters:
    uav_id: UAV identifier for TPTP facts (default: drone_alpha)
    home_lat: Home position latitude (for distance calculation)
    home_lon: Home position longitude
    battery_critical_threshold: Battery percentage for critical status (default: 0.20)
    battery_low_threshold: Battery percentage for low status (default: 0.30)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

from vampire_bridge.hot_fact_buffer import HotFactBuffer

# Try to import MAVROS messages - may not be available in all environments
try:
    from mavros_msgs.msg import State as MavrosState
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False
    MavrosState = None


class StateGroundingNode(Node):
    """
    ROS 2 node that grounds sensor state to TPTP facts.

    Converts ROS 2 messages from MAVROS topics into symbolic TPTP facts
    for the Vampire theorem prover. Facts are stored in a HotFactBuffer
    that provides thread-safe, atomic snapshots for query construction.
    """

    def __init__(self, hot_buffer: Optional[HotFactBuffer] = None):
        """
        Initialize the state grounding node.

        Args:
            hot_buffer: Optional pre-existing HotFactBuffer. If None,
                       creates a new one (useful for testing).
        """
        super().__init__('state_grounding')

        # Declare parameters
        self._declare_parameters()

        # Get parameter values
        self._uav_id = self.get_parameter('uav_id').value
        self._home_lat = self.get_parameter('home_lat').value
        self._home_lon = self.get_parameter('home_lon').value
        self._battery_critical = self.get_parameter('battery_critical_threshold').value
        self._battery_low = self.get_parameter('battery_low_threshold').value

        # Initialize or use provided HotFactBuffer
        self._hot_buffer = hot_buffer or HotFactBuffer(uav_id=self._uav_id)

        # Track current position for distance calculations
        self._current_position: Optional[tuple[float, float, float]] = None

        # QoS profile for sensor data (best effort, keep last)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions
        self._create_subscriptions(sensor_qos)

        # Statistics
        self._msg_count = {
            'battery': 0,
            'pose': 0,
            'state': 0,
        }

        self.get_logger().info(
            f'State grounding node initialized. UAV ID: {self._uav_id}'
        )

    def _declare_parameters(self) -> None:
        """Declare all node parameters with defaults."""
        self.declare_parameter('uav_id', 'drone_alpha')
        self.declare_parameter('home_lat', 0.0)
        self.declare_parameter('home_lon', 0.0)
        self.declare_parameter('battery_critical_threshold', 0.20)
        self.declare_parameter('battery_low_threshold', 0.30)

    def _create_subscriptions(self, sensor_qos: QoSProfile) -> None:
        """Create ROS 2 subscriptions for sensor topics."""

        # Battery state subscription
        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._on_battery,
            sensor_qos
        )

        # Local position subscription
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._on_pose,
            sensor_qos
        )

        # MAVROS state subscription (if available)
        if MAVROS_AVAILABLE:
            self.create_subscription(
                MavrosState,
                '/mavros/state',
                self._on_mavros_state,
                sensor_qos
            )
        else:
            self.get_logger().warning(
                'mavros_msgs not available - /mavros/state will not be subscribed'
            )

    def _on_battery(self, msg: BatteryState) -> None:
        """
        Handle battery state message.

        Converts battery percentage to symbolic status and numeric level.

        Args:
            msg: BatteryState message from /mavros/battery
        """
        self._msg_count['battery'] += 1

        # Determine symbolic battery status
        percentage = msg.percentage
        if percentage < self._battery_critical:
            status = 'battery_critical'
        elif percentage < self._battery_low:
            status = 'battery_low'
        else:
            status = 'battery_nominal'

        # Update hot buffer with both symbolic status and numeric level
        self._hot_buffer.update(
            'battery_status',
            source_topic='/mavros/battery',
            status=status
        )

        self._hot_buffer.update(
            'current_battery_level',
            source_topic='/mavros/battery',
            level=f'{percentage * 100:.1f}'
        )

        # Log critical battery warnings
        if status == 'battery_critical':
            self.get_logger().warning(
                f'CRITICAL: Battery at {percentage * 100:.1f}%'
            )

    def _on_pose(self, msg: PoseStamped) -> None:
        """
        Handle local position message.

        Updates position fact and calculates distance to home.

        Args:
            msg: PoseStamped message from /mavros/local_position/pose
        """
        self._msg_count['pose'] += 1

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self._current_position = (x, y, z)

        # Create position identifier (discretized for stability)
        pos_id = f'pos_{int(x)}_{int(y)}_{int(z)}'
        self._hot_buffer.update(
            'position',
            source_topic='/mavros/local_position/pose',
            pos_id=pos_id
        )

        # Calculate distance to home (in local frame, home is origin)
        dist_to_home = math.sqrt(x * x + y * y)
        self._hot_buffer.update(
            'distance_to_home',
            source_topic='/mavros/local_position/pose',
            dist=f'{dist_to_home:.1f}'
        )

    def _on_mavros_state(self, msg: 'MavrosState') -> None:
        """
        Handle MAVROS state message.

        Updates armed status, flight mode, and comms status.

        Args:
            msg: State message from /mavros/state
        """
        self._msg_count['state'] += 1

        # Armed status
        armed = 'true' if msg.armed else 'false'
        self._hot_buffer.update(
            'armed_status',
            source_topic='/mavros/state',
            armed=armed
        )

        # Flight mode
        mode = msg.mode.lower().replace(' ', '_') if msg.mode else 'unknown'
        self._hot_buffer.update(
            'flight_mode',
            source_topic='/mavros/state',
            mode=mode
        )

        # Comms status (based on connection to FCU)
        if msg.connected:
            comms_status = 'comms_operational'
        else:
            comms_status = 'comms_denied'

        self._hot_buffer.update(
            'comms_status',
            source_topic='/mavros/state',
            status=comms_status
        )

        # Log connection loss
        if not msg.connected:
            self.get_logger().error('CRITICAL: Lost connection to FCU!')

    @property
    def hot_buffer(self) -> HotFactBuffer:
        """Return the HotFactBuffer for external access."""
        return self._hot_buffer

    def get_statistics(self) -> dict:
        """
        Get node statistics.

        Returns:
            Dict with message counts and buffer stats
        """
        buffer_stats = self._hot_buffer.get_stats()
        return {
            'message_counts': self._msg_count.copy(),
            'buffer': buffer_stats,
            'uav_id': self._uav_id,
            'current_position': self._current_position,
        }


def main(args=None):
    """Main entry point for state_grounding_node."""
    rclpy.init(args=args)

    node = StateGroundingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down state grounding node')
    finally:
        stats = node.get_statistics()
        node.get_logger().info(f'Final statistics: {stats}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
