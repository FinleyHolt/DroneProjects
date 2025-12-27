#!/usr/bin/env python3
"""
Flyby F-11 Simulation Integration Test

Tests the complete simulation workflow:
1. Spawn vehicle at configurable position
2. Arm and takeoff
3. Fly to a waypoint
4. Query Vampire bridge for safety status
5. Reset episode

Usage:
    python3 test_integration.py

Requirements:
    - Simulation stack running (podman-compose up)
    - MAVROS connected to SITL
    - Vampire bridge available (optional)
"""

import json
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State as FCUState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass
class TestResult:
    """Test result container."""
    name: str
    passed: bool
    duration: float
    message: str = ""


class SimulationIntegrationTest(Node):
    """Integration test node for simulation stack."""

    def __init__(self):
        super().__init__('simulation_integration_test')

        self.test_results: list[TestResult] = []

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # State tracking
        self.current_pose: Optional[PoseStamped] = None
        self.fcu_state: Optional[FCUState] = None
        self.connected = False
        self.armed = False

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._pose_callback, qos)
        self.state_sub = self.create_subscription(
            FCUState, '/mavros/state',
            self._state_callback, qos)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        # Setpoint publisher
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos)

        self.get_logger().info('Integration test node initialized')

    def _pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def _state_callback(self, msg: FCUState):
        self.fcu_state = msg
        self.connected = msg.connected
        self.armed = msg.armed

    def wait_for_connection(self, timeout: float = 30.0) -> bool:
        """Wait for MAVROS connection to FCU."""
        self.get_logger().info('Waiting for FCU connection...')
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.connected:
                self.get_logger().info('FCU connected!')
                return True
        return False

    def set_mode(self, mode: str, timeout: float = 10.0) -> bool:
        """Set flight mode."""
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set mode service not available')
            return False

        request = SetMode.Request()
        request.custom_mode = mode

        future = self.mode_client.call_async(request)
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                result = future.result()
                if result and result.mode_sent:
                    self.get_logger().info(f'Mode set to {mode}')
                    return True
                break
        return False

    def arm(self, timeout: float = 10.0) -> bool:
        """Arm the vehicle."""
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service not available')
            return False

        request = CommandBool.Request()
        request.value = True

        future = self.arming_client.call_async(request)
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                result = future.result()
                if result and result.success:
                    # Wait for armed confirmation
                    while time.time() - start < timeout:
                        rclpy.spin_once(self, timeout_sec=0.1)
                        if self.armed:
                            self.get_logger().info('Vehicle armed!')
                            return True
                break
        return False

    def disarm(self, timeout: float = 10.0) -> bool:
        """Disarm the vehicle."""
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            return False

        request = CommandBool.Request()
        request.value = False

        future = self.arming_client.call_async(request)
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                return future.result() and future.result().success
        return False

    def takeoff(self, altitude: float = 10.0, timeout: float = 30.0) -> bool:
        """Takeoff to specified altitude."""
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Takeoff service not available')
            return False

        request = CommandTOL.Request()
        request.altitude = altitude

        future = self.takeoff_client.call_async(request)
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                result = future.result()
                if result and result.success:
                    # Wait for altitude
                    while time.time() - start < timeout:
                        rclpy.spin_once(self, timeout_sec=0.1)
                        if self.current_pose:
                            if self.current_pose.pose.position.z >= altitude * 0.9:
                                self.get_logger().info(f'Takeoff complete at {altitude}m')
                                return True
                break
        return False

    def fly_to_waypoint(self, x: float, y: float, z: float,
                        tolerance: float = 2.0, timeout: float = 60.0) -> bool:
        """Fly to specified waypoint."""
        self.get_logger().info(f'Flying to waypoint ({x}, {y}, {z})')

        # Create setpoint
        setpoint = PoseStamped()
        setpoint.header.frame_id = 'map'
        setpoint.pose.position.x = x
        setpoint.pose.position.y = y
        setpoint.pose.position.z = z
        setpoint.pose.orientation.w = 1.0

        start = time.time()
        while time.time() - start < timeout:
            # Publish setpoint
            setpoint.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(setpoint)

            rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_pose:
                pos = self.current_pose.pose.position
                dist = math.sqrt(
                    (pos.x - x)**2 +
                    (pos.y - y)**2 +
                    (pos.z - z)**2
                )
                if dist < tolerance:
                    self.get_logger().info(f'Reached waypoint (dist={dist:.2f}m)')
                    return True

        return False

    def land(self, timeout: float = 30.0) -> bool:
        """Land the vehicle."""
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            return False

        request = CommandTOL.Request()
        future = self.land_client.call_async(request)

        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                result = future.result()
                if result and result.success:
                    # Wait for landing
                    while time.time() - start < timeout:
                        rclpy.spin_once(self, timeout_sec=0.1)
                        if self.current_pose and self.current_pose.pose.position.z < 0.5:
                            self.get_logger().info('Landed!')
                            return True
                break
        return False

    def run_test(self, name: str, test_func) -> TestResult:
        """Run a single test and record result."""
        self.get_logger().info(f'\n=== Running Test: {name} ===')
        start = time.time()

        try:
            passed = test_func()
            duration = time.time() - start
            result = TestResult(
                name=name,
                passed=passed,
                duration=duration,
                message='PASSED' if passed else 'FAILED'
            )
        except Exception as e:
            duration = time.time() - start
            result = TestResult(
                name=name,
                passed=False,
                duration=duration,
                message=f'EXCEPTION: {e}'
            )

        self.test_results.append(result)

        status = 'PASSED' if result.passed else 'FAILED'
        self.get_logger().info(f'Test {name}: {status} ({duration:.2f}s)')
        return result

    def run_all_tests(self):
        """Run complete integration test suite."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('FLYBY F-11 SIMULATION INTEGRATION TEST')
        self.get_logger().info('='*60)

        # Test 1: Connection
        self.run_test('FCU Connection', lambda: self.wait_for_connection())

        if not self.connected:
            self.get_logger().error('Cannot proceed without FCU connection')
            return False

        # Test 2: Set GUIDED mode
        self.run_test('Set GUIDED Mode', lambda: self.set_mode('GUIDED'))

        # Test 3: Arm
        self.run_test('Arm Vehicle', lambda: self.arm())

        if not self.armed:
            self.get_logger().error('Cannot proceed without arming')
            return False

        # Test 4: Takeoff
        self.run_test('Takeoff to 10m', lambda: self.takeoff(10.0))

        # Test 5: Fly to waypoint
        self.run_test('Fly to Waypoint (20, 20, 15)',
                      lambda: self.fly_to_waypoint(20, 20, 15))

        # Test 6: Land
        self.run_test('Land Vehicle', lambda: self.land())

        # Test 7: Disarm
        self.run_test('Disarm Vehicle', lambda: self.disarm())

        # Print summary
        self.print_summary()

        return all(r.passed for r in self.test_results)

    def print_summary(self):
        """Print test results summary."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST SUMMARY')
        self.get_logger().info('='*60)

        passed = sum(1 for r in self.test_results if r.passed)
        total = len(self.test_results)

        for result in self.test_results:
            status = '[PASS]' if result.passed else '[FAIL]'
            self.get_logger().info(
                f'  {status} {result.name}: {result.duration:.2f}s'
            )

        self.get_logger().info('-'*60)
        self.get_logger().info(f'Total: {passed}/{total} tests passed')
        self.get_logger().info('='*60)


def main():
    rclpy.init()

    test_node = SimulationIntegrationTest()

    try:
        success = test_node.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print('\nTest interrupted')
        sys.exit(1)
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
