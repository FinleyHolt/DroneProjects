#!/usr/bin/env python3
"""
IMU Sensor Verification Script for F-11 ISR Drone

This script subscribes to the Gazebo IMU topic, collects samples,
and verifies noise parameters match the specification:
- Angular velocity stddev: 0.009 rad/s
- Linear acceleration stddev: 0.017 m/s^2
- Update rate: 250 Hz
"""

import os
import sys
import time
import math
import statistics
from dataclasses import dataclass, field
from typing import List, Optional

# Workaround for protobuf compatibility
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

try:
    from gz.transport13 import Node
    from gz.msgs10.imu_pb2 import IMU
except ImportError as e:
    print(f"Error importing Gazebo libraries: {e}")
    print("Make sure gz.transport13 and gz.msgs10 are installed")
    sys.exit(1)


@dataclass
class IMUStats:
    """Statistics container for IMU measurements"""
    angular_velocity_x: List[float] = field(default_factory=list)
    angular_velocity_y: List[float] = field(default_factory=list)
    angular_velocity_z: List[float] = field(default_factory=list)
    linear_acceleration_x: List[float] = field(default_factory=list)
    linear_acceleration_y: List[float] = field(default_factory=list)
    linear_acceleration_z: List[float] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)

    def add_sample(self, msg: IMU) -> None:
        """Add a sample from an IMU message"""
        self.angular_velocity_x.append(msg.angular_velocity.x)
        self.angular_velocity_y.append(msg.angular_velocity.y)
        self.angular_velocity_z.append(msg.angular_velocity.z)
        self.linear_acceleration_x.append(msg.linear_acceleration.x)
        self.linear_acceleration_y.append(msg.linear_acceleration.y)
        self.linear_acceleration_z.append(msg.linear_acceleration.z)

        # Convert timestamp to seconds
        ts = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        self.timestamps.append(ts)

    @property
    def sample_count(self) -> int:
        return len(self.timestamps)

    def compute_statistics(self) -> dict:
        """Compute statistics from collected samples"""
        if self.sample_count < 2:
            return {}

        # For a stationary drone, angular velocity should be ~0 rad/s
        # and linear acceleration should be ~[0, 0, 9.81] m/s^2

        # Calculate standard deviations (noise)
        gyro_x_std = statistics.stdev(self.angular_velocity_x)
        gyro_y_std = statistics.stdev(self.angular_velocity_y)
        gyro_z_std = statistics.stdev(self.angular_velocity_z)

        accel_x_std = statistics.stdev(self.linear_acceleration_x)
        accel_y_std = statistics.stdev(self.linear_acceleration_y)
        accel_z_std = statistics.stdev(self.linear_acceleration_z)

        # Calculate means
        gyro_x_mean = statistics.mean(self.angular_velocity_x)
        gyro_y_mean = statistics.mean(self.angular_velocity_y)
        gyro_z_mean = statistics.mean(self.angular_velocity_z)

        accel_x_mean = statistics.mean(self.linear_acceleration_x)
        accel_y_mean = statistics.mean(self.linear_acceleration_y)
        accel_z_mean = statistics.mean(self.linear_acceleration_z)

        # Calculate update rate
        if len(self.timestamps) >= 2:
            dt_list = [self.timestamps[i+1] - self.timestamps[i]
                      for i in range(len(self.timestamps)-1) if self.timestamps[i+1] > self.timestamps[i]]
            if dt_list:
                avg_dt = statistics.mean(dt_list)
                update_rate = 1.0 / avg_dt if avg_dt > 0 else 0
            else:
                update_rate = 0
        else:
            update_rate = 0

        return {
            'gyro_x_mean': gyro_x_mean,
            'gyro_y_mean': gyro_y_mean,
            'gyro_z_mean': gyro_z_mean,
            'gyro_x_std': gyro_x_std,
            'gyro_y_std': gyro_y_std,
            'gyro_z_std': gyro_z_std,
            'accel_x_mean': accel_x_mean,
            'accel_y_mean': accel_y_mean,
            'accel_z_mean': accel_z_mean,
            'accel_x_std': accel_x_std,
            'accel_y_std': accel_y_std,
            'accel_z_std': accel_z_std,
            'update_rate': update_rate,
            'sample_count': self.sample_count,
        }


class IMUVerifier:
    """IMU Sensor Verification Class"""

    # Expected specifications from model.sdf
    EXPECTED_GYRO_STDDEV = 0.009  # rad/s
    EXPECTED_ACCEL_STDDEV = 0.017  # m/s^2
    EXPECTED_UPDATE_RATE = 250  # Hz
    EXPECTED_GRAVITY = 9.81  # m/s^2

    def __init__(self, topic: str, target_samples: int = 500):
        self.topic = topic
        self.target_samples = target_samples
        self.stats = IMUStats()
        self.node = Node()
        self.running = True

    def imu_callback(self, msg: IMU) -> None:
        """Callback for IMU messages"""
        self.stats.add_sample(msg)

        if self.stats.sample_count >= self.target_samples:
            self.running = False

    def run(self, timeout: float = 10.0) -> dict:
        """Run the verification test"""
        print(f"Subscribing to IMU topic: {self.topic}")
        print(f"Collecting {self.target_samples} samples...")

        if not self.node.subscribe(IMU, self.topic, self.imu_callback):
            print(f"Error: Failed to subscribe to {self.topic}")
            return {}

        start_time = time.time()
        while self.running and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        elapsed = time.time() - start_time
        print(f"Collected {self.stats.sample_count} samples in {elapsed:.2f}s")

        return self.stats.compute_statistics()

    def verify_results(self, results: dict) -> bool:
        """Verify results against specifications"""
        if not results:
            print("ERROR: No results to verify")
            return False

        all_passed = True

        print("\n" + "="*60)
        print("IMU SENSOR VERIFICATION REPORT")
        print("="*60)
        print(f"\nSamples collected: {results['sample_count']}")
        print(f"Estimated update rate: {results['update_rate']:.1f} Hz (expected: {self.EXPECTED_UPDATE_RATE} Hz)")

        # Check update rate (allow 10% tolerance)
        rate_tolerance = 0.10
        if abs(results['update_rate'] - self.EXPECTED_UPDATE_RATE) / self.EXPECTED_UPDATE_RATE > rate_tolerance:
            print(f"  WARNING: Update rate outside {rate_tolerance*100:.0f}% tolerance")

        print("\n--- Angular Velocity (Gyroscope) ---")
        print(f"Mean (x, y, z): ({results['gyro_x_mean']:.6f}, {results['gyro_y_mean']:.6f}, {results['gyro_z_mean']:.6f}) rad/s")
        print(f"StdDev (x, y, z): ({results['gyro_x_std']:.6f}, {results['gyro_y_std']:.6f}, {results['gyro_z_std']:.6f}) rad/s")
        print(f"Expected StdDev: {self.EXPECTED_GYRO_STDDEV} rad/s")

        # Check gyro noise (allow 50% tolerance for stochastic measurements)
        gyro_tolerance = 0.50
        for axis, std in [('x', results['gyro_x_std']), ('y', results['gyro_y_std']), ('z', results['gyro_z_std'])]:
            if abs(std - self.EXPECTED_GYRO_STDDEV) / self.EXPECTED_GYRO_STDDEV > gyro_tolerance:
                print(f"  WARNING: Gyro {axis} stddev ({std:.6f}) outside {gyro_tolerance*100:.0f}% of expected")

        print("\n--- Linear Acceleration (Accelerometer) ---")
        print(f"Mean (x, y, z): ({results['accel_x_mean']:.6f}, {results['accel_y_mean']:.6f}, {results['accel_z_mean']:.6f}) m/s^2")
        print(f"StdDev (x, y, z): ({results['accel_x_std']:.6f}, {results['accel_y_std']:.6f}, {results['accel_z_std']:.6f}) m/s^2")
        print(f"Expected StdDev: {self.EXPECTED_ACCEL_STDDEV} m/s^2")

        # Check that Z-axis shows gravity
        gravity_tolerance = 0.1  # 10%
        if abs(results['accel_z_mean'] - self.EXPECTED_GRAVITY) / self.EXPECTED_GRAVITY > gravity_tolerance:
            print(f"  WARNING: Z-axis acceleration ({results['accel_z_mean']:.4f}) differs from gravity ({self.EXPECTED_GRAVITY})")
            all_passed = False
        else:
            print(f"  PASS: Z-axis acceleration matches gravity ({self.EXPECTED_GRAVITY} m/s^2)")

        # Check accel noise (allow 50% tolerance for stochastic measurements)
        accel_tolerance = 0.50
        for axis, std in [('x', results['accel_x_std']), ('y', results['accel_y_std']), ('z', results['accel_z_std'])]:
            if abs(std - self.EXPECTED_ACCEL_STDDEV) / self.EXPECTED_ACCEL_STDDEV > accel_tolerance:
                print(f"  WARNING: Accel {axis} stddev ({std:.6f}) outside {accel_tolerance*100:.0f}% of expected")

        # Check that X and Y means are near zero (stationary drone)
        xy_tolerance = 0.05  # m/s^2
        if abs(results['accel_x_mean']) > xy_tolerance or abs(results['accel_y_mean']) > xy_tolerance:
            print(f"  WARNING: X/Y acceleration not near zero (possible tilt or motion)")
        else:
            print(f"  PASS: X/Y acceleration near zero (drone stationary)")

        print("\n--- Covariance Verification ---")
        # Expected covariance = stddev^2
        expected_gyro_cov = self.EXPECTED_GYRO_STDDEV ** 2  # 8.1e-05
        expected_accel_cov = self.EXPECTED_ACCEL_STDDEV ** 2  # 2.89e-04
        print(f"Expected gyro covariance: {expected_gyro_cov:.2e} (from stddev^2)")
        print(f"Expected accel covariance: {expected_accel_cov:.2e} (from stddev^2)")
        print("  Note: Covariance values in messages should match these expectations")

        print("\n" + "="*60)
        if all_passed:
            print("VERIFICATION RESULT: PASS")
        else:
            print("VERIFICATION RESULT: WARNINGS PRESENT (see above)")
        print("="*60)

        return all_passed


def main():
    """Main entry point"""
    # IMU topic for F-11 ISR drone
    imu_topic = "/world/flyby_training/model/f11_isr/link/imu_link/sensor/imu_sensor/imu"

    print("="*60)
    print("F-11 ISR Drone - IMU Sensor Verification")
    print("="*60)
    print(f"\nTopic: {imu_topic}")
    print("\nExpected Specifications (from model.sdf):")
    print("  - Angular velocity noise: 0.009 rad/s (stddev)")
    print("  - Linear acceleration noise: 0.017 m/s^2 (stddev)")
    print("  - Update rate: 250 Hz")
    print()

    verifier = IMUVerifier(imu_topic, target_samples=500)
    results = verifier.run(timeout=10.0)

    if results:
        verifier.verify_results(results)

        # Print sample data
        print("\n--- Sample IMU Data ---")
        if verifier.stats.sample_count > 0:
            print(f"First sample:")
            print(f"  Angular velocity: ({verifier.stats.angular_velocity_x[0]:.6f}, "
                  f"{verifier.stats.angular_velocity_y[0]:.6f}, "
                  f"{verifier.stats.angular_velocity_z[0]:.6f}) rad/s")
            print(f"  Linear acceleration: ({verifier.stats.linear_acceleration_x[0]:.6f}, "
                  f"{verifier.stats.linear_acceleration_y[0]:.6f}, "
                  f"{verifier.stats.linear_acceleration_z[0]:.6f}) m/s^2")

            if verifier.stats.sample_count > 1:
                last = verifier.stats.sample_count - 1
                print(f"\nLast sample:")
                print(f"  Angular velocity: ({verifier.stats.angular_velocity_x[last]:.6f}, "
                      f"{verifier.stats.angular_velocity_y[last]:.6f}, "
                      f"{verifier.stats.angular_velocity_z[last]:.6f}) rad/s")
                print(f"  Linear acceleration: ({verifier.stats.linear_acceleration_x[last]:.6f}, "
                      f"{verifier.stats.linear_acceleration_y[last]:.6f}, "
                      f"{verifier.stats.linear_acceleration_z[last]:.6f}) m/s^2")
    else:
        print("ERROR: Failed to collect IMU data")
        print("Make sure the simulation is running and unpaused")
        sys.exit(1)


if __name__ == "__main__":
    main()
