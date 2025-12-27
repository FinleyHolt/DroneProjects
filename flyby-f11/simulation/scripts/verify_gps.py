#!/usr/bin/env python3
"""
GPS/NavSat Sensor Verification Script for F-11 ISR Drone

This script subscribes to the GPS/NavSat topic from Gazebo simulation
and verifies:
1. GPS data is publishing correctly
2. Coordinates match world spherical_coordinates config
3. Update rate matches sensor specification (10 Hz)

Expected Location (from training_world.sdf):
- Latitude: 33.3853 deg N
- Longitude: -117.5653 deg W
- Elevation: 100 m

Usage:
    PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 verify_gps.py
"""

import time
import math
from dataclasses import dataclass
from typing import Optional

# Gazebo transport bindings
from gz.transport13 import Node
from gz.msgs10.navsat_pb2 import NavSat

# Expected values from world file
EXPECTED_LAT = 33.3853
EXPECTED_LON = -117.5653
EXPECTED_ELEVATION = 100.0
EXPECTED_UPDATE_RATE = 10.0  # Hz

# Tolerance for coordinate comparison (degrees)
# 0.0001 degrees is approximately 11 meters
COORDINATE_TOLERANCE = 0.001
ALTITUDE_TOLERANCE = 5.0  # meters


@dataclass
class GPSSample:
    """Single GPS measurement sample."""
    timestamp_sec: float
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    velocity_east: float
    velocity_north: float
    velocity_up: float
    frame_id: str


class GPSVerifier:
    """Verifies GPS sensor data from Gazebo simulation."""

    def __init__(self):
        self.node = Node()
        self.samples: list[GPSSample] = []
        self.topic = "/world/flyby_training/model/f11_isr/link/gps_link/sensor/gps_sensor/navsat"
        self.collecting = False
        self.max_samples = 50  # Collect 50 samples (~5 seconds at 10 Hz)

    def _callback(self, msg: NavSat) -> None:
        """Process incoming GPS messages."""
        if not self.collecting:
            return

        if len(self.samples) >= self.max_samples:
            return

        timestamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        sample = GPSSample(
            timestamp_sec=timestamp,
            latitude_deg=msg.latitude_deg,
            longitude_deg=msg.longitude_deg,
            altitude_m=msg.altitude,
            velocity_east=msg.velocity_east,
            velocity_north=msg.velocity_north,
            velocity_up=msg.velocity_up,
            frame_id=msg.frame_id
        )
        self.samples.append(sample)

    def collect_samples(self, duration_sec: float = 5.0) -> bool:
        """Collect GPS samples for the specified duration."""
        print(f"Subscribing to: {self.topic}")

        if not self.node.subscribe(NavSat, self.topic, self._callback):
            print("ERROR: Failed to subscribe to GPS topic")
            return False

        print(f"Collecting GPS samples for {duration_sec} seconds...")
        self.collecting = True

        start_time = time.time()
        while time.time() - start_time < duration_sec and len(self.samples) < self.max_samples:
            time.sleep(0.05)

        self.collecting = False
        print(f"Collected {len(self.samples)} samples")
        return len(self.samples) > 0

    def calculate_update_rate(self) -> Optional[float]:
        """Calculate actual update rate from timestamps.

        Note: DDS/Gazebo transport may buffer old messages, so we filter
        to only use messages with timestamps close to the maximum timestamp
        (i.e., the most recent simulation time).
        """
        if len(self.samples) < 2:
            return None

        # Sort by timestamp
        sorted_samples = sorted(self.samples, key=lambda s: s.timestamp_sec)

        # Find the maximum timestamp and filter to recent samples only
        # This handles DDS buffering of old messages from previous sim runs
        max_ts = sorted_samples[-1].timestamp_sec
        recent_threshold = 10.0  # Only use samples within 10s of most recent
        recent_samples = [s for s in sorted_samples
                          if max_ts - s.timestamp_sec < recent_threshold]

        if len(recent_samples) < 2:
            return None

        # Calculate time differences between consecutive samples
        time_diffs = []
        for i in range(1, len(recent_samples)):
            dt = recent_samples[i].timestamp_sec - recent_samples[i-1].timestamp_sec
            # Only count differences that look like valid update intervals (0.05-0.5s)
            if 0.05 < dt < 0.5:
                time_diffs.append(dt)

        if not time_diffs:
            return None

        avg_dt = sum(time_diffs) / len(time_diffs)
        return 1.0 / avg_dt if avg_dt > 0 else None

    def verify_coordinates(self) -> dict:
        """Verify GPS coordinates match expected values."""
        if not self.samples:
            return {"passed": False, "error": "No samples collected"}

        # Sort by timestamp and use only recent samples
        sorted_samples = sorted(self.samples, key=lambda s: s.timestamp_sec)
        max_ts = sorted_samples[-1].timestamp_sec
        recent_samples = [s for s in sorted_samples
                          if max_ts - s.timestamp_sec < 10.0]

        # Use average of recent samples only
        avg_lat = sum(s.latitude_deg for s in recent_samples) / len(recent_samples)
        avg_lon = sum(s.longitude_deg for s in recent_samples) / len(recent_samples)
        avg_alt = sum(s.altitude_m for s in recent_samples) / len(recent_samples)

        lat_diff = abs(avg_lat - EXPECTED_LAT)
        lon_diff = abs(avg_lon - EXPECTED_LON)
        alt_diff = abs(avg_alt - EXPECTED_ELEVATION)

        lat_ok = lat_diff < COORDINATE_TOLERANCE
        lon_ok = lon_diff < COORDINATE_TOLERANCE
        alt_ok = alt_diff < ALTITUDE_TOLERANCE

        return {
            "passed": lat_ok and lon_ok and alt_ok,
            "average_latitude": avg_lat,
            "average_longitude": avg_lon,
            "average_altitude": avg_alt,
            "expected_latitude": EXPECTED_LAT,
            "expected_longitude": EXPECTED_LON,
            "expected_elevation": EXPECTED_ELEVATION,
            "latitude_error_deg": lat_diff,
            "longitude_error_deg": lon_diff,
            "altitude_error_m": alt_diff,
            "latitude_ok": lat_ok,
            "longitude_ok": lon_ok,
            "altitude_ok": alt_ok,
        }

    def print_sample_data(self, num_samples: int = 5) -> None:
        """Print sample GPS data."""
        print("\n" + "=" * 60)
        print("GPS SAMPLE DATA")
        print("=" * 60)

        for i, sample in enumerate(self.samples[:num_samples]):
            print(f"\nSample {i + 1}:")
            print(f"  Timestamp:    {sample.timestamp_sec:.3f} s")
            print(f"  Latitude:     {sample.latitude_deg:.6f} deg")
            print(f"  Longitude:    {sample.longitude_deg:.6f} deg")
            print(f"  Altitude:     {sample.altitude_m:.2f} m")
            print(f"  Velocity E:   {sample.velocity_east:.6f} m/s")
            print(f"  Velocity N:   {sample.velocity_north:.6f} m/s")
            print(f"  Velocity Up:  {sample.velocity_up:.6f} m/s")
            print(f"  Frame ID:     {sample.frame_id}")

    def print_verification_report(self) -> None:
        """Print full verification report."""
        print("\n" + "=" * 60)
        print("GPS SENSOR VERIFICATION REPORT")
        print("=" * 60)
        print(f"Topic: {self.topic}")
        print(f"Total samples collected: {len(self.samples)}")

        # Coordinate verification
        coord_result = self.verify_coordinates()
        print("\n--- Coordinate Verification ---")
        if "error" in coord_result:
            print(f"ERROR: {coord_result['error']}")
        else:
            print(f"Expected Latitude:  {coord_result['expected_latitude']:.4f} deg")
            print(f"Measured Latitude:  {coord_result['average_latitude']:.6f} deg")
            print(f"  Error: {coord_result['latitude_error_deg']:.6f} deg "
                  f"({'PASS' if coord_result['latitude_ok'] else 'FAIL'})")

            print(f"\nExpected Longitude: {coord_result['expected_longitude']:.4f} deg")
            print(f"Measured Longitude: {coord_result['average_longitude']:.6f} deg")
            print(f"  Error: {coord_result['longitude_error_deg']:.6f} deg "
                  f"({'PASS' if coord_result['longitude_ok'] else 'FAIL'})")

            print(f"\nExpected Elevation: {coord_result['expected_elevation']:.1f} m")
            print(f"Measured Altitude:  {coord_result['average_altitude']:.2f} m")
            print(f"  Error: {coord_result['altitude_error_m']:.2f} m "
                  f"({'PASS' if coord_result['altitude_ok'] else 'FAIL'})")

        # Update rate verification
        print("\n--- Update Rate Verification ---")
        update_rate = self.calculate_update_rate()
        if update_rate:
            rate_error = abs(update_rate - EXPECTED_UPDATE_RATE)
            rate_ok = rate_error < 2.0  # Allow 2 Hz tolerance
            print(f"Expected Rate: {EXPECTED_UPDATE_RATE:.1f} Hz")
            print(f"Measured Rate: {update_rate:.1f} Hz")
            print(f"  Error: {rate_error:.1f} Hz ({'PASS' if rate_ok else 'FAIL'})")
        else:
            print("Could not calculate update rate (insufficient samples)")

        # Overall result
        print("\n" + "=" * 60)
        overall_pass = coord_result.get("passed", False) and update_rate and rate_error < 2.0
        if overall_pass:
            print("OVERALL: PASS - GPS sensor is functioning correctly")
        else:
            print("OVERALL: FAIL - GPS sensor verification failed")
        print("=" * 60)


def main():
    print("F-11 ISR Drone - GPS/NavSat Sensor Verification")
    print("-" * 50)

    verifier = GPSVerifier()

    # Collect samples
    if not verifier.collect_samples(duration_sec=5.0):
        print("ERROR: Failed to collect GPS samples")
        print("Make sure the simulation is running and unpaused")
        return 1

    # Print sample data
    verifier.print_sample_data(num_samples=5)

    # Print verification report
    verifier.print_verification_report()

    return 0


if __name__ == "__main__":
    exit(main())
