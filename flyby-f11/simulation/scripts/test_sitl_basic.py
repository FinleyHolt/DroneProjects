#!/usr/bin/env python3
"""
Basic SITL connectivity test using pymavlink.

Tests:
1. Connect to SITL via MAVLink
2. Verify heartbeat
3. Request parameters
4. Arm vehicle
5. Command takeoff
6. Verify altitude change

Usage:
    python3 test_sitl_basic.py [--connection tcp:127.0.0.1:5762]
"""

import sys
import time
import argparse
from pymavlink import mavutil


class SITLBasicTest:
    def __init__(self, connection_string: str):
        self.connection_string = connection_string
        self.master = None
        self.test_results = []

    def log(self, message: str):
        """Print timestamped log message."""
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] {message}")

    def test_connection(self) -> bool:
        """Test 1: Connect to SITL."""
        self.log(f"Connecting to SITL at {self.connection_string}...")
        start = time.time()

        try:
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.log("✓ Connection established")
            duration = time.time() - start
            self.test_results.append(("Connection", True, duration))
            return True
        except Exception as e:
            self.log(f"✗ Connection failed: {e}")
            duration = time.time() - start
            self.test_results.append(("Connection", False, duration))
            return False

    def test_heartbeat(self, timeout: int = 30) -> bool:
        """Test 2: Verify heartbeat from autopilot."""
        self.log(f"Waiting for heartbeat (timeout: {timeout}s)...")
        start = time.time()

        try:
            msg = self.master.wait_heartbeat(timeout=timeout)
            duration = time.time() - start
            self.log(f"✓ Heartbeat received from system {msg.get_srcSystem()}")
            self.log(f"  Type: {msg.type}, Autopilot: {msg.autopilot}")
            self.test_results.append(("Heartbeat", True, duration))
            return True
        except Exception as e:
            duration = time.time() - start
            self.log(f"✗ No heartbeat received: {e}")
            self.test_results.append(("Heartbeat", False, duration))
            return False

    def test_parameters(self) -> bool:
        """Test 3: Request and verify parameter list."""
        self.log("Requesting parameter list...")
        start = time.time()

        try:
            # Request parameter list
            self.master.mav.param_request_list_send(
                self.master.target_system,
                self.master.target_component
            )

            # Wait for first parameter
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)

            if msg:
                duration = time.time() - start
                self.log(f"✓ Parameters available (total: {msg.param_count})")
                self.log(f"  First param: {msg.param_id} = {msg.param_value}")
                self.test_results.append(("Parameters", True, duration))
                return True
            else:
                raise TimeoutError("No parameter response")

        except Exception as e:
            duration = time.time() - start
            self.log(f"✗ Parameter request failed: {e}")
            self.test_results.append(("Parameters", False, duration))
            return False

    def test_mode_change(self, mode: str = "GUIDED") -> bool:
        """Test 4: Change flight mode."""
        self.log(f"Changing mode to {mode}...")
        start = time.time()

        try:
            # Get mode ID
            if not self.master.mode_mapping():
                raise ValueError("No mode mapping available")

            mode_id = self.master.mode_mapping()[mode]

            # Send mode change command
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )

            # Wait for mode change confirmation
            time.sleep(2)
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)

            if msg:
                duration = time.time() - start
                current_mode = self.master.flightmode
                self.log(f"✓ Mode changed to: {current_mode}")
                self.test_results.append(("Mode Change", True, duration))
                return True
            else:
                raise TimeoutError("No mode confirmation")

        except Exception as e:
            duration = time.time() - start
            self.log(f"✗ Mode change failed: {e}")
            self.test_results.append(("Mode Change", False, duration))
            return False

    def test_arm(self) -> bool:
        """Test 5: Arm the vehicle."""
        self.log("Arming vehicle...")
        start = time.time()

        try:
            # Send arm command
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )

            # Wait for arm confirmation
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)

            if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                duration = time.time() - start
                self.log("✓ Vehicle armed")
                self.test_results.append(("Arm", True, duration))
                return True
            else:
                raise ValueError(f"Arm rejected: {msg.result if msg else 'timeout'}")

        except Exception as e:
            duration = time.time() - start
            self.log(f"✗ Arming failed: {e}")
            self.test_results.append(("Arm", False, duration))
            return False

    def test_takeoff(self, altitude: float = 10.0) -> bool:
        """Test 6: Command takeoff."""
        self.log(f"Commanding takeoff to {altitude}m...")
        start = time.time()

        try:
            # Send takeoff command
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0, altitude
            )

            # Wait for command acknowledgment
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)

            if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.log("✓ Takeoff command accepted")

                # Monitor altitude for 10 seconds
                self.log("Monitoring altitude...")
                initial_alt = 0
                max_alt = 0

                for _ in range(20):  # 10 seconds at 0.5s intervals
                    msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                    if msg:
                        alt = msg.relative_alt / 1000.0  # Convert mm to m
                        max_alt = max(max_alt, alt)
                        self.log(f"  Altitude: {alt:.2f}m (max: {max_alt:.2f}m)")
                    time.sleep(0.5)

                duration = time.time() - start

                if max_alt > 1.0:  # Verify vehicle is climbing
                    self.log(f"✓ Takeoff successful (reached {max_alt:.2f}m)")
                    self.test_results.append(("Takeoff", True, duration))
                    return True
                else:
                    raise ValueError(f"No altitude gain detected (max: {max_alt:.2f}m)")

            else:
                raise ValueError(f"Takeoff rejected: {msg.result if msg else 'timeout'}")

        except Exception as e:
            duration = time.time() - start
            self.log(f"✗ Takeoff failed: {e}")
            self.test_results.append(("Takeoff", False, duration))
            return False

    def print_summary(self):
        """Print test summary."""
        self.log("")
        self.log("=" * 60)
        self.log("TEST SUMMARY")
        self.log("=" * 60)

        total_tests = len(self.test_results)
        passed_tests = sum(1 for _, passed, _ in self.test_results if passed)

        for name, passed, duration in self.test_results:
            status = "✓ PASS" if passed else "✗ FAIL"
            self.log(f"{status:8} {name:20} ({duration:.2f}s)")

        self.log("-" * 60)
        self.log(f"Total: {passed_tests}/{total_tests} tests passed")
        self.log("=" * 60)

        return passed_tests == total_tests

    def run_all_tests(self) -> bool:
        """Run all tests in sequence."""
        self.log("Starting SITL Basic Integration Tests")
        self.log("=" * 60)

        # Test sequence - stop on first failure
        if not self.test_connection():
            self.print_summary()
            return False

        if not self.test_heartbeat():
            self.print_summary()
            return False

        if not self.test_parameters():
            self.print_summary()
            return False

        if not self.test_mode_change():
            self.print_summary()
            return False

        if not self.test_arm():
            self.print_summary()
            return False

        if not self.test_takeoff():
            self.print_summary()
            return False

        # All tests passed
        return self.print_summary()


def main():
    parser = argparse.ArgumentParser(description="SITL Basic Integration Test")
    parser.add_argument(
        "--connection",
        default="tcp:127.0.0.1:5762",
        help="MAVLink connection string (default: tcp:127.0.0.1:5762)"
    )
    args = parser.parse_args()

    test = SITLBasicTest(args.connection)

    try:
        success = test.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        test.log("\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        test.log(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
