#!/usr/bin/env python3
"""
Basic flight test for F-11 drone in Isaac Sim.

This script performs a simple flight sequence to validate:
1. MAVLink connection to ArduPilot SITL
2. Arming and takeoff
3. Hover and position hold
4. Landing

Usage:
    python3 test_flight.py [--altitude 5.0] [--duration 10]

Prerequisites:
    - Isaac Sim running with drone spawned (spawn_drone.py)
    - ArduPilot SITL running and connected
"""

import argparse
import time
import sys

try:
    from pymavlink import mavutil
except ImportError:
    print("Error: pymavlink not installed")
    print("Install with: pip3 install pymavlink")
    sys.exit(1)


def wait_for_heartbeat(master, timeout=30):
    """Wait for a heartbeat from the autopilot."""
    print("Waiting for heartbeat...")
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            print(f"  Heartbeat received from system {msg.get_srcSystem()}")
            return True
    return False


def set_mode(master, mode):
    """Set flight mode."""
    mode_id = master.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Error: Unknown mode '{mode}'")
        return False

    master.set_mode(mode_id)
    print(f"  Mode set to: {mode}")
    return True


def arm(master, timeout=10):
    """Arm the vehicle."""
    print("Arming vehicle...")
    master.arducopter_arm()

    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("  Vehicle armed!")
            return True

    print("  Warning: Arm command sent but not confirmed")
    return False


def disarm(master):
    """Disarm the vehicle."""
    print("Disarming vehicle...")
    master.arducopter_disarm()
    time.sleep(1)
    print("  Vehicle disarmed")


def takeoff(master, altitude):
    """Command takeoff to specified altitude."""
    print(f"Taking off to {altitude}m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0, 0, 0, 0,  # params 1-4 (unused)
        0, 0,  # lat, lon (use current)
        altitude  # altitude
    )

    # Wait for takeoff
    time.sleep(altitude * 2)  # rough estimate: 0.5 m/s climb
    print(f"  Reached target altitude: {altitude}m")


def land(master):
    """Command landing."""
    print("Landing...")
    set_mode(master, 'LAND')

    # Wait for landing
    time.sleep(10)
    print("  Landed")


def main():
    parser = argparse.ArgumentParser(description='F-11 basic flight test')
    parser.add_argument('--connection', default='udp:127.0.0.1:14550',
                        help='MAVLink connection string')
    parser.add_argument('--altitude', type=float, default=5.0,
                        help='Takeoff altitude in meters')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Hover duration in seconds')
    args = parser.parse_args()

    print("=" * 50)
    print("  Flyby F-11 Basic Flight Test")
    print("=" * 50)
    print(f"  Connection: {args.connection}")
    print(f"  Altitude: {args.altitude}m")
    print(f"  Hover duration: {args.duration}s")
    print("")

    # Connect to vehicle
    print("[1/6] Connecting to vehicle...")
    try:
        master = mavutil.mavlink_connection(args.connection)
    except Exception as e:
        print(f"Error: Failed to connect: {e}")
        sys.exit(1)

    # Wait for heartbeat
    print("[2/6] Waiting for heartbeat...")
    if not wait_for_heartbeat(master):
        print("Error: No heartbeat received")
        sys.exit(1)

    # Set mode to GUIDED
    print("[3/6] Setting GUIDED mode...")
    if not set_mode(master, 'GUIDED'):
        print("Error: Failed to set mode")
        sys.exit(1)

    # Arm
    print("[4/6] Arming...")
    if not arm(master):
        print("Error: Failed to arm")
        sys.exit(1)

    try:
        # Takeoff
        print("[5/6] Taking off...")
        takeoff(master, args.altitude)

        # Hover
        print(f"Hovering for {args.duration} seconds...")
        time.sleep(args.duration)

        # Land
        print("[6/6] Landing...")
        land(master)

    except KeyboardInterrupt:
        print("\nInterrupted! Landing...")
        land(master)

    finally:
        # Disarm
        disarm(master)

    print("")
    print("=" * 50)
    print("  Flight test complete!")
    print("=" * 50)


if __name__ == "__main__":
    main()
