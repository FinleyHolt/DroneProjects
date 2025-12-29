#!/usr/bin/env python3
"""
Simple MAVLink script to arm and fly the drone.
Run this while Isaac Sim is already running with PX4.
"""

import time
import sys
from pymavlink import mavutil

print("=" * 50)
print("Connecting to PX4...")
print("=" * 50)

# Connect to PX4
mav = mavutil.mavlink_connection('udpin:localhost:14550', source_system=255)
mav.wait_heartbeat(timeout=10)
print(f"Connected! System {mav.target_system}, Component {mav.target_component}")

def send_position(x, y, z, yaw=0):
    """Send position setpoint."""
    mav.mav.set_position_target_local_ned_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        x, y, -z,  # NED (z down)
        0, 0, 0, 0, 0, 0, yaw, 0
    )

# Send setpoints before arming (required for OFFBOARD)
print("\nSending initial setpoints...")
for i in range(50):
    send_position(0, 0, 3.0)
    time.sleep(0.02)

# Set OFFBOARD mode
print("Setting OFFBOARD mode...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    6, 0, 0, 0, 0, 0  # 6 = OFFBOARD
)
time.sleep(0.5)

# Arm
print("Arming...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    1, 0, 0, 0, 0, 0, 0
)
time.sleep(1)

# Set OFFBOARD again
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    6, 0, 0, 0, 0, 0
)

print("\n>>> TAKING OFF to 3m...")
for i in range(150):  # 3 seconds
    send_position(0, 0, 3.0)
    time.sleep(0.02)

print(">>> Flying FORWARD 3m...")
for i in range(150):
    send_position(3, 0, 3.0)
    time.sleep(0.02)

print(">>> Flying RIGHT 3m...")
for i in range(150):
    send_position(3, 3, 3.0)
    time.sleep(0.02)

print(">>> Flying BACK 3m...")
for i in range(150):
    send_position(0, 3, 3.0)
    time.sleep(0.02)

print(">>> Flying LEFT (home)...")
for i in range(150):
    send_position(0, 0, 3.0)
    time.sleep(0.02)

print("\n>>> LANDING...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
    0, 0, 0, 0, 0, 0, 0
)

print("\nFlight complete!")
mav.close()
