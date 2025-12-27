#!/usr/bin/env python3
"""
Test script for F-11 ISR gimbal control system.

This script verifies gimbal movement by:
1. Unpausing the simulation
2. Commanding different gimbal angles (yaw, pitch, roll)
3. Capturing camera images at each position to verify movement

Gimbal Control Interface:
    Topics (gz.msgs.Double):
        /f11_isr/gimbal/yaw   - Yaw angle in radians (limits: +/- 6.02 rad)
        /f11_isr/gimbal/pitch - Pitch angle in radians (limits: +/- 2.09 rad)
        /f11_isr/gimbal/roll  - Roll angle in radians (limits: +/- 0.785 rad)

Usage:
    podman exec flyby-f11-isr python3 /workspace/simulation/test_gimbal_control.py

Author: Finley Holt
"""
import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import sys
import time
import subprocess
import numpy as np
from PIL import Image as PILImage
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.double_pb2 import Double


# Configuration
WORLD_NAME = "flyby_training"
MODEL_NAME = "f11_isr"
CAMERA_TOPIC = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/camera_link/sensor/isr_camera/image"
OUTPUT_DIR = "/simulation/gimbal_test_output"
IMAGE_TIMEOUT = 5.0  # seconds


class GimbalController:
    """Controller for F-11 ISR gimbal."""

    def __init__(self):
        self.node = Node()

        # Publishers for gimbal control
        self.yaw_pub = self.node.advertise(f"/f11_isr/gimbal/yaw", Double)
        self.pitch_pub = self.node.advertise(f"/f11_isr/gimbal/pitch", Double)
        self.roll_pub = self.node.advertise(f"/f11_isr/gimbal/roll", Double)

        # Allow time for publishers to be established
        time.sleep(0.5)

        # Image capture state
        self.image_received = False
        self.image_data = None
        self.width = 0
        self.height = 0

    def set_yaw(self, angle_rad: float):
        """Set gimbal yaw angle in radians. Limits: +/- 6.02 rad"""
        msg = Double()
        msg.data = max(-6.02, min(6.02, angle_rad))
        self.yaw_pub.publish(msg)
        print(f"  Yaw command: {msg.data:.3f} rad ({np.degrees(msg.data):.1f} deg)")

    def set_pitch(self, angle_rad: float):
        """Set gimbal pitch angle in radians. Limits: +/- 2.09 rad"""
        msg = Double()
        msg.data = max(-2.09, min(2.09, angle_rad))
        self.pitch_pub.publish(msg)
        print(f"  Pitch command: {msg.data:.3f} rad ({np.degrees(msg.data):.1f} deg)")

    def set_roll(self, angle_rad: float):
        """Set gimbal roll angle in radians. Limits: +/- 0.785 rad"""
        msg = Double()
        msg.data = max(-0.785, min(0.785, angle_rad))
        self.roll_pub.publish(msg)
        print(f"  Roll command: {msg.data:.3f} rad ({np.degrees(msg.data):.1f} deg)")

    def set_gimbal(self, yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0):
        """Set all gimbal angles at once."""
        self.set_yaw(yaw)
        self.set_pitch(pitch)
        self.set_roll(roll)

    def image_callback(self, msg):
        """Callback for image messages."""
        if self.image_received:
            return
        self.width = msg.width
        self.height = msg.height
        self.image_data = msg.data
        self.image_received = True

    def capture_image(self, filename: str) -> bool:
        """Capture a single image from the camera."""
        self.image_received = False
        self.image_data = None

        if not self.node.subscribe(Image, CAMERA_TOPIC, self.image_callback):
            print(f"ERROR: Failed to subscribe to {CAMERA_TOPIC}")
            return False

        start_time = time.time()
        while not self.image_received:
            time.sleep(0.05)
            if time.time() - start_time > IMAGE_TIMEOUT:
                print(f"ERROR: Timeout waiting for image after {IMAGE_TIMEOUT}s")
                return False

        # Convert and save image
        img_bytes = bytes(self.image_data)
        bytes_per_pixel = len(img_bytes) // (self.width * self.height)

        if bytes_per_pixel == 3:
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            img_array = img_array.reshape((self.height, self.width, 3))
            mode = 'RGB'
        elif bytes_per_pixel == 4:
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            img_array = img_array.reshape((self.height, self.width, 4))
            mode = 'RGBA'
        else:
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            img_array = img_array.reshape((self.height, self.width, 3))
            mode = 'RGB'

        pil_image = PILImage.fromarray(img_array, mode=mode)
        pil_image.save(filename)
        print(f"  Saved: {filename}")
        return True


def unpause_simulation():
    """Unpause the Gazebo simulation."""
    print("Unpausing simulation...")
    cmd = [
        "gz", "service",
        "-s", f"/world/{WORLD_NAME}/control",
        "--reqtype", "gz.msgs.WorldControl",
        "--reptype", "gz.msgs.Boolean",
        "--req", "pause: false",
        "--timeout", "5000"
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("  Simulation unpaused successfully")
            return True
        else:
            print(f"  Warning: {result.stderr}")
            return True  # Continue anyway, might already be unpaused
    except subprocess.TimeoutExpired:
        print("  Warning: Timeout unpausing simulation")
        return True
    except Exception as e:
        print(f"  Error unpausing: {e}")
        return False


def main():
    print("=" * 60)
    print("F-11 ISR Gimbal Control Test")
    print("=" * 60)

    # Create output directory
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"\nOutput directory: {OUTPUT_DIR}")

    # Unpause simulation
    if not unpause_simulation():
        print("Failed to unpause simulation")
        return 1

    # Wait for simulation to start
    time.sleep(1.0)

    # Initialize gimbal controller
    print("\nInitializing gimbal controller...")
    gimbal = GimbalController()

    # Test sequence: various gimbal positions
    test_positions = [
        # (name, yaw_rad, pitch_rad, roll_rad)
        ("center", 0.0, 0.0, 0.0),
        ("yaw_left", 1.0, 0.0, 0.0),           # ~57 degrees left
        ("yaw_right", -1.0, 0.0, 0.0),         # ~57 degrees right
        ("pitch_down", 0.0, 0.5, 0.0),         # ~29 degrees down
        ("pitch_up", 0.0, -0.5, 0.0),          # ~29 degrees up
        ("roll_left", 0.0, 0.0, 0.4),          # ~23 degrees left roll
        ("roll_right", 0.0, 0.0, -0.4),        # ~23 degrees right roll
        ("combined_1", 0.5, 0.3, 0.2),         # Combined movement
        ("combined_2", -0.5, -0.3, -0.2),      # Combined movement opposite
        ("extreme_yaw", 3.0, 0.0, 0.0),        # ~172 degrees yaw
        ("look_down", 0.0, 1.5, 0.0),          # ~86 degrees down (near limit)
    ]

    print("\n" + "-" * 60)
    print("Running gimbal test sequence...")
    print("-" * 60)

    successful_tests = 0
    failed_tests = 0

    for name, yaw, pitch, roll in test_positions:
        print(f"\nTest: {name}")
        gimbal.set_gimbal(yaw, pitch, roll)

        # Wait for gimbal to reach position
        time.sleep(1.5)

        # Capture image
        filename = os.path.join(OUTPUT_DIR, f"gimbal_{name}.png")
        if gimbal.capture_image(filename):
            successful_tests += 1
        else:
            failed_tests += 1

    # Return to center position
    print("\nReturning to center position...")
    gimbal.set_gimbal(0.0, 0.0, 0.0)
    time.sleep(1.0)

    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    print(f"  Successful: {successful_tests}/{len(test_positions)}")
    print(f"  Failed: {failed_tests}/{len(test_positions)}")
    print(f"  Images saved to: {OUTPUT_DIR}")

    if failed_tests == 0:
        print("\n[PASS] All gimbal tests completed successfully!")
        return 0
    else:
        print(f"\n[PARTIAL] {failed_tests} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
