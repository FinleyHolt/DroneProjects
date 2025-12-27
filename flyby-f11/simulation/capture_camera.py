#!/usr/bin/env python3
"""
Capture a single frame from the F-11 ISR camera in Gazebo simulation.
Saves the image as PNG to the specified output path.
"""
import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import sys
import time
import numpy as np
from PIL import Image as PILImage
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# Configuration
TOPIC = "/world/flyby_training/model/f11_isr/link/camera_link/sensor/isr_camera/image"
OUTPUT_PATH = "/workspace/simulation/test_camera_output.png"
TIMEOUT = 10.0  # seconds

class ImageCapture:
    def __init__(self):
        self.image_received = False
        self.image_data = None
        self.width = 0
        self.height = 0
        self.pixel_format = ""

    def callback(self, msg):
        """Callback for image messages."""
        if self.image_received:
            return

        self.width = msg.width
        self.height = msg.height
        self.pixel_format = msg.pixel_format_type
        self.image_data = msg.data
        self.image_received = True
        print(f"Received image: {self.width}x{self.height}, format: {self.pixel_format}")

def main():
    print(f"Subscribing to: {TOPIC}")
    print(f"Output path: {OUTPUT_PATH}")

    # Create node and subscribe
    node = Node()
    capture = ImageCapture()

    if not node.subscribe(Image, TOPIC, capture.callback):
        print(f"ERROR: Failed to subscribe to {TOPIC}")
        return 1

    print("Waiting for image...")
    start_time = time.time()

    while not capture.image_received:
        time.sleep(0.1)
        if time.time() - start_time > TIMEOUT:
            print(f"ERROR: Timeout waiting for image after {TIMEOUT}s")
            return 1

    # Convert to numpy array and save
    print("Processing image...")

    # The image data is raw bytes, format is typically RGB8 or similar
    img_bytes = bytes(capture.image_data)

    # Determine pixel size based on format
    # R8G8B8 = 3 bytes per pixel
    bytes_per_pixel = len(img_bytes) // (capture.width * capture.height)
    print(f"Bytes per pixel: {bytes_per_pixel}")

    if bytes_per_pixel == 3:
        # RGB format
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        img_array = img_array.reshape((capture.height, capture.width, 3))
        mode = 'RGB'
    elif bytes_per_pixel == 4:
        # RGBA format
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        img_array = img_array.reshape((capture.height, capture.width, 4))
        mode = 'RGBA'
    elif bytes_per_pixel == 1:
        # Grayscale
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        img_array = img_array.reshape((capture.height, capture.width))
        mode = 'L'
    else:
        print(f"WARNING: Unknown format with {bytes_per_pixel} bytes per pixel, assuming RGB")
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        img_array = img_array.reshape((capture.height, capture.width, 3))
        mode = 'RGB'

    # Create PIL image and save
    pil_image = PILImage.fromarray(img_array, mode=mode)
    pil_image.save(OUTPUT_PATH)

    print(f"Image saved to: {OUTPUT_PATH}")
    print(f"Image size: {capture.width}x{capture.height}")

    return 0

if __name__ == "__main__":
    sys.exit(main())
