# agents_perception

Perception pipeline for LLMDrone using Intel RealSense cameras.

## Overview

This package provides vision and tracking capabilities using:
- **Intel RealSense T265**: Visual-Inertial Odometry (VIO) for accurate 6-DOF pose tracking
- **Intel RealSense D455**: Stereo depth camera for object detection, obstacle avoidance, and 3D mapping

## Cameras

### T265 Tracking Camera
- **Purpose**: Visual odometry and localization in GPS-denied environments
- **Outputs**:
  - 6-DOF pose at 200Hz
  - Dual fisheye images (848x800 @ 30fps)
  - IMU data (gyro @ 200Hz, accel @ 62Hz)
- **Use Cases**: Navigation, position hold, return-to-home without GPS

### D455 Depth Camera
- **Purpose**: Perception, obstacle avoidance, object detection
- **Outputs**:
  - RGB image (640x480 @ 30fps)
  - Depth map (640x480 @ 30fps)
  - Stereo IR images
  - 3D point cloud
  - IMU data
- **Use Cases**: Terrain mapping, object detection, vision-based landing

## Quick Start

### Using Docker (Recommended)

```bash
# Build multi-arch RealSense image
./scripts/build_realsense.sh

# Run interactive container with cameras
docker compose up realsense

# Auto-start camera nodes
docker compose up realsense-autostart

# Check camera topics (from host or another container)
ros2 topic list | grep camera
```

### Native Installation

```bash
# Build the package
cd ros2_ws
colcon build --packages-select agents_perception

# Source workspace
source install/setup.bash

# Launch both cameras
ros2 launch agents_perception realsense.launch.py

# Launch only T265
ros2 launch agents_perception realsense.launch.py enable_d455:=false

# Launch only D455
ros2 launch agents_perception realsense.launch.py enable_t265:=false
```

## ROS 2 Topics

### T265 Topics
- `/t265/odom/sample` - Visual odometry
- `/t265/accel/sample` - Accelerometer data
- `/t265/gyro/sample` - Gyroscope data
- `/t265/fisheye1/image_raw` - Left fisheye image
- `/t265/fisheye2/image_raw` - Right fisheye image

### D455 Topics
- `/d455/color/image_raw` - RGB image
- `/d455/depth/image_rect_raw` - Depth image (aligned to RGB)
- `/d455/depth/color/points` - 3D point cloud
- `/d455/infra1/image_rect_raw` - Left IR image
- `/d455/infra2/image_rect_raw` - Right IR image
- `/d455/imu` - IMU data (gyro + accel)

## Portability: Laptop → Jetson

This package is designed for seamless portability between development (laptop) and deployment (Jetson):

### Multi-Architecture Support
The Docker images are built for both **x86_64** (laptop) and **ARM64** (Jetson) using Docker Buildx:

```bash
# Build for both architectures
docker buildx build --platform linux/amd64,linux/arm64 \
  -f docker/Dockerfile.realsense \
  -t llmdrone:realsense .
```

### Workflow

1. **Development on Laptop** (x86_64):
   ```bash
   docker compose up realsense
   ```

2. **Deploy to Jetson** (ARM64):
   ```bash
   # Same command, different architecture pulled automatically
   docker compose up realsense
   ```

The cameras "just work" because:
- USB passthrough configured in docker-compose.yml
- RealSense SDK built from source for each architecture
- udev rules included for automatic camera detection
- ROS 2 DDS uses `network_mode: host` for cross-container communication

## Troubleshooting

### Camera Not Detected
```bash
# Check USB devices
lsusb | grep Intel

# Verify camera serial numbers
rs-enumerate-devices

# Check permissions (inside container)
groups  # Should include 'video' and 'plugdev'
```

### Docker USB Passthrough Issues
```bash
# Ensure privileged mode or specific device passthrough
docker run --privileged --device=/dev/bus/usb llmdrone:realsense

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### ROS 2 Topics Not Publishing
```bash
# Check if nodes are running
ros2 node list

# Verify camera firmware
rs-fw-update -l

# Restart containers
docker compose restart realsense
```

## Integration with LLMDrone

This package integrates with the broader LLMDrone stack:
- **agents_control**: Uses T265 odometry for position control
- **agents_llm_bridge**: Processes D455 imagery for scene understanding
- **agents_mavsdk_bridge**: Fuses T265 pose with PX4 EKF for navigation

## Configuration

Edit [config/realsense_params.yaml](config/realsense_params.yaml) to tune:
- Camera resolutions and frame rates
- Depth filtering (spatial, temporal, decimation)
- Point cloud density
- IMU rates
- TF publishing settings

## Future Enhancements

- [ ] Sensor fusion (T265 + D455 IMUs)
- [ ] Object detection pipeline (YOLO/DepthAI)
- [ ] Semantic segmentation for terrain analysis
- [ ] Visual-inertial SLAM integration
- [ ] Edge AI model deployment (NVIDIA Jetson inference)
