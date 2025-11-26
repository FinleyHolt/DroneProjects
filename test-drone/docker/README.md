# Test Drone Docker Configurations

This directory contains Docker configurations for test-drone development and deployment.

## Overview

Two Docker environments:
- **Dockerfile.simulation** - Gazebo + PX4 SITL for simulation
- **Dockerfile.flight_test** - Real hardware with T265 + D455 sensors

## Quick Start

### Simulation
```bash
cd test-drone
docker compose up simulation
```

This launches:
- PX4 SITL (Software-in-the-Loop)
- Gazebo with simulated quadcopter
- Simulated T265 (visual odometry) and D455 (depth camera)
- ROS 2 Humble
- MAVSDK for flight control
- BehaviorTree.CPP for mission logic

### Flight Test (Real Hardware)
```bash
cd test-drone
docker compose up flight-test
```

This supports:
- **Platforms**: NVIDIA Jetson (ARM64) and Ubuntu laptop (x86_64)
- **Sensors**: Intel RealSense T265 + D455
- **Flight Controller**: PX4 via serial (typically `/dev/ttyACM0`)

## Architecture Support

### Dockerfile.flight_test
- **x86_64**: Ubuntu laptop development
- **ARM64**: NVIDIA Jetson Orin/Xavier deployment
- Multi-platform build automatically detects architecture

### Dockerfile.simulation
- **x86_64 only**: Simulation with Gazebo Classic
- GPU-accelerated (optional but recommended)

## Environment Details

### Simulation Container
- **Base**: Ubuntu 22.04
- **ROS 2**: Humble
- **Simulator**: Gazebo Classic 11
- **PX4**: v1.14.0 built from source
- **Autonomy**: MAVSDK, BehaviorTree.CPP, RTAB-Map

### Flight Test Container
- **Base**: Ubuntu 22.04
- **ROS 2**: Humble
- **Sensors**: librealsense2 v2.50.0 (for T265 compatibility)
- **Autonomy**: MAVSDK, BehaviorTree.CPP, RTAB-Map
- **Hardware Access**: Privileged mode for USB devices

## Usage

### Build Containers
```bash
# Simulation
docker compose build simulation

# Flight test (current platform)
docker compose build flight-test

# Flight test for specific platform
docker buildx build --platform linux/arm64 -f Dockerfile.flight_test -t test-drone:flight-test-arm64 ../..
```

### Interactive Shell
```bash
# Simulation
docker compose run --rm simulation bash

# Flight test
docker compose run --rm flight-test bash
```

### Development Workflow

**Simulation**:
1. `docker compose up simulation` - Start container
2. Inside container: `cd /opt/PX4-Autopilot && make px4_sitl gazebo-classic`
3. In another terminal: Build and launch ROS 2 workspace
   ```bash
   docker exec -it test-drone-simulation bash
   cd /workspace/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch test_drone_bringup simulation.launch.py
   ```

**Flight Test**:
1. Connect hardware (T265, D455, PX4 flight controller)
2. `docker compose up flight-test` - Start container
3. Inside container:
   ```bash
   cd /workspace/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch test_drone_bringup real_hardware.launch.py
   ```

## Hardware Requirements

### Simulation
- **CPU**: 4+ cores recommended
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: NVIDIA GPU optional but recommended for Gazebo
- **Display**: X11 forwarded for Gazebo GUI

### Flight Test
- **Platform**: Ubuntu laptop (x86_64) or NVIDIA Jetson (ARM64)
- **USB**: Multiple USB 3.0 ports for cameras
- **Serial**: USB serial for PX4 flight controller
- **Permissions**: udev rules for RealSense (configured in Dockerfile)

## Volumes

### Simulation
- `/workspace/ros2_ws` - ROS 2 workspace (mounted from host)
- `/workspace/simulation` - Gazebo worlds/models
- `/workspace/config` - PX4 parameters

### Flight Test
- `/workspace/ros2_ws` - ROS 2 workspace (mounted from host)
- `/workspace/config` - Sensor calibration, PX4 parameters
- `/run/udev` - Hotplug events (read-only from host)

## Environment Variables

### Common
- `ROS_DOMAIN_ID=42` - Isolates ROS 2 DDS traffic
- `ROS_DISTRO=humble` - ROS 2 distribution

### Simulation
- `PX4_HOME_LAT` - Home latitude (default: SF)
- `PX4_HOME_LON` - Home longitude
- `PX4_HOME_ALT` - Home altitude
- `DISPLAY` - X11 display for Gazebo GUI

### Flight Test
- `REALSENSE_SDK_PATH=/opt/realsense` - RealSense installation path

## Troubleshooting

### Simulation Issues

**Gazebo won't start**:
```bash
# Check X11 forwarding
xhost +local:docker
# Verify GPU support
nvidia-smi
```

**PX4 build fails**:
```bash
# Clean build
docker compose run --rm simulation bash
cd /opt/PX4-Autopilot
make clean
make px4_sitl gazebo-classic
```

### Flight Test Issues

**Cameras not detected**:
```bash
# Inside container
rs-enumerate-devices
# Should show T265 and D455

# If not found, check USB permissions on host
lsusb | grep Intel
```

**Permission denied for serial port**:
```bash
# On host, add user to dialout group
sudo usermod -a -G dialout $USER
# Re-login required
```

**T265 not working with D455**:
The T265 requires librealsense2 v2.50.0, which is built from source in the container. The D455 uses the ROS 2 wrapper which supports both.

## Notes

- **T265 EOL**: Intel discontinued T265. Container builds v2.50.0 from source for compatibility.
- **Multi-platform**: Flight test container supports both x86_64 and ARM64 (Jetson).
- **GPU**: Simulation benefits from GPU but can run on CPU-only.
- **Network**: Host network mode used for ROS 2 DDS discovery between containers.
