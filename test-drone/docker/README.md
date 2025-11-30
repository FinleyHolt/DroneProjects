# Test Drone Docker Environment

Complete Docker setup for test-drone development and simulation.

## Quick Start

```bash
cd test-drone

# Build the Docker image
./sim build

# Start simulation (PX4 SITL + Gazebo + QGroundControl)
./sim

# First run will take 5-10 minutes to build PX4
# Subsequent runs are instant
```

That's it! The `./sim` script handles everything automatically.

**See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) if you encounter issues.**

---

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Commands](#commands)
- [Architecture](#architecture)
- [Build Optimization](#build-optimization)
- [Development Workflow](#development-workflow)
- [Environment Variables](#environment-variables)
- [Volumes](#volumes)

---

## Overview

Two Docker environments are provided:

### 1. Simulation (`Dockerfile.simulation`)
- **Purpose**: PX4 SITL + Gazebo for development and testing
- **Includes**: ROS 2 Humble, Gazebo Harmonic, PX4, MAVSDK, QGroundControl
- **Platform**: x86_64 only
- **GPU**: Optional NVIDIA GPU support for better Gazebo performance

### 2. Flight Test (`Dockerfile.flight_test`)
- **Purpose**: Real hardware testing with Intel RealSense cameras
- **Includes**: ROS 2 Humble, librealsense2, MAVSDK
- **Platforms**: x86_64 (laptop) and ARM64 (Jetson Orin/Xavier)
- **Hardware**: T265 (visual odometry) + D455 (depth camera) + PX4 flight controller

---

## Prerequisites

### For Simulation

**Required:**
- Docker and Docker Compose
- X11 display server (for GUI)
- 8GB+ RAM, 16GB recommended
- 4+ CPU cores

**Optional:**
- NVIDIA GPU with drivers installed
- 16GB+ RAM for faster builds

**Setup X11 access:**
```bash
xhost +local:docker
export DISPLAY=:0  # Or your display number
```

### For Flight Test

**Required:**
- Ubuntu laptop (x86_64) or NVIDIA Jetson (ARM64)
- Intel RealSense T265 + D455 cameras
- PX4 flight controller connected via USB serial
- USB 3.0 ports

**Permissions:**
```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER
# Re-login required
```

---

## Commands

All commands run from `test-drone/` directory.

### Using `./sim` Script (Recommended)

```bash
./sim          # Start simulation (default)
./sim build    # Build Docker image
./sim down     # Stop simulation
./sim shell    # Open shell in running container
```

### Manual Docker Compose Commands

```bash
# Simulation
docker compose -f docker/docker-compose.yml up simulation
docker compose -f docker/docker-compose.yml build simulation
docker compose -f docker/docker-compose.yml down

# Flight test
docker compose -f docker/docker-compose.yml up flight-test
docker compose -f docker/docker-compose.yml build flight-test
```

### Interactive Shells

```bash
# Enter running simulation container
docker exec -it test-drone-simulation bash

# Or start fresh container
docker compose run --rm simulation bash
```

---

## Architecture

### Simulation Container

**Base:** Ubuntu 22.04

**Key Components:**
- **ROS 2 Humble** - Robot Operating System
- **Gazebo Harmonic** - 3D simulation environment
- **PX4 Autopilot** - Flight control software (built on first run)
- **MAVSDK** - MAVLink library for flight control
- **QGroundControl** - Ground control station GUI
- **BehaviorTree.CPP** - Mission planning framework
- **RTAB-Map** - SLAM library

**Why PX4 builds on first run:**
- Keeps Docker image size smaller (saves ~2GB)
- Faster Docker build times (12-18 min vs 60+ min)
- Allows PX4 source changes without rebuilding entire image
- Uses ccache for fast incremental rebuilds

### Flight Test Container

**Base:** Ubuntu 22.04

**Key Components:**
- **ROS 2 Humble**
- **librealsense2 v2.50.0** - Built from source for T265 compatibility
- **MAVSDK**
- **BehaviorTree.CPP**
- **RTAB-Map**

**Multi-platform Support:**
- Automatically detects x86_64 or ARM64 architecture
- Single Dockerfile works on both laptop and Jetson

---

## Build Optimization

The Docker image build is optimized for speed and developer experience.

### Performance Features

1. **Compiler Cache (ccache)**
   - 5GB cache in `/tmp/ccache`
   - Speeds up rebuilds by 10-100x
   - Only recompiles changed files

2. **Parallel Builds**
   - All builds use `$(nproc)` cores
   - MAVSDK: ~8x faster
   - ros_gz: ~6x faster
   - PX4: ~12x faster (when built manually)

3. **Parallel Git Operations**
   - PX4 submodules clone with `--jobs $(nproc)`
   - Reduces clone time from ~5min to ~1min

### Expected Build Times

**System**: 16+ cores, 64GB RAM

| Build Stage | Time (Optimized) |
|-------------|------------------|
| Docker image build | 12-18 min |
| PX4 build (first run inside container) | 5-8 min |
| **Total first-time setup** | **17-26 min** |

**Subsequent runs:** Instant (PX4 already built)

### Build PX4 Manually (Inside Container)

If you need to rebuild PX4:

```bash
docker exec -it test-drone-simulation bash

# Inside container:
/opt/build_px4_sitl.sh

# Or limit parallel jobs (if low RAM):
PX4_BUILD_JOBS=4 /opt/build_px4_sitl.sh

# Or clean rebuild:
cd /opt/PX4-Autopilot
make clean
/opt/build_px4_sitl.sh
```

---

## Development Workflow

### Simulation Development

1. **Start simulation:**
   ```bash
   cd test-drone
   ./sim
   ```

2. **Develop ROS 2 packages:**
   ```bash
   # In another terminal
   docker exec -it test-drone-simulation bash
   cd /workspace/ros2_ws

   # Build your packages
   colcon build --symlink-install --packages-select my_package

   # Source and run
   source install/setup.bash
   ros2 launch my_package my_launch.py
   ```

3. **Iterate:**
   - Edit code on host (files mounted from `test-drone/ros2_ws/`)
   - Rebuild inside container
   - Test in simulation

### Flight Test Development

1. **Connect hardware:**
   - T265 and D455 cameras via USB 3.0
   - PX4 flight controller via USB serial

2. **Start container:**
   ```bash
   cd test-drone
   docker compose up flight-test
   ```

3. **Verify sensors:**
   ```bash
   # Inside container
   rs-enumerate-devices  # Should show T265 and D455
   ls /dev/ttyACM*       # Should show flight controller
   ```

4. **Build and launch:**
   ```bash
   cd /workspace/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch test_drone_bringup real_hardware.launch.py
   ```

---

## Environment Variables

### Common (Both Containers)

```bash
ROS_DOMAIN_ID=42        # Isolates ROS 2 DDS traffic
ROS_DISTRO=humble       # ROS 2 distribution
```

### Simulation Only

```bash
DISPLAY=:0              # X11 display for GUIs
PX4_HOME_LAT=37.7749    # Home latitude (San Francisco)
PX4_HOME_LON=-122.4194  # Home longitude
PX4_HOME_ALT=0.0        # Home altitude (meters)
QT_X11_NO_MITSHM=1      # Qt compatibility fix
NVIDIA_VISIBLE_DEVICES=all         # GPU access (if enabled)
NVIDIA_DRIVER_CAPABILITIES=all     # GPU capabilities
```

### Flight Test Only

```bash
REALSENSE_SDK_PATH=/opt/realsense  # RealSense install path
```

---

## Volumes

### Simulation Container Mounts

| Host Path | Container Path | Purpose |
|-----------|----------------|---------|
| `test-drone/ros2_ws/` | `/workspace/ros2_ws` | ROS 2 workspace (development) |
| `test-drone/simulation/` | `/workspace/simulation` | Gazebo models/worlds |
| `test-drone/config/` | `/workspace/config` | PX4 parameters, calibration |
| `test-drone/scripts/` | `/opt/*.sh` | Helper scripts |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 socket for GUI |

### Flight Test Container Mounts

| Host Path | Container Path | Purpose |
|-----------|----------------|---------|
| `test-drone/ros2_ws/` | `/workspace/ros2_ws` | ROS 2 workspace |
| `test-drone/config/` | `/workspace/config` | Sensor calibration, params |
| `/dev/bus/usb` | `/dev/bus/usb` | USB devices (cameras) |
| `/dev/ttyACM0` | `/dev/ttyACM0` | Flight controller serial |
| `/run/udev` | `/run/udev` | Hotplug events (read-only) |

**Note:** Files are mounted as volumes, so changes on host appear immediately in container (and vice versa).

---

## GPU Support (Optional)

For better Gazebo performance, enable NVIDIA GPU support:

### 1. Install nvidia-docker2

```bash
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### 2. Uncomment GPU config in `docker-compose.yml`

```yaml
simulation:
  # Uncomment this section:
  deploy:
    resources:
      reservations:
        devices:
          - driver: nvidia
            count: all
            capabilities: [gpu, graphics, compute, utility]
```

### 3. Verify GPU access

```bash
docker exec -it test-drone-simulation nvidia-smi
```

---

## Next Steps

- **Having issues?** See [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **Custom Gazebo models:** See [../simulation/README.md](../simulation/README.md)
- **ROS 2 packages:** See [../ros2_ws/src/](../ros2_ws/src/)
- **PX4 documentation:** https://docs.px4.io/

---

## Notes

- **Network Mode**: Host networking used for ROS 2 DDS discovery
- **T265 EOL**: Intel discontinued T265; we build librealsense v2.50.0 from source
- **Multi-platform**: Flight test supports both x86_64 and ARM64 (Jetson)
- **Security**: The container runs in privileged mode for hardware access (flight test only)
