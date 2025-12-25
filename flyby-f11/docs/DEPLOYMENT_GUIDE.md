---
title: "Flyby-F11 Deployment Guide"
author: "Finley Holt"
date: 2025-12-25
format:
  pdf:
    documentclass: article
    geometry:
      - margin=1in
    fontsize: 11pt
    number-sections: true
    toc: true
    include-in-header:
      text: |
        \usepackage{fancyhdr}
        \pagestyle{fancy}
        \fancyhf{}
        \fancyhead[L]{\textit{Deployment Guide}}
        \fancyhead[R]{\thepage}
        \renewcommand{\headrulewidth}{0.4pt}
---

# Deployment Guide

This guide covers deployment of the flyby-f11 autonomy system to the Jetson Orin NX 16GB onboard compute module, including JetPack setup, Docker containerization, ROS 2 workspace deployment, and hardware integration.

## Jetson Orin NX 16GB Setup

### Hardware Prerequisites

**Jetson Orin NX 16GB Module**:
- 50 TOPS AI performance
- 1,024 CUDA cores (Ampere architecture)
- 32 Tensor cores
- 16GB unified memory (CPU + GPU shared)
- 8-core ARM Cortex-A78AE CPU

**Carrier Board**: Compatible with Jetson Orin NX form factor

**Storage**: 256GB+ NVMe SSD recommended (OS + models + logs)

**Power**: 15W-25W TDP depending on mode (ensure adequate power supply)

**Cooling**: Active cooling (fan) required for continuous operation

### JetPack 6.1 Installation

**Option 1: NVIDIA SDK Manager** (development host required):

```bash
# On Ubuntu host machine (x86_64)
# Download SDK Manager from NVIDIA Developer Portal
# https://developer.nvidia.com/sdk-manager

# Install SDK Manager
sudo dpkg -i sdkmanager_<version>_amd64.deb
sudo apt install -f

# Launch SDK Manager
sdkmanager

# Follow GUI:
# 1. Select Jetson Orin NX
# 2. Select JetPack 6.1
# 3. Connect Jetson via USB in recovery mode
# 4. Flash and install components
```

**Option 2: JetPack via SD Card** (for development):

```bash
# Download JetPack 6.1 image from NVIDIA
# https://developer.nvidia.com/embedded/jetpack

# Flash to SD card (Linux)
sudo dd if=jetpack-6.1-orin-nx.img of=/dev/sdX bs=4M status=progress
sync

# Insert SD card into Jetson
# Boot and follow on-screen setup (username, password, network)
```

**Post-Installation Verification**:

```bash
# Check JetPack version
cat /etc/nv_tegra_release

# Check CUDA version
nvcc --version

# Check GPU status
tegrastats

# Check available memory
free -h

# Set power mode (15W for efficiency, 25W for performance)
sudo nvpmodel -m 0  # MAXN mode (25W)
sudo nvpmodel -m 2  # 15W mode

# Enable on-boot
sudo systemctl enable nvpmodel
```

### System Configuration

**Increase swap** (recommended for 16GB memory):

```bash
# Disable existing swap
sudo swapoff -a

# Create 16GB swap file
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Verify
free -h
```

**Optimize memory management**:

```bash
# Reduce swappiness (prefer RAM over swap)
sudo sysctl vm.swappiness=10
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf

# Set overcommit (allow memory allocation beyond physical RAM)
sudo sysctl vm.overcommit_memory=1
echo 'vm.overcommit_memory=1' | sudo tee -a /etc/sysctl.conf
```

**Configure NVIDIA power settings**:

```bash
# Lock GPU/CPU clocks for consistent performance
sudo jetson_clocks --show
sudo jetson_clocks  # Enable maximum performance

# Create systemd service to enable on boot
sudo nano /etc/systemd/system/jetson-clocks.service
```

Add:
```ini
[Unit]
Description=Jetson Clocks Service
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl daemon-reload
sudo systemctl enable jetson-clocks.service
sudo systemctl start jetson-clocks.service
```

**Network configuration** (WiFi/Ethernet):

```bash
# Configure static IP (optional, for consistent connections)
sudo nmcli con mod <connection-name> ipv4.addresses 192.168.1.100/24
sudo nmcli con mod <connection-name> ipv4.gateway 192.168.1.1
sudo nmcli con mod <connection-name> ipv4.dns "8.8.8.8 8.8.4.4"
sudo nmcli con mod <connection-name> ipv4.method manual
sudo nmcli con up <connection-name>

# Verify
ip addr show
ping 8.8.8.8
```

### Install Core Dependencies

**Docker and Docker Compose**:

```bash
# Add Docker's official GPG key
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Install Docker Compose
sudo apt update
sudo apt install docker-compose-plugin

# Verify
docker --version
docker compose version

# Test GPU access in Docker
docker run --rm --runtime nvidia --gpus all ubuntu:22.04 nvidia-smi
```

**NVIDIA Container Toolkit**:

```bash
# Configure Docker for NVIDIA runtime
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install nvidia-container-toolkit
sudo systemctl restart docker

# Verify GPU in container
docker run --rm --gpus all nvcr.io/nvidia/l4t-base:r35.4.1 tegrastats
```

**ROS 2 Humble** (if not using Docker):

```bash
# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-ros-base ros-dev-tools

# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Docker Container Deployment

### Docker Image Architecture

**Multi-stage build strategy**:

1. **Base stage**: JetPack 6.1 + ROS 2 Humble + system dependencies
2. **Build stage**: Compile ROS 2 workspace + vision models
3. **Runtime stage**: Minimal runtime dependencies + compiled artifacts

**Directory structure**:
```
flyby-f11/
├── docker/
│   ├── Dockerfile.base         # Base image (JetPack + ROS 2)
│   ├── Dockerfile.runtime      # Runtime image (execution mode)
│   ├── Dockerfile.planning     # Planning image (heavyweight reasoners)
│   ├── docker-compose.yml      # Multi-container orchestration
│   └── .dockerignore
```

### Building Docker Images

**Dockerfile.base** (example):

```dockerfile
# docker/Dockerfile.base
FROM nvcr.io/nvidia/l4t-jetpack:r36.2.0  # JetPack 6.1

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y ros-humble-ros-base python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    swi-prolog \
    libopencv-dev \
    librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --upgrade pip \
    && pip3 install \
    numpy \
    pyswip \
    opencv-python \
    stable-baselines3 \
    gymnasium

# Source ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
```

**Dockerfile.runtime** (execution mode):

```dockerfile
# docker/Dockerfile.runtime
FROM flyby-f11-base:latest

# Copy ROS 2 workspace
COPY ros2_ws /root/ros2_ws

# Build workspace
WORKDIR /root/ros2_ws
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Copy ontology files
COPY ontology/execution_mode /root/ontology

# Copy vision models (TensorRT engines)
COPY models/ /root/models/

# Entry point
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "flyby_f11_bringup", "real_hardware.launch.py"]
```

**Build images**:

```bash
cd ~/Github/DroneProjects/flyby-f11

# Build base image
docker build -f docker/Dockerfile.base -t flyby-f11-base:latest .

# Build runtime image
docker build -f docker/Dockerfile.runtime -t flyby-f11-runtime:latest .

# Verify
docker images | grep flyby-f11
```

### Docker Compose Configuration

**docker-compose.yml**:

```yaml
# docker/docker-compose.yml
version: '3.8'

services:
  # Execution mode (in-flight)
  runtime:
    image: flyby-f11-runtime:latest
    runtime: nvidia
    privileged: true
    network_mode: host
    volumes:
      - /dev:/dev  # Serial devices
      - ./logs:/root/logs  # Persistent logs
      - ./config:/root/config  # Configuration files
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
      - CUDA_VISIBLE_DEVICES=0
    devices:
      - /dev/ttyTHS0  # ArduPilot serial (example)
      - /dev/video0   # Camera (example)
    command: ros2 launch flyby_f11_bringup real_hardware.launch.py

  # Planning mode (pre-flight, optional separate container)
  planning:
    image: flyby-f11-planning:latest
    runtime: nvidia
    network_mode: host
    volumes:
      - ./ontology:/root/ontology
      - ./missions:/root/missions
    environment:
      - ROS_DOMAIN_ID=42
    command: python3 /root/mission_planner.py
    profiles:
      - planning  # Only start when explicitly requested

  # Monitoring/debugging (optional)
  monitor:
    image: osrf/ros:humble-desktop
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    command: rviz2
    profiles:
      - debug
```

**Deploy with Docker Compose**:

```bash
cd ~/Github/DroneProjects/flyby-f11

# Start runtime container
docker compose up -d runtime

# View logs
docker compose logs -f runtime

# Stop
docker compose down

# Start planning mode
docker compose --profile planning up planning

# Start with debugging tools
docker compose --profile debug up monitor
```

## ROS 2 Workspace Deployment

### Direct Deployment (No Docker)

**Transfer workspace to Jetson**:

```bash
# On development machine
cd ~/Github/DroneProjects/flyby-f11
tar czf flyby-f11-ws.tar.gz ros2_ws/ ontology/ models/ config/

# Copy to Jetson
scp flyby-f11-ws.tar.gz jetson@192.168.1.100:~/

# On Jetson
ssh jetson@192.168.1.100
cd ~
tar xzf flyby-f11-ws.tar.gz
```

**Build on Jetson**:

```bash
# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build (may take 30+ minutes on first build)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Optimize build for ARM**:

```bash
# Use all CPU cores
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

# Enable compiler optimizations
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-O3 -march=native"
```

### Model Deployment

**TensorRT Engine Conversion** (YOLO11):

```bash
# On Jetson (or cross-compile on x86 with TensorRT)
cd ~/models

# Convert ONNX to TensorRT engine
trtexec --onnx=yolo11n.onnx \
        --saveEngine=yolo11n.engine \
        --fp16 \
        --workspace=4096 \
        --verbose

# Verify engine
trtexec --loadEngine=yolo11n.engine --verbose
```

**Prolog Knowledge Base**:

```bash
# Copy compiled rules
cp ~/ontology/execution_mode/compiled_rules.pl /opt/flyby_f11/ontology/

# Test
swipl
?- ['/opt/flyby_f11/ontology/compiled_rules.pl'].
?- canExecute(moveForward).
```

**RL Policy Checkpoints**:

```bash
# Copy trained policies
mkdir -p ~/models/rl_policies
cp mission_planner.zip ~/models/rl_policies/
cp behavior_selector.zip ~/models/rl_policies/
cp trajectory_optimizer.zip ~/models/rl_policies/

# Verify with Stable-Baselines3
python3 -c "from stable_baselines3 import PPO; model = PPO.load('~/models/rl_policies/behavior_selector'); print('Loaded successfully')"
```

## Memory Profiling and Optimization

### Memory Budget Planning

**16GB unified memory allocation**:

| Component | Planning Mode | Execution Mode |
|-----------|---------------|----------------|
| OS + System | ~2 GB | ~2 GB |
| ROS 2 Middleware | ~1 GB | ~1 GB |
| SUMO Ontology | ~500 MB | 0 MB (unloaded) |
| Vampire/Clingo | ~3 GB | 0 MB (unloaded) |
| SWI-Prolog | ~50 MB | ~100 MB |
| YOLO11 (TensorRT) | 0 MB | ~2.5 GB |
| Segmentation Model | 0 MB | ~1.5 GB |
| VLM (optional) | 0 MB | ~4 GB |
| Depth Processing | 0 MB | ~500 MB |
| RL Policies | ~200 MB | ~200 MB |
| Buffers/Cache | ~2 GB | ~4 GB |
| **Total** | ~9 GB | ~16 GB |

### Memory Monitoring

**tegrastats** (real-time monitoring):

```bash
# Basic stats
tegrastats

# Log to file
tegrastats --interval 1000 --logfile ~/logs/tegrastats.log

# Parse log
grep "RAM" ~/logs/tegrastats.log | awk '{print $2}'
```

**Custom memory profiler**:

```python
# scripts/memory_profiler.py
import psutil
import os

def get_memory_usage():
    """Get current memory usage in GB."""
    process = psutil.Process(os.getpid())
    mem_info = process.memory_info()
    return mem_info.rss / (1024 ** 3)  # Convert to GB

def log_memory(label):
    """Log memory usage with label."""
    mem = get_memory_usage()
    print(f"[{label}] Memory: {mem:.2f} GB")

# Usage in nodes
log_memory("Before loading YOLO")
# ... load model ...
log_memory("After loading YOLO")
```

**ROS 2 memory monitoring**:

```bash
# Monitor specific node
ros2 run demo_nodes_cpp talker &
PID=$!
watch -n 1 "cat /proc/$PID/status | grep VmRSS"

# Or use custom script
ros2 run flyby_f11_bringup memory_monitor.py
```

### Optimization Techniques

**Model quantization** (reduce memory footprint):

```bash
# FP16 inference (half precision)
trtexec --onnx=yolo11n.onnx --saveEngine=yolo11n_fp16.engine --fp16

# INT8 quantization (requires calibration)
trtexec --onnx=yolo11n.onnx --saveEngine=yolo11n_int8.engine --int8 --calib=calibration.cache
```

**Memory-mapped models** (lazy loading):

```python
# Use torch.load with mmap
import torch
model = torch.load('model.pth', map_location='cuda:0', mmap=True)
```

**Prolog memory optimization**:

```prolog
% Set stack limits in SWI-Prolog
:- set_prolog_flag(stack_limit, 100_000_000).  % 100MB

% Use indexing for faster queries
:- dynamic objectType/2.
:- index(objectType(1, 0)).  % Index on first argument
```

**Phase transition** (critical for memory management):

```python
# scripts/phase_transition.py
import subprocess
import psutil

def transition_to_execution_mode():
    """Unload planning tools, load vision models."""
    # 1. Save planning results
    save_mission_plan()
    save_compiled_rules()

    # 2. Kill heavyweight reasoners
    subprocess.run(['pkill', '-9', 'vampire'])
    subprocess.run(['pkill', '-9', 'clingo'])

    # 3. Clear cache
    subprocess.run(['sync'])
    subprocess.run(['echo', '3', '>', '/proc/sys/vm/drop_caches'], shell=True)

    # 4. Load vision models
    load_yolo_model()
    load_segmentation_model()

    # 5. Start execution nodes
    subprocess.Popen(['ros2', 'launch', 'flyby_f11_bringup', 'execution_mode.launch.py'])
```

## Hardware Connections

### Flight Controller Serial Connection

**ArduPilot via UART**:

```bash
# Jetson Orin NX UART pins (check carrier board pinout)
# Example: UART1 on 40-pin header
# TX: Pin 8
# RX: Pin 10
# GND: Pin 6

# Enable UART
sudo usermod -aG dialout $USER  # Add user to dialout group
sudo systemctl disable nvgetty  # Disable console on UART

# Verify serial device
ls -l /dev/ttyTHS*  # THS0, THS1, etc.

# Test connection
sudo apt install minicom
minicom -D /dev/ttyTHS0 -b 921600
```

**ArduPilot parameter configuration**:

```
# On flight controller
SERIAL2_PROTOCOL = 2        # MAVLink 2
SERIAL2_BAUD = 921600       # Match Jetson UART baud
```

**MAVSDK connection** (in ROS 2 launch file):

```python
# flyby_f11_bringup/launch/real_hardware.launch.py
Node(
    package='ardupilot_interface',
    executable='ardupilot_bridge',
    parameters=[{
        'connection_url': 'serial:///dev/ttyTHS0:921600',
        'system_id': 1,
        'component_id': 191
    }],
    output='screen'
)
```

### Camera Connections

**Intel RealSense T265** (Visual Odometry):

```bash
# USB 3.0 connection (Type-C on T265)
# Check device
lsusb | grep Intel

# Install RealSense SDK
sudo apt install librealsense2-dev librealsense2-utils

# Test camera
realsense-viewer

# ROS 2 wrapper
sudo apt install ros-humble-realsense2-camera

# Launch
ros2 launch realsense2_camera rs_launch.py \
    device_type:=t265 \
    enable_pose:=true \
    enable_fisheye1:=true \
    enable_fisheye2:=true
```

**Intel RealSense D455** (Depth Camera):

```bash
# USB 3.0 connection
# Check device
lsusb | grep Intel

# Launch
ros2 launch realsense2_camera rs_launch.py \
    device_type:=d455 \
    enable_color:=true \
    enable_depth:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30
```

**USB bandwidth considerations**:

```bash
# Multiple cameras require USB bandwidth management
# Use separate USB controllers if available
# Reduce resolution/framerate if bandwidth limited

# Check USB tree
lsusb -t
```

### Power Management

**Power budget**:

- Jetson Orin NX: 15-25W
- T265: 1.5W
- D455: 3.5W
- Total: ~22-30W

**Power supply requirements**:

```bash
# Ensure adequate supply (recommend 60W total for system + peripherals)
# Use barrel jack (5.5mm/2.1mm) 19V DC or USB-C PD

# Monitor power draw
sudo tegrastats | grep POM
```

**Battery monitoring** (if using battery power):

```bash
# Access battery via I2C/SMBus (platform-dependent)
# Example: TI BQ series fuel gauge
i2cdetect -y -r 1

# Read battery percentage via MAVLink
ros2 topic echo /ardupilot/telemetry/battery
```

## Pre-Flight Checklists

### Hardware Checklist

**Physical Inspection**:
- [ ] Propellers installed correctly (CW/CCW orientation)
- [ ] Motors spin freely
- [ ] Frame/arms structurally sound
- [ ] Cameras securely mounted
- [ ] Jetson Orin NX firmly seated on carrier board
- [ ] All cables secured with strain relief
- [ ] Battery charged (>80% recommended)
- [ ] SD card/NVMe SSD inserted and functional

**Electrical**:
- [ ] Battery voltage nominal (4S: 16.8V full, 14.4V min)
- [ ] Power switch functional
- [ ] No loose connections
- [ ] ESCs armed beep sequence correct
- [ ] Jetson boots successfully
- [ ] Cooling fan operational

### Software Checklist

**System Health**:
```bash
# Run pre-flight diagnostics script
cd ~/Github/DroneProjects/flyby-f11
./scripts/preflight_check.sh
```

**Checklist items**:
- [ ] Jetson booted (check via SSH or console)
- [ ] Docker containers running: `docker ps`
- [ ] ROS 2 nodes active: `ros2 node list`
- [ ] Cameras detected: `ros2 topic list | grep camera`
- [ ] ArduPilot connected: `ros2 topic echo /ardupilot/telemetry/heartbeat -n 1`
- [ ] GPS lock (outdoor): `ros2 topic echo /ardupilot/telemetry/gps -n 1`
- [ ] T265 tracking (indoor): `ros2 topic echo /camera/odom/sample -n 1`
- [ ] Ontology reasoner ready: `ros2 service call /ontology/query_safety ...`
- [ ] RL policies loaded: Check node logs for "Policy loaded successfully"

**ArduPilot Parameters**:
```bash
# Verify critical parameters via MAVLink
ros2 run ardupilot_interface param_check.py
```

Expected values:
- [ ] `SERIAL2_PROTOCOL = 2` (MAVLink)
- [ ] `SERIAL2_BAUD = 921600`
- [ ] `ARMING_CHECK = 1` (all checks, or 16384 for testing)
- [ ] `FS_THR_ENABLE = 1` (RC failsafe)
- [ ] `FS_BATT_ENABLE = 1` (Battery failsafe)
- [ ] `EKF3_SRC1_POSXY` set correctly (3=GPS, 6=External Nav)

### Mission Planning Checklist

**Planning Mode**:
- [ ] Mission definition uploaded: `mission.json`
- [ ] Ontology verification passed (no safety violations)
- [ ] Waypoints within geofence boundaries
- [ ] Battery sufficient for mission + 20% reserve
- [ ] Weather conditions acceptable
- [ ] Airspace authorized (if required)

**Execution Mode**:
- [ ] Phase transition to execution mode completed
- [ ] Vision models loaded (YOLO, segmentation)
- [ ] Prolog knowledge base initialized
- [ ] Memory usage <70% (check `tegrastats`)
- [ ] CPU/GPU temperatures normal (<70°C)

### Safety Checklist

**Operational Safety**:
- [ ] Safety pilot ready with RC transmitter
- [ ] Emergency stop procedure reviewed
- [ ] Clear flight area (no people, obstacles)
- [ ] Visual line of sight maintained (VLOS)
- [ ] First aid kit available
- [ ] Fire extinguisher accessible (LiPo safety)

**Autonomous Mode Safety**:
- [ ] Geofence configured and active
- [ ] Return-to-home (RTL) altitude set
- [ ] Low battery failsafe tested
- [ ] RC failsafe tested (switch off RC, verify RTL)
- [ ] Ontology constraints verified (canExecute queries)
- [ ] Collision avoidance active (depth camera functional)

**Abort Conditions** (trigger immediate landing or RTL):
- Battery <20%
- Lost T265 tracking (GPS-denied environments)
- Lost ArduPilot connection
- Ontology safety constraint violated
- Unrecoverable error in RL policy
- High CPU/GPU temperature (>80°C)
- Memory allocation failure (OOM)

## Troubleshooting Common Deployment Issues

### Jetson won't boot
- Check power supply (19V, 60W+)
- Verify SD card/NVMe SSD properly seated
- Try recovery mode (force USB boot)

### Docker container fails to start
- Check NVIDIA runtime: `docker run --rm --gpus all ubuntu nvidia-smi`
- Verify GPU access: `nvidia-smi`
- Check logs: `docker compose logs runtime`

### ROS 2 nodes not discovering each other
- Verify `ROS_DOMAIN_ID` consistent across nodes
- Check network: `ping <other-device>`
- Disable firewall: `sudo ufw disable` (temporarily)

### ArduPilot connection timeout
- Check serial device: `ls -l /dev/ttyTHS0`
- Verify baud rate matches (921600)
- Test with MAVProxy: `mavproxy.py --master=/dev/ttyTHS0,921600`

### Camera not detected
- Check USB: `lsusb | grep Intel`
- Verify permissions: `sudo usermod -aG video $USER`
- Test with `realsense-viewer`

### Out of memory (OOM)
- Check `tegrastats` for memory usage
- Reduce model size (quantization)
- Kill unnecessary processes
- Increase swap size

### High latency / low FPS
- Check CPU/GPU frequency: `jetson_clocks --show`
- Enable max performance: `sudo jetson_clocks`
- Profile with `tegrastats`, `nvprof`

## Deployment Automation

**Automated deployment script** (`scripts/deploy.sh`):

```bash
#!/bin/bash
set -e

echo "Flyby-F11 Deployment Script"

# 1. Build Docker images
echo "Building Docker images..."
docker build -f docker/Dockerfile.base -t flyby-f11-base:latest .
docker build -f docker/Dockerfile.runtime -t flyby-f11-runtime:latest .

# 2. Transfer to Jetson
echo "Transferring to Jetson..."
docker save flyby-f11-runtime:latest | ssh jetson@192.168.1.100 docker load

# 3. Deploy config files
echo "Deploying configuration..."
scp -r config/ jetson@192.168.1.100:~/flyby-f11/

# 4. Start containers
echo "Starting containers..."
ssh jetson@192.168.1.100 "cd ~/flyby-f11 && docker compose up -d runtime"

echo "Deployment complete!"
```

---

**Last Updated**: 2025-12-25
**Status**: Deployment guide for Jetson Orin NX 16GB
