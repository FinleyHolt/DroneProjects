# NVIDIA JetPack and Docker Reference for Jetson Development

## JetPack 6.1 Overview

JetPack 6.1 is NVIDIA's comprehensive SDK for Jetson platforms, providing essential tools and libraries for AI and robotics applications.

### Supported Platforms
- Jetson Orin Nano (8GB, 4GB)
- Jetson Orin Nano Super (8GB) - flyby-f11 development platform
- Jetson Orin NX (16GB, 8GB) - flyby-f11 target platform
- Jetson Orin AGX (64GB, 32GB)
- Jetson Xavier series

### Key Components

#### 1. Linux for Tegra (L4T)
- **Version**: L4T 36.4 (JetPack 6.1)
- **Kernel**: Linux 5.15
- **Ubuntu Base**: 22.04 LTS (Jammy Jellyfish)

#### 2. CUDA Toolkit
- **Version**: CUDA 12.6
- **Features**: Full CUDA runtime and development libraries
- **Libraries**: cuBLAS, cuDNN, cuFFT, cuSPARSE, etc.

#### 3. Deep Learning Frameworks
- **TensorRT**: 10.3.0 - High-performance inference engine
- **cuDNN**: 9.3.0 - CUDA Deep Neural Network library
- **PyTorch**: 2.5.0 (via pip with CUDA support)
- **TensorFlow**: 2.17.0 (via pip)
- **ONNX Runtime**: For model deployment

#### 4. Computer Vision
- **VPI (Vision Programming Interface)**: 3.2
  - Hardware-accelerated CV algorithms
  - Supports PVA, VIC, GPU, CPU backends
- **OpenCV**: 4.10.0 with CUDA support
- **Deepstream SDK**: 7.1 for video analytics

#### 5. Multimedia
- **GStreamer**: 1.20.3 with hardware encoding/decoding
- **V4L2**: Video4Linux2 camera interface
- **NVMM**: NVIDIA Multimedia API

#### 6. AI/ML Tools
- **NVIDIA Triton Inference Server**: Production deployment
- **TAO Toolkit**: Transfer learning and model optimization
- **Riva**: Speech AI SDK

## JetPack Installation and Configuration

### Flash JetPack to Jetson

```bash
# Using SDK Manager (GUI)
# 1. Download SDK Manager from developer.nvidia.com
# 2. Connect Jetson in recovery mode
# 3. Select JetPack 6.1 and target device
# 4. Flash OS and install SDK components

# Using command line (L4T)
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.0/...
sudo ./flash.sh jetson-orin-nano-devkit mmcblk0p1
```

### Post-Installation Configuration

```bash
# Update system
sudo apt update
sudo apt upgrade

# Install jtop (Jetson stats monitoring)
sudo pip3 install -U jetson-stats
sudo systemctl restart jtop.service

# Check installed versions
jetson_release
jtop

# Configure power mode (Jetson Orin Nano Super: 25W mode)
sudo nvpmodel -m 0  # Maximum performance
sudo jetson_clocks   # Lock clocks to max
```

## Docker on Jetson

### NVIDIA Container Runtime

JetPack includes Docker with NVIDIA runtime for GPU-accelerated containers.

#### Installation Verification

```bash
# Check Docker installation
docker --version
docker info | grep -i runtime

# Verify NVIDIA runtime
docker run --rm --runtime nvidia nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi
```

#### Default Runtime Configuration

Edit `/etc/docker/daemon.json`:

```json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia",
    "storage-driver": "overlay2"
}
```

Restart Docker:
```bash
sudo systemctl restart docker
```

### Base Images for Jetson

NVIDIA provides L4T-based container images:

```dockerfile
# JetPack 6.1 base image
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# Minimal L4T base
FROM nvcr.io/nvidia/l4t-base:r36.4.0

# L4T with CUDA runtime
FROM nvcr.io/nvidia/l4t-cuda:12.6.0-runtime

# L4T with PyTorch
FROM nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3
```

## dusty-nv/jetson-containers

**Repository**: https://github.com/dusty-nv/jetson-containers

This is the **essential reference** for containerized Jetson development. Maintained by Dusty Franklin (NVIDIA), it provides pre-built containers for common frameworks.

### Key Features

1. **Pre-built Containers**: Ready-to-use images for PyTorch, TensorFlow, ROS, etc.
2. **Automated Building**: Build scripts for custom configurations
3. **Multi-architecture**: Supports all Jetson platforms
4. **Documentation**: Extensive guides and examples

### Repository Structure

```
jetson-containers/
├── packages/          # Individual package definitions
│   ├── pytorch/
│   ├── tensorflow/
│   ├── ros/
│   ├── opencv/
│   └── ...
├── scripts/           # Build and utility scripts
├── docker/            # Dockerfiles and build contexts
├── docs/              # Documentation
└── README.md
```

### Common Containers

#### 1. PyTorch Container

```bash
# Pull pre-built PyTorch container
docker pull dustynv/pytorch:2.5-r36.4.0

# Run with GPU support
docker run -it --runtime nvidia dustynv/pytorch:2.5-r36.4.0

# Mount workspace
docker run -it --runtime nvidia \
    -v /home/finley/workspace:/workspace \
    dustynv/pytorch:2.5-r36.4.0
```

#### 2. ROS 2 Container

```bash
# ROS 2 Humble (matches JetPack 6.1 / Ubuntu 22.04)
docker pull dustynv/ros:humble-desktop-r36.4.0

# Run ROS 2 with GUI support
docker run -it --runtime nvidia \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    dustynv/ros:humble-desktop-r36.4.0
```

#### 3. Combined ROS + PyTorch

```bash
# ROS 2 with PyTorch for perception
docker pull dustynv/ros:humble-pytorch-r36.4.0
```

### Building Custom Containers

```bash
# Clone repository
git clone https://github.com/dusty-nv/jetson-containers.git
cd jetson-containers

# Build specific package
./build.sh pytorch tensorflow ros:humble

# Build with custom configuration
./build.sh --name my_custom_image pytorch opencv ros:humble
```

### Using with Docker Compose

Example `docker-compose.yml` for flyby-f11:

```yaml
version: "3.9"

services:
  autonomy:
    image: dustynv/ros:humble-pytorch-r36.4.0
    runtime: nvidia
    network_mode: host
    privileged: true  # For device access
    volumes:
      - ./ros2_ws:/workspace/ros2_ws
      - /dev:/dev  # Camera and sensor access
    environment:
      - DISPLAY=$DISPLAY
      - ROS_DOMAIN_ID=42
    devices:
      - /dev/video0  # USB cameras
      - /dev/ttyUSB0  # Serial devices
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        cd /workspace/ros2_ws &&
        colcon build --symlink-install &&
        source install/setup.bash &&
        ros2 launch flyby_f11_bringup simulation.launch.py
      "

  px4_sitl:
    image: px4io/px4-dev-simulation-focal:latest
    network_mode: host
    volumes:
      - ./simulation:/simulation
      - ./config:/config
    command: >
      bash -c "
        cd /PX4-Autopilot &&
        make px4_sitl gazebo-classic
      "
```

## Performance Optimization

### GPU Memory Management

```python
# PyTorch: Limit GPU memory usage
import torch
torch.cuda.set_per_process_memory_fraction(0.7)  # Use 70% of GPU memory

# Monitor GPU usage
import subprocess
subprocess.run(['tegrastats'])
```

### Power Modes (Jetson Orin Nano Super)

```bash
# List available power modes
sudo nvpmodel -q

# Mode 0: MAXN (25W) - Maximum performance
sudo nvpmodel -m 0

# Mode 1: 15W
sudo nvpmodel -m 1

# Mode 2: 7W (power-efficient)
sudo nvpmodel -m 2
```

### Frequency Scaling

```bash
# Enable max clocks
sudo jetson_clocks --show
sudo jetson_clocks

# Disable max clocks (return to dynamic scaling)
sudo jetson_clocks --restore
```

## JetPack Components for Robotics

### ROS 2 Integration

```bash
# Install ROS 2 Humble on JetPack 6.1
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### Vision Processing with VPI

```python
import vpi

# Hardware-accelerated stereo disparity
with vpi.Backend.CUDA:
    left_img = vpi.asimage(left_np)
    right_img = vpi.asimage(right_np)

    stereo = vpi.StereoDisparityEstimator()
    disparity = stereo(left_img, right_img)
```

### DeepStream Integration

```bash
# Run DeepStream detection pipeline
deepstream-app -c /opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app/config_infer_primary.txt
```

## TensorRT Model Optimization

### Converting ONNX to TensorRT

```python
import tensorrt as trt

# Create builder
logger = trt.Logger(trt.Logger.WARNING)
builder = trt.Builder(logger)
network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
parser = trt.OnnxParser(network, logger)

# Parse ONNX model
with open('model.onnx', 'rb') as f:
    parser.parse(f.read())

# Build engine with FP16 precision
config = builder.create_builder_config()
config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB
config.set_flag(trt.BuilderFlag.FP16)

# Build and serialize
engine = builder.build_serialized_network(network, config)
with open('model.engine', 'wb') as f:
    f.write(engine)
```

### PyTorch to TensorRT with torch2trt

```python
from torch2trt import torch2trt

# Convert PyTorch model
model_trt = torch2trt(model, [x], fp16_mode=True)

# Save
torch.save(model_trt.state_dict(), 'model_trt.pth')
```

## Camera Access

### V4L2 USB Cameras

```bash
# List cameras
v4l2-ctl --list-devices

# Check formats
v4l2-ctl -d /dev/video0 --list-formats-ext

# Capture with GStreamer
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! xvimagesink
```

### CSI Cameras (MIPI)

```bash
# NVIDIA Argus camera API
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! nvvidconv ! xvimagesink
```

## Networking for Multi-Robot Systems

### DDS Configuration for ROS 2

Create `/etc/cyclonedds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

Set environment:
```bash
export CYCLONEDDS_URI=/etc/cyclonedds.xml
```

## Monitoring and Debugging

### System Monitoring

```bash
# jetson-stats (jtop)
jtop

# tegrastats (NVIDIA tool)
tegrastats

# Check temperature
cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

### Docker Resource Limits

```yaml
services:
  autonomy:
    deploy:
      resources:
        limits:
          memory: 6G
        reservations:
          devices:
            - capabilities: [gpu]
```

## Best Practices for flyby-f11

1. **Use dusty-nv Containers**: Start with proven base images
2. **Layer Custom Code**: Add ROS 2 workspace as volume mount
3. **Optimize Models**: Convert to TensorRT for inference
4. **Monitor Resources**: Use jtop to track GPU/CPU/memory
5. **Power Management**: Select appropriate nvpmodel for mission
6. **Network Configuration**: Configure DDS for reliable communication
7. **Camera Integration**: Use GStreamer with hardware encoding
8. **Persistent Storage**: Mount volumes for logs and data

## Reference Links

- **JetPack Documentation**: https://developer.nvidia.com/embedded/jetpack
- **L4T Documentation**: https://docs.nvidia.com/jetson/archives/r36.4/
- **dusty-nv jetson-containers**: https://github.com/dusty-nv/jetson-containers
- **NVIDIA NGC Catalog**: https://catalog.ngc.nvidia.com/containers
- **Jetson Developer Forums**: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/
- **JetPack Release Notes**: https://developer.nvidia.com/embedded/jetpack-sdk-61

## Troubleshooting

### Container Can't Access GPU

```bash
# Verify NVIDIA runtime
docker run --rm --runtime nvidia nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi

# Check daemon.json configuration
sudo systemctl restart docker
```

### Camera Not Detected

```bash
# Check permissions
sudo usermod -aG video $USER
sudo chmod 666 /dev/video*
```

### ROS 2 Nodes Not Discovering

```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Check network interface
ros2 daemon stop
ros2 daemon start
```

## Notes for flyby-f11 Project

- Target platform: Jetson Orin NX 16GB (50 TOPS)
- Use dusty-nv/ros:humble-pytorch container as base
- Optimize perception models with TensorRT for real-time performance
- Configure power mode based on mission duration vs performance needs
- Test Docker Compose workflow in simulation before hardware deployment
- Use jetson-containers build system for custom image creation
