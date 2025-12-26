# Containerization Strategy: Podman-First Development

## Overview

All development for the flyby-f11 project uses **Podman** for fully self-contained, portable, and reproducible environments. This approach ensures complete dependency documentation and eliminates "works on my machine" issues.

## Two-Tool Approach: Development vs Deployment

We use **different tools for different phases**:

- **Development** (dev machine): **Podman Compose** - fast iteration, easy orchestration
- **Deployment** (Jetson): **Quadlet** - systemd integration, auto-start, production-grade

This gives us the best of both worlds: developer productivity + production reliability.

## Key Benefits

### 1. **Complete Portability**
- All dependencies documented in `Containerfile` specifications
- No hidden host dependencies
- Identical environment on any machine with Podman
- Easy handoff to collaborators and deployment to target hardware

### 2. **GPU Passthrough Support**
- NVIDIA GPU access for vision models (YOLO, segmentation)
- TensorRT optimization inside containers
- Gazebo simulation with hardware acceleration
- Training RL models with GPU acceleration

### 3. **Rootless Security**
- No daemon required (unlike Docker)
- Containers run as user processes
- Better security model for development
- Compatible with restrictive security policies

### 4. **Reproducible Research**
- All dependencies versioned in container specs
- Bit-for-bit reproducible builds
- Critical for scientific research and collaboration
- Enables verification of results by others

### 5. **Clean Host System**
- No ROS 2, Python, or library installation on host
- Host only needs Podman + NVIDIA drivers
- No dependency conflicts between projects
- Easy cleanup: delete container, done

### 6. **Multi-Architecture Support**
- Build for x86_64 (development) and ARM64 (Jetson) from same specs
- Test on development machine, deploy to Jetson without changes
- Cross-platform compatibility built-in

## Container Structure

### Planning Mode Container

**File**: `ontology/Containerfile.planning`

**Purpose**: Heavyweight reasoning for mission planning and safety verification

**Contents**:
- SUMO ontology (SUO-KIF format)
- Vampire theorem prover
- Clingo answer set programming solver
- E-Prover first-order logic reasoner
- Java runtime for ontology tools
- Python for automation scripts

**Usage**:
```bash
# Build planning container
podman build -f ontology/Containerfile.planning -t flyby-f11-planning .

# Run Vampire proof verification
podman run --rm -v ./ontology:/workspace flyby-f11-planning \
  vampire /workspace/planning_mode/tests/safety_proof.tptp

# Interactive development
podman run --rm -it -v ./ontology:/workspace flyby-f11-planning bash
```

### Execution Mode Container

**File**: `ontology/Containerfile.execution`

**Purpose**: Lightweight runtime reasoning with SWI-Prolog

**Contents**:
- SWI-Prolog runtime
- Python with pyswip bindings
- Compiled Prolog rules
- Minimal dependencies for fast startup

**Usage**:
```bash
# Build execution container
podman build -f ontology/Containerfile.execution -t flyby-f11-execution .

# Run Prolog tests
podman run --rm -v ./ontology:/workspace flyby-f11-execution \
  python3 /workspace/execution_mode/test_prolog.py
```

### Simulation Container

**File**: `simulation/Containerfile.sitl`

**Purpose**: Software-in-the-loop simulation with ArduPilot

**Contents**:
- ArduPilot build from source
- MAVProxy ground station software
- pymavlink for programmatic control
- Gazebo for 3D simulation (with GPU support)
- MAVSDK-Python for flight control

**Usage**:
```bash
# Build SITL container
podman build -f simulation/Containerfile.sitl -t flyby-f11-sitl .

# Run SITL with network passthrough
podman run --rm --network=host -it flyby-f11-sitl \
  sim_vehicle.py -w --console --map
```

### ROS 2 Development Container

**File**: `ros2_ws/Containerfile.ros2`

**Purpose**: ROS 2 Humble workspace with GPU passthrough

**Contents**:
- ROS 2 Humble base
- NVIDIA Container Toolkit for GPU access
- colcon build tools
- ROS 2 development packages
- Python 3.10 with ROS 2 bindings

**Usage**:
```bash
# Build ROS 2 container with GPU support
podman build -f ros2_ws/Containerfile.ros2 -t flyby-f11-ros2 .

# Run with GPU passthrough
podman run --rm -it \
  --device nvidia.com/gpu=all \
  -v ./ros2_ws:/workspace \
  flyby-f11-ros2 bash

# Inside container - verify GPU
nvidia-smi

# Build ROS 2 workspace
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

### Vision/Perception Container

**File**: `ros2_ws/Containerfile.vision`

**Purpose**: TensorRT-optimized vision models with GPU acceleration

**Contents**:
- CUDA toolkit
- cuDNN libraries
- TensorRT for inference optimization
- Ultralytics YOLO
- PyTorch with CUDA support
- OpenCV with CUDA support

**Usage**:
```bash
# Build vision container
podman build -f ros2_ws/Containerfile.vision -t flyby-f11-vision .

# Run with GPU passthrough for TensorRT export
podman run --rm -it \
  --device nvidia.com/gpu=all \
  -v ./models:/models \
  flyby-f11-vision bash

# Inside container - export YOLO to TensorRT
python3 -c "
from ultralytics import YOLO
model = YOLO('yolo11n.pt')
model.export(format='engine', device=0, half=True)
"
```

## GPU Passthrough Setup

### One-Time Host Configuration

```bash
# Install NVIDIA container toolkit
sudo apt update
sudo apt install -y nvidia-container-toolkit

# Configure CDI (Container Device Interface) for Podman
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Verify GPU access
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

### Using GPU in Containers

All GPU-enabled containers use the `--device nvidia.com/gpu=all` flag:

```bash
# Run with GPU access
podman run --rm -it \
  --device nvidia.com/gpu=all \
  -v ./workspace:/workspace \
  <image-name> bash
```

For development containers that need GPU:
- Vision/perception containers (YOLO, TensorRT)
- Simulation containers (Gazebo with GPU rendering)
- Training containers (RL policy training)

## Container Orchestration

### Podman Compose

**File**: `podman-compose.yml`

Orchestrates multiple containers for complex workflows:

```yaml
version: '3'
services:
  planning:
    build:
      context: .
      dockerfile: ontology/Containerfile.planning
    volumes:
      - ./ontology:/workspace
    command: /bin/bash

  execution:
    build:
      context: .
      dockerfile: ontology/Containerfile.execution
    volumes:
      - ./ontology:/workspace
    depends_on:
      - planning

  ros2:
    build:
      context: .
      dockerfile: ros2_ws/Containerfile.ros2
    volumes:
      - ./ros2_ws:/workspace
    devices:
      - nvidia.com/gpu=all
    network_mode: host

  vision:
    build:
      context: .
      dockerfile: ros2_ws/Containerfile.vision
    volumes:
      - ./models:/models
      - ./ros2_ws:/workspace
    devices:
      - nvidia.com/gpu=all

  sitl:
    build:
      context: .
      dockerfile: simulation/Containerfile.sitl
    network_mode: host
```

**Usage**:
```bash
# Start all services
podman-compose up

# Start specific service
podman-compose up ros2

# Build and start
podman-compose up --build

# Stop all services
podman-compose down

# Stop and remove volumes
podman-compose down -v
```

## Multi-Architecture Support

### Building for Jetson (ARM64)

```bash
# Build ARM64 image on x86_64 host
podman build --platform linux/arm64 \
  -f ros2_ws/Containerfile.ros2 \
  -t flyby-f11-ros2:arm64 .

# Save image for transfer to Jetson
podman save -o flyby-f11-ros2-arm64.tar flyby-f11-ros2:arm64

# On Jetson - load and run
podman load -i flyby-f11-ros2-arm64.tar
podman run --rm -it \
  --device nvidia.com/gpu=all \
  -v ./ros2_ws:/workspace \
  flyby-f11-ros2:arm64
```

### Cross-Compilation

For components requiring cross-compilation (e.g., optimized C++ ROS 2 nodes):

```dockerfile
# In Containerfile
FROM --platform=linux/arm64 ros:humble

# ARM64-specific optimizations
RUN apt-get update && apt-get install -y \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu
```

## Development Workflow

### Typical Development Session

1. **Start container**:
   ```bash
   podman run --rm -it \
     --device nvidia.com/gpu=all \
     -v ./ros2_ws:/workspace \
     --name flyby-dev \
     flyby-f11-ros2 bash
   ```

2. **Inside container - develop**:
   ```bash
   cd /workspace
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch flyby_f11_bringup simulation.launch.py
   ```

3. **In separate terminal - interact with running container**:
   ```bash
   podman exec -it flyby-dev bash
   # Now inside same container
   ros2 topic list
   ros2 node list
   ```

4. **Clean up**:
   ```bash
   # Exit container
   exit
   # Container auto-removed (--rm flag)
   ```

### Persistent Development Container

For long-running development sessions:

```bash
# Start container without --rm
podman run -dit \
  --device nvidia.com/gpu=all \
  -v ./ros2_ws:/workspace \
  --name flyby-dev \
  flyby-f11-ros2

# Attach to running container
podman exec -it flyby-dev bash

# Stop container (preserves state)
podman stop flyby-dev

# Restart later
podman start flyby-dev
podman exec -it flyby-dev bash

# Remove when done
podman rm flyby-dev
```

## Benefits for Research Reproducibility

### 1. **Exact Dependency Versions**
Every container specification includes exact versions:
```dockerfile
RUN pip install \
    ultralytics==8.0.200 \
    torch==2.1.0 \
    numpy==1.24.3
```

### 2. **Build-Time Testing**
Container builds can include verification:
```dockerfile
RUN python3 -c "import torch; assert torch.cuda.is_available()"
```

### 3. **Archival and Versioning**
Tag and save containers for long-term reproducibility:
```bash
# Tag specific version
podman tag flyby-f11-ros2 flyby-f11-ros2:v1.0.0

# Save for archival
podman save -o flyby-f11-ros2-v1.0.0.tar flyby-f11-ros2:v1.0.0

# Years later - exact reproduction
podman load -i flyby-f11-ros2-v1.0.0.tar
```

## Troubleshooting

### GPU Not Available

**Symptom**: `nvidia-smi` fails inside container

**Solution**:
```bash
# Regenerate CDI configuration
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Verify on host
nvidia-smi

# Test in container
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

### Permission Issues with Volumes

**Symptom**: Cannot write to mounted volumes

**Solution**: Podman rootless containers run as your user by default, but ensure ownership:
```bash
# Check ownership of mounted directory
ls -la ./ros2_ws

# If needed, fix ownership
chown -R $USER:$USER ./ros2_ws
```

### Container Build Fails

**Symptom**: Build errors during `podman build`

**Solution**:
```bash
# Clean build cache
podman system prune -a

# Rebuild with verbose output
podman build --no-cache -f <containerfile> -t <tag> .
```

### Out of Disk Space

**Symptom**: No space left on device

**Solution**:
```bash
# Check Podman storage usage
podman system df

# Remove unused images
podman image prune -a

# Remove unused containers
podman container prune

# Remove unused volumes
podman volume prune
```

## Best Practices

### 1. **Layer Caching**
Order Dockerfile instructions from least to most frequently changing:
```dockerfile
# Good: dependencies first (cached)
FROM ros:humble
RUN apt-get update && apt-get install -y python3-pip
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . /workspace  # Source code last (changes often)
```

### 2. **Minimize Image Size**
```dockerfile
# Combine RUN commands to reduce layers
RUN apt-get update && apt-get install -y \
    package1 \
    package2 \
    && rm -rf /var/lib/apt/lists/*  # Clean cache
```

### 3. **Use .containerignore**
Create `.containerignore` to exclude unnecessary files:
```
build/
install/
log/
.git/
*.pyc
__pycache__/
```

### 4. **Multi-Stage Builds**
For smaller runtime images:
```dockerfile
# Build stage
FROM ros:humble AS builder
COPY . /workspace
RUN colcon build

# Runtime stage
FROM ros:humble
COPY --from=builder /workspace/install /workspace/install
```

## Deployment to Jetson

### Transfer Container to Target Hardware

```bash
# On development machine
podman save -o flyby-f11-complete.tar flyby-f11-ros2

# Transfer to Jetson (via scp/rsync)
scp flyby-f11-complete.tar jetson@jetson-ip:/tmp/

# On Jetson
podman load -i /tmp/flyby-f11-complete.tar

# Run on Jetson with GPU
podman run --rm -it \
  --device nvidia.com/gpu=all \
  -v /dev/video0:/dev/video0 \
  -v /dev/ttyACM0:/dev/ttyACM0 \
  --network=host \
  flyby-f11-ros2
```

### Jetson-Specific Optimizations

In Containerfile for Jetson:
```dockerfile
# Use NVIDIA L4T base for Jetson
FROM nvcr.io/nvidia/l4t-base:r35.4.1

# Install Jetson-optimized libraries
RUN apt-get update && apt-get install -y \
    nvidia-jetpack
```

## Quadlet for Production Deployment

For production deployment on the Jetson Orin NX, we use **Quadlet** - Podman's native systemd integration.

### Why Quadlet for Production?

1. **Systemd Integration** - Full lifecycle management with familiar `systemctl` commands
2. **Auto-Start on Boot** - Services start automatically when Jetson powers on
3. **Rootless-First** - Built specifically for rootless Podman (better security)
4. **Lightweight** - No Python dependency (podman-compose needs Python)
5. **Production-Grade** - Designed for server/embedded deployments

### Quadlet Configuration Files

Located in [`containers/quadlet/`](containers/quadlet/) directory:

- **`flyby-f11-execution.container`** - Runtime Prolog reasoning (auto-start)
- **`flyby-f11-ros2.container`** - ROS 2 autonomy stack (auto-start)
- **`flyby-f11-vision.container`** - GPU vision pipeline (auto-start)
- **`flyby-f11-planning.container`** - Heavyweight planning (on-demand)
- **`flyby-f11-sitl.container`** - ArduPilot SITL (development only)

See [containers/quadlet/README.md](containers/quadlet/README.md) for complete deployment instructions.

### Quick Deployment on Jetson

```bash
# 1. Build images on dev machine
podman build -f ros2_ws/Containerfile.ros2 -t flyby-f11-ros2 .

# 2. Save and transfer to Jetson
podman save -o flyby-f11-ros2.tar flyby-f11-ros2
scp flyby-f11-ros2.tar jetson@jetson-ip:/tmp/

# 3. On Jetson - load image
podman load -i /tmp/flyby-f11-ros2.tar

# 4. Install Quadlet files
cp containers/quadlet/*.container ~/.config/containers/systemd/
systemctl --user daemon-reload

# 5. Enable auto-start
systemctl --user enable flyby-f11-ros2.service
systemctl --user enable flyby-f11-vision.service

# 6. Start services
systemctl --user start flyby-f11-ros2.service

# 7. Check status
systemctl --user status flyby-f11-ros2.service
```

### Systemd Service Management

```bash
# Start services
systemctl --user start flyby-f11-ros2.service

# Stop services
systemctl --user stop flyby-f11-ros2.service

# Restart after updates
systemctl --user restart flyby-f11-ros2.service

# View logs
journalctl --user -u flyby-f11-ros2.service -f

# Check status
systemctl --user status 'flyby-f11-*'
```

## Podman Compose for Development

For development on your dev machine, we use **Podman Compose** for fast iteration.

### Development Workflow

```bash
# Start all services
podman-compose up

# Rebuild and start
podman-compose up --build

# Run in background
podman-compose up -d

# View logs
podman-compose logs -f

# Stop all services
podman-compose down

# Stop and remove volumes
podman-compose down -v
```

### Interactive Development

```bash
# Start services in background
podman-compose up -d

# Enter ROS 2 development container
podman exec -it flyby-f11-ros2-dev bash

# Inside container - build and test
cd /workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch flyby_f11_bringup simulation.launch.py
```

### Running Specific Services

```bash
# Only start execution mode + ROS 2
podman-compose up execution ros2

# Run SITL simulation (uses profile)
podman-compose --profile simulation up sitl

# Run planning mode (uses profile)
podman-compose --profile planning up planning
```

## Comparison: Quadlet vs Podman Compose

| Feature | Quadlet (Jetson) | Podman Compose (Dev) |
|---------|------------------|----------------------|
| **Primary Use** | Production deployment | Development iteration |
| **Auto-start on boot** | ✅ Yes | ❌ No |
| **Systemd integration** | ✅ Native | ❌ No |
| **Restart policies** | ✅ Full systemd | ⚠️ Limited |
| **Logging** | ✅ Journald | ⚠️ Container logs |
| **Health checks** | ✅ Systemd watchdog | ⚠️ Manual |
| **Quick rebuild** | ❌ Requires rebuild | ✅ `up --build` |
| **Multi-container orchestration** | ⚠️ Via dependencies | ✅ Single command |
| **Resource limits** | ✅ Systemd cgroups | ⚠️ Container-level |
| **Familiar commands** | `systemctl` | `docker-compose` |
| **Dependencies** | None (systemd built-in) | Python |

## Development → Production Workflow

### 1. Develop with Podman Compose

```bash
cd flyby-f11

# Fast iteration cycle
podman-compose up --build
# Make code changes
podman-compose restart ros2
# Test
podman-compose logs -f ros2
```

### 2. Test Quadlet Locally (Optional)

Before deploying to Jetson, test Quadlet on dev machine:

```bash
# Install Quadlet files locally
mkdir -p ~/.config/containers/systemd
cp containers/quadlet/flyby-f11-ros2.container ~/.config/containers/systemd/
systemctl --user daemon-reload

# Test systemd management
systemctl --user start flyby-f11-ros2.service
systemctl --user status flyby-f11-ros2.service
journalctl --user -u flyby-f11-ros2.service -f

# Clean up after testing
systemctl --user stop flyby-f11-ros2.service
systemctl --user disable flyby-f11-ros2.service
rm ~/.config/containers/systemd/flyby-f11-ros2.container
systemctl --user daemon-reload
```

### 3. Deploy to Jetson

```bash
# Save images
podman save -o flyby-f11-all.tar \
  flyby-f11-execution \
  flyby-f11-ros2 \
  flyby-f11-vision

# Transfer and deploy (see Quadlet README for full instructions)
scp flyby-f11-all.tar containers/quadlet/*.container jetson@jetson-ip:/tmp/
ssh jetson@jetson-ip 'bash -s' < scripts/deploy-to-jetson.sh
```

## Summary

Podman-first development for flyby-f11 provides:

✅ **Complete portability** - runs identically on any machine
✅ **GPU passthrough** - full NVIDIA GPU access for vision and training
✅ **Reproducibility** - exact dependencies documented and versioned
✅ **Clean host** - no pollution of host system
✅ **Security** - rootless containers, no daemon
✅ **Multi-arch** - build once, deploy to x86 and ARM
✅ **Dual workflow** - Podman Compose for dev, Quadlet for production

This approach ensures the project is:
- **Shareable**: collaborators can reproduce environment instantly
- **Deployable**: same containers run on dev machine and Jetson
- **Production-ready**: systemd-managed services with auto-start
- **Maintainable**: all dependencies explicitly documented
- **Verifiable**: research results reproducible by others
