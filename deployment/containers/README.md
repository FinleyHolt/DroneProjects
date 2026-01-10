# Flyby F-11 Container Infrastructure

This directory contains all containerization configurations for the Flyby F-11 project. We use a dual-workflow approach optimized for both development iteration and production deployment.

## Directory Structure

```
containers/
├── compose/            # Podman Compose (development workflow)
│   ├── podman-compose.yml
│   └── README.md
├── quadlet/            # Quadlet (production deployment)
│   ├── flyby-f11-planning.container
│   ├── flyby-f11-execution.container
│   ├── flyby-f11-ros2.container
│   ├── flyby-f11-vision.container
│   ├── flyby-f11-sitl.container
│   └── README.md
└── README.md           # This file
```

## Quick Start

### Development (on your laptop/desktop)

```bash
cd /home/finley/Github/DroneProjects/flyby-f11

# Start all services
podman-compose -f containers/compose/podman-compose.yml up

# Enter ROS 2 container for development
podman exec -it flyby-f11-ros2-dev bash
```

### Production (on Jetson)

```bash
# Install Quadlet files
cp containers/quadlet/*.container ~/.config/containers/systemd/
systemctl --user daemon-reload

# Start services
systemctl --user start flyby-f11-ros2.service
```

## Container Architecture

The project uses **separate containers per concern** for maximum modularity:

### Planning Mode (flyby-f11-planning)
- **Purpose**: Heavyweight ontological reasoning for mission planning
- **Contents**: SUMO ontology framework, Vampire theorem prover
- **When**: On-demand, pre-flight mission verification
- **Resource**: 8GB RAM, CPU-intensive

### Execution Mode (flyby-f11-execution)
- **Purpose**: Lightweight Prolog runtime for real-time constraint checking
- **Contents**: SWI-Prolog, compiled mission rules
- **When**: Always running during flight
- **Resource**: 512MB RAM, microsecond query latency

### ROS 2 Stack (flyby-f11-ros2)
- **Purpose**: Core autonomy and navigation
- **Contents**: ROS 2 Humble, navigation stack, behavior trees, MAVSDK
- **When**: Always running during flight
- **Resource**: 4GB RAM, GPU access for some nodes

### Vision Pipeline (flyby-f11-vision)
- **Purpose**: GPU-accelerated perception
- **Contents**: YOLO, depth processing, TensorRT
- **When**: Always running during flight
- **Resource**: 4GB RAM, GPU required

### SITL Simulation (flyby-f11-sitl)
- **Purpose**: ArduPilot Software-in-the-Loop testing
- **Contents**: ArduPilot, MAVProxy, Gazebo
- **When**: Development and testing only
- **Resource**: 8GB RAM, GPU for rendering

## Workflow Comparison

| Feature | Compose (Dev) | Quadlet (Production) |
|---------|---------------|----------------------|
| **Use Case** | Development iteration | Jetson deployment |
| **Start Command** | `podman-compose up` | `systemctl --user start` |
| **Auto-restart** | No | Yes (systemd) |
| **Boot startup** | No | Yes (optional) |
| **Logging** | Container logs | journald |
| **Rebuild** | `up --build` | Manual image transfer |
| **Multi-service** | Easy | Via dependencies |

## Development Workflow

1. **Code Changes**: Edit files in `ros2_ws/`, `ontology/`, etc.
2. **Rebuild**: `podman-compose -f containers/compose/podman-compose.yml up --build ros2`
3. **Test**: Enter container and run tests
4. **Iterate**: Repeat until feature complete

See [containers/compose/README.md](compose/README.md) for detailed development instructions.

## Production Deployment

1. **Build Images**: Build on development machine
2. **Transfer**: Save and SCP to Jetson
3. **Install Quadlet**: Copy `.container` files to `~/.config/containers/systemd/`
4. **Enable Services**: `systemctl --user enable flyby-f11-*.service`
5. **Start**: Services start on boot or manually

See [containers/quadlet/README.md](quadlet/README.md) for detailed deployment instructions.

## Container Images

All containers are built from Containerfiles in their respective directories:

- `ontology/Containerfile.planning` → flyby-f11-planning
- `ontology/Containerfile.execution` → flyby-f11-execution
- `ros2_ws/Containerfile.ros2` → flyby-f11-ros2
- `ros2_ws/Containerfile.vision` → flyby-f11-vision
- `simulation/Containerfile.sitl` → flyby-f11-sitl

## GPU Passthrough

Both workflows use NVIDIA CDI (Container Device Interface) for GPU access:

```bash
# Generate CDI configuration
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Test GPU access
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

In containers, use: `Device=nvidia.com/gpu=all` (Quadlet) or `devices: [nvidia.com/gpu=all]` (Compose)

## Further Reading

- [CONTAINERIZATION_STRATEGY.md](../CONTAINERIZATION_STRATEGY.md) - Detailed architecture and rationale
- [QUICK_START.md](../QUICK_START.md) - Quick reference for common commands
- [containers/compose/README.md](compose/README.md) - Development workflow details
- [containers/quadlet/README.md](quadlet/README.md) - Production deployment details
