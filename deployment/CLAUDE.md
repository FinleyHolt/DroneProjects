# Deployment Guide

This file provides guidance for deploying the autonomy stack to Jetson hardware.

## Purpose

Production deployment infrastructure for NVIDIA Jetson platforms using Podman containers and systemd quadlet files.

**Current Status**: Partially configured. Only 2 of 5+ expected containers defined.

## Target Hardware

### Flyby F-11 (Production)
- NVIDIA Jetson Orin NX 16GB (50 TOPS)
- 3kg payload capacity
- NDAA-compliant for government applications

### project-drone (Development)
- NVIDIA Jetson Orin Nano Super 8GB (67 TOPS)
- T265 visual odometry + D455 depth camera
- Custom 7-inch FPV quadcopter

## Directory Structure

```
deployment/
├── containers/
│   ├── compose/           # Development workflow (Podman Compose)
│   │   └── podman-compose.yml
│   └── quadlet/           # Production workflow (systemd)
│       ├── mission-orchestrator.container
│       └── vampire-reasoning.container
└── quadlet/               # Legacy location
```

## Workflows

### Development (Podman Compose)
```bash
cd isaac-sim
podman-compose up --build
podman exec -it isaac-sim-px4 bash
```
- Live code mounting for rapid iteration
- GPU passthrough via CDI
- Full Isaac Sim environment

### Production (Quadlet)
```bash
# Install quadlet files
cp deployment/containers/quadlet/*.container ~/.config/containers/systemd/

# Reload systemd
systemctl --user daemon-reload

# Start services
systemctl --user start vampire-reasoning.service
systemctl --user start mission-orchestrator.service
```

## Current Quadlet Files

### vampire-reasoning.container
- **Image**: `localhost/flyby-f11-planning:latest`
- **Purpose**: Vampire ATP reasoning service
- **Resources**: 2GB RAM, 3 CPUs
- **Health check**: `/vampire/query` service

### mission-orchestrator.container
- **Image**: `localhost/flyby-f11-ros2:latest`
- **Purpose**: Mission coordination
- **Depends on**: vampire-reasoning.service
- **Resources**: 1GB RAM, 2 CPUs

## Missing Containers (TODO)

| Container | Purpose | Priority |
|-----------|---------|----------|
| `flyby-f11-ros2.container` | Core ROS 2 autonomy | High |
| `flyby-f11-vision.container` | GPU perception (TensorRT) | High |
| `flyby-f11-execution.container` | Prolog runtime | Medium |
| `flyby-f11-sitl.container` | PX4 SITL (dev only) | Low |

## Deployment Steps (NOT YET VALIDATED)

1. **Train** policy in Isaac Sim
2. **Export** to TensorRT format (`rl_inference` package)
3. **Build** Jetson-optimized containers (ARM64)
4. **Transfer** images to Jetson via SCP or registry
5. **Install** quadlet files to systemd
6. **Enable** services for auto-start
7. **Validate** with bench testing
8. **Flight test** with safety pilot

## Container Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    JETSON DEPLOYMENT                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │   vampire    │    │    ros2      │    │   vision     │  │
│  │  reasoning   │◄──►│   autonomy   │◄──►│  pipeline    │  │
│  │   (CPU)      │    │   (CPU)      │    │   (GPU)      │  │
│  └──────────────┘    └──────────────┘    └──────────────┘  │
│         │                   │                   │           │
│         └───────────────────┼───────────────────┘           │
│                             │                                │
│                    ┌────────▼────────┐                      │
│                    │   Flight        │                      │
│                    │   Controller    │                      │
│                    │   (PX4)         │                      │
│                    └─────────────────┘                      │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## GPU Passthrough

```bash
# Generate CDI spec (one-time on Jetson)
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Verify GPU access
podman run --rm --device nvidia.com/gpu=all nvidia/cuda:12.0-base nvidia-smi
```

## Troubleshooting

### Container won't start
```bash
# Check systemd status
systemctl --user status <service>

# View logs
journalctl --user -u <service> -f
```

### GPU not detected
- Verify `/etc/cdi/nvidia.yaml` exists
- Ensure `nvidia-container-toolkit` installed
- Check container has `--device nvidia.com/gpu=all`

## Related Documentation

- `containers/README.md`: Detailed container architecture
- `containers/quadlet/README.md`: Quadlet file format
- `../ros2_ws/CLAUDE.md`: ROS 2 package documentation
