# Drone Projects

Personal drone autonomy development portfolio focused on Isaac Sim-based simulation, reinforcement learning, and edge AI deployment on NVIDIA Jetson platforms.

## Portfolio Overview

This repository showcases autonomous drone development work focused on edge AI, computer vision, ontology-guided safety, and embedded systems integration.

**Developer**: Finley Holt
**Focus Areas**: Isaac Sim simulation, Reinforcement learning, Ontology-guided safety, Edge deployment

## Repository Structure

```
DroneProjects/
├── isaac-sim/              # Primary: Isaac Sim + PX4 simulation and RL training
│   ├── environments/       # Gymnasium RL environments
│   ├── perception/         # YOLO11, ByteTrack, frustum culling
│   ├── training/           # RL training infrastructure
│   └── scripts/            # Test and utility scripts
│
├── ros2_ws/                # ROS 2 Jazzy packages for real hardware (14 packages)
│   └── src/                # flyby_msgs, perception, autonomy, planning, etc.
│
├── ontology/               # Ontology-based reasoning (SUMO + Vampire ATP)
│   ├── planning_mode/      # Heavyweight offline reasoning
│   └── execution_mode/     # Lightweight runtime (Prolog)
│
├── deployment/             # Jetson deployment configs
│   └── containers/         # Quadlet files for systemd
│
├── project-drone/          # Personal dev platform (Jetson Orin Nano Super)
│
├── CLAUDE.md               # AI assistant guidance with capability roadmap
└── README.md               # This file
```

## Technology Stack

**Simulation**:
- Isaac Sim 5.1.0 (photorealistic simulation)
- PX4 Autopilot v1.14.3 SITL
- Pegasus Simulator v5.1.0

**Autonomy Software**:
- ROS 2 Jazzy (middleware)
- BehaviorTree.CPP v4 (mission logic)
- Stable Baselines3 (RL: SAC, PPO, TD3)
- Gymnasium (RL environment API)

**Perception**:
- YOLO11 (object detection)
- ByteTrack (multi-object tracking)
- Intel RealSense SDK (depth + VIO)

**Reasoning**:
- SUMO + Vampire ATP (formal planning)
- SWI-Prolog (runtime safety)

**Development**:
- Podman (containerized workflows)
- Python 3.10+, C++17

## Target Hardware

### Flyby F-11 (Production)
- NVIDIA Jetson Orin NX 16GB (50 TOPS)
- 3kg payload capacity
- NDAA-compliant for government applications

### project-drone (Development)
- NVIDIA Jetson Orin Nano Super 8GB (67 TOPS)
- T265 visual odometry + D455 depth camera
- Custom 7-inch FPV quadcopter

## Quick Start

```bash
# Clone repository
git clone https://github.com/finleyholt/DroneProjects.git
cd DroneProjects

# Build and run Isaac Sim container
cd isaac-sim
podman-compose up --build

# Enter container
podman exec -it isaac-sim-px4 bash

# Run a test
/isaac-sim/python.sh scripts/depth_avoidance_test.py
```

See [isaac-sim/README.md](isaac-sim/README.md) for detailed setup.

## Current Status (January 2026)

**Working**:
- Isaac Sim + PX4 SITL container
- YOLO11 + ByteTrack perception
- 3 canonical ISR environments defined
- Gimbal control

**In Progress**:
- Depth avoidance calibration
- Behavior tree architecture (RL as leaf nodes)

**Proof of Concept**:
- RL training infrastructure (runs, no good model yet)
- Ontology safety filter (integrated, needs rigorous testing)

**Planned**:
- Complete deployment containers
- Real hardware flight testing

See [CLAUDE.md](CLAUDE.md) for detailed capability roadmap.

## Documentation

- **[CLAUDE.md](CLAUDE.md)**: Project structure, capability roadmap, AI guidance
- **[isaac-sim/CLAUDE.md](isaac-sim/CLAUDE.md)**: Simulation and training guide
- **[ros2_ws/CLAUDE.md](ros2_ws/CLAUDE.md)**: ROS 2 package documentation
- **[ontology/CLAUDE.md](ontology/CLAUDE.md)**: Reasoning system guide
- **[deployment/CLAUDE.md](deployment/CLAUDE.md)**: Jetson deployment guide

---

**Last Updated**: 2026-01-10
