# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Drone autonomy development portfolio focused on Isaac Sim-based simulation, reinforcement learning, and edge AI deployment. The primary development environment is Isaac Sim with PX4 SITL for photorealistic drone simulation and RL training.

**Developer**: Finley Holt
**Focus**: Isaac Sim simulation, reinforcement learning for autonomous navigation, ontology-guided mission planning, and edge deployment on NVIDIA Jetson platforms.

**Technical Foundation**: Isaac Sim for photorealistic simulation, PX4 SITL for flight control, Stable Baselines3 for RL training, ROS 2 Jazzy for autonomy software, and Gymnasium for environment wrappers.

## Repository Structure

```
DroneProjects/
├── isaac-sim/                   # Primary development - Isaac Sim + PX4 simulation
│   ├── environments/            # Gymnasium RL environments
│   ├── perception/              # YOLO detection, tracking, frustum culling
│   ├── training/                # RL training infrastructure (SAC, PPO, TD3)
│   ├── scripts/                 # Test scripts, utilities
│   ├── config/                  # Training hyperparameters, configs
│   ├── ros2_packages/           # ROS 2 bridge packages for sim-to-real
│   ├── tests/                   # Pytest test suite
│   ├── extensions/              # Isaac Sim world generation extensions
│   ├── Containerfile            # Container definition
│   ├── podman-compose.yml       # Development orchestration
│   └── README.md
│
├── ros2_ws/                     # ROS 2 workspace for real hardware
│   └── src/
│       ├── flyby_msgs/          # Custom ROS 2 message definitions
│       ├── flyby_perception/    # Perception ROS 2 nodes
│       └── flyby_autonomy/      # Autonomy ROS 2 nodes
│
├── ontology/                    # Ontology-based reasoning for mission planning
│   └── [ontology pipeline files]
│
├── deployment/                  # Production deployment configs
│   ├── quadlet/                 # Systemd quadlet files for Jetson
│   └── containers/              # Production container definitions
│
├── project-drone/               # Personal dev platform (Jetson Orin Nano Super)
│   └── [self-contained ROS 2 project]
│
├── CLAUDE.md                    # This file
├── README.md                    # Project overview
└── .gitignore
```

## Development Commands

### Working with Isaac Sim (Primary)

```bash
cd isaac-sim

# Build and start container with GPU passthrough
podman-compose up --build

# Enter container for interactive development
podman exec -it isaac-sim-px4 bash

# Inside container - run a test script
/isaac-sim/python.sh scripts/phase2_procgen_test.py

# Run training
/isaac-sim/python.sh training/train_canonical.py

# Run tests
pytest tests/
```

### Working with ROS 2 Workspace

```bash
# Inside the Isaac Sim container, ROS 2 packages are pre-built
# Source the workspace
source /workspace/ros2_ws/install/setup.bash

# Or build from scratch
cd /workspace/ros2_ws
colcon build --symlink-install
```

### Testing

```bash
# Enter container
podman exec -it isaac-sim-px4 bash

# Run Isaac Sim tests
cd /workspace
pytest tests/

# Run specific test
pytest tests/test_environments.py -v
```

## Key Architectural Decisions

- **Isaac Sim First**: Photorealistic simulation with PX4 SITL for high-fidelity training
- **Gymnasium Wrapper**: All environments follow OpenAI Gym API for RL compatibility
- **Ontology-Guided**: Mission planning incorporates ontological reasoning for safety constraints
- **Edge Deployment**: Models trained in simulation deploy to Jetson hardware
- **Podman Containers**: Reproducible, GPU-accelerated development environments

## Code Style and Conventions

- **Python**: Use `ruff format` with snake_case for modules, 4-space indentation
- **ROS 2 C++**: Use `ament_uncrustify` with 4-space indentation
- **ROS 2 Messages**: UpperCamelCase for message types
- **Experimental Code**: Prefix branches/folders with `exp-` to signal unstable code

## Environment Organization (isaac-sim/environments/)

Core environments in `isaac-sim/environments/`:
- `base_isr_env.py`: Base ISR (Intelligence, Surveillance, Reconnaissance) environment
- `comms_denied_env.py`: Communications-denied mission environment
- `gymnasium_wrapper.py`: Gymnasium API wrapper
- `action_bridge.py`: Z-up (Isaac Sim) to NED (PX4) coordinate conversion
- `safety_filter.py`: Ontology-based safety constraints
- `perception_manager.py`: YOLO detection integration
- `zone_manager.py`: No-fly zone management

## Training Infrastructure (isaac-sim/training/)

- `configs/`: YAML training hyperparameter configs
- `callbacks/`: TensorBoard, checkpoint handlers
- `wrappers/`: Gymnasium wrapper extensions
- `deployment/`: Model export and inference utilities

## Perception Pipeline (isaac-sim/perception/)

- YOLO11 object detection (ultralytics)
- ByteTrack multi-object tracking
- Frustum culling for efficient detection
- ROS 2 camera bridge for sim-to-real parity

## Container Environment

The Isaac Sim container includes:
- Isaac Sim 5.1.0 (NVIDIA Omniverse)
- PX4 Autopilot v1.14.3 SITL
- Pegasus Simulator v5.1.0
- ROS 2 Jazzy
- Stable Baselines3 (SAC, PPO, TD3)
- YOLO11 (ultralytics)
- Gymnasium

Key environment variables:
- `ISAACSIM_PATH=/isaac-sim`
- `PX4_HOME=/px4`
- `YOLO_MODEL_PATH=/workspace/models/yolo11x.pt`

## Common Pitfalls

- **NumPy Version**: Isaac Sim requires NumPy 1.26.4 - do NOT upgrade to NumPy 2.x (breaks camera operations)
- **Podman Volumes**: Use `podman-compose down -v` to reset persistent caches
- **GPU Passthrough**: Verify `/etc/cdi/nvidia.yaml` exists for GPU access
- **Shader Warmup**: First container run compiles shaders (~2-3 min); subsequent runs are instant
- **Coordinate Systems**: Isaac Sim uses Z-up, PX4 uses NED - use action_bridge.py for conversion

## Target Hardware

### Flyby F-11 (Production Platform)
- NVIDIA Jetson Orin NX 16GB (50 TOPS)
- 3kg payload capacity
- NDAA-compliant for government applications

### project-drone (Personal Development)
- NVIDIA Jetson Orin Nano Super 8GB (67 TOPS)
- T265 visual odometry + D455 depth camera
- Custom 7-inch FPV quadcopter

## Git Workflow

- **Commit Style**: Imperative mood ("Add obstacle avoidance", "Fix depth correction")
- **Logical Grouping**: Group related changes, reference issues when applicable
- **Ignored Files**: Build artifacts, model weights, test outputs excluded via `.gitignore`
