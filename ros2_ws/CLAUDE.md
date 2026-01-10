# ROS 2 Workspace Guide

This file provides guidance for working with the ROS 2 packages for real hardware deployment.

## Purpose

ROS 2 Jazzy packages for autonomous drone operation on Jetson platforms (F-11 Orin NX, project-drone Orin Nano Super). Designed for sim-to-real transfer from Isaac Sim.

## Quick Start

```bash
# Inside Isaac Sim container or on Jetson
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Package Index (14 packages)

### Core - Complete
| Package | Description | Status |
|---------|-------------|--------|
| `flyby_msgs` | 13 messages, 1 action, 1 service | Complete |
| `flyby_perception` | YOLO11 + ByteTrack detection/tracking | Complete |
| `flyby_f11_sensors` | ISR camera, gimbal controller | Complete |
| `flyby_f11_bringup` | Launch files (simulation, hardware) | Complete |

### Autonomy - Partial
| Package | Description | Status |
|---------|-------------|--------|
| `flyby_autonomy` | State estimator, ontology controller | Basic |
| `flyby_depth` | Depth scale correction | In Progress |
| `behavior_trees` | BT executor @ 50Hz, nodes | Partial |

### Planning - Scaffolded
| Package | Description | Status |
|---------|-------------|--------|
| `path_planning` | OMPL Informed RRT*, NFZ manager | Scaffolded |
| `local_avoidance` | 3D-VFH+, OctoMap | Scaffolded |

### RL & Reasoning - Scaffolded
| Package | Description | Status |
|---------|-------------|--------|
| `rl_inference` | TensorRT policy inference | Scaffolded |
| `vampire_bridge` | Vampire ATP ROS 2 bridge | Scaffolded |
| `ontology_rl` | Ontology-RL integration | Scaffolded |
| `mission_orchestrator` | Mission coordination | Scaffolded |

## Key Message Types (flyby_msgs)

```
UAVState.msg          # Full vehicle state (60Hz)
DetectionArray.msg    # YOLO detections with tracking
MissionTasking.msg    # Mission objectives, NFZs, targets
BehaviorCommand.msg   # Behavior tree commands
ThreatZone.msg        # NFZ/threat zone definitions
DepthMap.msg          # Depth sensor data
```

## Key Actions/Services

```
PlanPath.action       # Global path planning request
OntologyQuery.srv     # Safety/validity queries to Vampire
```

## Building

```bash
# Full build
colcon build --symlink-install

# Single package
colcon build --packages-select flyby_perception

# With debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Launch Files

```bash
# Isaac Sim bridge
ros2 launch flyby_f11_bringup isaac_sim_bridge.launch.py

# Full simulation
ros2 launch flyby_f11_bringup simulation.launch.py

# Navigation (scaffolded)
ros2 launch flyby_f11_bringup navigation.launch.py
```

## Integration with Isaac Sim

The `isaac-sim/ros2_packages/` directory contains bridge packages that:
- Publish camera images to ROS 2 topics
- Bridge MAVLink commands via MAVSDK
- Synchronize state between simulation and ROS 2

## Code Style

- **C++**: `ament_uncrustify` with 4-space indentation
- **Python**: `ruff format` with snake_case
- **Messages**: UpperCamelCase for types

## Hardware Deployment Path

1. Train/validate in Isaac Sim
2. Build packages for ARM64 (Jetson)
3. Deploy via quadlet containers (see `deployment/`)
4. Launch using systemd services

## Current Priorities

1. Complete `flyby_depth` scale correction
2. Implement behavior tree action nodes
3. Test `vampire_bridge` integration
4. Validate `rl_inference` on Jetson

## Related Documentation

- `src/flyby_msgs/msg/`: All message definitions
- `src/behavior_trees/`: BT node implementations
- `../deployment/CLAUDE.md`: Container deployment guide
