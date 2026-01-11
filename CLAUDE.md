# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Session Coordination (Automatic via Hooks)

This repository uses automatic multi-session coordination via Claude Code hooks. No manual steps required.

### What Happens Automatically

- **On session start**: Session manifest created in `.claude/sessions/active/`, open GitHub issues shown
- **Before file edits**: Path conflicts checked against registry, warnings shown if another session has claimed the path
- **Path tracking**: Edited file paths are auto-tracked in your session manifest

### Commands Available

| Command | Purpose |
|---------|---------|
| `/session-status` | View all active sessions and claimed paths |
| `/claim-task` | Explicitly claim a task with GitHub issue integration |
| `/handoff` | Hand off incomplete work to another session |
| `/complete-task` | Mark task done, create PR, move issue to Done |
| `/whats-next` | Check GitHub project board for available tasks |

### Commit Prefix Convention

Use these prefixes when committing:
- `WIP:` - work in progress
- `CHECKPOINT:` - stable state, can be picked up by others
- `HANDOFF:` - explicitly releasing for another session
- `COMPLETE:` - task finished, ready for PR

### How It Works

1. **SessionStart hook** (`.claude/hooks/session-init.sh`) creates session manifest and shows coordination context
2. **PreToolUse hook** (`.claude/hooks/check-path-conflicts.sh`) checks for conflicts before Edit/Write operations
3. Conflicts show warnings but don't block - you can proceed if needed
4. Run `/claim-task` to formally claim paths and integrate with GitHub Issues/Projects

---

## Project Overview

Drone autonomy development portfolio focused on Isaac Sim-based simulation, reinforcement learning, and edge AI deployment. The primary development environment is Isaac Sim with PX4 SITL for photorealistic drone simulation and RL training.

**Developer**: Finley Holt
**Focus**: Isaac Sim simulation, reinforcement learning for autonomous navigation, ontology-guided mission planning, and edge deployment on NVIDIA Jetson platforms.

**Technical Foundation**: Isaac Sim for photorealistic simulation, PX4 SITL for flight control, Stable Baselines3 for RL training, ROS 2 Jazzy for autonomy software, and Gymnasium for environment wrappers.

## Current Development Status (January 2026)

### Working
- Isaac Sim 5.1.0 + PX4 SITL container with GPU passthrough
- YOLO11 + ByteTrack perception pipeline
- Ground truth frustum filtering (1000+ Hz)
- 3 canonical ISR environments defined (comms-denied, dynamic NFZ, multi-objective)
- ROS 2 camera bridge
- Gimbal control

### Proof of Concept (demonstrated but not production-ready)
- SAC/PPO/TD3 training infrastructure - runs but no good trained model yet
- Safety filter with Vampire ATP - integrated but not rigorously tested
- Ontology reasoning - defined but needs significant testing/refinement

### In Progress
- Depth perception calibration and obstacle avoidance
- Behavior tree architecture with RL as leaf nodes (for specific tasks like area exploration)

### Planned
- Rigorous ontology testing and validation
- RL agent for area exploration (behavior tree leaf node)
- Complete deployment containers for Jetson
- MAVSDK bridge for project-drone
- Real hardware flight testing

## Capability Roadmap

### M1: Simulation Infrastructure
- [x] Isaac Sim + PX4 SITL container
- [x] 3 canonical ISR environments defined
- [x] Gymnasium wrapper for RL

### M2: Perception Pipeline
- [x] YOLO11 + ByteTrack detection/tracking
- [x] Frustum culling (ground truth)
- [x] Gimbal control
- [ ] Depth avoidance calibration (in progress)

### M3: Behavior Tree Architecture
- [ ] BT executor with action nodes
- [ ] RL leaf nodes for specific tasks (area exploration)
- [ ] Ontology-guided behavior preemption

### M4: Local Planning
- [ ] 3D-VFH+ obstacle avoidance
- [ ] OctoMap integration
- [ ] NFZ-aware path planning

### M5: Safety & Reasoning (needs rigorous testing)
- [~] Safety filter (Vampire ATP) - integrated, not validated
- [~] Ontology behavior controller - defined, not tested
- [ ] Rigorous ontology test suite
- [ ] Full ontology-RL integration

### M6: RL Training
- [~] SAC/PPO/TD3 infrastructure - runs, no good model
- [ ] Area exploration RL agent (BT leaf node)
- [ ] Fast training (direct dynamics)

### M7: ROS 2 Stack
- [x] Core messages (flyby_msgs)
- [x] Perception nodes
- [ ] Complete behavior tree nodes
- [ ] Mission orchestrator

### M8: Deployment
- [x] Isaac Sim container
- [ ] Complete quadlet containers
- [ ] Jetson-optimized builds

### M9: Hardware Integration
- [ ] MAVSDK bridge for project-drone
- [ ] VIO flight (T265)
- [ ] Real flight validation

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
├── ros2_ws/                     # ROS 2 workspace for real hardware (14 packages)
│   └── src/
│       ├── flyby_msgs/          # Custom message definitions (13 msgs, 1 action, 1 srv)
│       ├── flyby_perception/    # YOLO + ByteTrack perception
│       ├── flyby_depth/         # Depth processing (in progress)
│       ├── flyby_autonomy/      # State estimator, ontology controller
│       ├── flyby_f11_sensors/   # ISR camera, gimbal control
│       ├── flyby_f11_bringup/   # Launch files
│       ├── behavior_trees/      # BT executor, action/condition nodes
│       ├── path_planning/       # OMPL Informed RRT* (scaffolded)
│       ├── local_avoidance/     # 3D-VFH+, OctoMap (scaffolded)
│       ├── rl_inference/        # TensorRT policy inference (scaffolded)
│       ├── mission_orchestrator/ # Mission coordination (scaffolded)
│       ├── vampire_bridge/      # Ontology reasoning bridge (scaffolded)
│       └── ontology_rl/         # Ontology-RL integration (scaffolded)
│
├── ontology/                    # Ontology-based reasoning (needs rigorous testing)
│   ├── planning_mode/           # SUMO + Vampire ATP (heavyweight, offline)
│   ├── execution_mode/          # SWI-Prolog (lightweight, runtime)
│   └── translation/             # KIF to Prolog translator
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

### Local Development Environment (micromamba)

For local linting, formatting, and tools that don't require Isaac Sim, use the `flyby-f11-eval` micromamba environment. **Do NOT install packages system-wide.**

```bash
# Activate the environment
micromamba activate flyby-f11-eval

# Run ruff format check
ruff format --check isaac-sim/

# Run ruff lint
ruff check isaac-sim/

# Run static tests (no Isaac Sim required)
pytest isaac-sim/tests/test_static.py -v

# Deactivate when done
micromamba deactivate
```

Environment location: `/home/finley/micromamba/envs/flyby-f11-eval`

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

## ROS 2 Packages (ros2_ws/src/)

### Core (Complete)
- **flyby_msgs**: 13 message types, 1 action (PlanPath), 1 service (OntologyQuery)
- **flyby_perception**: YOLO11 + ByteTrack detection/tracking nodes
- **flyby_f11_sensors**: ISR camera node, gimbal controller
- **flyby_f11_bringup**: Launch files for simulation and hardware

### Autonomy (Partial)
- **flyby_autonomy**: State estimator, ontology controller (basic)
- **flyby_depth**: Depth scale correction (in progress)
- **behavior_trees**: BT executor @ 50Hz, action/condition nodes (partial)

### Planning (Scaffolded)
- **path_planning**: OMPL Informed RRT*, NFZ manager
- **local_avoidance**: 3D-VFH+, OctoMap integration

### RL & Reasoning (Scaffolded)
- **rl_inference**: TensorRT policy inference for Jetson
- **vampire_bridge**: ROS 2 interface to Vampire ATP
- **ontology_rl**: Ontology-guided RL integration
- **mission_orchestrator**: Mission-level coordination

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

## Developer Preferences

- **Shell Scripts over Compose**: Prefer standalone shell scripts (like `run_depth_avoidance_test.sh`) over podman-compose for container orchestration. Shell scripts provide better visibility and control.
- **Container Execution**: Use `podman run` with explicit flags rather than compose files.
- **Container Dependencies**: When installing packages into the container at runtime (e.g., `apt-get install`), always update the Containerfile (`isaac-sim/Containerfile`) so fresh builds include all dependencies. Keep the container self-contained.

## Git Workflow

- **Commit Style**: Imperative mood ("Add obstacle avoidance", "Fix depth correction")
- **Logical Grouping**: Group related changes, reference issues when applicable
- **Ignored Files**: Build artifacts, model weights, test outputs excluded via `.gitignore`

## Multi-Session Coordination

See "Session Coordination (Automatic via Hooks)" at the top of this file for details.

### Coordination Files

```
.claude/
├── hooks/                   # Claude Code hooks for auto-coordination
│   ├── session-init.sh      # SessionStart hook
│   └── check-path-conflicts.sh  # PreToolUse hook for Edit/Write
├── tasks/registry.json      # Task claims (path -> session mapping)
├── sessions/active/         # Session manifests
├── handoffs/                # Handoff documents
└── coordination.log         # Event log
```

### Stale Session Recovery

Sessions inactive >2 hours can be recovered. Use `/claim-task --recover <task-slug>` to take over.
