# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LLMDrone is a reasoning-enabled autonomy framework for small unmanned aerial systems (sUAS) developed under MCTSSA Digital Solutions Branch. The project enables drones to interpret natural-language mission intent and execute autonomous missions in communications-denied environments.

**Mission**: Convert natural-language tasking (e.g., "Survey NAI 3 and identify vehicle movement") into executable flight plans with onboard reasoning, adaptive planning, and context-aware decision-making—without requiring external connectivity.

**Technical Foundation**: Locally hosted LLM with Mixture-of-Experts (MoE) architecture on edge compute devices (NVIDIA Jetson Orin/Thor), integrated with ROS 2 for autonomy software, MAVSDK for flight control, PX4 autopilot, and Gazebo simulation.

## Key Architectural Decisions

- **Edge-First Autonomy**: All reasoning and planning runs locally on embedded compute; no cloud dependency or persistent connectivity required
- **Modular Stack**: ROS 2 for inter-component communication, MAVSDK for MAVLink/flight control, PX4 for autopilot firmware
- **LLM Architecture**: Mixture-of-Experts model maintains all expert weights onboard but activates only relevant pathways to minimize compute/power demand
- **External Dependencies**: PX4-Autopilot and Gazebo binaries cloned into `.deps/` via scripts (not committed to reduce repo size)
- **Configuration Management**: Custom configs (params, mixers, launch files) versioned per drone variant in tracked folders
- **Workspace Separation**: ROS 2 autonomy packages in `ros2_ws/src/`, LLM reasoning/planning in `llm/`, perception models separate
- **Asset Management**: Large Gazebo worlds/models managed with Git LFS
- **Logging**: SITL/flight logs stay outside repo by default; curated logs in `logs/<date>-<scenario>/` for analysis

## Development Environment

**Primary Platform**: Ubuntu LTS with ROS 2 Humble, PX4 SITL, and Gazebo
**Target Deployment**: NVIDIA Jetson Orin or Jetson Thor edge compute module

Development assumes:
- NVIDIA/graphics drivers for GPU acceleration (simulation and vision models)
- Network access during setup for fetching PX4, ROS dependencies, LLM weights
- LLM model weights and API keys managed via environment variables or secure storage (never committed)
- Python 3.10+ for LLM inference frameworks

## Build, Setup, and Development Commands

### Initial Setup (test-drone)

The primary development platform is `test-drone/`. Start here:

```bash
# Install PX4 + Gazebo SITL toolchain (fresh Ubuntu LTS)
bash setup/clone_px4.sh
# Or manually:
git clone https://github.com/PX4/PX4-Autopilot.git --recursive .deps/PX4-Autopilot
bash .deps/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Install ROS 2 tools and pip dependencies
bash setup/bootstrap.sh

# Navigate to test-drone project
cd test-drone
```

### Building and Running test-drone

```bash
# Using Docker (recommended)
cd test-drone
docker compose up

# OR building locally
cd test-drone/ros2_ws
colcon build --symlink-install

# Launch simulation
source install/setup.bash
ros2 launch test_drone_bringup simulation.launch.py

# Launch on real hardware
ros2 launch test_drone_bringup real_hardware.launch.py
```

### Testing

```bash
# Run tests for specific ROS 2 package
cd test-drone/ros2_ws
colcon test --packages-select <pkg>

# Run all ROS 2 tests
colcon test

# Test results
colcon test-result --verbose
```

### Working with flyby (future)

When flyby hardware becomes available:

```bash
cd flyby/ros2_ws/src

# Link shared packages from test-drone
ln -s ../../../test-drone/ros2_ws/src/autonomy_core ./
ln -s ../../../test-drone/ros2_ws/src/behavior_trees ./
ln -s ../../../test-drone/ros2_ws/src/px4_interface ./
ln -s ../../../test-drone/ros2_ws/src/perception_pipeline ./

# Build flyby workspace
cd ..
colcon build --symlink-install
```

## Repository Structure

The repository is organized into **self-contained platform projects** with shared autonomy components:

```
LLMDrone/
├── test-drone/              # Development platform (8GB GPU, custom quadcopter)
│   ├── ros2_ws/             # Complete ROS 2 workspace for test-drone
│   │   └── src/
│   │       ├── test_drone_bringup/      # Hardware-specific launch files
│   │       ├── test_drone_sensors/      # T265, D455, experimental sensors
│   │       ├── autonomy_core/           # SHARED: Core autonomy logic
│   │       ├── behavior_trees/          # SHARED: BehaviorTree.CPP mission logic
│   │       ├── perception_pipeline/     # SHARED: Vision models
│   │       └── px4_interface/           # SHARED: MAVSDK/MAVLink bridge
│   ├── docker/              # Test-drone container configs
│   ├── simulation/          # Gazebo worlds/models for test-drone
│   ├── launch/              # Top-level launch files
│   ├── config/              # PX4 params, sensor calibration
│   └── README.md
│
├── flyby/                   # Production flyby mission (16GB GPU, different sensors)
│   ├── ros2_ws/             # Flyby-specific workspace
│   │   └── src/
│   │       ├── flyby_bringup/           # Mission-specific launch files
│   │       ├── flyby_sensors/           # Flyby sensor suite drivers
│   │       ├── flyby_mission/           # Flyby mission logic
│   │       ├── autonomy_core/           # Symlink to test-drone version
│   │       ├── behavior_trees/          # Symlink to test-drone version
│   │       ├── perception_pipeline/     # Symlink or optimized version
│   │       └── px4_interface/           # Symlink to test-drone version
│   ├── docker/              # Deployment container (optimized)
│   ├── simulation/          # Flyby scenario worlds
│   ├── launch/
│   ├── config/
│   ├── deployment/          # Preflight checks, field configs
│   └── README.md
│
├── llm/                     # Shared LLM integration (future)
│   ├── prompts/
│   ├── configs/
│   └── notebooks/
│
├── docs/                    # Project-wide documentation
└── .deps/                   # External dependencies (PX4, Gazebo)
```

### Directory Organization Philosophy

1. **Platform Projects** (`test-drone/`, `flyby/`):
   - Each is a **complete, self-contained** drone project
   - Has its own ROS 2 workspace with all needed packages
   - Hardware-specific packages live only in that platform
   - Can be independently built, tested, and deployed

2. **Shared Components**:
   - Core autonomy packages developed in `test-drone/ros2_ws/src/`
   - Shared to `flyby/` via symlinks when ready for production
   - Packages prefixed with platform name (e.g., `test_drone_*`) are NOT shared
   - Generic packages (e.g., `autonomy_core`, `px4_interface`) ARE shared

3. **Development Workflow**:
   - Active development happens in `test-drone/` (accessible hardware)
   - Shared packages are designed to be platform-agnostic
   - When flyby hardware becomes available, link shared packages
   - Platform-specific code stays isolated in respective projects

## Code Style and Conventions

- **PX4 C/C++**: Use `clang-format` per PX4 upstream conventions
- **ROS 2 C++**: Use `ament_uncrustify` with 4-space indentation
- **Python**: Use `ruff format` with snake_case for modules, 4-space indentation
- **ROS 2 Messages**: UpperCamelCase for message types
- **Launch Files**: kebab-case naming
- **Experimental Code**: Prefix branches/folders with `exp-` to signal unstable code

## ROS 2 Package Organization

### test-drone Packages

Packages in `test-drone/ros2_ws/src/` are organized by shareability:

**Platform-Specific (NOT shared)**:
- `test_drone_bringup/`: Launch files, hardware bring-up configurations
- `test_drone_sensors/`: T265, D455, and experimental sensor drivers/wrappers

**Shared Autonomy Components** (designed to be platform-agnostic):
- `autonomy_core/`: Core mission planning, waypoint navigation, state machines
- `behavior_trees/`: BehaviorTree.CPP mission logic, BT node definitions
- `perception_pipeline/`: Vision models (object detection, segmentation), inference nodes
- `px4_interface/`: MAVSDK/MAVLink bridge, flight command abstraction

**Supporting Packages**:
- `agents_interface/`: Custom message/service definitions (standard across platforms)
- `agents_llm_bridge/`: LLM reasoning integration (future)

### flyby Packages

Packages in `flyby/ros2_ws/src/`:

**Flyby-Specific**:
- `flyby_bringup/`: Mission-specific launch configurations
- `flyby_sensors/`: Flyby sensor suite drivers
- `flyby_mission/`: Mission-specific logic (route planning, target recognition)

**Symlinked from test-drone**:
- `autonomy_core/` → `../../../test-drone/ros2_ws/src/autonomy_core`
- `behavior_trees/` → `../../../test-drone/ros2_ws/src/behavior_trees`
- `px4_interface/` → `../../../test-drone/ros2_ws/src/px4_interface`
- `perception_pipeline/` → May be symlinked or copied/optimized for 16GB GPU

### Package Naming Convention

- Packages prefixed with platform name (e.g., `test_drone_*`, `flyby_*`) are platform-specific
- Generic names (e.g., `autonomy_core`, `px4_interface`) indicate shared components
- Design shared packages to be hardware-agnostic (depend only on standard ROS 2 message types)

## PX4 Configuration Management

Each platform project contains its own PX4 configuration in `<platform>/config/`:
- `px4_params.params`: QGroundControl-exported parameter file
- `sensor_calibration.yaml`: Sensor-specific calibration data
- `README.md`: Platform-specific tuning notes and rationale

Load custom parameters when launching SITL by referencing these files in launch configurations.

## Environment Variables

Key variables set by `setup/env.sh` or `env/px4_sitl.env`:
- `PX4_HOME_*`: PX4 home coordinates for SITL
- `GAZEBO_MODEL_PATH`: Includes `sim/models/` for custom assets
- `ROS_DOMAIN_ID`: Isolates ROS 2 DDS traffic
- LLM API credentials (never commit these)

## Testing Strategy

- **SITL Smoke Tests**: Run headless via PX4's test framework
- **ROS 2 Unit Tests**: Place `test_*.py` or `test_*.cpp` in package `test/` directories
- **Integration Tests**: Located in `tests/integration/`, validate full stack behavior
- **Hardware-in-the-Loop**: Target minimum one HIL session per release, logs in `logs/<date>-<scenario>/`

## Git Workflow

- **Commit Style**: Imperative mood ("Add offboard controller", "Fix SITL topic sync")
- **Logical Grouping**: Group related changes, reference PX4/ROS tickets when applicable
- **Pull Requests**: Must include context, reproduction steps, test evidence, simulator screenshots for behavioral changes
- **Review**: Require at least one review from someone familiar with affected subsystem
- **Ignored Files**: `.deps/` directory and build artifacts excluded via `.gitignore`

## Common Pitfalls

- **Environment Sourcing**: Always source `setup/env.sh` before running simulations or ROS nodes
- **PX4 Build**: First build can take 10+ minutes; subsequent builds are incremental
- **ROS 2 Overlay**: Remember to source `ros2_ws/install/setup.bash` in addition to ROS 2 base installation
- **Gazebo Models**: Custom models must be in `GAZEBO_MODEL_PATH`; verify with `echo $GAZEBO_MODEL_PATH`
- **DDS Domain**: If nodes can't discover each other, check `ROS_DOMAIN_ID` is consistent

## Development Objectives (Project Deliverables)

1. **Natural-Language Mission Parsing**: Translate intent (e.g., "Survey NAI 3") into structured sub-tasks and update plans dynamically
2. **Modular Containerized Stack**: ROS 2-based architecture integrating reasoning, perception, and control via MAVSDK bridges
3. **Onboard Sensor Integration**: Vision models for task verification, vehicle/object detection, anomaly detection, local replanning
4. **Validation Pipeline**: Prove reasoning reliability, latency, and mission robustness in PX4-SITL + Gazebo before live flight testing

## Project Deliverables

- Edge-deployable reasoning module and containerized software stack
- Simulation and live-flight demonstrations of autonomous mission execution from natural-language tasking
- Publishable research paper on reasoning-enabled autonomy framework design and evaluation

## Development Platforms

### test-drone (Primary Development Platform)
- **Hardware**: Custom quadcopter with T265 visual odometry, D455 depth camera
- **Compute**: 8GB GPU memory
- **Purpose**: Experimentation, sensor testing, behavior tree development
- **Status**: Active development
- **Location**: Available for immediate testing

### flyby (Production Mission Platform)
- **Hardware**: Quadcopter with PX4 autopilot, different sensor suite than test-drone
- **Compute**: 16GB GPU memory (can run heavier models)
- **Purpose**: Autonomous flyby mission with obstacle avoidance
- **Status**: Hardware not yet accessible (several months)
- **Strategy**: Shared autonomy components developed on test-drone will be linked when ready

## Phase-Aware Development

Project follows phased approach (see Planning/ directory for detailed phase plans):
- **Phase 1**: Baseline PX4 SITL bring-up, setup scripts, basic integration
- **Phase 2**: ROS 2 workspace scaffolding, MAVSDK bridge, stub packages
- **Phase 3**: LLM reasoning engine integration, natural-language parsing
- **Phase 4**: Perception pipeline, vision models, sensor fusion
- **Phase 5**: Mission planning and adaptive replanning
- **Phase 6**: Simulation validation, edge deployment, live-flight testing

**Current Focus**: Development on test-drone platform. All shared autonomy components are designed to be platform-agnostic for future use on flyby.

When adding features, align with current phase objectives. Reference `Planning/README.md` for project overview and detailed phase plans in `Planning/Phase-N.md` files.
