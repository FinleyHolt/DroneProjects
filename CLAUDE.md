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

### Initial Setup
```bash
# Install PX4 + Gazebo SITL toolchain (fresh Ubuntu LTS)
bash setup/clone_px4.sh
# Or manually:
git clone https://github.com/PX4/PX4-Autopilot.git --recursive .deps/PX4-Autopilot
bash .deps/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Install ROS 2 tools and pip dependencies
bash setup/bootstrap.sh

# Source environment variables
source setup/env.sh
# or
source env/px4_sitl.env
```

### Building and Running

```bash
# Build PX4 and launch Gazebo SITL (default vehicle)
make -C .deps/PX4-Autopilot px4_sitl gazebo

# Build all ROS 2 packages with fast iteration symlinks
cd ros2_ws && colcon build --symlink-install

# Launch autonomy stack against running simulator
source ros2_ws/install/setup.bash
ros2 launch agents bringup.launch.py

# Run SITL smoke tests headless
make -C .deps/PX4-Autopilot px4_sitl gazebo_headless test
```

### Testing

```bash
# Run tests for specific ROS 2 package
cd ros2_ws
colcon test --packages-select <pkg>

# Run all ROS 2 tests
colcon test

# Test results
colcon test-result --verbose
```

## Directory Structure

```
px4-config/          # Custom PX4 parameter files, mixers per drone variant
  llm_drone_v1/      # Specific drone configuration with params, mixer, tuning notes
sim/
  worlds/            # Custom Gazebo worlds (e.g., cityscape.world)
  models/            # Custom Gazebo models with meshes (managed via Git LFS)
  scripts/           # World generation helpers
ros2_ws/
  src/
    agents_control/       # Offboard controllers, planners
    agents_interface/     # Custom ROS messages/services
    agents_llm_bridge/    # LLM orchestration nodes
    agents_sim_tools/     # SITL utilities, loggers
llm/
  prompts/           # Grounding instructions (e.g., safety_sops.md)
  configs/           # Model + tool settings (llm_drone.yaml)
  notebooks/         # Jupyter prototyping (large files via Git LFS)
scripts/             # High-level wrappers (run_sim.sh sources env + launches)
tests/               # Integration and unit tests
  integration/
  unit/
```

## Code Style and Conventions

- **PX4 C/C++**: Use `clang-format` per PX4 upstream conventions
- **ROS 2 C++**: Use `ament_uncrustify` with 4-space indentation
- **Python**: Use `ruff format` with snake_case for modules, 4-space indentation
- **ROS 2 Messages**: UpperCamelCase for message types
- **Launch Files**: kebab-case naming
- **Experimental Code**: Prefix branches/folders with `exp-` to signal unstable code

## ROS 2 Package Organization

Packages under `ros2_ws/src/` follow standard ROS 2 structure aligned with project objectives:
- `agents_control/`: Offboard flight controllers, trajectory planners, waypoint navigation
- `agents_interface/`: Custom message/service definitions (mission tasks, reasoning results, perception outputs)
- `agents_llm_bridge/`: LLM reasoning engine integration, natural-language parsing, task decomposition, dynamic replanning
- `agents_perception/`: Onboard vision models (object detection, segmentation), sensor fusion, anomaly detection
- `agents_mavsdk_bridge/`: MAVSDK integration for MAVLink telemetry and flight commands
- `agents_sim_tools/`: SITL utilities, logging, mission replay, evaluation metrics

## PX4 Configuration Management

Each drone variant gets a folder under `px4-config/` containing:
- `params.params`: QGroundControl-exported parameter file
- `mixer.main.mix`: Actuator mixer configuration
- `README.md`: Rationale and tuning notes

Load custom parameters when launching SITL by referencing these files.

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

## Phase-Aware Development

Project follows phased approach (see Planning/ directory for detailed phase plans):
- **Phase 1**: Baseline PX4 SITL bring-up, setup scripts, basic integration
- **Phase 2**: ROS 2 workspace scaffolding, MAVSDK bridge, stub packages
- **Phase 3**: LLM reasoning engine integration, natural-language parsing
- **Phase 4**: Perception pipeline, vision models, sensor fusion
- **Phase 5**: Mission planning and adaptive replanning
- **Phase 6**: Simulation validation, edge deployment, live-flight testing

When adding features, align with current phase objectives. Reference `Planning/README.md` for project overview and detailed phase plans in `Planning/Phase-N.md` files.
