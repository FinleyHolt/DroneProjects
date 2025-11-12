# Repository Guidelines

## Project Overview
LLMDrone is a reasoning-enabled autonomy framework for small unmanned aerial systems (sUAS) developed under MCTSSA Digital Solutions Branch. The system converts natural-language mission tasking into executable flight plans using locally hosted LLMs, enabling autonomous operations in communications-denied environments without external connectivity.

## Project Structure & Module Organization
- **PX4 Firmware**: External dependency in `.deps/PX4-Autopilot/` (cloned via setup scripts, not committed)
- **ROS 2 Packages**: Autonomy software in `ros2_ws/src/` with modular packages for control, perception, LLM reasoning, and MAVSDK integration
- **LLM Assets**: Reasoning engine, prompts, and model configs in `llm/` directory
- **Simulation Assets**: Gazebo worlds and models in `sim/` with Git LFS for large files
- **Configuration**: PX4 params/mixers per drone variant in `px4-config/`
- **Documentation**: Architecture docs and runbooks in `docs/`; detailed phase plans in `Planning/`

## Build, Test, and Development Commands

### Initial Setup
- `bash setup/clone_px4.sh` — clone PX4 into `.deps/` and run ubuntu.sh setup
- `bash setup/bootstrap.sh` — install ROS 2 tools and Python dependencies
- `source setup/env.sh` — export required environment variables (PX4_HOME, GAZEBO_MODEL_PATH, ROS_DOMAIN_ID)

### Building
- `make -C .deps/PX4-Autopilot px4_sitl gazebo` — build PX4 and launch Gazebo SITL (default vehicle)
- `cd ros2_ws && colcon build --symlink-install` — compile all ROS 2 packages with fast iteration symlinks

### Running
- `source ros2_ws/install/setup.bash && ros2 launch agents bringup.launch.py` — start autonomy stack against running simulator
- `scripts/run_sim.sh` — wrapper script that sources environment and launches simulation

### Testing
- `make -C .deps/PX4-Autopilot px4_sitl gazebo_headless test` — run SITL smoke tests headless
- `cd ros2_ws && colcon test --packages-select <pkg>` — run tests for specific ROS 2 package
- `colcon test-result --verbose` — view detailed test results

## Coding Style & Naming Conventions
- **PX4 C/C++**: Use `clang-format` per PX4 upstream conventions
- **ROS 2 C++**: Use `ament_uncrustify` with 4-space indentation
- **Python**: Use `ruff format` with snake_case for modules, 4-space indentation
- **ROS 2 Messages**: UpperCamelCase for message types
- **Launch Files**: kebab-case naming
- **Experimental Code**: Prefix branches/folders with `exp-` to signal unstable code

## ROS 2 Package Organization
Packages in `ros2_ws/src/` aligned with project objectives:
- **agents_control**: Offboard flight controllers, trajectory planners, waypoint navigation
- **agents_interface**: Custom message/service definitions (mission tasks, reasoning results, perception outputs)
- **agents_llm_bridge**: LLM reasoning engine, natural-language parsing, task decomposition, dynamic replanning
- **agents_perception**: Onboard vision models (object detection, segmentation), sensor fusion, anomaly detection
- **agents_mavsdk_bridge**: MAVSDK integration for MAVLink telemetry and flight commands
- **agents_sim_tools**: SITL utilities, logging, mission replay, evaluation metrics

## Testing Guidelines
- **SITL Smoke Tests**: Run headless via `make -C .deps/PX4-Autopilot px4_sitl gazebo_headless test`
- **ROS 2 Unit Tests**: Create `test_*.py` or `test_*.cpp` in each package's `test/` directory; run with `colcon test --packages-select <pkg>`
- **Integration Tests**: Located in `tests/integration/`, validate full stack behavior (LLM → planning → control → flight)
- **Mission Scenarios**: Test natural-language parsing, adaptive replanning, perception integration
- **Performance Metrics**: Track reasoning latency, mission completion time, replanning frequency
- **Hardware-in-the-Loop**: Target minimum one HIL session per release; capture logs in `logs/<date>-<scenario>/`

## Commit & Pull Request Guidelines
- **Commit Style**: Imperative mood ("Add offboard controller", "Fix SITL topic sync", "Integrate YOLOv8 detection")
- **Logical Grouping**: Group related changes; reference PX4/ROS tickets or research papers when applicable
- **Pull Request Requirements**:
  - Context: What problem does this solve? Which phase objective does it address?
  - Reproduction steps or usage instructions
  - Test evidence: Command outputs, log snippets, reasoning latency measurements
  - Simulator screenshots or recordings for behavioral changes
  - Documentation updates if architecture or APIs change
- **Review Process**: At least one review from someone familiar with affected subsystem (autonomy, perception, flight control)
- **Phase Alignment**: Ensure changes align with current phase objectives in `Planning/` directory

## Development Objectives
1. **Natural-Language Mission Parsing**: Translate intent into structured sub-tasks, update plans dynamically
2. **Modular Containerized Stack**: ROS 2 architecture integrating reasoning, perception, control via MAVSDK
3. **Onboard Sensor Integration**: Vision models for verification, detection, anomaly detection, replanning
4. **Validation Pipeline**: Prove reliability, latency, and robustness in SITL before live-flight testing

## Project Deliverables
- Edge-deployable reasoning module and containerized software stack
- Simulation and live-flight demonstrations of autonomous mission execution from natural-language tasking
- Publishable research paper on reasoning-enabled autonomy framework design and evaluation

## Phase-Aware Development
Project follows phased approach with detailed plans in `Planning/` directory:
- **Phase 1**: Baseline PX4 SITL bring-up, setup scripts, basic integration
- **Phase 2**: ROS 2 workspace scaffolding, MAVSDK bridge, stub packages
- **Phase 3**: LLM reasoning engine integration, natural-language parsing
- **Phase 4**: Perception pipeline, vision models, sensor fusion
- **Phase 5**: Mission planning and adaptive replanning
- **Phase 6**: Simulation validation, edge deployment, live-flight testing

Reference `Planning/README.md` for project overview and detailed phase plans before starting new work to ensure alignment with project timeline.
