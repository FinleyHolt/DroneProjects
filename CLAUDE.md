# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Personal drone autonomy development portfolio showcasing edge AI, computer vision, and autonomous navigation projects. The repository contains multiple self-contained drone projects, each targeting specific hardware platforms.

**Developer**: Finley Holt
**Focus**: Edge-based autonomous navigation, visual SLAM, behavior trees, and embedded AI for small unmanned aerial systems (sUAS)

**Technical Foundation**: ROS 2 for autonomy software, MAVLink/MAVROS for flight control, ArduPilot autopilot firmware, and Gazebo simulation. All processing runs locally on embedded NVIDIA Jetson compute modules.

## Key Architectural Decisions

- **Edge-First Autonomy**: All processing runs locally on embedded compute; no cloud dependency required
- **Modular Stack**: ROS 2 for inter-component communication, MAVROS for MAVLink/flight control, ArduPilot for autopilot firmware
- **Podman-First Development**: All platforms use Podman (rootless containers) for self-contained, portable development and deployment
  - GPU passthrough support for vision models and simulation
  - All dependencies documented and versioned in container specifications
  - No host-level dependency conflicts; completely reproducible environments
- **External Dependencies**: ArduPilot and Gazebo binaries managed via setup scripts (not committed to reduce repo size)
- **Configuration Management**: Custom configs (params, mixers, launch files) versioned per drone variant
- **Workspace Separation**: Each platform has its own ROS 2 workspace; shared packages linked as needed
- **Asset Management**: Large Gazebo worlds/models managed with Git LFS
- **Platform Independence**: Each drone folder is completely self-contained and can be developed/deployed independently

## Development Environment

**Primary Platform**: Ubuntu LTS with ROS 2 Humble, ArduPilot SITL, and Gazebo
**Target Deployment**: NVIDIA Jetson Orin Nano Super 8GB (project-drone), NVIDIA Jetson Orin NX (flyby-f11)

Development assumes:
- NVIDIA/graphics drivers for GPU acceleration (simulation and vision models)
- Network access during setup for fetching ArduPilot, ROS dependencies
- Podman for rootless containerized workflows with GPU passthrough
- All language runtimes and tools managed inside containers (no host dependencies)

## Build, Setup, and Development Commands

### Working with project-drone

The primary development platform is `project-drone/` (Jetson Orin Nano Super with T265 + D455):

```bash
# Navigate to project-drone
cd project-drone

# Using Podman (recommended - fully self-contained)
podman-compose up

# OR build and run with GPU passthrough for vision models
podman-compose up --build

# Enter container for interactive development
podman exec -it project-drone-dev bash

# Inside container - build ROS 2 workspace
cd /workspace/ros2_ws
colcon build --symlink-install

# Launch simulation (GPU passthrough enabled)
source install/setup.bash
ros2 launch project_drone_bringup simulation.launch.py

# Launch on real hardware
ros2 launch project_drone_bringup real_hardware.launch.py
```

### Working with flyby-f11

When flyby-f11 hardware becomes available:

```bash
cd flyby-f11

# Using Podman (self-contained development)
podman-compose up

# Enter container for development
podman exec -it flyby-f11-dev bash

# Inside container
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch flyby_f11_bringup simulation.launch.py
```

### Testing

All tests run inside Podman containers for consistent, reproducible results:

```bash
# Enter development container
podman exec -it <platform>-dev bash

# Inside container - run tests for specific ROS 2 package
cd /workspace/ros2_ws
colcon test --packages-select <pkg>

# Run all ROS 2 tests
colcon test

# Test results
colcon test-result --verbose
```

## Repository Structure

The repository is organized into **self-contained platform projects**:

```
DroneProjects/
├── project-drone/           # Jetson Orin Nano Super development platform
│   ├── ros2_ws/            # Complete ROS 2 workspace
│   │   └── src/
│   │       ├── project_drone_bringup/     # Hardware-specific launch files
│   │       ├── project_drone_sensors/     # T265, D455 drivers
│   │       ├── autonomy_core/             # Core autonomy logic (shareable)
│   │       ├── behavior_trees/            # BehaviorTree.CPP logic (shareable)
│   │       ├── perception_pipeline/       # Vision models (shareable)
│   │       └── ardupilot_interface/        # MAVROS bridge (shareable)
│   ├── docker/             # Container configurations
│   ├── simulation/         # Gazebo worlds/models
│   ├── config/             # ArduPilot params, sensor calibration
│   └── README.md
│
├── flyby-f11/              # Flyby F-11 production platform
│   ├── ros2_ws/            # Flyby-specific workspace
│   │   └── src/
│   │       ├── flyby_f11_bringup/         # Mission-specific launch
│   │       ├── flyby_f11_sensors/         # Platform sensor drivers
│   │       ├── flyby_f11_mission/         # Mission-specific logic
│   │       ├── autonomy_core/             # Symlink to project-drone
│   │       ├── behavior_trees/            # Symlink to project-drone
│   │       ├── perception_pipeline/       # Symlink or optimized
│   │       └── ardupilot_interface/        # Symlink to project-drone
│   ├── docker/             # Deployment containers
│   ├── simulation/         # Flyby scenario worlds
│   ├── config/             # Mission-specific configs
│   └── README.md
│
├── AGENTS.md               # Development guidelines
├── CLAUDE.md               # This file
└── .gitignore
```

### Directory Organization Philosophy

1. **Platform Projects** (`project-drone/`, `flyby-f11/`):
   - Each is a **complete, self-contained** drone project
   - Has its own ROS 2 workspace with all needed packages
   - Hardware-specific packages live only in that platform
   - Can be independently built, tested, and deployed
   - Uses Podman for fully self-contained, reproducible environments
   - All dependencies documented in container specifications (no hidden host dependencies)

2. **Shared Components**:
   - Core autonomy packages developed in `project-drone/ros2_ws/src/`
   - Can be shared to `flyby-f11/` via symlinks when ready
   - Packages prefixed with platform name (e.g., `project_drone_*`) are NOT shared
   - Generic packages (e.g., `autonomy_core`, `ardupilot_interface`) ARE shareable

3. **Development Workflow**:
   - Active development happens in `project-drone/` (accessible hardware)
   - Shared packages are designed to be platform-agnostic
   - When flyby-f11 hardware becomes available, link shared packages
   - Platform-specific code stays isolated in respective projects

## Code Style and Conventions

- **ArduPilot C/C++**: Use `clang-format` per ArduPilot upstream conventions
- **ROS 2 C++**: Use `ament_uncrustify` with 4-space indentation
- **Python**: Use `ruff format` with snake_case for modules, 4-space indentation
- **ROS 2 Messages**: UpperCamelCase for message types
- **Launch Files**: snake_case naming with `.launch.py` extension
- **Experimental Code**: Prefix branches/folders with `exp-` to signal unstable code

## Documentation Conventions

- **Format**: All technical documentation uses Quarto Markdown (`.qmd` files)
- **YAML Header**: Every `.qmd` file must include YAML frontmatter with custom formatting:
  ```yaml
  ---
  title: "Document Title"
  author: "Finley Holt"
  date: today
  format:
    pdf:
      documentclass: article
      geometry:
        - margin=1in
      fontsize: 11pt
      number-sections: true
      toc: true
      include-in-header:
        text: |
          \usepackage{fancyhdr}
          \pagestyle{fancy}
          \fancyhf{}
          \fancyhead[L]{\textit{Short Document Title}}
          \fancyhead[R]{\thepage}
          \renewcommand{\headrulewidth}{0.4pt}
  ---
  ```
- **Rendering**: Use `quarto render filename.qmd` to generate PDF outputs
- **Purpose**: Ensures consistent professional formatting across all project documentation
- **Examples**: See `flyby-f11/APPROACH.qmd`, `flyby-f11/ONTOLOGY_FOUNDATION.qmd`, `flyby-f11/literature_review/*.qmd`

## ROS 2 Package Organization

### project-drone Packages

Packages in `project-drone/ros2_ws/src/` are organized by shareability:

**Platform-Specific (NOT shared)**:
- `project_drone_bringup/`: Launch files, hardware bring-up configurations
- `project_drone_sensors/`: T265, D455, and experimental sensor drivers/wrappers

**Shared Autonomy Components** (designed to be platform-agnostic):
- `autonomy_core/`: Core mission planning, waypoint navigation, state machines
- `behavior_trees/`: BehaviorTree.CPP mission logic, BT node definitions
- `perception_pipeline/`: Vision models (object detection, segmentation), inference nodes
- `ardupilot_interface/`: MAVROS/MAVLink bridge, flight command abstraction

**Supporting Packages**:
- `agents_interface/`: Custom message/service definitions (standard across platforms)

### flyby-f11 Packages

Packages in `flyby-f11/ros2_ws/src/`:

**Flyby-Specific**:
- `flyby_f11_bringup/`: Mission-specific launch configurations
- `flyby_f11_sensors/`: Flyby F-11 sensor suite drivers
- `flyby_f11_mission/`: Mission-specific logic

**Symlinked from project-drone** (when ready):
- `autonomy_core/` → `../../../project-drone/ros2_ws/src/autonomy_core`
- `behavior_trees/` → `../../../project-drone/ros2_ws/src/behavior_trees`
- `ardupilot_interface/` → `../../../project-drone/ros2_ws/src/ardupilot_interface`
- `perception_pipeline/` → May be symlinked or copied/optimized for Jetson Orin NX

### Package Naming Convention

- Packages prefixed with platform name (e.g., `project_drone_*`, `flyby_f11_*`) are platform-specific
- Generic names (e.g., `autonomy_core`, `ardupilot_interface`) indicate shared components
- Design shared packages to be hardware-agnostic (depend only on standard ROS 2 message types)

## ArduPilot Configuration Management

Each platform project contains its own ArduPilot configuration in `<platform>/config/`:
- `ardupilot_params.param`: Mission Planner/QGroundControl-exported parameter file
- `sensor_calibration.yaml`: Sensor-specific calibration data
- `README.md`: Platform-specific tuning notes and rationale

Load custom parameters when launching SITL by referencing these files in launch configurations.

## Environment Variables

Key variables set by Docker or local environment:
- `ARDUPILOT_HOME`: ArduPilot home coordinates for SITL
- `GAZEBO_MODEL_PATH`: Includes `simulation/models/` for custom assets
- `ROS_DOMAIN_ID`: Isolates ROS 2 DDS traffic

## Testing Strategy

- **SITL Smoke Tests**: Run headless via ArduPilot's SITL framework
- **ROS 2 Unit Tests**: Place `test_*.py` or `test_*.cpp` in package `test/` directories
- **Integration Tests**: Validate full stack behavior in simulation
- **Hardware-in-the-Loop**: Target minimum one HIL session per major feature

## Git Workflow

- **Commit Style**: Imperative mood ("Add offboard controller", "Fix SITL topic sync")
- **Logical Grouping**: Group related changes, reference issues when applicable
- **Pull Requests**: Include context, reproduction steps, test evidence, simulator screenshots for behavioral changes
- **Ignored Files**: Build artifacts, `.deps/`, Docker volumes excluded via `.gitignore`

## Common Pitfalls

- **Podman Volumes**: Containers maintain persistent volumes; use `podman-compose down -v` to reset
- **GPU Passthrough**: If GPU not accessible, verify `nvidia-ctk cdi generate` was run and `/etc/cdi/nvidia.yaml` exists
- **ArduPilot Build**: First build can take 10+ minutes; subsequent builds are incremental
- **ROS 2 Overlay**: Remember to source `ros2_ws/install/setup.bash` inside container
- **Gazebo Models**: Custom models must be in `GAZEBO_MODEL_PATH`; verify with `echo $GAZEBO_MODEL_PATH` inside container
- **DDS Domain**: If nodes can't discover each other, check `ROS_DOMAIN_ID` is consistent
- **Host vs Container**: All development commands run inside containers; don't install ROS 2/Python packages on host

## Development Platforms

### project-drone (Primary Development Platform)
- **Hardware**: Custom-built 7-inch FPV quadcopter with 3D-printed components, T265 visual odometry, D455 depth camera
- **Compute**: NVIDIA Jetson Orin Nano Super (8GB, 67 TOPS)
- **GPU**: 1,024 CUDA cores, 32 Tensor cores (Ampere architecture)
- **CPU**: 6-core ARM CPU
- **Purpose**: Personal autonomy development platform - full simulation-to-flight workflow for testing navigation algorithms
- **Status**: Hardware complete, sensors verified working, active algorithm development
- **Location**: Personal build, available for immediate testing

### flyby-f11 (Collaboration Platform)
- **Hardware**: Flyby Robotics F-11 developer quadcopter (MCTSSA)
- **Compute**: NVIDIA Jetson Orin NX 16GB (50 TOPS)
- **Purpose**: Advanced autonomy testing with mission-intent interpretation, communications-denied operations
- **Status**: Development planning (hardware access via MCTSSA)
- **Strategy**: Shared autonomy components developed on project-drone will be adapted and deployed

## Podman-First Development

All platforms use Podman for fully self-contained, rootless container development.

We use **two tools for different phases**:
- **Development** (dev machine): **Podman Compose** - fast iteration, easy orchestration
- **Deployment** (Jetson): **Quadlet** - systemd integration, auto-start on boot, production-grade

**Benefits**:
- **Complete portability**: All dependencies documented in container specs
- **No host pollution**: No manual ROS 2, Python, or library installation on host
- **GPU passthrough**: NVIDIA GPU access for vision models and simulation
- **Rootless security**: No daemon, containers run as user processes
- **Reproducibility**: Identical environment on any machine with Podman
- **Production-ready**: Systemd-managed services with auto-restart and logging
- **Easy deployment**: Same containers run on dev machine and target hardware

**GPU Passthrough Setup**:
```bash
# Install nvidia-container-toolkit (one-time setup)
sudo apt install nvidia-container-toolkit

# Configure CDI for Podman GPU access
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

# Verify GPU access
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

**Development Usage** (Podman Compose):
```bash
cd <platform>
podman-compose up          # Start all services with GPU passthrough
podman-compose up --build  # Rebuild and start
podman-compose down        # Stop services
podman-compose down -v     # Stop and remove volumes (reset state)

# Interactive development
podman exec -it <platform>-dev bash

# View logs
podman-compose logs -f
```

**Production Usage** (Quadlet on Jetson):
```bash
# One-time setup: install Quadlet files
cp quadlet/*.container ~/.config/containers/systemd/
systemctl --user daemon-reload

# Enable auto-start on boot
systemctl --user enable flyby-f11-ros2.service

# Manage services
systemctl --user start flyby-f11-ros2.service
systemctl --user status flyby-f11-ros2.service
systemctl --user restart flyby-f11-ros2.service

# View logs
journalctl --user -u flyby-f11-ros2.service -f
```

**Container Structure**:
- `Containerfile` (Podman equivalent of Dockerfile): Defines environment
- `podman-compose.yml`: Development orchestration with GPU passthrough
- `quadlet/*.container`: Production systemd service definitions
- All ROS 2, ArduPilot, Gazebo dependencies specified in containers
- Host only needs Podman + NVIDIA drivers

## Platform-Specific Notes

### project-drone
- Jetson Orin Nano Super provides 67 TOPS with 8GB RAM
- Ampere GPU architecture with 1,024 CUDA cores and 32 Tensor cores
- Capable of running generative AI models (vision transformers, LLMs, VLMs)
- T265 provides robust visual odometry indoors
- D455 depth camera for obstacle avoidance
- All sensors verified working; ready for algorithm testing in simulation

### flyby-f11
- Jetson Orin NX 16GB provides 50 TOPS (slightly less than Orin Nano Super but with 2x memory: 16GB vs 8GB)
- Additional memory enables larger models and more concurrent processes
- Can run heavier perception models (larger YOLO, transformer-based detectors)
- Open flight controller and GPU access for custom development
- 3kg payload capacity allows additional sensors/compute
- NDAA-compliant for government/defense applications
- Hardware access via MCTSSA collaboration
- Note: Compute is similar (67 vs 50 TOPS), but flyby has more memory and payload capacity
