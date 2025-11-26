# LLMDrone

Reasoning-enabled autonomy framework for small unmanned aerial systems (sUAS) enabling natural-language mission tasking and autonomous execution in communications-denied environments.

## Overview

LLMDrone integrates locally hosted Large Language Models (LLM) with drone autonomy software to enable tactical operators to task drones using natural language. The system runs entirely on edge compute (NVIDIA Jetson) without requiring external connectivity, making it suitable for contested or communications-denied operational environments.

**Example Mission**: *"Survey Named Area of Interest (NAI) 3 and identify vehicle movement before returning home."*

The onboard reasoning engine parses this command, plans the mission, executes autonomous flight, detects vehicles using onboard vision, and returns home—all without human intervention or network connectivity.

## Project Context

- **Organization**: Marine Corps Tactical Systems Support Activity (MCTSSA), Digital Solutions Branch
- **Purpose**: Support Force Design 2030 with distributed, resilient sUAS operations
- **Lead**: 2ndLt Finley Holt
- **Supervisor**: Shaun Monera, MCTSSA Digital Solutions Branch

## Key Features

- **Natural-Language Tasking**: Convert tactical mission commands to executable flight plans
- **Edge Autonomy**: All reasoning and planning runs locally on NVIDIA Jetson (no cloud dependency)
- **Adaptive Replanning**: Dynamic mission adjustment based on obstacles, failures, or perception feedback
- **Onboard Perception**: Computer vision for object detection, tracking, and geospatial tagging
- **Safety-Critical Design**: Multi-layer constraints (geofencing, battery management, collision avoidance)

## Technology Stack

- **Edge Compute**: NVIDIA Jetson Orin or Thor
- **LLM**: Mixture-of-Experts architecture for efficient on-device inference
- **Middleware**: ROS 2 Humble
- **Flight Control**: MAVSDK + PX4 autopilot
- **Simulation**: Gazebo + PX4 SITL
- **Perception**: YOLOv8/RT-DETR for object detection
- **Development**: Ubuntu 22.04 LTS, Python 3.10+, C++

## Project Status

**Current Phase**: Phase 1 - Foundation (Repository setup and PX4 SITL integration)

See [Planning/README.md](Planning/README.md) for comprehensive project plan, timeline, and detailed phase breakdowns.

## Documentation

- **[Planning/README.md](Planning/README.md)**: Comprehensive project plan with 6-phase roadmap (18-24 week timeline)
- **[CLAUDE.md](CLAUDE.md)**: Guidance for Claude Code AI assistant when working in this repository
- **[AGENTS.md](AGENTS.md)**: Developer guidelines, coding standards, and contribution workflow
- **[Project_scope.md](Project_scope.md)**: Original project proposal and research objectives

## Repository Structure

LLMDrone is organized into **self-contained platform projects** with shared autonomy components:

- **[test-drone/](test-drone/)** - Primary development platform (custom quadcopter, 8GB GPU)
  - Hardware: T265 visual odometry + D455 depth camera
  - Purpose: Experimentation, sensor testing, behavior tree development
  - Status: Active development

- **[flyby/](flyby/)** - Production flyby mission (16GB GPU, different sensors)
  - Purpose: Autonomous flyby with obstacle avoidance
  - Status: Hardware not yet accessible (several months out)
  - Strategy: Will share autonomy components developed on test-drone

- **Shared Resources**: `llm/`, `docs/`, `.deps/` (PX4, Gazebo)

See [CLAUDE.md](CLAUDE.md) for detailed architecture and package organization.

## Quick Start

### Prerequisites
- Ubuntu 22.04 or 24.04 LTS
- 16+ GB RAM
- 50+ GB free disk space
- NVIDIA GPU with driver >=525.x (recommended but not required for SITL)
- Internet connection for initial setup

### Initial Setup (test-drone)

```bash
# Clone repository
git clone https://github.com/[org]/LLMDrone.git
cd LLMDrone

# Install PX4 and dependencies
bash setup/clone_px4.sh
bash setup/bootstrap.sh

# Navigate to test-drone platform
cd test-drone

# Build workspace
cd ros2_ws
colcon build --symlink-install

# Launch simulation (or use Docker - see test-drone/README.md)
source install/setup.bash
ros2 launch test_drone_bringup simulation.launch.py
```

For detailed test-drone setup and usage, see [test-drone/README.md](test-drone/README.md).

### Makefile Commands (Legacy)

Some top-level Makefile commands are still available:
- `make check` - Validate system requirements
- `make setup` - Install PX4 dependencies
- `make sim` - Launch PX4 SITL + Gazebo (standalone)

### Troubleshooting

**Gazebo doesn't open or crashes:**
- Ensure NVIDIA drivers are installed: `nvidia-smi`
- Try headless mode: `make sim-headless`
- Check GPU requirements: `make check`

**PX4 fails to build:**
- Ensure all dependencies installed: `make bootstrap`
- Check disk space: `df -h`
- Clean and rebuild: `make clean && make sim`

**Setup scripts fail:**
- Check internet connectivity and proxy settings
- Re-run with clean state: `make clean-all && make setup`

**Simulation is slow or laggy:**
- Close unnecessary applications to free RAM
- Reduce Gazebo graphics quality in GUI settings
- Use headless mode: `make sim-headless`

For more details, see [CLAUDE.md](CLAUDE.md) development guidelines.

## Development Phases

1. **Phase 1 - Foundation** (2-3 weeks): PX4 SITL, repository structure, automation scripts
2. **Phase 2 - Integration** (2-3 weeks): ROS 2 workspace, MAVSDK bridge, offboard control
3. **Phase 3 - Reasoning** (4-5 weeks): LLM integration, natural-language parsing
4. **Phase 4 - Perception** (3-4 weeks): Vision pipeline, object detection, sensor fusion
5. **Phase 5 - Planning** (4-5 weeks): Mission planning, adaptive replanning, safety constraints
6. **Phase 6 - Validation** (3-4 weeks): Simulation testing, edge deployment, live-flight demos

See detailed phase plans in [Planning/](Planning/) directory.

## Project Deliverables

1. Edge-deployable reasoning module and containerized software stack
2. Simulation and live-flight demonstrations of natural-language autonomous missions
3. Publishable research paper on reasoning-enabled autonomy framework

## Contributing

This is a research project under active development. For contribution guidelines, see [AGENTS.md](AGENTS.md).

## License

[To be determined]

## Contact

**Project Lead**: 2ndLt Finley Holt
**Organization**: MCTSSA Digital Solutions Branch

---

**Status**: Phase 1 in progress | **Last Updated**: 2025-11-12
