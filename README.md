# Drone Projects

Personal drone autonomy development projects targeting various hardware platforms. Each folder represents a specific drone build with its own compute, sensors, and mission objectives.

## Portfolio Overview

This repository showcases autonomous drone development work focused on edge AI, computer vision, and embedded systems integration. All projects run ROS 2 for autonomy software, MAVSDK for flight control, and PX4 autopilot.

**Developer**: Finley Holt
**Focus Areas**: Edge AI for drones, Visual SLAM, Autonomous navigation, Embedded systems

## Hardware Platforms

### project-drone/
**Hardware**: Custom-built 7-inch FPV quadcopter with 3D-printed components
**Sensors**:
- Intel RealSense T265 (Visual Odometry)
- Intel RealSense D455 (Depth Camera)
- FPV system for manual flight testing

**Compute**: NVIDIA Jetson Orin Nano Super (8GB, 67 TOPS)
**Purpose**: Personal autonomy development platform - built from ground up for testing autonomous navigation algorithms
**Status**: Hardware complete, sensors verified, active algorithm development

**Key Features**:
- Full simulation-to-flight workflow (PX4 SITL + Gazebo + ROS 2 + MAVSDK)
- Indoor navigation and mapping
- Visual odometry-based localization (T265)
- Obstacle avoidance using depth sensing (D455)
- BehaviorTree.CPP for mission logic
- Docker-based development environment
- Capable of running generative AI models (vision transformers, LLMs, VLMs)

See [project-drone/README.md](project-drone/README.md) for setup and usage.

---

### flyby-f11/
**Hardware**: [Flyby Robotics F-11](https://www.flybyrobotics.com/) developer platform
**Sensors**: TBD based on mission requirements
**Compute**: NVIDIA Jetson Orin NX 16GB (50 TOPS)
**Purpose**: Advanced autonomy platform with high payload capacity and processing power
**Status**: Development planning (hardware access via MCTSSA)

**Platform Specifications**:
- 3 kg (6.6 lbs) payload capacity with 12 mounting hardpoints
- Up to 50 minutes flight time with hot-swappable batteries
- NVIDIA Jetson Orin NX 16GB onboard (50 trillion ops/sec)
- Fully open flight controller and GPU for custom development
- NDAA-compliant supply chain

**Planned Capabilities**:
- Mission-intent interpretation and autonomous execution
- Advanced perception and object detection
- Real-time path planning and obstacle avoidance
- Multi-sensor fusion
- Communications-denied operations

See [flyby-f11/README.md](flyby-f11/README.md) for mission details and development roadmap.

---

## Technology Stack

**Flight Control**:
- PX4 Autopilot (firmware)
- MAVSDK (MAVLink interface)
- Gazebo (simulation environment)

**Autonomy Software**:
- ROS 2 Humble (middleware)
- BehaviorTree.CPP (mission logic)
- RTAB-Map / ORB-SLAM3 (Visual SLAM)

**Computer Vision**:
- Intel RealSense SDK
- OpenCV
- YOLOv8 / RT-DETR (object detection)

**Development**:
- Ubuntu 22.04 LTS
- Docker (containerized deployments)
- Python 3.10+, C++17

## Repository Structure

```
DroneProjects/
├── project-drone/          # Jetson Orin Nano Super development platform
│   ├── ros2_ws/           # ROS 2 workspace
│   ├── docker/            # Container configurations
│   ├── simulation/        # Gazebo worlds and models
│   ├── config/            # PX4 params and sensor calibration
│   └── README.md
│
├── flyby-f11/             # Flyby F-11 production platform
│   ├── ros2_ws/           # ROS 2 workspace (will link shared packages)
│   ├── docker/            # Deployment containers
│   ├── config/            # Mission-specific configurations
│   └── README.md
│
├── AGENTS.md              # Development guidelines
└── CLAUDE.md              # AI assistant guidance
```

## Quick Start

Each drone project is self-contained and uses Docker for consistent development environments:

```bash
# Clone repository
git clone https://github.com/finleyholt/DroneProjects.git
cd DroneProjects

# Navigate to specific platform
cd project-drone  # or flyby-f11

# Launch with Docker (recommended)
docker compose up

# OR build locally
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch <platform>_bringup simulation.launch.py
```

See individual platform README files for detailed setup instructions.

## Development Approach

**Modular Design**: Core autonomy packages are designed to be hardware-agnostic and can be shared between platforms via symlinks or git submodules.

**Simulation First**: All features are validated in Gazebo simulation before hardware deployment.

**Containerized Workflows**: Docker ensures consistent development and deployment environments across different host systems.

**Edge-First**: All processing runs locally on embedded compute - no cloud dependency required.

## Documentation

- **[AGENTS.md](AGENTS.md)**: Developer guidelines, coding standards, and best practices
- **[CLAUDE.md](CLAUDE.md)**: Project structure and guidance for Claude Code AI assistant
- **Platform READMEs**: Specific setup and usage for each drone build

## Current Status

**Active Development**: project-drone (7-inch FPV quadcopter with Jetson Orin Nano Super 8GB)
- Custom-built airframe with 3D-printed components
- Full simulation-to-flight workflow (PX4 SITL, Gazebo, ROS 2, MAVSDK, Docker)
- Sensors verified working, ready for algorithm development

**Planning**: flyby-f11 (development via MCTSSA collaboration)

---

**Last Updated**: 2025-12-08
