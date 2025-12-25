# Flyby-F11 Implementation Documentation

This directory contains all technical documentation needed for implementing the ontology-constrained RL architecture.

## 📋 Complete Documentation Index

**👉 See [DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md) for comprehensive navigation guide**

The complete index provides:
- Full file listing with descriptions
- Phase-based implementation guides
- Two-phase architecture walkthrough
- Getting started guides for each phase
- Documentation status and verification

## Documentation Organization

### Phase 1: Ontology Foundation (Weeks 1-6)
**Location**: [`ontology/`](ontology/)
- SUMO ontology files and specifications
- SWI-Prolog manual and API references
- Vampire, Clingo, E-Prover theorem prover documentation
- SUO-KIF format specifications
- TPTP format documentation

### Phase 2: Perception-to-Reasoning Bridge (Weeks 7-10)
**Location**: [`vision/`](vision/), [`ros2/`](ros2/)
- YOLO11 Jetson deployment guides
- TensorRT optimization documentation
- Intel RealSense ROS 2 wrappers
- Semantic segmentation model documentation
- VLM deployment guides
- ROS 2 Humble tutorials and API references
- PySwip Prolog integration documentation

### Phase 3: Flight Control Integration
**Location**: [`flight_control/`](flight_control/)
- **ArduPilot Copter** documentation (primary autopilot)
- ArduPilot SITL simulation setup
- MAVSDK with ArduPilot GUIDED mode
- MAVLink protocol reference
- Gazebo with ArduPilot integration

### Phase 4: Reinforcement Learning (Weeks 11-14)
**Location**: [`rl/`](rl/)
- Gymnasium custom environment creation
- Stable-Baselines3 algorithm documentation (PPO, SAC, TD3)
- BehaviorTree.CPP with ROS 2 integration
- Hierarchical multi-agent RL research papers
- Domain randomization and sim-to-real transfer

### Supporting Documentation
**Location**: [`standards/`](standards/), [`tools/`](tools/)
- IEEE 1872.2 AuR ontology standard
- TPTP theorem prover format
- JetPack 6.1 and Jetson containers
- Quarto documentation rendering

## Download Status

Documentation is being downloaded by automated agents. Check individual directory `manifest.txt` files for details.

**Last Updated**: 2024-12-25

## Quick Reference by Component

### Two-Phase Architecture Implementation

#### Planning Mode Tools
- **SUMO**: `ontology/sumo/`
- **Vampire**: `ontology/vampire/`
- **Clingo**: `ontology/clingo/`
- **E-Prover**: `ontology/eprover/`

#### Execution Mode Tools
- **SWI-Prolog**: `ontology/swi-prolog-docs/`
- **PySwip Integration**: `ros2/pyswip/`

### Perception-to-Reasoning Bridge

#### Vision Models
- **YOLO11**: `vision/ultralytics/`, `vision/yolo-jetson-guides/`
- **TensorRT**: `vision/tensorrt-optimization/`
- **Segmentation**: `vision/jetson-inference/`, `vision/segmentation-papers/`
- **VLMs**: `vision/vlm-jetson/`

#### Sensor Integration
- **RealSense SDK**: `vision/realsense-sdk/`
- **T265 ROS 2**: `vision/realsense_t265_shelfino/`
- **D455 ROS 2**: `vision/realsense-ros/`

#### Grounding Nodes
- **ROS 2 Tutorials**: `ros2/tutorials/`
- **Custom Messages**: `ros2/custom-interfaces/`
- **Action Servers**: `ros2/action-servers/`

### Flight Control

#### ArduPilot Stack
- **Copter Docs**: `flight_control/ardupilot-copter/`
- **SITL Setup**: `flight_control/ardupilot-sitl/`
- **Gazebo Integration**: `flight_control/ardupilot-gazebo/`
- **GUIDED Mode**: `flight_control/ardupilot-copter/` (flight modes)

#### MAVLink Interface
- **MAVSDK**: `flight_control/MAVSDK/`
- **ROS 2 Bridge**: `flight_control/drone_mavsdk/`

### Reinforcement Learning

#### Training Frameworks
- **Gymnasium**: `rl/Gymnasium/`
- **Stable-Baselines3**: `rl/stable-baselines3/`
- **Custom Environments**: `rl/Gymnasium/` (tutorials)

#### Mission Execution
- **BehaviorTree.CPP**: `rl/BehaviorTree.CPP/`
- **BT ROS 2**: `rl/BehaviorTree.ROS2/`

#### Research Papers
- **Hierarchical MARL**: `rl/papers/hierarchical-marl/`
- **Domain Randomization**: `rl/papers/sim2real/`
- **Multi-Objective RL**: `rl/papers/morl/`

## Implementation Workflow

### Starting Phase 1 (Ontology Foundation)
1. Read: `ontology/sumo/Merge.kif` - SUMO upper ontology
2. Read: `ontology/vampire/README.md` - Vampire usage
3. Read: `ontology/clingo/guide.pdf` - Clingo/ASP programming
4. Read: `ontology/swi-prolog-docs/manual.html` - Prolog reference

### Starting Phase 2 (Perception Bridge)
1. Read: `vision/yolo-jetson-guides/jetson-deployment.html` - YOLO11 on Jetson
2. Read: `vision/realsense-ros/README.md` - RealSense ROS 2 setup
3. Read: `ros2/tutorials/custom-interfaces.html` - Message definitions
4. Read: `ros2/pyswip/README.md` - Prolog integration

### Starting Phase 3 (Flight Control)
1. Read: `flight_control/ardupilot-copter/flight-modes.html` - GUIDED mode
2. Read: `flight_control/MAVSDK/offboard-guide.html` - Velocity control
3. Read: `flight_control/ardupilot-sitl/setup.html` - Simulation

### Starting Phase 4 (RL Training)
1. Read: `rl/Gymnasium/custom-environments.html` - Environment creation
2. Read: `rl/stable-baselines3/algorithms/` - PPO, SAC, TD3
3. Read: `rl/BehaviorTree.CPP/ros2-integration.html` - BT integration

## Usage Notes

- All GitHub repositories are shallow clones (`--depth 1`) to save space
- HTML documentation mirrored with `wget --convert-links` for offline use
- PDFs saved directly where available
- Each subdirectory has `manifest.txt` listing downloaded content

## Maintenance

When updating documentation:
1. Note version/date in manifest.txt
2. Keep old versions if API changes significantly
3. Update this README with new quick reference paths

---

**For Implementation Claude**: This documentation set provides complete offline reference for all tools and frameworks in the flyby-f11 architecture. Start with the Quick Reference section above to find relevant docs for your current implementation phase.
