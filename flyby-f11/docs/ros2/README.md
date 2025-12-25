# ROS 2 Humble Integration for Flyby-F11

## Overview

This directory contains ROS 2 Humble documentation and integration guides for the Flyby-F11 autonomous drone project. The ROS 2 stack serves as the critical perception-to-reasoning bridge, connecting vision system outputs to the Prolog-based ontological reasoning engine.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Flyby-F11 Autonomy Stack                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐  │
│  │   Vision     │      │   Grounding  │      │   Prolog     │  │
│  │  Pipeline    │─────▶│    Nodes     │─────▶│  Reasoning   │  │
│  │  (YOLO, etc) │      │  (ROS 2)     │      │   Engine     │  │
│  └──────────────┘      └──────────────┘      └──────────────┘  │
│         │                     │                      │          │
│         │                     │                      │          │
│         ▼                     ▼                      ▼          │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              ROS 2 Topics & Services                     │  │
│  │  - /perception/objects                                   │  │
│  │  - /perception/scene                                     │  │
│  │  - /reasoning/query (service)                            │  │
│  │  - /mission/execute (action)                             │  │
│  └──────────────────────────────────────────────────────────┘  │
│         │                                                        │
│         ▼                                                        │
│  ┌──────────────┐                                               │
│  │  PX4 Flight  │                                               │
│  │  Controller  │                                               │
│  └──────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
```

## System Components

### 1. Grounding Nodes
ROS 2 nodes that convert perception outputs to symbolic facts for the Prolog reasoning engine:

- **ObjectGroundingNode**: Converts detected objects (YOLO bounding boxes) to Prolog facts
- **SceneGroundingNode**: Converts scene understanding to spatial relations
- **MissionGroundingNode**: Converts mission state to queryable facts

### 2. Custom Interfaces
Message/service/action definitions for perception data:

- `PerceptionObject.msg`: Object detections with class, confidence, bbox
- `PerceptionScene.msg`: Scene graph with spatial relationships
- `PrologQuery.srv`: Query Prolog knowledge base
- `ExecuteMission.action`: Execute mission with symbolic goals

### 3. PySwip Integration
Python-based Prolog interface for ROS 2 nodes:

- Query Prolog knowledge base from ROS 2 callbacks
- Assert/retract facts dynamically
- Execute reasoning queries in real-time

### 4. Launch Configuration
Multi-node launch files for the complete autonomy stack:

- Perception pipeline nodes
- Grounding nodes
- Reasoning interface
- Mission execution

## Platform Specifications

- **Hardware**: Flyby F-11 quadcopter with Jetson Orin NX 16GB
- **Compute**: 50 TOPS, 16GB RAM
- **ROS Version**: ROS 2 Humble Hawksbill
- **Python Version**: 3.10+
- **Prolog**: SWI-Prolog 8.4+

## Key Features

1. **Edge-Based Reasoning**: All processing runs locally on Jetson Orin NX
2. **Real-Time Grounding**: Perception → symbolic facts at 10-30 Hz
3. **Bi-Directional Interface**: ROS 2 ↔ Prolog communication
4. **Mission-Intent Interpretation**: Natural language → symbolic goals → execution
5. **Communications-Denied Operations**: No cloud dependency required

## Directory Structure

```
/home/finley/Github/DroneProjects/flyby-f11/docs/ros2/
├── tutorials/              # ROS 2 tutorials (publishers, services, actions)
├── concepts/               # Core ROS 2 concepts documentation
├── client_libs/            # rclpy API documentation
├── interfaces/             # Custom message/service/action definitions
├── pyswip/                 # PySwip repository and integration guides
├── README.md               # This file
├── GROUNDING_NODES_GUIDE.md
├── PYSWIP_INTEGRATION.md
├── CUSTOM_MESSAGES.md
└── manifest.txt
```

## Quick Start

### 1. Install ROS 2 Humble
```bash
# On Jetson Orin NX (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 2. Install PySwip
```bash
sudo apt install swi-prolog
pip install pyswip
```

### 3. Build Flyby-F11 Workspace
```bash
cd /home/finley/Github/DroneProjects/flyby-f11/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch Grounding Nodes
```bash
ros2 launch flyby_f11_bringup grounding_nodes.launch.py
```

## Key Documentation

1. **GROUNDING_NODES_GUIDE.md**: How to implement perception → Prolog grounding nodes
2. **PYSWIP_INTEGRATION.md**: How to call Prolog from ROS 2 Python nodes
3. **CUSTOM_MESSAGES.md**: Message definitions for perception data

## ROS 2 Topics

### Published Topics
- `/perception/objects` (PerceptionObject[]): Detected objects from vision pipeline
- `/perception/scene` (PerceptionScene): Scene understanding with spatial relations
- `/mission/status` (MissionStatus): Current mission execution state

### Subscribed Topics
- `/px4/odometry` (Odometry): Vehicle position/velocity
- `/px4/battery` (BatteryState): Battery status for mission planning

### Services
- `/reasoning/query` (PrologQuery): Query Prolog knowledge base
- `/reasoning/assert` (PrologAssert): Add facts to knowledge base
- `/reasoning/retract` (PrologRetract): Remove facts from knowledge base

### Actions
- `/mission/execute` (ExecuteMission): Execute mission with symbolic goals

## Development Workflow

1. **Vision Pipeline**: Develop object detection models (YOLO, etc.)
2. **Grounding Nodes**: Implement ROS 2 nodes to convert detections to Prolog facts
3. **Ontology Development**: Define Prolog rules for reasoning
4. **Mission Planning**: Implement mission execution with symbolic goals
5. **Testing**: Validate perception → reasoning → execution pipeline in simulation
6. **Deployment**: Deploy to Flyby-F11 hardware

## Integration Points

### With Vision System
- Subscribe to `/perception/detections` from YOLO nodes
- Convert detections to symbolic facts (object, class, location)

### With Prolog Reasoning
- Use PySwip to query knowledge base
- Assert facts from perception
- Retrieve reasoning results for mission planning

### With PX4 Flight Controller
- Publish flight commands based on reasoning results
- Subscribe to vehicle state for situational awareness

## Performance Targets

- **Grounding Latency**: < 10ms per perception frame
- **Reasoning Query**: < 50ms per query
- **End-to-End**: < 100ms perception → reasoning → command

## References

- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- PySwip Documentation: https://github.com/yuce/pyswip
- SWI-Prolog Manual: https://www.swi-prolog.org/pldoc/doc_for?object=manual
- Flyby F-11 Specifications: https://flybyrobotics.com/f-11

## License

This documentation is part of the Flyby-F11 project developed by Finley Holt.
