# LLMDrone Project Planning

This directory contains comprehensive planning documentation for the LLMDrone reasoning-enabled autonomy framework.

## Mission Statement

Develop a reasoning-enabled autonomy framework that converts natural-language mission intent into executable flight plans for small unmanned aerial systems (sUAS), enabling autonomous operations in communications-denied environments without requiring external connectivity.

## Project Context

**Sponsor**: Marine Corps Tactical Systems Support Activity (MCTSSA), Digital Solutions Branch
**Purpose**: Enable distributed, resilient sUAS operations supporting Force Design 2030
**Target Environment**: Tactical operations with intermittent or unavailable control links
**Technical Foundation**: Locally hosted LLM with Mixture-of-Experts architecture on NVIDIA Jetson edge compute

## Development Objectives

1. **Natural-Language Mission Parsing**: Translate intent (e.g., "Survey NAI 3 and identify vehicle movement") into structured sub-tasks with dynamic plan updates
2. **Modular Containerized Stack**: ROS 2-based architecture integrating reasoning, perception, and control via MAVSDK bridges
3. **Onboard Sensor Integration**: Vision models for task verification, vehicle detection, anomaly detection, and local replanning
4. **Validation Pipeline**: Prove reasoning reliability, latency, and mission robustness in PX4-SITL + Gazebo before live-flight testing

## Project Deliverables

1. Edge-deployable reasoning module and containerized software stack
2. Simulation and live-flight demonstrations of autonomous mission execution from natural-language tasking
3. Publishable research paper on reasoning-enabled autonomy framework design and evaluation

## Technology Stack

- **Edge Compute**: NVIDIA Jetson Orin or Jetson Thor
- **LLM Architecture**: Mixture-of-Experts (MoE) with local inference
- **Middleware**: ROS 2 Humble for autonomy software communication
- **Flight Control**: MAVSDK for MAVLink telemetry and commands
- **Autopilot**: PX4 firmware
- **Simulation**: Gazebo with PX4 SITL (Software-in-the-Loop)
- **Perception**: Onboard vision models (object detection, segmentation)
- **Development**: Ubuntu 22.04 LTS, Python 3.10+, C++ for real-time components

## Operational Vignette

A reconnaissance team tasks a quadrotor: **"Survey Named Area of Interest (NAI) 3 and identify vehicle movement before returning home."**

The onboard reasoning engine:
1. Parses the natural-language command into structured actions
2. Plans navigation to the designated area
3. Executes search patterns optimized for coverage
4. Runs onboard vision models to detect vehicles
5. Geotags detections with position and timestamp
6. Returns home autonomously

If communications are lost during the mission, the system continues execution in accordance with commander's intent. If environmental changes or obstacles are encountered, the system replans locally to maintain mission continuity and aircraft safety.

## Key Architectural Principles

- **Edge-First**: All reasoning and planning runs locally; no cloud dependency
- **Modular Design**: Clear separation between reasoning, perception, planning, and control
- **Safety-Critical**: Multiple layers of safety constraints and fallback behaviors
- **Transparent**: Open-source frameworks (ROS 2, MAVSDK, PX4, Gazebo) for reproducible development
- **Validated**: Simulation-first development with metrics for latency, reliability, and robustness

## Success Metrics

- **Natural-Language Understanding**: >90% accurate parsing of tactical mission commands
- **Reasoning Latency**: <2 seconds for initial task decomposition, <500ms for replanning
- **Mission Completion**: >85% success rate in simulation scenarios with dynamic conditions
- **Edge Performance**: Real-time inference on Jetson Orin within power budget
- **Safety**: Zero critical failures in simulation; comprehensive fallback coverage

---

## Phase Structure

The project is divided into **five development phases** followed by validation/deployment, using a **bottom-up capability approach**:

### Phase 1 - Foundation (Current)
**Goal**: Get PX4 SITL + Gazebo working with version control
**Duration**: 1-2 weeks
**Deliverables**: Working simulator, setup scripts, clean git workflow

**Key Idea**: Just get a drone flying in simulation without overengineering the repository.

[Detailed Plan →](Phase-1-Foundation.md)

### Phase 2 - Environment & Mission Design
**Goal**: Create operational context before building capabilities
**Duration**: 2-3 weeks
**Deliverables**: Custom drone config, 2-3 Gazebo worlds, mission definitions with success criteria

**Key Idea**: Define **what** we want the drone to do before building **how** it does it. These missions drive Phase 3 requirements.

[Detailed Plan →](Phase-2-Environment-Mission-Design.md)

### Phase 3 - Action Vocabulary
**Goal**: Build the fundamental capabilities from the ground up
**Duration**: 4-6 weeks
**Deliverables**: ROS 2 + MAVSDK bridge, navigation (waypoints, A*), perception (object detection), 8-12 action primitives, manual mission execution

**Key Idea**: Create the **vocabulary** of actions the LLM will use. Build navigation, perception, and atomic actions. Prove it works by flying missions with manual scripts (no LLM yet).

[Detailed Plan →](Phase-3-Action-Vocabulary.md)

### Phase 4 - LLM Integration
**Goal**: Enable natural-language mission planning
**Duration**: 3-4 weeks
**Deliverables**: Local LLM deployment, structured output generation, mission parser, Mission 1 from natural-language command

**Key Idea**: The LLM translates language into the action vocabulary from Phase 3. Use constrained generation (LangChain/guidance) or custom output tokens to ensure valid actions.

[Detailed Plan →](Phase-4-LLM-Integration.md)

### Phase 5 - Dynamic Replanning
**Goal**: Enable adaptive behavior under changing conditions
**Duration**: 3-4 weeks
**Deliverables**: Event detection, mid-flight replanning, obstacle avoidance, curveball testing

**Key Idea**: The LLM becomes a continuous reasoning engine that adapts when things go wrong (obstacles, failures, battery constraints). Test with "curveball" scenarios.

[Detailed Plan →](Phase-5-Dynamic-Replanning.md)

### Phase 6 - Validation & Deployment (TBD)
**Goal**: Comprehensive testing, edge deployment, live-flight preparation
**Duration**: 3-4 weeks
**Deliverables**: Test suite, containerization, Jetson deployment, live-flight demonstrations

**Key Idea**: Validate everything in simulation, package for edge deployment, fly it for real.

[Detailed Plan →](Phase-6-Validation-Deployment.md) _(To be created)_

## Timeline

**Total Estimated Duration**: 18-24 weeks (4.5-6 months) from start to live-flight demonstrations

**Current Phase**: Phase 1 - Foundation

## Key Milestones

- **M1**: PX4 SITL running with custom Gazebo world (End of Phase 1)
- **M2**: ROS 2 nodes controlling drone via MAVSDK (End of Phase 2)
- **M3**: Natural-language command → autonomous flight (End of Phase 3)
- **M4**: Object detection integrated with mission execution (End of Phase 4)
- **M5**: Adaptive replanning under dynamic conditions (End of Phase 5)
- **M6**: Live-flight demonstration with edge deployment (End of Phase 6)

## Phase Dependencies

```
Phase 1 (Foundation)
    ↓
Phase 2 (Integration)
    ↓
Phase 3 (Reasoning) ← Phase 4 (Perception)
    ↓                      ↓
    └──────→ Phase 5 (Planning) ←────┘
                ↓
          Phase 6 (Validation)
```

**Note**: Phases 3 and 4 can partially overlap once Phase 2 is complete, as they work on different parts of the system (reasoning vs. perception).

## Success Criteria by Phase

### Phase 1
- ✓ PX4 SITL launches successfully
- ✓ Gazebo displays drone in custom world
- ✓ Setup scripts work on fresh Ubuntu install

### Phase 2
- ✓ ROS 2 nodes receive telemetry from PX4
- ✓ Offboard control mode enabled via MAVSDK
- ✓ Waypoint navigation works in simulation

### Phase 3
- ✓ Natural-language commands parsed to structured tasks
- ✓ LLM generates valid navigation waypoints
- ✓ Task decomposition matches mission intent

### Phase 4
- ✓ Vision model detects vehicles in Gazebo
- ✓ Detections tagged with GPS coordinates
- ✓ Perception runs in real-time (<100ms latency)

### Phase 5
- ✓ Search patterns generated for area coverage
- ✓ Replanning triggered by obstacles/failures
- ✓ Safety constraints enforced (geofencing, battery)

### Phase 6
- ✓ End-to-end mission succeeds in simulation
- ✓ Containerized stack deployed to Jetson
- ✓ Live-flight demonstration with natural-language tasking

## Risk Management

### High-Risk Areas

1. **LLM Inference Performance**: MoE model may not meet latency requirements on Jetson
   - **Mitigation**: Benchmark multiple model architectures early in Phase 3; have fallback to lighter models

2. **Perception Reliability**: Object detection may have high false-positive rate
   - **Mitigation**: Multi-stage verification, confidence thresholds, sensor fusion

3. **Safety Validation**: Ensuring safe autonomous behavior in all edge cases
   - **Mitigation**: Extensive simulation testing, formal safety constraints, manual override always available

### Medium-Risk Areas

1. **MAVSDK Integration Complexity**: MAVLink protocol nuances may cause issues
   - **Mitigation**: Use MAVSDK examples, leverage upstream community support

2. **Edge Deployment**: Container size and resource constraints on Jetson
   - **Mitigation**: Optimize model quantization, test early on target hardware

3. **Live-Flight Safety**: Risk of drone crash or unsafe behavior
   - **Mitigation**: Progressive testing, manual RC override, comprehensive pre-flight checklists

## Resource Requirements

### Hardware
- Development workstation with NVIDIA GPU (for simulation and vision model training)
- NVIDIA Jetson Orin or Jetson Thor (target deployment platform)
- Quadrotor drone compatible with PX4 (Phase 6 live testing)
- RC transmitter and receiver for manual override
- Battery charging equipment and spares

### Software
- Ubuntu 22.04 LTS
- ROS 2 Humble
- PX4 v1.14+ (SITL and firmware)
- Gazebo Classic or Gazebo (Ignition)
- Python 3.10+
- MAVSDK (C++ or Python bindings)
- LLM inference framework (TensorRT-LLM, vLLM, or llama.cpp)
- Git LFS for large asset management

### Personnel
- Primary developer (2ndLt Finley Holt)
- MCTSSA Digital Solutions Branch supervision (Shaun Monera)
- Optional: Subject matter experts for LLM optimization, flight control validation

## Documentation Requirements

Each phase should produce:
- **Implementation Notes**: Key technical decisions, architecture diagrams, code organization
- **Test Results**: Performance metrics, failure mode analysis, benchmark data
- **User Guides**: How to build, run, configure, and debug the system
- **Research Findings**: Analysis and insights for final research paper

## Current Status

**Active Phase**: Phase 1 - Foundation
**Start Date**: [To be filled]
**Expected Completion**: [To be filled based on 2-3 week estimate]

## Next Steps

1. Review detailed Phase 1 plan: [Phase-1-Foundation.md](Phase-1-Foundation.md)
2. Set up development environment following Phase 1 tasks
3. Create repository structure and setup scripts
4. Verify PX4 SITL + Gazebo working end-to-end
5. Document progress, decisions, and blockers in Phase 1 plan
6. Upon Phase 1 completion, proceed to Phase 2

## Directory Contents

- **[Phase-1-Foundation.md](Phase-1-Foundation.md)**: Detailed plan for repository setup and PX4 SITL integration
- **[Phase-2-Integration.md](Phase-2-Integration.md)**: ROS 2 workspace and MAVSDK bridge implementation
- **[Phase-3-Reasoning.md](Phase-3-Reasoning.md)**: LLM reasoning engine and natural-language parsing
- **[Phase-4-Perception.md](Phase-4-Perception.md)**: Vision pipeline and object detection
- **[Phase-5-Planning.md](Phase-5-Planning.md)**: Mission planning and adaptive replanning
- **[Phase-6-Validation.md](Phase-6-Validation.md)**: Comprehensive testing, deployment, and live-flight demonstrations

---

**Last Updated**: 2025-11-10
**Project Lead**: 2ndLt Finley Holt
**Organization**: MCTSSA Digital Solutions Branch
