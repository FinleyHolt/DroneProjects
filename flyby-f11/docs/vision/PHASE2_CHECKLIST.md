# Pre-RL Scaffolding Checklist

**Platform**: Flyby F-11 (NVIDIA Jetson Orin NX 16GB)
**Purpose**: Verify all ROS 2 packages and simulation infrastructure exist before starting RL training
**Last Updated**: 2025-12-27

---

## Overview

This checklist tracks the scaffolding required before Phase 7 (Mission Planner RL) can begin. All items must be complete for RL training to have the necessary environment, sensors, and interfaces.

**Hardware Reference**: See [SYSTEM_CONSTRAINTS.qmd](../../SYSTEM_CONSTRAINTS.qmd) for F-11 specs.

---

## Section 1: F-11 ISR Sensor Payloads

The F-11 uses mission-configurable ISR payloads (one at a time):

| Payload | Sensors | Primary Use | Simulation Model |
|---------|---------|-------------|------------------|
| **Gremsy VIO** | 640p FLIR thermal + 4K RGB + 20x zoom | ISR surveillance, day/night | `f11_isr_camera` |
| **RESEPI LiDAR** | Ouster OS1-64 3D point cloud | Terrain mapping, obstacles | `f11_lidar` |
| **NextVision Raptor** | EO-IR gimbal, 1280x720p thermal | Recon, target tracking | `f11_multispectral` |

### 1.1 Gazebo Simulation Models

- [x] **f11_isr_camera model exists**
  - Location: `simulation/models/f11_isr_camera/model.sdf`
  - Includes: 3-axis gimbal, Sony A7R camera (1920x1080 @ 30Hz)
  - Verified: Gimbal yaw/pitch/roll joints with PID control

- [x] **f11_lidar model exists**
  - Location: `simulation/models/f11_lidar/model.sdf`
  - Includes: Ouster OS1-64 (64 channels, 360° FOV)

- [x] **f11_multispectral model exists**
  - Location: `simulation/models/f11_multispectral/model.sdf`
  - Includes: MicaSense RedEdge-P (6 spectral bands)

- [ ] **Gimbal control verified in Gazebo**
  - Test script: `simulation/test_gimbal_control.py`
  - ROS topics: `/f11_isr/gimbal/{yaw,pitch,roll}`
  - Camera image published to `/f11_isr/camera/image_raw`

---

## Section 2: ROS 2 Package Structure

### 2.1 Core Packages (Must Exist)

| Package | Purpose | Status |
|---------|---------|--------|
| `vampire_bridge` | Vampire prover ROS 2 integration | [x] Exists |
| `mission_orchestrator` | Mission state machine | [x] Exists |
| `flyby_f11_bringup` | Launch files for F-11 | [ ] **NOT CREATED** |
| `flyby_f11_sensors` | ISR sensor drivers/interfaces | [ ] **NOT CREATED** |
| `perception_grounding` | Vision-to-TPTP grounding | [ ] **NOT CREATED** |
| `ontology_rl` | RL agents and Gym environments | [ ] **NOT CREATED** |

### 2.2 flyby_f11_bringup Package

- [ ] **Package created**
  ```bash
  ros2 pkg create flyby_f11_bringup --build-type ament_python
  ```

- [ ] **Launch files created**
  - [ ] `launch/simulation.launch.py` - Gazebo + ArduPilot SITL
  - [ ] `launch/f11_isr.launch.py` - ISR camera payload
  - [ ] `launch/f11_lidar.launch.py` - LiDAR payload
  - [ ] `launch/perception.launch.py` - All perception nodes
  - [ ] `launch/full_stack.launch.py` - Complete autonomy stack

### 2.3 flyby_f11_sensors Package

- [ ] **Package created**
  ```bash
  ros2 pkg create flyby_f11_sensors --build-type ament_python
  ```

- [ ] **Sensor interface nodes**
  - [ ] `isr_camera_node.py` - Gremsy VIO camera interface
  - [ ] `gimbal_controller_node.py` - 3-axis gimbal control
  - [ ] `lidar_interface_node.py` - Ouster LiDAR interface
  - [ ] `thermal_processor_node.py` - FLIR thermal processing

- [ ] **Message definitions**
  - [ ] `msg/GimbalCommand.msg` - Gimbal angle commands
  - [ ] `msg/GimbalState.msg` - Current gimbal angles
  - [ ] `msg/ThermalImage.msg` - Thermal camera output

### 2.4 perception_grounding Package

- [ ] **Package created** (as specified in Phase 5)
  ```bash
  ros2 pkg create perception_grounding --build-type ament_python
  ```

- [ ] **Core nodes implemented**
  - [ ] `object_grounding_node.py` - Detection to TPTP facts
  - [ ] `spatial_relation_node.py` - Compute spatial relations
  - [ ] `event_detection_node.py` - Zone entry/exit events
  - [ ] `tptp_fact_builder.py` - TPTP format generation

- [ ] **Integration with vampire_bridge**
  - [ ] Facts published to `/perception/tptp_facts`
  - [ ] vampire_bridge subscribes and processes

### 2.5 ontology_rl Package

- [ ] **Package created**
  ```bash
  ros2 pkg create ontology_rl --build-type ament_python
  ```

- [ ] **Directory structure**
  ```
  ontology_rl/
  ├── envs/
  │   ├── __init__.py
  │   ├── base_env.py
  │   └── mission_planner_env.py
  ├── agents/
  │   ├── __init__.py
  │   └── mission_planner_sac.py
  ├── training/
  │   ├── __init__.py
  │   └── train_mission_planner.py
  └── nodes/
      ├── __init__.py
      └── mission_planner_node.py
  ```

- [ ] **Dependencies installed**
  - [ ] `gymnasium` - RL environment API
  - [ ] `stable-baselines3` - RL algorithms
  - [ ] `torch` - Neural network backend

---

## Section 3: Simulation Environment

### 3.1 Gazebo World

- [ ] **Training world created**
  - Location: `simulation/worlds/training_arena.world`
  - Features: Open area, obstacles, NFZ regions, landing zones

- [ ] **Test scenarios defined**
  - [ ] Simple waypoint navigation
  - [ ] Obstacle avoidance
  - [ ] NFZ boundary testing
  - [ ] Battery-constrained mission

### 3.2 ArduPilot SITL Integration

- [ ] **SITL configuration**
  - [ ] ArduPilot parameters for F-11 (`config/ardupilot_f11.param`)
  - [ ] MAVROS launch included in simulation

- [ ] **Containerized simulation**
  - [ ] `Containerfile.sitl` builds and runs
  - [ ] GPU passthrough for Gazebo rendering
  - [ ] ROS 2 topics accessible from host

### 3.3 Simulation Verification

- [ ] **End-to-end test passes**
  ```bash
  # Launch simulation
  ros2 launch flyby_f11_bringup simulation.launch.py

  # Verify topics
  ros2 topic list | grep -E "(camera|gimbal|mavros)"

  # Test gimbal control
  ros2 topic pub /f11_isr/gimbal/pitch std_msgs/Float64 "data: 0.5"
  ```

---

## Section 4: Vampire Integration Verification

### 4.1 vampire_bridge Functionality

- [x] **vampire_bridge package exists**
  - Location: `ros2_ws/src/vampire_bridge/`

- [ ] **Core functionality verified**
  - [ ] `vampire_node.py` starts without errors
  - [ ] Queries processed within 50ms latency target
  - [ ] Results published to `/vampire/results`

- [ ] **Perception grounding integration**
  - [ ] TPTP facts from perception accepted
  - [ ] Safety queries trigger on perception events

### 4.2 Ontology Axioms

- [x] **ISR extensions exist**
  - Location: `ontology/planning_mode/isr_extensions.kif`
  - Includes: CommsStatus, ThreatZone, DynamicNoFlyZone, gimbal limits

- [x] **Canonical test scenarios pass**
  - [x] canonical_1_comms_denied.tptp
  - [x] canonical_2_dynamic_nfz.tptp
  - [x] canonical_3_multi_objective.tptp

---

## Section 5: RL Training Prerequisites

### 5.1 Gymnasium Environment

- [ ] **MissionPlannerEnv implemented**
  - [ ] State space defined (mission progress, resources, environment)
  - [ ] Action space defined (waypoint selection, mission adaptation)
  - [ ] Reward function implemented (completion + efficiency + safety)
  - [ ] Ontology constraint filtering works

- [ ] **Environment registration**
  ```python
  import gymnasium as gym
  gym.register(
      id='FlybyMissionPlanner-v0',
      entry_point='ontology_rl.envs:MissionPlannerEnv'
  )
  ```

### 5.2 Training Infrastructure

- [ ] **Training script created**
  - Location: `ontology_rl/training/train_mission_planner.py`
  - Uses Stable-Baselines3 SAC
  - Checkpointing and logging

- [ ] **Simulation-training loop verified**
  - [ ] Gym env connects to ROS 2 simulation
  - [ ] Actions sent to simulation
  - [ ] Observations received from simulation
  - [ ] Episode reset works

---

## Section 6: Documentation

### 6.1 Architecture Documentation

- [ ] **Perception pipeline documented**
  - ISR sensor → ROS 2 → TPTP facts → Vampire
  - Latency requirements per tier

- [ ] **RL integration documented**
  - MDP formulation
  - State/action spaces
  - Reward shaping rationale

### 6.2 Quick Start Guides

- [ ] **Simulation quick start**
  - How to launch simulation
  - How to verify sensors working
  - How to test gimbal control

- [ ] **Training quick start**
  - How to start RL training
  - How to monitor progress
  - How to deploy trained policy

---

## Completion Checklist

Before starting Phase 7 (Mission Planner RL), verify:

- [ ] All ROS 2 packages created and build successfully
- [ ] Simulation launches with F-11 model and sensors
- [ ] Gimbal control works in simulation
- [ ] vampire_bridge processes TPTP facts correctly
- [ ] Gymnasium environment connects to simulation
- [ ] At least one training episode completes without error

---

## Notes

### Key Differences from Original Checklist

The original checklist referenced RealSense T265/D455 cameras which are used by `project-drone`, not `flyby-f11`. The F-11 uses mission-specific ISR payloads:

| Original (Wrong) | Actual F-11 Hardware |
|------------------|---------------------|
| RealSense T265 | Gremsy VIO gimbal + camera |
| RealSense D455 | RESEPI LiDAR or NextVision thermal |
| Visual odometry | ArduPilot EKF + GPS/INS |
| Depth-based obstacles | LiDAR point cloud or thermal |

### Hardware Access Timeline

Physical F-11 hardware is accessed via MCTSSA collaboration. Until then:
- All development in Gazebo simulation
- Train RL agents in simulation
- Deploy to hardware when available

---

**Maintainer**: Finley Holt
**Last Updated**: 2025-12-27
