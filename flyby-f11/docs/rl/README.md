# Hierarchical Reinforcement Learning Architecture for Flyby F-11

## Overview

This documentation directory contains comprehensive resources for implementing a hierarchical multi-agent reinforcement learning system for autonomous UAV navigation on the Flyby F-11 platform.

## System Architecture

The flyby-f11 autonomy stack uses a three-tier hierarchical RL architecture:

### 1. Mission Planner (High-Level Agent)
- **Temporal Horizon**: Long-term (mission-level decisions)
- **Observation Space**: Mission context, strategic goals, environmental conditions
- **Action Space**: Mission phase transitions, high-level waypoint selection
- **Algorithm**: PPO (on-policy, suited for discrete/continuous mixed spaces)
- **Training Frequency**: Episodes (complete missions)

### 2. Behavior Selector (Mid-Level Agent)
- **Temporal Horizon**: Medium-term (behavior-level decisions)
- **Observation Space**: Current mission phase, sensor data, vehicle state
- **Action Space**: Behavior primitives (follow waypoint, avoid obstacle, hover, search pattern)
- **Algorithm**: SAC (off-policy, continuous exploration)
- **Training Frequency**: Sub-episodes (behavior sequences)

### 3. Trajectory Optimizer (Low-Level Agent)
- **Temporal Horizon**: Short-term (control-level decisions)
- **Observation Space**: Vehicle state, immediate sensor readings, desired behavior
- **Action Space**: Velocity commands, position setpoints
- **Algorithm**: TD3 (off-policy, continuous control with stability)
- **Training Frequency**: Steps (high-frequency control)

## Key Features

### Ontology-Constrained Action Spaces
The action spaces are constrained by the UAV ontology defined in `/home/finley/Github/DroneProjects/flyby-f11/ONTOLOGY_FOUNDATION.qmd`:
- Actions must respect physical constraints (velocity limits, acceleration bounds)
- State transitions must be valid within the ontological model
- Behaviors must compose into mission-legal sequences

### Experience Sharing
- **Shared Replay Buffer**: All three agents contribute to a common experience pool
- **Policy Distillation**: Transfer learning from simulation to real-world deployment
- **Meta-Learning**: Cross-mission knowledge transfer for faster adaptation

### BehaviorTree.CPP Integration
Trained policies are executed through BehaviorTree.CPP nodes:
- Each behavior primitive is a custom BT action node
- BT provides fallback/recovery logic around learned policies
- ROS 2 integration via BehaviorTree.ROS2 for sensor/actuator access

## Training Pipeline

### Development Environment
- **Simulation**: ArduPilot SITL with Gazebo (custom UAV models)
- **Training Cluster**: NPS HPC resources for parallel environment sampling
- **Deployment Target**: NVIDIA Jetson Orin NX 16GB (50 TOPS)

### Training Workflow
1. **Environment Development**: Custom Gymnasium environments for UAV navigation
2. **Parallel Training**: Vectorized environments on NPS cluster
3. **Policy Export**: Convert trained models to ONNX/TorchScript
4. **BT Integration**: Wrap policies in BehaviorTree.CPP action nodes
5. **SITL Validation**: Full mission testing in simulation
6. **Hardware Deployment**: Deploy to Jetson Orin NX on flyby-f11

## Directory Structure

```
rl/
├── README.md                          # This file
├── CUSTOM_ENVIRONMENTS.md             # UAV Gymnasium environment design
├── TRAINING_GUIDE.md                  # Training workflow and hyperparameters
├── BEHAVIOR_TREE_INTEGRATION.md       # BT.CPP + ROS 2 integration
├── EXPERIENCE_SHARING.md              # Shared replay and meta-learning
├── gymnasium/
│   ├── repo/                          # Gymnasium source code
│   ├── custom_environments.md         # Custom env documentation
│   └── vectorized_environments.md     # Parallel training docs
├── stable-baselines3/
│   ├── repo/                          # SB3 source code
│   ├── ppo_reference.md               # PPO algorithm details
│   ├── sac_reference.md               # SAC algorithm details
│   ├── td3_reference.md               # TD3 algorithm details
│   └── custom_env_integration.md      # SB3 environment integration
├── behavior-trees/
│   ├── BehaviorTree.CPP/              # BT.CPP source code
│   ├── BehaviorTree.ROS2/             # ROS 2 integration source
│   └── overview.md                    # BT.CPP overview
├── papers/
│   └── (research papers on hierarchical RL, UAV navigation)
└── tutorials/
    └── (step-by-step implementation tutorials)
```

## Quick Start

### 1. Review Core Concepts
Start with these documents in order:
1. `gymnasium/custom_environments.md` - Learn Gymnasium environment structure
2. `stable-baselines3/ppo_reference.md` - Understand PPO for mission planning
3. `CUSTOM_ENVIRONMENTS.md` - UAV-specific environment design

### 2. Implement Training Environment
Follow `TRAINING_GUIDE.md` to:
- Define observation/action spaces for each agent
- Implement reward shaping for UAV navigation
- Set up vectorized environments for parallel training

### 3. Train Hierarchical Agents
Use the three-tier training approach:
- Train trajectory optimizer first (low-level control)
- Train behavior selector with frozen trajectory optimizer
- Train mission planner with frozen lower-level agents

### 4. Deploy to BehaviorTree.CPP
Follow `BEHAVIOR_TREE_INTEGRATION.md` to:
- Wrap trained policies in BT action nodes
- Integrate with ROS 2 sensor/actuator topics
- Build mission-level behavior trees

## Hardware Requirements

### Training (NPS Cluster)
- Multi-core CPU for vectorized environments
- GPU for neural network training (CUDA-compatible)
- 32GB+ RAM for large replay buffers

### Deployment (Flyby F-11)
- NVIDIA Jetson Orin NX 16GB (50 TOPS)
- ROS 2 Humble
- ArduPilot with MAVROS interface

## Key References

### Algorithm Papers
- PPO: [Proximal Policy Optimization Algorithms](https://arxiv.org/abs/1707.06347)
- SAC: [Soft Actor-Critic](https://arxiv.org/abs/1801.01290)
- TD3: [Addressing Function Approximation Error](https://arxiv.org/abs/1802.09477)

### Implementation Resources
- Gymnasium Documentation: `gymnasium/repo/docs/`
- Stable-Baselines3 Docs: `stable-baselines3/repo/docs/`
- BehaviorTree.CPP Docs: `behavior-trees/BehaviorTree.CPP/docs/`

### Project-Specific
- Ontology Foundation: `/home/finley/Github/DroneProjects/flyby-f11/ONTOLOGY_FOUNDATION.qmd`
- System Constraints: `/home/finley/Github/DroneProjects/flyby-f11/SYSTEM_CONSTRAINTS.qmd`
- Approach Document: `/home/finley/Github/DroneProjects/flyby-f11/APPROACH.qmd`

## Development Notes

### Algorithm Selection Rationale
- **PPO for Mission Planner**: On-policy algorithm suited for discrete/mixed action spaces, stable training
- **SAC for Behavior Selector**: Off-policy with entropy regularization, encourages exploration
- **TD3 for Trajectory Optimizer**: Off-policy continuous control with reduced overestimation, stable low-level control

### Computational Constraints
- **Training**: Leverage NPS cluster for data-parallel training
- **Deployment**: Jetson Orin NX provides 50 TOPS, sufficient for real-time inference
- **Model Optimization**: Use ONNX/TensorRT for optimized inference on Jetson

### Safety Considerations
- BehaviorTree.CPP provides fail-safe fallback behaviors
- Learned policies operate within ontology-constrained action spaces
- Manual override always available via RC transmitter
- Geofencing enforced at flight controller level (ArduPilot)

## Contributing

When adding new documentation or code examples:
1. Follow the directory structure above
2. Include complete, runnable code examples
3. Document hyperparameters and design decisions
4. Reference relevant papers in `papers/` directory
5. Update this README with new resources

## Contact

**Developer**: Finley Holt
**Project**: Flyby F-11 Autonomous Navigation
**Collaboration**: MCTSSA / NPS
