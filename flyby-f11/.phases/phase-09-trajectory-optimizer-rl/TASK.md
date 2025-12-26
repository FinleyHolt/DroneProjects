# Phase 9: Trajectory Optimizer RL Agent

## Overview

Implement the Level 3 Trajectory Optimizer agent using Twin Delayed DDPG (TD3). This agent operates on a 100ms horizon, generating velocity commands [vx, vy, vz, yaw_rate] while respecting actuator constraints and safety margins.

## Human Description

The Trajectory Optimizer is the lowest level in the three-level hierarchy:

1. Mission Planner (Level 1) - 10s horizon, waypoint selection
2. Behavior Selector (Level 2) - 1s horizon, behavior selection
3. **Trajectory Optimizer (Level 3)** - 100ms horizon, velocity commands ← This phase

This agent receives behavior commands and generates smooth, safe trajectories that:
- Track setpoints accurately
- Minimize jerk for smooth flight
- Maintain obstacle clearance
- Optimize energy consumption
- Respect actuator limits

The output is velocity commands sent directly to PX4/ArduPilot.

## AI Agent Instructions

### Prerequisites
- Phase 7-8 progress (shares ontology_rl package)
- Understanding of TD3 algorithm
- Familiarity with quadrotor dynamics
- Knowledge of PX4/MAVSDK interfaces

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Define MDP Formulation**
   - State space: pose, velocity, setpoints, obstacles
   - Action space: continuous velocity [vx, vy, vz, yaw_rate]
   - Reward function: tracking + smoothness + safety + efficiency
   - Document in `docs/trajectory_optimizer_mdp.md`

2. **Create Gymnasium Environment**
   - Location: `ros2_ws/src/ontology_rl/envs/trajectory_optimizer_env.py`
   - Continuous observation and action spaces
   - Physics-based dynamics simulation
   - Ontology constraints on velocity limits

3. **Implement TD3 Agent**
   - Location: `ros2_ws/src/ontology_rl/agents/trajectory_optimizer_td3.py`
   - Twin Q-networks for stability
   - Delayed policy updates
   - Target network smoothing

4. **Create Dynamics Model**
   - Location: `ros2_ws/src/ontology_rl/dynamics/quadrotor_model.py`
   - Simplified quadrotor dynamics for training
   - Domain randomization parameters
   - Sim-to-real transfer considerations

5. **Implement ROS 2 Node**
   - Location: `ros2_ws/src/ontology_rl/nodes/trajectory_optimizer_node.py`
   - 100Hz control loop
   - Subscribe to behavior setpoints
   - Publish velocity commands to PX4
   - Real-time performance critical

6. **Develop Training Scenarios**
   - Waypoint tracking
   - Obstacle avoidance trajectories
   - Wind disturbance rejection
   - Emergency stop scenarios

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] MDP formulation documented
- [ ] Gymnasium environment implemented
- [ ] TD3 agent trains successfully
- [ ] Velocity limits enforced by ontology
- [ ] ROS 2 node runs at 100Hz
- [ ] Smooth trajectory generation
- [ ] Collision-free paths in simulation

### Verification

Run automated verification:
```bash
bash .phases/phase-09-trajectory-optimizer-rl/verification.sh
```

### Time Estimate
10-14 hours (includes dynamics modeling and real-time optimization)

### Common Pitfalls

- **Control frequency**: Must maintain 100Hz for stable flight
- **Action saturation**: Clipping can cause instability
- **Sim-to-real gap**: Simulated dynamics differ from reality
- **Jerk limits**: Smooth acceleration is critical for safety
- **Latency**: Observation delays affect performance
- **Coordinate frames**: Must handle frame transformations

### References

- [TD3 Paper](https://arxiv.org/abs/1802.09477)
- [Stable-Baselines3 TD3](https://stable-baselines3.readthedocs.io/en/master/modules/td3.html)
- [PX4 Offboard Control](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [MAVSDK Offboard](https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html)

### Dependencies
See `dependencies.json` - can run in parallel with Phase 7 and 8.

### Next Phase
After completion, proceed to Phase 10: AGGM Integration
