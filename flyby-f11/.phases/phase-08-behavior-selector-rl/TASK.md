# Phase 8: Behavior Selector RL Agent

## Overview

Implement the Level 2 Behavior Selector agent using Proximal Policy Optimization (PPO). This agent operates on a 1-second horizon, selecting high-level behaviors (navigate, loiter, land, avoid) within ontology-defined constraints.

## Human Description

The Behavior Selector is the middle layer in the three-level hierarchy:

1. Mission Planner (Level 1) - 10s horizon, waypoint selection
2. **Behavior Selector (Level 2)** - 1s horizon, behavior selection ← This phase
3. Trajectory Optimizer (Level 3) - 100ms horizon, velocity commands

This agent receives waypoint goals from the Mission Planner and selects appropriate behaviors:
- **Navigate**: Transit to target waypoint
- **Loiter**: Station-keeping at position
- **Land**: Controlled descent to ground
- **Avoid**: Collision avoidance maneuver
- **ReturnToHome**: Emergency return

Behavior selection is constrained by ontology rules about valid transitions.

## AI Agent Instructions

### Prerequisites
- Phase 7 completed (Mission Planner RL - shares ontology_rl package)
- Understanding of PPO algorithm
- Familiarity with behavior trees
- Knowledge of flight behavior modes

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Define MDP Formulation**
   - State space: vehicle state, context, current behavior
   - Action space: behavior selection with parameters
   - Reward function: appropriateness + smoothness + safety
   - Document in `docs/behavior_selector_mdp.md`

2. **Create Gymnasium Environment**
   - Location: `ros2_ws/src/ontology_rl/envs/behavior_selector_env.py`
   - Discrete behavior actions
   - Continuous behavior parameters
   - Ontology-based transition validation

3. **Implement PPO Agent**
   - Location: `ros2_ws/src/ontology_rl/agents/behavior_selector_ppo.py`
   - Mixed discrete-continuous action space
   - Behavior tree integration hooks
   - Experience sharing with Mission Planner

4. **Define Behavior Specifications**
   - Location: `ros2_ws/src/ontology_rl/behaviors/`
   - Formal definition of each behavior
   - Entry/exit conditions
   - Parameter ranges and defaults

5. **Implement ROS 2 Node**
   - Location: `ros2_ws/src/ontology_rl/nodes/behavior_selector_node.py`
   - Subscribe to Mission Planner goals
   - Publish behavior commands
   - Interface with trajectory optimizer

6. **Develop Training Scenarios**
   - Behavior transition scenarios
   - Emergency behavior triggers
   - Multi-behavior sequences
   - Obstacle avoidance situations

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] MDP formulation documented
- [ ] Gymnasium environment implemented
- [ ] PPO agent trains successfully
- [ ] Behavior transitions validated by ontology
- [ ] ROS 2 node selects appropriate behaviors
- [ ] Smooth transitions between behaviors
- [ ] Emergency behaviors trigger correctly

### Verification

Run automated verification:
```bash
bash .phases/phase-08-behavior-selector-rl/verification.sh
```

### Time Estimate
10-14 hours (includes behavior definitions and transition logic)

### Common Pitfalls

- **Behavior oscillation**: Agent rapidly switching behaviors
- **Transition validity**: Must respect ontology transition rules
- **Parameter tuning**: Continuous parameters need careful bounds
- **Hierarchy coordination**: Must coordinate with Level 1 and 3
- **Emergency priority**: Safety behaviors must preempt normal
- **Behavior state**: Some behaviors have internal state

### References

- [PPO Paper](https://arxiv.org/abs/1707.06347)
- [Stable-Baselines3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [flyby-f11 APPROACH.qmd - Behavior Selector Agent](../APPROACH.qmd)

### Dependencies
See `dependencies.json` - can run in parallel with Phase 7 and 9.

### Next Phase
After completion, proceed to Phase 9: Trajectory Optimizer RL Agent
