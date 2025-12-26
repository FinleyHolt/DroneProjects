# Phase 7: Mission Planner RL Agent

## Overview

Implement the Level 1 Mission Planner agent using Soft Actor-Critic (SAC) reinforcement learning. This agent operates on a 10-second horizon, selecting waypoints and adapting mission plans within ontology-defined constraints.

## Human Description

The Mission Planner is the highest-level RL agent in the three-level hierarchy:

1. **Mission Planner (Level 1)** - 10s horizon, waypoint selection ← This phase
2. Behavior Selector (Level 2) - 1s horizon, behavior mode selection
3. Trajectory Optimizer (Level 3) - 100ms horizon, velocity commands

This agent receives the verified mission plan from planning mode and makes runtime decisions about:
- Which waypoint to pursue next
- When to adapt the mission plan
- How to allocate resources (battery, time)
- When to abort if constraints are violated

All actions are filtered through the ontology to ensure safety.

## AI Agent Instructions

### Prerequisites
- Phase 4-6 completed (execution runtime, perception, transitions)
- Understanding of reinforcement learning (SAC algorithm)
- Familiarity with Gymnasium environment design
- Knowledge of ROS 2 action servers

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Define MDP Formulation**
   - State space: mission progress, resources, environment
   - Action space: waypoint selection, mission adaptation
   - Reward function: completion + efficiency + safety
   - Document in `docs/mission_planner_mdp.md`

2. **Create Gymnasium Environment**
   - Location: `ros2_ws/src/ontology_rl/envs/mission_planner_env.py`
   - Wrap ROS 2 topics/services as Gym interface
   - Implement Prolog constraint checking
   - Add ontology-based action filtering

3. **Implement SAC Agent**
   - Location: `ros2_ws/src/ontology_rl/agents/mission_planner_sac.py`
   - Use Stable-Baselines3 or custom implementation
   - Add experience replay buffer
   - Implement policy and Q-function networks

4. **Create Training Infrastructure**
   - Location: `ros2_ws/src/ontology_rl/training/`
   - Simulation environment connector
   - Curriculum learning support
   - Checkpointing and logging

5. **Implement ROS 2 Node**
   - Location: `ros2_ws/src/ontology_rl/nodes/mission_planner_node.py`
   - Load trained policy
   - Subscribe to state topics
   - Publish action commands
   - Query Prolog for constraint validation

6. **Develop Training Scenarios**
   - Simple waypoint missions
   - Multi-objective missions
   - Resource-constrained scenarios
   - Dynamic environment changes

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] MDP formulation documented
- [ ] Gymnasium environment implemented
- [ ] SAC agent trains successfully
- [ ] Ontology constraint filtering works
- [ ] ROS 2 node publishes valid actions
- [ ] Training converges on simple missions
- [ ] Trained policy can be loaded and executed

### Verification

Run automated verification:
```bash
bash .phases/phase-07-mission-planner-rl/verification.sh
```

### Time Estimate
12-16 hours (includes MDP design, training, and integration)

### Common Pitfalls

- **Reward shaping**: Poor rewards lead to undesired behavior
- **Action filtering**: Must not break exploration during training
- **State representation**: Ontological state vs. raw sensor state
- **Sample efficiency**: RL is sample-hungry, use simulation
- **Policy collapse**: SAC can collapse to deterministic suboptimal
- **ROS timing**: Ensure agent loop matches planning horizon

### References

- [SAC Paper](https://arxiv.org/abs/1801.01290)
- [Stable-Baselines3 SAC](https://stable-baselines3.readthedocs.io/en/master/modules/sac.html)
- [Gymnasium API](https://gymnasium.farama.org/)
- [flyby-f11 APPROACH.qmd - Multi-Agent RL](../APPROACH.qmd)

### Dependencies
See `dependencies.json` - requires Phase 6 completion.

### Next Phase
After completion, proceed to Phase 8: Behavior Selector RL Agent
