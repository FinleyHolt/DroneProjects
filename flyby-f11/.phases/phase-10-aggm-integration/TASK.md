# Phase 10: AGGM Integration

## Overview

Integrate the Automatic Goal Generation Model (AGGM) that enables runtime adaptation to unseen situations without retraining. This implements the six-stage reasoning process for handling novel scenarios through forward and backward ontological reasoning.

## Human Description

The AGGM is the key innovation that allows the system to handle situations never seen during training:

1. **Observe**: Ground perceptions to ontological concepts
2. **Evaluate**: Compute Q-values and state distances
3. **Identify**: Detect significant changes triggering adaptation
4. **Reason**: Forward/backward reasoning for goal generation
5. **Generate**: Ontology-constrained action selection
6. **Execute**: Translate to flight commands

This phase integrates AGGM with all three RL agents, enabling:
- Runtime goal generation for novel situations
- Backward reasoning to return to known safe states
- Priority balancing between safety and task completion

## AI Agent Instructions

### Prerequisites
- Phase 7-9 completed (all three RL agents)
- Phase 5 completed (perception grounding)
- Understanding of ontological reasoning
- Familiarity with importance weighting

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Implement AGGM Core**
   - Location: `ros2_ws/src/ontology_rl/aggm/`
   - Six-stage process implementation
   - State distance computation
   - Importance weighting functions

2. **Create Change Detection Module**
   - Location: `ros2_ws/src/ontology_rl/aggm/change_detector.py`
   - Q-value discrepancy detection
   - State distance thresholds
   - High-importance concept detection

3. **Implement Goal Reasoner**
   - Location: `ros2_ws/src/ontology_rl/aggm/goal_reasoner.py`
   - Forward reasoning (predefined goal-set)
   - Backward reasoning (safe state return)
   - Priority function F(B, J)

4. **Integrate with RL Agents**
   - Modify mission planner for AGGM goals
   - Update behavior selector for new goals
   - Adjust trajectory optimizer for goal changes
   - Handle goal switching gracefully

5. **Create AGGM ROS 2 Node**
   - Location: `ros2_ws/src/ontology_rl/nodes/aggm_node.py`
   - Continuous monitoring loop
   - Goal publication to agents
   - Event logging for analysis

6. **Develop Test Scenarios**
   - Unexpected obstacle scenarios
   - GPS loss scenarios
   - Environment change scenarios
   - Emergency situations

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] Six-stage AGGM process implemented
- [ ] Change detection triggers correctly
- [ ] Forward reasoning selects valid goals
- [ ] Backward reasoning returns to safe states
- [ ] Priority function balances safety/task
- [ ] Integration with all three agents
- [ ] Test scenarios pass

### Verification

Run automated verification:
```bash
bash .phases/phase-10-aggm-integration/verification.sh
```

### Time Estimate
12-16 hours (includes integration with all agents and extensive testing)

### Common Pitfalls

- **False triggers**: Change detection too sensitive
- **Goal conflicts**: Multiple goals competing
- **Priority tuning**: Safety vs. task balance
- **Reasoning latency**: Must be fast enough for real-time
- **State similarity**: Poor metrics lead to bad goals
- **Agent coordination**: All agents must accept new goals

### References

- [Ghanadbashi & Golpayegani (2022)](../literature_review/02_ontology_guided_rl_unseen_situations.qmd)
- [AGGM Paper](https://link.springer.com/article/10.1007/s10489-021-02449-5)
- [flyby-f11 APPROACH.qmd - AGGM](../APPROACH.qmd)

### Dependencies
See `dependencies.json` - requires all three RL agents completed.

### Next Phase
After completion, proceed to Phase 11: Simulation Benchmark Suite
