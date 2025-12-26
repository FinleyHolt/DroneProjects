# Phase 6: Phase Transition Manager

## Overview

Implement the orchestration system that manages transitions between Planning Mode (heavyweight reasoning) and Execution Mode (lightweight runtime). This enables the two-phase compute strategy on the 16GB Jetson Orin NX.

## Human Description

The Flyby-F11 architecture uses two distinct compute modes:

1. **Planning Mode** (~14GB): SUMO + Vampire/Clingo for mission verification
2. **Execution Mode** (~14GB): Vision models + SWI-Prolog for real-time control

These cannot run simultaneously due to memory constraints. This phase creates:

1. Phase transition controller that orchestrates container swapping
2. Memory manager that ensures safe transitions
3. Mission handoff protocol from planning to execution
4. Runtime replanning triggers for major mission changes

## AI Agent Instructions

### Prerequisites
- Phase 4 completed (execution runtime)
- Phase 5 completed (perception bridge)
- Understanding of container orchestration
- Familiarity with ROS 2 lifecycle nodes

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Create Phase Transition Controller**
   - Location: `ros2_ws/src/phase_transition/`
   - ROS 2 lifecycle node for transition management
   - State machine: PLANNING → TRANSITION → EXECUTION
   - Handle graceful shutdown of planning containers
   - Orchestrate vision model loading

2. **Implement Memory Manager**
   - Monitor system memory usage
   - Ensure planning mode cleanup before execution
   - Validate memory availability for vision models
   - Implement memory pressure warnings

3. **Create Mission Handoff Protocol**
   - Define mission plan message format
   - Serialize verified mission from planning
   - Load compiled Prolog rules in execution
   - Validate mission integrity after transfer

4. **Implement Container Orchestration**
   - Location: `containers/compose/transition-compose.yml`
   - Define planning and execution service groups
   - Implement container lifecycle hooks
   - Handle GPU resource allocation

5. **Add Replanning Triggers**
   - Define conditions requiring replanning
   - Implement pause-replan-resume workflow
   - Handle emergency transitions
   - Log transition events for debugging

6. **Create Integration Tests**
   - Test full planning → execution transition
   - Validate memory is freed correctly
   - Test mission handoff integrity
   - Benchmark transition latency

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `phase_transition` ROS 2 package implemented
- [ ] Memory manager ensures <16GB total usage
- [ ] Planning → Execution transition completes in <30s
- [ ] Mission plan integrity preserved across transition
- [ ] Replanning triggers work correctly
- [ ] Container orchestration files created
- [ ] Integration tests pass

### Verification

Run automated verification:
```bash
bash .phases/phase-06-phase-transition/verification.sh
```

### Time Estimate
8-10 hours (includes container orchestration and integration testing)

### Common Pitfalls

- **Memory leaks**: Planning containers must fully release memory
- **Race conditions**: Ensure clean shutdown before starting execution
- **GPU conflicts**: TensorRT and other GPU processes need coordination
- **State loss**: Mission state must be serialized correctly
- **Timeout handling**: Transitions can hang if containers don't respond
- **Partial failures**: Handle cases where only some services start

### References

- [ROS 2 Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)
- [Podman Compose](https://docs.podman.io/en/latest/markdown/podman-compose.1.html)
- [flyby-f11 APPROACH.qmd - Two-Phase Compute Strategy](../APPROACH.qmd)
- [Jetson Memory Management](https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonOrinNxSeriesAndJetsonAgxOrinSeries.html)

### Dependencies
See `dependencies.json` - requires Phase 4 and 5 completion.

### Next Phase
After completion, proceed to Phase 7: Mission Planner RL Agent
