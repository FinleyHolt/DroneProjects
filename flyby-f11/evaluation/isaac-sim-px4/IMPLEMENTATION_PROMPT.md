# Implementation Prompt for Fast Training Architecture

Copy this prompt to start a new Claude Code session for implementation.

---

## Prompt

I need help implementing a faster-than-realtime RL training architecture for my drone ISR simulation. I have a detailed implementation plan at:

```
flyby-f11/evaluation/isaac-sim-px4/FAST_TRAINING_IMPLEMENTATION_PLAN.md
```

Please read that document first to understand the full scope.

### Context

I'm building RL training for drone ISR (Intelligence, Surveillance, Reconnaissance) missions using Isaac Sim 5.1 with PX4 SITL. Currently the simulation runs at ~60 FPS due to PX4 lockstep synchronization. Research showed that **removing PX4 from the training loop** and using direct dynamics can achieve 100-1000x speedup.

### Current State

The existing codebase is at `flyby-f11/evaluation/isaac-sim-px4/`:
- `environments/base_isr_env.py` - Core environment with PX4 SITL integration
- `environments/gymnasium_wrapper.py` - Gym wrapper
- `environments/perception_manager.py` - Camera + detection (GT and YOLO)
- `scripts/drone_functions_check.py` - Full-stack video validation test
- `scripts/run_test.sh` - Podman container runner

### What I Need Implemented

The plan has 4 phases. Implement them in order, with a working full-stack test after each phase:

**Phase 1: Quick Wins**
- Add render frame skipping to existing environment
- Add resolution presets (training vs evaluation)
- Full-stack test that produces video showing speedup

**Phase 2: Direct Dynamics**
- Create `training/environments/direct_dynamics.py` - simplified flight model
- Create `training/environments/fast_base_env.py` - env without PX4
- Full-stack test comparing direct vs PX4 dynamics (side-by-side video)

**Phase 3: Multi-Drone**
- Create `training/environments/fast_multi_drone.py` - vectorized multi-drone
- Implement tiled camera rendering for N drones
- Full-stack test showing grid of drone camera views

**Phase 4: Pipeline Integration**
- Create `evaluation/scripts/evaluate_policy.py` - run trained policy with PX4
- Create `scripts/run_full_pipeline.sh` - train → eval → video workflow
- Full-stack test of complete pipeline

### Repository Restructure Required

The plan includes a new directory structure separating `training/` (fast, no PX4) from `evaluation/` (full fidelity, with PX4). Please:

1. Create the new directory structure as specified in the plan
2. Move/refactor existing files to new locations
3. Keep backwards compatibility where possible (symlinks or re-exports)
4. Update imports throughout

### Testing Approach

I prefer full-stack integration tests that produce video evidence rather than unit tests. Each phase should have a test script that:
1. Runs in the Isaac Sim container via Podman
2. Produces a video file showing the feature works
3. Includes overlays (FPS, detections, metrics)
4. Can be run headless

Example of my existing test approach:
```bash
./scripts/run_test.sh --profile detection
# Produces: output/functions_check_detection.mp4
```

### Key Files to Reference

Before starting, please read:
1. `FAST_TRAINING_IMPLEMENTATION_PLAN.md` - Full plan with code examples
2. `environments/base_isr_env.py` - Current environment to understand patterns
3. `scripts/drone_functions_check.py` - Example of full-stack test with video
4. `environments/perception_manager.py` - Camera/detection integration

### Implementation Guidelines

1. **Incremental**: Complete one phase fully before starting the next
2. **Test-driven**: Each feature needs a working full-stack test
3. **Video evidence**: Tests should produce annotated video output
4. **Preserve PX4 path**: The full PX4 SITL integration must remain functional in `evaluation/`
5. **Shared components**: Reward functions, observation encoding should be shared between training and evaluation

### Hardware Context

My dev machine:
- NVIDIA RTX 5090 Laptop GPU (24GB VRAM)
- 64GB RAM
- AMD Ryzen AI 9 HX 370 (12 cores)

Target: 16-32 drones per world during training.

### Start Here

Please begin by:
1. Reading `FAST_TRAINING_IMPLEMENTATION_PLAN.md`
2. Understanding the current codebase structure
3. Creating the new directory structure
4. Implementing Phase 1 (render skipping + resolution presets)
5. Creating the Phase 1 full-stack test

After Phase 1 is working with a passing test video, we'll move to Phase 2.

---

## Additional Notes

- The Isaac Sim container is built with `scripts/build_container.sh`
- Tests run via `podman run` with GPU passthrough
- All paths inside container are `/workspace/...`
- Python scripts run with `/isaac-sim/python.sh`
- PX4 is installed at `/px4` in the container

## Success Criteria

Phase 1 is complete when:
- [ ] `./scripts/run_phase1_test.sh` produces video
- [ ] Video shows FPS ≥ 120 (was 60)
- [ ] Video shows detections still work correctly
- [ ] Existing `./scripts/run_test.sh` still works (backwards compat)
