# Isaac Sim Development Guide

This file provides guidance for working with the Isaac Sim simulation and training environment.

## Purpose

Primary simulation environment for drone autonomy development using Isaac Sim 5.1.0, PX4 SITL, and Pegasus Simulator. Used for perception development, RL training infrastructure, and validation before hardware deployment.

## Quick Start

```bash
# Build and start container
podman-compose up --build

# Enter container
podman exec -it isaac-sim-px4 bash

# Run a test script
/isaac-sim/python.sh scripts/depth_avoidance_test.py

# Run tests
pytest tests/
```

## Directory Structure

```
isaac-sim/
├── environments/        # Gymnasium RL environments
│   ├── base_isr_env.py  # Core ISR environment (1000+ LOC)
│   ├── comms_denied_env.py     # Canonical problem 1
│   ├── dynamic_nfz_env.py      # Canonical problem 2 (defined, not tested)
│   ├── multi_objective_env.py  # Canonical problem 3 (defined, not tested)
│   ├── gymnasium_wrapper.py    # Gym API wrapper
│   ├── action_bridge.py        # Z-up to NED conversion
│   ├── safety_filter.py        # Vampire ATP integration (not rigorously tested)
│   └── zone_manager.py         # NFZ management
├── perception/          # Detection and tracking
│   ├── detector.py             # YOLO11 wrapper
│   ├── ground_truth_detector.py # Frustum-based GT (1000+ Hz)
│   ├── dual_mode_perception.py # GT/YOLO mode switching
│   └── frustum.py              # Camera frustum math
├── training/            # RL training infrastructure (proof of concept)
│   ├── environments/           # Fast training envs
│   ├── scripts/                # Training launchers
│   └── callbacks/              # TensorBoard, checkpoints
├── scripts/             # Test and utility scripts
│   ├── depth_avoidance_test.py # Active development
│   ├── phase*_test.py          # Phase validation scripts
│   └── nav_obstacle_test.py    # Obstacle avoidance testing
├── extensions/          # Procedural world generation
├── tests/               # Pytest test suite
└── config/              # Training hyperparameters
```

## Current Focus (January 2026)

1. **Depth Avoidance**: Active work on `scripts/depth_avoidance_test.py` and related calibration
2. **E2E Validation**: Testing full comms-denied mission (Phase 9)
3. **Behavior Tree Integration**: Planning RL as leaf nodes in BT architecture

## Key Files to Understand

| File | Purpose | Status |
|------|---------|--------|
| `environments/base_isr_env.py` | Core environment orchestrator | Working |
| `perception/dual_mode_perception.py` | GT/YOLO switching | Working |
| `environments/safety_filter.py` | Ontology-based safety | Needs testing |
| `training/environments/isr_training_env.py` | Fast training env | Proof of concept |
| `scripts/depth_avoidance_test.py` | Depth calibration | Active development |

## Common Tasks

### Running Training (Proof of Concept)
```bash
/isaac-sim/python.sh training/scripts/train_isr_ppo.py
```
Note: Training infrastructure runs but no good model has been trained yet.

### Testing Perception
```bash
# Ground truth mode (fast)
/isaac-sim/python.sh scripts/phase3_camera_gt_test.py

# YOLO mode
/isaac-sim/python.sh scripts/phase8_yolo_test.py
```

### Adding New Environments
1. Extend `BaseISREnvironment` in `environments/`
2. Define observation/action spaces
3. Implement `_compute_reward()` and `_is_done()`
4. Create test script in `scripts/`

## Code Conventions

- All environments extend `BaseISREnvironment`
- Use `ObservationSpaceManager` for consistent observation bounds
- Safety filter is optional (graceful degradation if Vampire unavailable)
- Always use `action_bridge.py` for coordinate conversion (Z-up to NED)
- Continuous stepping required - no `time.sleep()` (PX4 starves)

## Pitfalls

- **NumPy 1.26.4**: Required - NumPy 2.x breaks Isaac Sim cameras
- **Shader Warmup**: First container run takes 2-3 min for shader compilation
- **PX4 Lockstep**: Limits simulation to ~60 FPS (hence fast training work)
- **Coordinate Systems**: Isaac Sim Z-up, PX4 NED - always convert

## Related Documentation

- `environments/README.md`: Canonical ISR problems
- `FAST_TRAINING_IMPLEMENTATION_PLAN.md`: Performance optimization roadmap
- `README.md`: Container setup and troubleshooting
