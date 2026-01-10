# Fast Training Architecture

## Overview

This package provides faster-than-realtime RL training environments that bypass PX4 SITL overhead while maintaining compatibility with the full-fidelity evaluation stack.

## Directory Structure

```
isaac-sim-px4/
├── training/                    # FAST TRAINING (this package)
│   ├── environments/
│   │   ├── isr_training_env.py  # Main training env (DirectRLEnv pattern)
│   │   └── __init__.py
│   ├── configs/                 # Training-specific configs
│   ├── tests/                   # Video-producing validation tests
│   └── scripts/                 # Training launch scripts
│
├── evaluation/                  # FULL-FIDELITY EVALUATION
│   ├── environments/            # PX4 SITL-integrated envs
│   ├── configs/                 # Evaluation configs
│   └── tests/                   # Sim-to-real validation
│
├── environments/                # LEGACY (PX4 SITL envs)
│   ├── base_isr_env.py         # Full-fidelity with PX4
│   └── ...
│
├── shared/                      # SHARED COMPONENTS
│   ├── reward_functions.py      # Reward computation
│   ├── observation_space.py     # Observation encoding
│   ├── action_space.py          # Action normalization
│   └── frustum_filter.py        # Camera FOV filtering
│
└── extensions/                  # Isaac Sim extensions
    └── forest_generator/        # World generation
```

## Key Differences: Training vs Evaluation

| Aspect | Training (Fast) | Evaluation (Full-Fidelity) |
|--------|-----------------|---------------------------|
| Physics | `set_external_force_and_torque()` | PX4 SITL + MAVLink |
| Perception | Ground truth + frustum filter | YOLO inference on RGB |
| Rendering | Optional (raycast depth) | Full RTX rendering |
| Parallelization | 512-1024 envs | Single env |
| Target FPS | 10k-50k | 60-128 |
| Use Case | Policy training | Sim-to-real validation |

## Training Environment Architecture

### ISRTrainingEnv (DirectRLEnv Pattern)

```
┌─────────────────────────────────────────────────────────────────┐
│                     ISRTrainingEnv                               │
├─────────────────────────────────────────────────────────────────┤
│  Drone Dynamics                                                  │
│  ├── RigidObject (drone body)                                   │
│  ├── set_external_force_and_torque() for thrust/moment          │
│  └── ArticulationView for gimbal joints                         │
├─────────────────────────────────────────────────────────────────┤
│  Perception (Ground Truth)                                       │
│  ├── Target positions from simulation state                     │
│  ├── Frustum filtering (camera FOV check)                       │
│  ├── Occlusion checking (optional raycast)                      │
│  └── Encoded as observation tensor                              │
├─────────────────────────────────────────────────────────────────┤
│  Observations                                                    │
│  ├── Drone state: position, velocity, orientation               │
│  ├── Gimbal state: pitch, yaw angles                            │
│  ├── Visible targets: [N x (rel_pos, bbox, confidence)]         │
│  ├── Mission state: battery, time, coverage                     │
│  └── Zone state: NFZ distances, threat levels                   │
├─────────────────────────────────────────────────────────────────┤
│  Actions                                                         │
│  ├── [0]: Collective thrust (normalized)                        │
│  ├── [1-3]: Roll, pitch, yaw moment                             │
│  └── [4-5]: Gimbal pitch, yaw commands                          │
└─────────────────────────────────────────────────────────────────┘
```

### Frustum Filtering

Ground truth targets are filtered to only include what the camera could see:

```python
def is_in_frustum(target_pos, camera_pos, camera_orientation, fov_h, fov_v, max_range):
    """
    Check if target is visible in camera frustum.

    1. Transform target to camera frame
    2. Check if within horizontal FOV
    3. Check if within vertical FOV
    4. Check if within max range
    5. (Optional) Raycast for occlusion
    """
    # Transform to camera frame
    rel_pos = target_pos - camera_pos
    cam_frame_pos = camera_rotation_inv @ rel_pos

    # Target must be in front of camera
    if cam_frame_pos[2] <= 0:
        return False

    # Check horizontal angle
    angle_h = np.arctan2(cam_frame_pos[0], cam_frame_pos[2])
    if abs(angle_h) > fov_h / 2:
        return False

    # Check vertical angle
    angle_v = np.arctan2(cam_frame_pos[1], cam_frame_pos[2])
    if abs(angle_v) > fov_v / 2:
        return False

    # Check range
    distance = np.linalg.norm(rel_pos)
    if distance > max_range:
        return False

    return True
```

### Gimbal Control

The gimbal is modeled as a 2-DOF articulation:
- Pitch joint: -90 to +30 degrees
- Yaw joint: -180 to +180 degrees (relative to body)

Camera pose = Drone pose * Gimbal transform

## Performance Targets

| Configuration | Parallel Envs | Expected FPS |
|---------------|---------------|--------------|
| State-only (no perception) | 4096 | 50k-100k |
| Ground truth perception | 1024 | 10k-50k |
| With raycast depth | 512 | 5k-20k |

## Workflow

### Training

```bash
# Fast training with ground truth perception
./scripts/run_phase2_test.sh

# Inside container
python training/scripts/train_isr.py \
    --num_envs 1024 \
    --perception_mode gt \
    --headless
```

### Evaluation

```bash
# Full-fidelity with PX4 + YOLO
./scripts/run_test.sh --profile full

# Inside container
python evaluation/scripts/evaluate_policy.py \
    --checkpoint checkpoints/best.pt \
    --perception_mode yolo \
    --record_video
```

## Shared Components

Components that must be identical between training and evaluation:

1. **Observation encoding**: Same tensor format
2. **Action space**: Same normalization
3. **Reward function**: Same computation (can use GT during training)
4. **Episode termination**: Same conditions

These live in `shared/` and are imported by both packages.
