# Fast Training Implementation Plan

## Overview

This document outlines the plan to implement faster-than-realtime RL training for drone ISR missions in Isaac Sim 5.1. The key insight from research is that **PX4 SITL lockstep is the primary bottleneck**, limiting simulation to ~60 FPS regardless of GPU capability.

**Target outcome**: 100-1000x speedup during training while retaining full PX4 SITL integration for policy evaluation/deployment validation.

## Current State Analysis

### Existing Architecture
```
flyby-f11/evaluation/isaac-sim-px4/
├── environments/           # Gymnasium wrappers, PX4 integration
│   ├── base_isr_env.py    # Core env with PX4 lockstep
│   ├── gymnasium_wrapper.py
│   ├── perception_manager.py
│   └── simulation_core.py
├── scripts/
│   ├── drone_functions_check.py  # Full-stack video validation
│   ├── flight_profiles/          # Modular test scenarios
│   └── run_test.sh              # Podman test runner
├── perception/            # YOLO, ByteTrack, ground truth
└── extensions/           # World generation
```

### Current Performance (Baseline)
- Physics rate: 250 Hz
- Render rate: 60 Hz
- Effective training FPS: ~60 (limited by PX4 lockstep)
- Camera resolution: 640x480
- Single drone per world

### Target Performance
| Metric | Current | Phase 1 | Phase 2 | Phase 3 |
|--------|---------|---------|---------|---------|
| Training FPS | 60 | 120-240 | 2,000-5,000 | 10,000-50,000 |
| Drones/world | 1 | 1 | 1 | 16-32 |
| Camera res | 640x480 | 128x128 | 128x128 | 80x80 |
| PX4 in loop | Yes | Yes (skip renders) | No | No |

---

## Repository Restructure

### Proposed New Structure
```
flyby-f11/evaluation/isaac-sim-px4/
├── core/                           # Shared components
│   ├── world_generator/           # Procedural world (moved from extensions)
│   ├── perception/                # Detectors, encoders, trackers
│   ├── domain_randomization/      # DR managers
│   └── configs/                   # Shared configuration schemas
│
├── training/                       # FAST training (no PX4)
│   ├── environments/
│   │   ├── fast_base_env.py       # Direct dynamics, no PX4
│   │   ├── fast_single_drone.py   # Single drone fast training
│   │   ├── fast_multi_drone.py    # Multi-drone vectorized
│   │   └── direct_dynamics.py     # Simplified flight model
│   ├── configs/
│   │   ├── training_config.yaml   # Training hyperparameters
│   │   └── dynamics_config.yaml   # Simplified dynamics params
│   ├── scripts/
│   │   ├── train_fast.py          # Main training script
│   │   └── train_fast.sh          # Podman runner
│   └── tests/
│       └── test_fast_training_stack.py
│
├── evaluation/                     # FULL-FIDELITY eval (with PX4)
│   ├── environments/
│   │   ├── px4_base_env.py        # Current base_isr_env.py (renamed)
│   │   ├── gymnasium_wrapper.py   # Current wrapper
│   │   └── simulation_core.py     # PX4 SITL integration
│   ├── scripts/
│   │   ├── drone_functions_check.py  # Current video validation
│   │   ├── flight_profiles/          # Current profiles
│   │   ├── evaluate_policy.py        # Run trained policy with PX4
│   │   └── run_evaluation.sh
│   └── tests/
│       └── test_px4_stack.py
│
├── shared/                         # Used by both training and eval
│   ├── reward_functions.py        # Reward computation
│   ├── observation_space.py       # Observation encoding
│   ├── action_space.py            # Action definitions
│   └── ontology_integration.py    # Safety filter, behavior controller
│
├── scripts/                        # Top-level runners
│   ├── run_fast_training.sh       # Fast training pipeline
│   ├── run_evaluation.sh          # Full PX4 evaluation
│   ├── run_full_pipeline.sh       # Train → Eval → Video
│   └── build_container.sh
│
└── output/                         # All outputs
    ├── models/                    # Trained policies
    ├── videos/                    # Validation recordings
    ├── logs/                      # Training logs
    └── reports/                   # Test reports
```

### Key Separation Principles

1. **Training** (`training/`): Optimized for speed
   - No PX4 SITL
   - Direct velocity/attitude control
   - Lower resolution cameras
   - Multi-drone vectorized environments
   - Skip rendering on most frames

2. **Evaluation** (`evaluation/`): Optimized for fidelity
   - Full PX4 SITL integration
   - Full resolution cameras
   - YOLO inference (not just GT)
   - Video recording for visual validation
   - Single drone (matches deployment)

3. **Shared** (`shared/`): Common abstractions
   - Same reward functions
   - Same observation encoding
   - Same action space semantics
   - Ensures training→eval transfer

---

## Implementation Phases

### Phase 1: Quick Wins (Estimated: 2-4x speedup)

**Goal**: Minimal code changes, immediate benefits within current PX4 architecture.

#### 1.1 Render Frame Skipping

Add configurable render interval to `base_isr_env.py`:

```python
# In EnvironmentConfig
render_interval: int = 4  # Render every Nth frame

# In step()
def step(self, action):
    self._step_count += 1
    need_render = (self._step_count % self.config.render_interval == 0)
    self.world.step(render=need_render)

    if need_render:
        obs = self.state_to_observation(state)  # Full observation
    else:
        obs = self._state_only_observation(state)  # No camera
```

#### 1.2 Training Resolution Config

Add resolution presets to `PerceptionConfig`:

```python
@dataclass
class PerceptionConfig:
    # Resolution presets
    resolution_preset: str = "training"  # "training", "evaluation", "full"

    @property
    def camera_resolution(self) -> Tuple[int, int]:
        presets = {
            "training": (128, 128),
            "evaluation": (320, 240),
            "full": (640, 480),
        }
        return presets[self.resolution_preset]
```

#### 1.3 Headless Optimization

Ensure headless mode is fully optimized:

```python
simulation_config = {
    "headless": True,
    "disable_viewport_updates": True,
    "anti_aliasing": 0,
    "renderer": "PathTracing",  # or "RayTracedLighting"
}
```

#### 1.4 Full-Stack Test: `test_render_skipping.py`

```python
"""
Full-stack test for Phase 1 render skipping optimization.

Runs: ./scripts/run_phase1_test.sh
Validates:
  - Training completes 1000 steps in <30 seconds (was ~60 seconds)
  - Video output shows correct detections
  - Policy performance comparable to baseline
"""
```

**Test script creates video output showing**:
- Frame counter with render/skip annotations
- FPS overlay showing effective throughput
- Detection overlay on rendered frames
- Timestamp verification

---

### Phase 2: Direct Dynamics (Estimated: 100x speedup)

**Goal**: Remove PX4 from training loop, use simplified flight dynamics.

#### 2.1 Direct Dynamics Module

Create `training/environments/direct_dynamics.py`:

```python
@dataclass
class DirectDynamicsConfig:
    """Configuration for simplified drone dynamics."""
    max_velocity: float = 10.0        # m/s
    max_vertical_velocity: float = 5.0
    max_yaw_rate: float = 90.0        # deg/s
    velocity_damping: float = 0.95    # Per-step decay
    position_noise_std: float = 0.01  # Domain randomization
    velocity_noise_std: float = 0.05

class DirectDynamics:
    """
    Simplified drone dynamics for fast RL training.

    Replaces PX4 SITL with direct velocity integration.
    Designed for policy pre-training; fine-tune with PX4 for deployment.
    """

    def __init__(self, config: DirectDynamicsConfig):
        self.config = config
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.yaw = 0.0

    def step(self, action: np.ndarray, dt: float) -> Tuple[np.ndarray, float]:
        """
        Apply action and integrate dynamics.

        Args:
            action: [vx, vy, vz, yaw_rate] normalized [-1, 1]
            dt: Time step in seconds

        Returns:
            (new_position, new_yaw)
        """
        # Scale action to physical units
        target_velocity = np.array([
            action[0] * self.config.max_velocity,
            action[1] * self.config.max_velocity,
            action[2] * self.config.max_vertical_velocity,
        ])
        yaw_rate = action[3] * np.radians(self.config.max_yaw_rate)

        # Simple first-order response (could be tuned to match PX4)
        self.velocity = (
            self.config.velocity_damping * self.velocity +
            (1 - self.config.velocity_damping) * target_velocity
        )

        # Add noise for domain randomization
        if self.config.velocity_noise_std > 0:
            self.velocity += np.random.normal(
                0, self.config.velocity_noise_std, 3
            )

        # Integrate
        self.position += self.velocity * dt
        self.yaw += yaw_rate * dt

        if self.config.position_noise_std > 0:
            self.position += np.random.normal(
                0, self.config.position_noise_std, 3
            )

        return self.position.copy(), self.yaw

    def reset(self, position: np.ndarray, yaw: float = 0.0):
        self.position = position.copy()
        self.velocity = np.zeros(3)
        self.yaw = yaw
```

#### 2.2 Fast Training Environment

Create `training/environments/fast_base_env.py`:

```python
class FastISREnvironment(gym.Env):
    """
    Fast training environment without PX4 SITL.

    Key differences from PX4 environment:
    - Direct dynamics (no MAVLink)
    - No lockstep synchronization
    - Physics runs as fast as GPU allows
    - Render only when needed for observations
    """

    def __init__(self, config: FastEnvConfig):
        self.config = config
        self.dynamics = DirectDynamics(config.dynamics)
        self.render_interval = config.render_interval
        self._step_count = 0

        # Same observation/action space as PX4 env for transfer
        self.observation_space = self._build_observation_space()
        self.action_space = spaces.Box(-1, 1, (4,), dtype=np.float32)

    def step(self, action):
        self._step_count += 1

        # Update dynamics (fast, no PX4)
        position, yaw = self.dynamics.step(action, self.config.dt)

        # Update Isaac Sim prim directly
        self._set_drone_pose(position, yaw)

        # Step physics (no render unless needed)
        need_render = (self._step_count % self.render_interval == 0)
        self.world.step(render=need_render)

        # Get observation
        if need_render:
            obs = self._get_full_observation()
        else:
            obs = self._get_state_observation()

        # Same reward as PX4 env
        reward = self._compute_reward()
        done = self._check_done()

        return obs, reward, done, False, {}
```

#### 2.3 Dynamics Tuning Script

Create `training/scripts/tune_dynamics.py`:

```python
"""
Tune DirectDynamics parameters to match PX4 SITL behavior.

Process:
1. Run PX4 environment with random actions, record trajectories
2. Run DirectDynamics with same actions
3. Optimize DirectDynamics params to minimize trajectory error
4. Output tuned config
"""
```

#### 2.4 Full-Stack Test: `test_direct_dynamics_parity.py`

**Test validates**:
- Same policy achieves similar reward in both environments
- Trajectory statistics match within tolerance
- Video comparison: Direct vs PX4 side-by-side

```python
"""
Full-stack parity test between Direct and PX4 dynamics.

Runs: ./scripts/run_parity_test.sh
Output: output/videos/parity_comparison.mp4

Video shows:
  - Left: Direct dynamics environment
  - Right: PX4 SITL environment
  - Both running same policy
  - Overlay: reward, position error, trajectory
"""
```

---

### Phase 3: Multi-Drone Single World (Estimated: 10-50x additional)

**Goal**: Run multiple drones in parallel within single Isaac Sim world.

#### 3.1 Multi-Drone Environment

Create `training/environments/fast_multi_drone.py`:

```python
class MultiDroneISREnvironment(gym.Env):
    """
    Vectorized multi-drone environment for maximum throughput.

    Architecture:
    - Single Isaac Sim world with shared static geometry
    - N drones with independent physics bodies
    - Tiled camera rendering (single render pass for all cameras)
    - Batched observations and rewards
    """

    def __init__(self, config: MultiDroneConfig):
        self.num_drones = config.num_drones
        self.drones = []
        self.dynamics = []
        self.tiled_camera = None

    def _spawn_drones(self, n: int):
        """Spawn N drones at different positions."""
        spawn_positions = self._compute_spawn_grid(n)

        for i in range(n):
            drone_path = f"/World/drones/drone_{i}"
            camera_path = f"{drone_path}/camera"

            # Spawn drone USD
            drone = self._create_drone_prim(drone_path, spawn_positions[i])
            self.drones.append(drone)

            # Create dynamics instance
            dynamics = DirectDynamics(self.config.dynamics)
            dynamics.reset(spawn_positions[i])
            self.dynamics.append(dynamics)

    def _setup_tiled_camera(self):
        """Setup tiled rendering for all drone cameras."""
        import omni.replicator.core as rep

        camera_paths = [f"/World/drones/drone_{i}/camera"
                       for i in range(self.num_drones)]

        # Create render products
        self.render_products = []
        for path in camera_paths:
            rp = rep.create.render_product(
                path,
                self.config.camera_resolution
            )
            self.render_products.append(rp)

        # Single annotator for all cameras
        self.bbox_annotator = rep.AnnotatorRegistry.get_annotator(
            "bounding_box_2d_tight"
        )
        for rp in self.render_products:
            self.bbox_annotator.attach(rp)

    def step(self, actions: np.ndarray):
        """
        Vectorized step for all drones.

        Args:
            actions: (num_drones, 4) array of actions

        Returns:
            obs: (num_drones, obs_dim)
            rewards: (num_drones,)
            dones: (num_drones,)
            ...
        """
        # Update all dynamics
        positions = np.zeros((self.num_drones, 3))
        for i, (dynamics, action) in enumerate(zip(self.dynamics, actions)):
            pos, yaw = dynamics.step(action, self.config.dt)
            positions[i] = pos
            self._set_drone_pose(i, pos, yaw)

        # Single world step (renders all cameras in one pass)
        self.world.step(render=True)

        # Get batched observations
        camera_obs = self._get_tiled_camera_obs()  # (N, H, W, C)
        state_obs = self._get_state_obs()          # (N, state_dim)

        # Compute rewards for all drones
        rewards = self._compute_rewards_batch(positions)
        dones = self._compute_dones_batch(positions)

        obs = np.concatenate([state_obs, camera_obs.reshape(self.num_drones, -1)], axis=1)

        return obs, rewards, dones, np.zeros(self.num_drones, dtype=bool), {}
```

#### 3.2 Tiled Camera Integration

```python
def _get_tiled_camera_obs(self) -> np.ndarray:
    """
    Get camera observations using tiled rendering.

    Returns:
        (num_drones, height, width, channels) tensor
    """
    # Isaac Lab style tiled camera returns batched tensor
    # Each camera rendered as tile in single GPU framebuffer

    # Get RGB data (already batched by tiled renderer)
    rgb_data = []
    for i, rp in enumerate(self.render_products):
        # Get frame from render product
        frame = rp.get_image()
        rgb_data.append(frame)

    return np.stack(rgb_data, axis=0)
```

#### 3.3 Full-Stack Test: `test_multi_drone_throughput.py`

**Test validates**:
- Throughput scales with drone count
- All drones have independent perception
- Shared world geometry collision works
- Video shows grid of all drone camera views

```python
"""
Multi-drone throughput and correctness test.

Runs: ./scripts/run_multi_drone_test.sh
Output: output/videos/multi_drone_grid.mp4

Video shows:
  - 4x4 grid of 16 drone camera views
  - Each drone flying independently
  - Overlaid: per-drone reward, detection count
  - Footer: aggregate FPS, VRAM usage
"""
```

---

### Phase 4: Integration and Validation Pipeline

**Goal**: Seamless train→evaluate→validate workflow.

#### 4.1 Full Pipeline Script

Create `scripts/run_full_pipeline.sh`:

```bash
#!/bin/bash
# Full training and evaluation pipeline
#
# 1. Fast training (direct dynamics, multi-drone)
# 2. Policy evaluation (PX4 SITL)
# 3. Video validation (annotated recording)
# 4. Metrics report

set -e

MODEL_NAME="${1:-default}"
TRAINING_STEPS="${2:-1000000}"
EVAL_EPISODES="${3:-100}"

echo "=========================================="
echo "ISR Drone Training Pipeline"
echo "=========================================="
echo "Model: $MODEL_NAME"
echo "Training steps: $TRAINING_STEPS"
echo "Eval episodes: $EVAL_EPISODES"
echo ""

# Phase 1: Fast Training
echo "[1/4] Fast Training (Direct Dynamics)..."
./scripts/run_fast_training.sh \
    --model-name "$MODEL_NAME" \
    --steps "$TRAINING_STEPS" \
    --num-drones 16 \
    --output output/models/${MODEL_NAME}_fast.pt

# Phase 2: PX4 Evaluation
echo "[2/4] PX4 Evaluation..."
./scripts/run_evaluation.sh \
    --model output/models/${MODEL_NAME}_fast.pt \
    --episodes "$EVAL_EPISODES" \
    --output output/logs/${MODEL_NAME}_eval.json

# Phase 3: Video Validation
echo "[3/4] Video Validation..."
./scripts/run_test.sh \
    --profile full \
    --model output/models/${MODEL_NAME}_fast.pt \
    --output output/videos/${MODEL_NAME}_validation.mp4

# Phase 4: Metrics Report
echo "[4/4] Generating Report..."
python3 scripts/generate_report.py \
    --training-log output/logs/${MODEL_NAME}_training.json \
    --eval-log output/logs/${MODEL_NAME}_eval.json \
    --video output/videos/${MODEL_NAME}_validation.mp4 \
    --output output/reports/${MODEL_NAME}_report.html

echo ""
echo "=========================================="
echo "Pipeline Complete"
echo "=========================================="
echo "Model: output/models/${MODEL_NAME}_fast.pt"
echo "Video: output/videos/${MODEL_NAME}_validation.mp4"
echo "Report: output/reports/${MODEL_NAME}_report.html"
```

#### 4.2 Policy Evaluation Script

Create `evaluation/scripts/evaluate_policy.py`:

```python
"""
Evaluate trained policy with full PX4 SITL stack.

Validates that policy trained with fast direct dynamics
transfers to full-fidelity simulation.
"""

def evaluate_policy(
    policy_path: str,
    num_episodes: int = 100,
    record_video: bool = True,
) -> Dict[str, Any]:
    """
    Run policy evaluation with PX4 SITL.

    Returns:
        Metrics dict with reward stats, success rate, etc.
    """
    # Load policy
    policy = load_policy(policy_path)

    # Create PX4 environment (full fidelity)
    env_config = EnvironmentConfig(
        headless=True,
        physics_dt=1.0 / 250.0,
        perception=PerceptionConfig(
            resolution_preset="full",
            mode="yolo",  # Real inference, not GT
        )
    )
    env = IsaacSimGymWrapper(PX4BaseISREnvironment(env_config))

    # Run evaluation
    metrics = {
        "rewards": [],
        "episode_lengths": [],
        "success_rate": 0,
        "detections": [],
    }

    for episode in range(num_episodes):
        obs, info = env.reset()
        done = False
        episode_reward = 0

        while not done:
            action = policy(obs)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            episode_reward += reward

        metrics["rewards"].append(episode_reward)
        metrics["episode_lengths"].append(info["step"])

    # Compute aggregate stats
    metrics["mean_reward"] = np.mean(metrics["rewards"])
    metrics["std_reward"] = np.std(metrics["rewards"])

    return metrics
```

---

## Full-Stack Tests Summary

Each phase includes full-stack tests that produce video evidence:

| Phase | Test Script | Video Output | Validates |
|-------|-------------|--------------|-----------|
| 1 | `test_render_skipping.py` | `render_skip_validation.mp4` | 2-4x speedup, detection quality |
| 2 | `test_direct_dynamics_parity.py` | `parity_comparison.mp4` | Dynamics match, policy transfer |
| 3 | `test_multi_drone_throughput.py` | `multi_drone_grid.mp4` | Scaling, independent perception |
| 4 | `test_full_pipeline.py` | `pipeline_validation.mp4` | End-to-end workflow |

### Test Video Requirements

All test videos must include:
- **Header overlay**: Test name, phase, timestamp
- **FPS counter**: Actual simulation throughput
- **Detection overlay**: Bounding boxes with GT comparison
- **Metrics sidebar**: Reward, step count, VRAM usage

---

## Configuration Files

### Training Config (`training/configs/training_config.yaml`)

```yaml
# Fast training configuration
environment:
  type: "multi_drone"
  num_drones: 16
  render_interval: 4

dynamics:
  max_velocity: 10.0
  max_yaw_rate: 90.0
  velocity_damping: 0.95
  position_noise_std: 0.02
  velocity_noise_std: 0.1

perception:
  resolution: [80, 80]
  skip_frames: 3
  use_ground_truth: true

training:
  algorithm: "SAC"
  total_steps: 1_000_000
  batch_size: 4096
  learning_rate: 3e-4
```

### Evaluation Config (`evaluation/configs/evaluation_config.yaml`)

```yaml
# Full-fidelity evaluation configuration
environment:
  type: "px4_sitl"
  lockstep: true

physics:
  dt: 0.004  # 250 Hz

perception:
  resolution: [640, 480]
  mode: "yolo"  # Real inference
  model: "yolo11x.pt"

recording:
  enabled: true
  fps: 30
  include_overlays: true
```

---

## Success Criteria

### Phase 1 Success
- [ ] Training FPS ≥ 120 (was 60)
- [ ] Video shows correct frame skip behavior
- [ ] Detection quality maintained

### Phase 2 Success
- [ ] Training FPS ≥ 2,000
- [ ] Policy trained with direct dynamics scores ≥ 90% of PX4-trained baseline
- [ ] Parity video shows matched behavior

### Phase 3 Success
- [ ] Training FPS ≥ 10,000 with 16 drones
- [ ] VRAM usage < 20 GB
- [ ] Multi-drone video shows independent agents

### Phase 4 Success
- [ ] Full pipeline completes without manual intervention
- [ ] Final policy achieves target reward in PX4 evaluation
- [ ] Validation video demonstrates ISR capabilities

---

## Dependencies and Prerequisites

### New Dependencies
- None required (uses existing Isaac Sim 5.1 APIs)
- Optional: `pynvml` for VRAM monitoring in tests

### Container Updates
- Update `build_container.sh` to include test utilities
- Add VRAM monitoring tools

---

## Risk Mitigation

### Risk: Direct Dynamics Doesn't Transfer
**Mitigation**:
- Extensive domain randomization in training
- Dynamics tuning script to match PX4 response
- Fine-tuning stage with PX4 (slower but short)

### Risk: Multi-Drone VRAM Overflow
**Mitigation**:
- Start with 8 drones, scale up
- Monitor VRAM in tests
- Reduce resolution if needed

### Risk: Tiled Rendering Quality Issues
**Mitigation**:
- Test resolution thresholds
- Compare detection rates between resolutions
- Use full resolution for final evaluation

---

## Timeline Estimate

| Phase | Tasks |
|-------|-------|
| Phase 1 | Add render skipping, resolution config, headless optimization |
| Phase 2 | Direct dynamics, fast env, parity testing |
| Phase 3 | Multi-drone env, tiled cameras, throughput testing |
| Phase 4 | Pipeline integration, final validation |

---

## Files to Create/Modify

### New Files
```
training/
├── environments/
│   ├── __init__.py
│   ├── direct_dynamics.py
│   ├── fast_base_env.py
│   └── fast_multi_drone.py
├── configs/
│   ├── training_config.yaml
│   └── dynamics_config.yaml
├── scripts/
│   ├── train_fast.py
│   ├── train_fast.sh
│   └── tune_dynamics.py
└── tests/
    ├── test_render_skipping.py
    ├── test_direct_dynamics_parity.py
    ├── test_multi_drone_throughput.py
    └── test_full_pipeline.py

evaluation/
├── scripts/
│   ├── evaluate_policy.py
│   └── run_evaluation.sh
└── configs/
    └── evaluation_config.yaml

scripts/
├── run_fast_training.sh
├── run_full_pipeline.sh
└── generate_report.py
```

### Modified Files
```
environments/base_isr_env.py      → evaluation/environments/px4_base_env.py
environments/gymnasium_wrapper.py → evaluation/environments/gymnasium_wrapper.py
environments/perception_manager.py → Shared (imported by both)
```

---

## Next Steps

1. Review this plan and adjust priorities
2. Start with Phase 1 (quick wins, minimal risk)
3. Validate Phase 1 with full-stack test before Phase 2
4. Iterate through phases with validation gates
