# Training Guide: Hierarchical RL for Flyby F-11

## Overview

This guide provides a complete workflow for training the three-tier hierarchical RL system for autonomous UAV navigation. Training occurs in stages, with lower-level agents trained first to provide a stable foundation for higher-level decision-making.

## Training Infrastructure

### NPS Cluster Setup
```bash
# SSH to NPS HPC cluster
ssh username@nps-cluster.navy.mil

# Load required modules
module load cuda/11.8
module load python/3.10
module load gazebo/11

# Create virtual environment
python3 -m venv ~/flyby_rl_env
source ~/flyby_rl_env/bin/activate

# Install dependencies
pip install gymnasium stable-baselines3[extra] torch tensorboard
pip install mavsdk airsim opencv-python
```

### PX4 SITL Headless Setup
```bash
# Clone PX4-Autopilot (if not already available)
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.14.3

# Build for SITL
make px4_sitl_default

# Run headless (no GUI) for parallel instances
HEADLESS=1 make px4_sitl_default gazebo
```

### Multi-Instance PX4 for Vectorized Training
```bash
# Script to launch 8 parallel PX4 instances
#!/bin/bash

NUM_INSTANCES=8
BASE_PORT=14540

for i in $(seq 0 $((NUM_INSTANCES-1))); do
    PX4_SIM_INSTANCE=$i \
    PX4_SIM_PORT=$((BASE_PORT + i)) \
    HEADLESS=1 \
    make px4_sitl_default gazebo_iris &
done

wait
```

## Training Stage 1: Trajectory Optimizer (Low-Level)

### Purpose
Train the low-level controller to execute smooth, dynamically feasible velocity commands.

### Algorithm: TD3
- Off-policy continuous control
- Twin critics reduce overestimation
- Delayed policy updates for stability

### Hyperparameters
```python
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
import numpy as np

# Action noise for exploration
n_actions = 3  # [vx, vy, vz]
action_noise = NormalActionNoise(
    mean=np.zeros(n_actions),
    sigma=0.5 * np.ones(n_actions)  # 0.5 m/s noise
)

# TD3 configuration
model = TD3(
    policy="MultiInputPolicy",  # Dict observation space
    env=trajectory_env,
    learning_rate=1e-4,
    buffer_size=1_000_000,
    batch_size=256,
    tau=0.005,  # Soft update coefficient
    gamma=0.99,  # Discount factor
    policy_delay=2,  # Update actor every 2 critic updates
    target_policy_noise=0.2,
    target_noise_clip=0.5,
    action_noise=action_noise,
    tensorboard_log="./logs/trajectory_optimizer/",
    verbose=1,
)
```

### Training Loop
```python
from gymnasium.vector import AsyncVectorEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

# Create vectorized environment (8 parallel instances)
num_envs = 8
env = AsyncVectorEnv([make_trajectory_env(i) for i in range(num_envs)])

# Evaluation environment (single instance)
eval_env = make_trajectory_env(0)

# Callbacks for monitoring
checkpoint_callback = CheckpointCallback(
    save_freq=10_000,
    save_path="./models/trajectory_optimizer/",
    name_prefix="td3_trajectory"
)

eval_callback = EvalCallback(
    eval_env,
    best_model_save_path="./models/trajectory_optimizer/best/",
    log_path="./logs/trajectory_optimizer/eval/",
    eval_freq=5_000,
    deterministic=True,
    render=False
)

# Train for 2M timesteps
model.learn(
    total_timesteps=2_000_000,
    callback=[checkpoint_callback, eval_callback],
    log_interval=10,
    progress_bar=True
)

# Save final model
model.save("./models/trajectory_optimizer/final/td3_trajectory_final")
```

### Evaluation Metrics
```python
def evaluate_trajectory_optimizer(model, eval_env, num_episodes=100):
    """Evaluate trained trajectory optimizer."""
    metrics = {
        "position_error": [],
        "velocity_error": [],
        "jerk": [],
        "collision_rate": 0,
        "success_rate": 0,
    }

    for episode in range(num_episodes):
        obs, info = eval_env.reset()
        done = False
        episode_metrics = []

        while not done:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = eval_env.step(action)
            done = terminated or truncated

            # Collect metrics
            episode_metrics.append({
                "position_error": info.get("position_error", 0),
                "velocity_error": info.get("velocity_error", 0),
                "jerk": info.get("jerk", 0),
            })

        # Aggregate episode metrics
        if info.get("success", False):
            metrics["success_rate"] += 1
        if info.get("collision", False):
            metrics["collision_rate"] += 1

        metrics["position_error"].extend([m["position_error"] for m in episode_metrics])
        metrics["velocity_error"].extend([m["velocity_error"] for m in episode_metrics])
        metrics["jerk"].extend([m["jerk"] for m in episode_metrics])

    # Calculate statistics
    metrics["success_rate"] /= num_episodes
    metrics["collision_rate"] /= num_episodes
    metrics["mean_position_error"] = np.mean(metrics["position_error"])
    metrics["mean_velocity_error"] = np.mean(metrics["velocity_error"])
    metrics["mean_jerk"] = np.mean(metrics["jerk"])

    return metrics
```

## Training Stage 2: Behavior Selector (Mid-Level)

### Purpose
Train behavior primitive selection using the trained trajectory optimizer.

### Algorithm: SAC
- Off-policy with entropy regularization
- Encourages exploration of behavior space
- Automatic temperature tuning

### Integration with Trajectory Optimizer
```python
# Load trained trajectory optimizer
trajectory_model = TD3.load("./models/trajectory_optimizer/best/best_model.zip")

# Freeze trajectory optimizer (use as black box)
trajectory_model.policy.eval()  # Set to evaluation mode

# Wrap in environment
class BehaviorSelectorEnvWithTrajectory(BehaviorSelectorEnv):
    """Behavior selector that uses trained trajectory optimizer."""

    def __init__(self, trajectory_model, **kwargs):
        super().__init__(**kwargs)
        self.trajectory_model = trajectory_model
        self.trajectory_steps = 10  # Execute behavior for 1 second (10 Hz)

    def step(self, action):
        """Execute behavior primitive using trajectory optimizer."""
        # Map discrete behavior to target velocity/position
        target_vel, target_pos = self._behavior_to_target(action)

        # Execute trajectory optimizer for multiple steps
        cumulative_reward = 0.0
        for _ in range(self.trajectory_steps):
            # Get low-level action from trajectory optimizer
            traj_obs = self._get_trajectory_observation(target_vel, target_pos)
            low_level_action, _ = self.trajectory_model.predict(traj_obs, deterministic=True)

            # Execute in environment
            obs, reward, terminated, truncated, info = self._execute_low_level_action(low_level_action)
            cumulative_reward += reward

            if terminated or truncated:
                break

        # Return behavior-level observation and reward
        behavior_obs = self._get_behavior_observation()
        behavior_reward = self._compute_behavior_reward(cumulative_reward, info)

        return behavior_obs, behavior_reward, terminated, truncated, info
```

### Hyperparameters
```python
from stable_baselines3 import SAC

model = SAC(
    policy="MultiInputPolicy",
    env=behavior_env,
    learning_rate=3e-4,
    buffer_size=1_000_000,
    batch_size=256,
    tau=0.005,
    gamma=0.99,
    ent_coef="auto",  # Automatic entropy tuning
    target_entropy="auto",
    use_sde=True,  # State-dependent exploration
    sde_sample_freq=4,
    tensorboard_log="./logs/behavior_selector/",
    verbose=1,
)
```

### Training Loop
```python
# Create vectorized environment with trajectory optimizer
num_envs = 8
env = AsyncVectorEnv([
    make_behavior_env_with_trajectory(i, trajectory_model)
    for i in range(num_envs)
])

# Evaluation environment
eval_env = make_behavior_env_with_trajectory(0, trajectory_model)

# Callbacks
checkpoint_callback = CheckpointCallback(
    save_freq=10_000,
    save_path="./models/behavior_selector/",
    name_prefix="sac_behavior"
)

eval_callback = EvalCallback(
    eval_env,
    best_model_save_path="./models/behavior_selector/best/",
    log_path="./logs/behavior_selector/eval/",
    eval_freq=5_000,
    deterministic=True,
    render=False
)

# Train for 3M timesteps (behaviors are longer horizon)
model.learn(
    total_timesteps=3_000_000,
    callback=[checkpoint_callback, eval_callback],
    log_interval=10,
    progress_bar=True
)

# Save final model
model.save("./models/behavior_selector/final/sac_behavior_final")
```

## Training Stage 3: Mission Planner (High-Level)

### Purpose
Train mission-level decision-making using trained behavior selector and trajectory optimizer.

### Algorithm: PPO
- On-policy algorithm
- Stable training for mixed action spaces
- Suitable for episodic mission tasks

### Integration with Lower Levels
```python
class MissionPlannerEnvWithHierarchy(MissionPlannerEnv):
    """Mission planner that uses trained behavior selector and trajectory optimizer."""

    def __init__(self, behavior_model, trajectory_model, **kwargs):
        super().__init__(**kwargs)
        self.behavior_model = behavior_model
        self.trajectory_model = trajectory_model
        self.mission_steps = 100  # Execute mission phase for ~10 seconds

    def step(self, action):
        """Execute mission-level action using behavior hierarchy."""
        # Decode mission action (phase transition, waypoint, behavior mode)
        next_phase = action["next_phase"]
        waypoint_offset = action["waypoint_offset"]
        behavior_mode = action["behavior_mode"]

        # Execute mission phase
        cumulative_reward = 0.0
        for _ in range(self.mission_steps):
            # Get behavior-level action from behavior selector
            behavior_obs = self._get_behavior_observation(next_phase, waypoint_offset, behavior_mode)
            behavior_action, _ = self.behavior_model.predict(behavior_obs, deterministic=True)

            # Behavior selector internally uses trajectory optimizer
            obs, reward, terminated, truncated, info = self._execute_behavior(behavior_action)
            cumulative_reward += reward

            if terminated or truncated:
                break

        # Return mission-level observation and reward
        mission_obs = self._get_mission_observation()
        mission_reward = self._compute_mission_reward(cumulative_reward, info)

        return mission_obs, mission_reward, terminated, truncated, info
```

### Hyperparameters
```python
from stable_baselines3 import PPO

model = PPO(
    policy="MultiInputPolicy",
    env=mission_env,
    learning_rate=3e-4,
    n_steps=2048,  # Collect 2048 steps per environment before update
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.01,  # Encourage exploration
    vf_coef=0.5,
    max_grad_norm=0.5,
    use_sde=False,  # Not needed for mission-level
    tensorboard_log="./logs/mission_planner/",
    verbose=1,
)
```

### Training Loop
```python
# Create vectorized environment with full hierarchy
num_envs = 4  # Fewer envs due to longer episodes
env = AsyncVectorEnv([
    make_mission_env_with_hierarchy(i, behavior_model, trajectory_model)
    for i in range(num_envs)
])

# Evaluation environment
eval_env = make_mission_env_with_hierarchy(0, behavior_model, trajectory_model)

# Callbacks
checkpoint_callback = CheckpointCallback(
    save_freq=20_000,
    save_path="./models/mission_planner/",
    name_prefix="ppo_mission"
)

eval_callback = EvalCallback(
    eval_env,
    best_model_save_path="./models/mission_planner/best/",
    log_path="./logs/mission_planner/eval/",
    eval_freq=10_000,
    deterministic=True,
    render=False
)

# Train for 5M timesteps (missions are long episodes)
model.learn(
    total_timesteps=5_000_000,
    callback=[checkpoint_callback, eval_callback],
    log_interval=10,
    progress_bar=True
)

# Save final model
model.save("./models/mission_planner/final/ppo_mission_final")
```

## Curriculum Learning

### Progressive Difficulty
```python
class CurriculumTrainingCallback:
    """Gradually increase task difficulty during training."""

    def __init__(self, env, difficulty_schedule):
        self.env = env
        self.difficulty_schedule = difficulty_schedule
        self.current_step = 0

    def on_step(self):
        """Update environment difficulty based on training progress."""
        for threshold, difficulty in self.difficulty_schedule:
            if self.current_step >= threshold:
                self.env.set_difficulty(difficulty)

        self.current_step += 1

# Example difficulty schedule
difficulty_schedule = [
    (0, "easy"),          # Start with simple scenarios
    (500_000, "medium"),  # Increase complexity
    (1_500_000, "hard"),  # Full difficulty
]

# Apply to trajectory optimizer training
curriculum_callback = CurriculumTrainingCallback(env, difficulty_schedule)
```

### Difficulty Parameters
```python
class TrajectoryOptimizerEnv(BaseUAVEnv):
    def set_difficulty(self, difficulty):
        """Adjust environment difficulty."""
        if difficulty == "easy":
            self.wind_speed = 0.0
            self.obstacle_density = 0.1
            self.target_tolerance = 2.0  # 2m tolerance
        elif difficulty == "medium":
            self.wind_speed = 5.0
            self.obstacle_density = 0.3
            self.target_tolerance = 1.0
        elif difficulty == "hard":
            self.wind_speed = 10.0
            self.obstacle_density = 0.5
            self.target_tolerance = 0.5
```

## Hyperparameter Tuning

### Optuna Integration
```python
import optuna
from optuna.pruners import MedianPruner
from optuna.samplers import TPESampler

def objective(trial):
    """Objective function for hyperparameter optimization."""
    # Suggest hyperparameters
    learning_rate = trial.suggest_float("learning_rate", 1e-5, 1e-3, log=True)
    gamma = trial.suggest_float("gamma", 0.95, 0.9999)
    batch_size = trial.suggest_categorical("batch_size", [128, 256, 512])
    tau = trial.suggest_float("tau", 0.001, 0.02)

    # Create model with suggested hyperparameters
    model = TD3(
        policy="MultiInputPolicy",
        env=env,
        learning_rate=learning_rate,
        gamma=gamma,
        batch_size=batch_size,
        tau=tau,
        verbose=0,
    )

    # Train for short duration
    model.learn(total_timesteps=100_000)

    # Evaluate
    mean_reward, _ = evaluate_policy(model, eval_env, n_eval_episodes=10)

    return mean_reward

# Run optimization
study = optuna.create_study(
    direction="maximize",
    sampler=TPESampler(),
    pruner=MedianPruner()
)
study.optimize(objective, n_trials=50, n_jobs=4)

print(f"Best hyperparameters: {study.best_params}")
```

## Monitoring Training Progress

### TensorBoard
```bash
# Launch TensorBoard
tensorboard --logdir ./logs/

# Access at http://localhost:6006
```

### Key Metrics to Monitor
1. **Trajectory Optimizer**:
   - Episode reward
   - Position error
   - Velocity error
   - Jerk (smoothness)
   - Collision rate

2. **Behavior Selector**:
   - Episode reward
   - Waypoint success rate
   - Obstacle avoidance success
   - Behavior diversity (entropy)

3. **Mission Planner**:
   - Mission success rate
   - Mission duration
   - Battery efficiency
   - Phase transition quality

## Model Export for Deployment

### Convert to ONNX
```python
import torch

# Load trained model
model = TD3.load("./models/trajectory_optimizer/best/best_model.zip")

# Extract policy network
policy = model.policy

# Create dummy input
dummy_input = {
    "position": torch.randn(1, 3),
    "velocity": torch.randn(1, 3),
    "acceleration": torch.randn(1, 3),
    # ... other observation components
}

# Export to ONNX
torch.onnx.export(
    policy,
    dummy_input,
    "./models/deployment/trajectory_optimizer.onnx",
    opset_version=11,
    input_names=list(dummy_input.keys()),
    output_names=["action"],
    dynamic_axes={k: {0: "batch_size"} for k in dummy_input.keys()}
)
```

### Optimize for Jetson
```python
import tensorrt as trt

# Convert ONNX to TensorRT engine
def build_engine(onnx_path, engine_path):
    """Build TensorRT engine from ONNX model."""
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)

    # Parse ONNX
    with open(onnx_path, 'rb') as model:
        if not parser.parse(model.read()):
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            return None

    # Build engine
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB
    config.set_flag(trt.BuilderFlag.FP16)  # Enable FP16 for Jetson

    engine = builder.build_serialized_network(network, config)

    # Save engine
    with open(engine_path, 'wb') as f:
        f.write(engine)

    return engine

# Build engines for all three agents
build_engine("./models/deployment/trajectory_optimizer.onnx", "./models/deployment/trajectory_optimizer.trt")
build_engine("./models/deployment/behavior_selector.onnx", "./models/deployment/behavior_selector.trt")
build_engine("./models/deployment/mission_planner.onnx", "./models/deployment/mission_planner.trt")
```

## Troubleshooting

### Common Issues

1. **Training Instability**:
   - Reduce learning rate
   - Increase batch size
   - Add gradient clipping
   - Check reward scaling

2. **Poor Exploration**:
   - Increase entropy coefficient
   - Add action noise
   - Use State-Dependent Exploration (SDE)

3. **Slow Convergence**:
   - Increase number of parallel environments
   - Tune discount factor (gamma)
   - Implement curriculum learning

4. **Simulation Crashes**:
   - Reduce number of parallel PX4 instances
   - Increase system resources
   - Check PX4 SITL port conflicts

## Next Steps

1. **Begin with Trajectory Optimizer**: Train low-level controller first
2. **Validate in Simulation**: Test extensively in PX4 SITL
3. **Progress to Hierarchy**: Add behavior selector once trajectory optimizer is stable
4. **Full System Training**: Train mission planner with complete hierarchy
5. **Deploy to Hardware**: Export models and integrate with BehaviorTree.CPP

## References

- PPO Reference: `stable-baselines3/ppo_reference.md`
- SAC Reference: `stable-baselines3/sac_reference.md`
- TD3 Reference: `stable-baselines3/td3_reference.md`
- Custom Environments: `CUSTOM_ENVIRONMENTS.md`
