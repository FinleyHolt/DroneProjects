# TD3 Algorithm Documentation Summary

## Overview
TD3 (Twin Delayed DDPG) is an advanced reinforcement learning algorithm designed to address function approximation errors in actor-critic methods. It builds upon DDPG by implementing three key improvements.

## Core Improvements Over DDPG

TD3 enhances its predecessor through:
1. **Clipped Double Q-Learning** - Uses twin critic networks to reduce overestimation
2. **Delayed Policy Updates** - Updates the actor less frequently than critics
3. **Target Policy Smoothing** - Adds noise to target actions for stability

## Key Hyperparameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `learning_rate` | 0.001 | Optimizer learning rate (supports schedules) |
| `buffer_size` | 1,000,000 | Replay buffer capacity |
| `batch_size` | 256 | Gradient update batch size |
| `tau` | 0.005 | Soft update coefficient (Polyak averaging) |
| `gamma` | 0.99 | Discount factor |
| `policy_delay` | 2 | Actor updates every N training steps |
| `target_policy_noise` | 0.2 | Gaussian noise standard deviation |
| `target_noise_clip` | 0.5 | Noise absolute value limit |

## Supported Environments

**Supported action spaces:** Box (continuous)
**Supported observation spaces:** Box, Discrete, MultiDiscrete, MultiBinary, Dict
**Limitations:** No recurrent or discrete action support; multi-processing enabled

## Available Policies

- `MlpPolicy` - Fully connected networks for continuous control
- `CnnPolicy` - Convolutional networks for image inputs
- `MultiInputPolicy` - Handles dictionary observation spaces

## Usage Example

```python
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
import numpy as np

env = gym.make("Pendulum-v1")
action_noise = NormalActionNoise(mean=np.zeros(1), sigma=0.1)
model = TD3("MlpPolicy", env, action_noise=action_noise)
model.learn(total_timesteps=10000)
```

## Implementation Details

**Activation Function:** ReLU (differs from other algorithms' tanh, matching the original paper)

**Training Configuration:**
- Supports both step-based and episode-based training frequencies
- Gradient steps adjustable per rollout (`gradient_steps` parameter)
- Includes memory-efficient replay buffer variant

## Performance Results

On PyBullet benchmarks (1M steps, 3 seeds), TD3 demonstrates competitive performance across locomotion tasks, with hyperparameters from the gSDE paper optimized for robotic environments.

## Additional Features

- Replay buffer save/load functionality
- TensorBoard logging integration
- Custom policy network configuration
- Deterministic and stochastic action prediction modes
