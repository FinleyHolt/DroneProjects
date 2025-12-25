# PPO Algorithm Documentation Summary

## Overview
Proximal Policy Optimization (PPO) is an on-policy reinforcement learning algorithm that "combines ideas from A2C (having multiple workers) and TRPO (it uses a trust region to improve the actor)." The core innovation involves using clipping to prevent overly large policy updates.

## Key Implementation Details

**Main Features:**
- Advantage normalization (automatic)
- Optional value function clipping
- Trust region enforcement through clipping mechanism
- Support for gSDE (generalized State-Dependent Exploration)

## When to Use PPO

**Supported Environments:**
- Discrete action/observation spaces ✔️
- Continuous action/observation spaces ✔️
- MultiDiscrete and MultiBinary spaces ✔️
- Dict observation spaces ✗

**Processing:**
- Multi-processing capable
- Primarily CPU-optimized (especially for non-CNN policies)
- Recurrent policies available in contrib library (though frame-stacking often performs comparably)

## Core Hyperparameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `learning_rate` | 0.0003 | Can be constant or scheduled |
| `n_steps` | 2048 | Steps per environment per update |
| `batch_size` | 64 | Minibatch size |
| `n_epochs` | 10 | Optimization epochs per rollout |
| `gamma` | 0.99 | Discount factor |
| `gae_lambda` | 0.95 | Bias-variance tradeoff for GAE |
| `clip_range` | 0.2 | Clipping parameter |
| `clip_range_vf` | None | Optional value function clipping |
| `ent_coef` | 0.0 | Entropy regularization |
| `vf_coef` | 0.5 | Value function loss weight |
| `max_grad_norm` | 0.5 | Gradient clipping magnitude |
| `use_sde` | False | State-dependent exploration |
| `target_kl` | None | KL divergence limit |

## Policy Network Options

1. **MlpPolicy**: Fully connected networks for vector observations
2. **CnnPolicy**: CNN architecture for image-based observations
3. **MultiInputPolicy**: Handles Dict observation spaces with combined feature extraction

All policies implement both actor and critic networks for value estimation.

## Usage Example

```python
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

vec_env = make_vec_env("CartPole-v1", n_envs=4)
model = PPO("MlpPolicy", vec_env, verbose=1)
model.learn(total_timesteps=25000)
model.save("ppo_cartpole")
```

## Performance Benchmarks

PPO demonstrates strong results across domains:
- **PyBullet (2M steps)**: Competitive with A2C, particularly strong with gSDE variant
- **Atari**: Comprehensive benchmark results available in PR #110

## Critical Notes

**gSDE Inference Behavior**: When using models trained with `use_sde=True`, noise resetting that occurs during training doesn't apply during inference with `model.predict()`, resulting in deterministic behavior even when `deterministic=False`. Manual noise reset via `model.policy.reset_noise()` is needed for stochastic inference behavior.

**Computational Efficiency**: For optimal CPU utilization, disable GPU and use `SubprocVecEnv` instead of the default `DummyVecEnv` with multiple parallel environments.

## Core Methods

- `learn()`: Train the model
- `predict()`: Get actions from observations
- `save()`/`load()`: Model persistence
- `set_parameters()`/`get_parameters()`: Weight manipulation
- `collect_rollouts()`: Experience gathering

## References

- Original paper: https://arxiv.org/abs/1707.06347
- Spinning Up guide: https://spinningup.openai.com/en/latest/algorithms/ppo.html
- 37 implementation details: https://iclr-blog-track.github.io/2022/03/25/ppo-implementation-details/
