# SAC (Soft Actor-Critic) Algorithm Documentation Summary

## Overview
SAC is an off-policy deep reinforcement learning algorithm that maximizes a trade-off between expected return and entropy. It's particularly effective for continuous control tasks and serves as the successor to Soft Q-Learning.

## Key Characteristics

**Core Innovation:** The algorithm balances reward maximization with policy entropy, encouraging exploration through randomness in action selection.

**Continuous Action Support:** SAC explicitly supports Box action spaces, making it ideal for continuous control problems like robotics and physics simulations.

**Exploration Method:** Uses stochastic policies by default, with optional generalized State-Dependent Exploration (gSDE) for enhanced exploration flexibility.

## Implementation Highlights

**Learning Setup:**
- Default learning rate: 0.0003
- Replay buffer size: 1,000,000
- Batch size: 256
- Soft update coefficient (tau): 0.005
- Discount factor (gamma): 0.99

**Entropy Regularization:** The algorithm uses an entropy coefficient that can be automatically tuned, described as "the equivalent to the inverse of reward scale in the original SAC paper." This helps balance exploration-exploitation trade-offs adaptively.

**Architecture Details:**
- Employs double Q-learning (two critic networks)
- Uses ReLU activation instead of tanh (differing from typical MLP policies)
- Supports multiple policy types: MlpPolicy, CnnPolicy, and MultiInputPolicy

## Practical Usage Example

```python
import gymnasium as gym
from stable_baselines3 import SAC

env = gym.make("Pendulum-v1")
model = SAC("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10000)
model.save("sac_pendulum")
```

## Limitations

- **Recurrent policies:** Not supported
- **Discrete actions:** Cannot be used with discrete action spaces
- **Inference behavior:** When using gSDE during training, `model.predict()` requires manual noise reset for stochastic inference behavior

## Performance Benchmark
On PyBullet environments (1M steps), SAC with gSDE achieved competitive results: HalfCheetah (2984±202), Ant (3102±37), Hopper (2262±1), and Walker2D (2136±67).
