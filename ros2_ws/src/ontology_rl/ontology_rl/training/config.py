"""
Training configuration for Stable-Baselines3 RL agents.

Provides default hyperparameters and configuration management for
training RL agents on the F-11 ontology-constrained environments.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Type
import os
import yaml


@dataclass
class TrainingConfig:
    """
    Configuration for RL agent training.

    This class holds all hyperparameters and settings for training
    agents using Stable-Baselines3. Configurations can be loaded from
    YAML files or created programmatically.

    Attributes:
        env_id: Gymnasium environment ID
        algorithm: SB3 algorithm name (PPO, SAC, TD3, etc.)
        total_timesteps: Total training timesteps
        learning_rate: Learning rate
        n_steps: Steps per update (for on-policy algorithms)
        batch_size: Minibatch size
        n_epochs: Epochs per update (for PPO)
        gamma: Discount factor
        gae_lambda: GAE lambda
        clip_range: PPO clip range
        ent_coef: Entropy coefficient
        vf_coef: Value function coefficient
        max_grad_norm: Maximum gradient norm
        policy_kwargs: Additional policy network kwargs
        seed: Random seed
        tensorboard_log: Tensorboard log directory
        verbose: Verbosity level
    """

    # Environment
    env_id: str = 'FlybyMissionPlanner-v0'
    env_kwargs: Dict[str, Any] = field(default_factory=dict)

    # Algorithm
    algorithm: str = 'PPO'

    # Training
    total_timesteps: int = 100000
    n_envs: int = 1  # Number of parallel environments

    # PPO hyperparameters
    learning_rate: float = 3e-4
    n_steps: int = 2048
    batch_size: int = 64
    n_epochs: int = 10
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    clip_range_vf: Optional[float] = None
    ent_coef: float = 0.01
    vf_coef: float = 0.5
    max_grad_norm: float = 0.5

    # SAC/TD3 hyperparameters (for continuous actions)
    buffer_size: int = 100000
    learning_starts: int = 1000
    tau: float = 0.005
    train_freq: int = 1

    # Network architecture
    policy_kwargs: Dict[str, Any] = field(default_factory=lambda: {
        'net_arch': [64, 64]
    })

    # Reproducibility
    seed: Optional[int] = None

    # Logging
    tensorboard_log: Optional[str] = None
    log_interval: int = 10
    verbose: int = 1

    # Checkpointing
    save_freq: int = 10000
    save_path: Optional[str] = None

    # Evaluation
    eval_freq: int = 5000
    n_eval_episodes: int = 10

    @classmethod
    def from_yaml(cls, path: str) -> 'TrainingConfig':
        """Load configuration from YAML file."""
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return cls(**data)

    def to_yaml(self, path: str) -> None:
        """Save configuration to YAML file."""
        data = {
            k: v for k, v in self.__dict__.items()
            if not k.startswith('_')
        }
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def get_algorithm_kwargs(self) -> Dict[str, Any]:
        """Get kwargs for SB3 algorithm constructor."""
        common_kwargs = {
            'learning_rate': self.learning_rate,
            'gamma': self.gamma,
            'verbose': self.verbose,
            'tensorboard_log': self.tensorboard_log,
            'seed': self.seed,
            'policy_kwargs': self.policy_kwargs,
        }

        if self.algorithm == 'PPO':
            return {
                **common_kwargs,
                'n_steps': self.n_steps,
                'batch_size': self.batch_size,
                'n_epochs': self.n_epochs,
                'gae_lambda': self.gae_lambda,
                'clip_range': self.clip_range,
                'clip_range_vf': self.clip_range_vf,
                'ent_coef': self.ent_coef,
                'vf_coef': self.vf_coef,
                'max_grad_norm': self.max_grad_norm,
            }
        elif self.algorithm in ['SAC', 'TD3']:
            return {
                **common_kwargs,
                'buffer_size': self.buffer_size,
                'learning_starts': self.learning_starts,
                'batch_size': self.batch_size,
                'tau': self.tau,
                'train_freq': self.train_freq,
            }
        else:
            return common_kwargs


# Predefined configurations for each environment
MISSION_PLANNER_CONFIG = TrainingConfig(
    env_id='FlybyMissionPlanner-v0',
    algorithm='PPO',
    total_timesteps=100000,
    n_steps=1024,
    batch_size=64,
    learning_rate=1e-4,
    gamma=0.99,
    ent_coef=0.01,
    policy_kwargs={'net_arch': [128, 128]},
)

BEHAVIOR_SELECTOR_CONFIG = TrainingConfig(
    env_id='FlybyBehaviorSelector-v0',
    algorithm='PPO',
    total_timesteps=200000,
    n_steps=512,
    batch_size=32,
    learning_rate=3e-4,
    gamma=0.98,
    ent_coef=0.02,
    policy_kwargs={'net_arch': [64, 64]},
)

TRAJECTORY_OPTIMIZER_CONFIG = TrainingConfig(
    env_id='FlybyTrajectoryOptimizer-v0',
    algorithm='SAC',  # Continuous action space
    total_timesteps=500000,
    buffer_size=100000,
    batch_size=256,
    learning_rate=3e-4,
    gamma=0.99,
    tau=0.005,
    policy_kwargs={'net_arch': [256, 256]},
)
