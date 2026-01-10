"""
Stable-Baselines3 agent implementations for Flyby F-11 ISR training.

Implements three agent types for hierarchical RL:
- SAC (mission_planner): High-level waypoint selection
- PPO (behavior_selector): Mid-level behavior selection
- TD3 (trajectory_optimizer): Low-level trajectory control
"""
from typing import Dict, Any, Optional


def create_sac_agent(
    env,
    config: Dict[str, Any],
    tensorboard_log: Optional[str] = None
):
    """
    Create SAC agent for mission planning (Level 1).

    SAC is chosen for mission planning because:
    - Sample efficient off-policy learning
    - Automatic entropy tuning for exploration
    - Stable learning for sparse rewards

    Args:
        env: Gymnasium environment
        config: Agent configuration from training_config.yaml
        tensorboard_log: Path for tensorboard logs

    Returns:
        Configured SAC agent
    """
    from stable_baselines3 import SAC
    import torch

    policy_kwargs = dict(
        net_arch=config.get('hidden_dims', [256, 256, 128]),
        activation_fn=torch.nn.ReLU
    )

    agent = SAC(
        policy="MlpPolicy",
        env=env,
        learning_rate=config.get('learning_rate', 3e-4),
        buffer_size=config.get('buffer_size', 1_000_000),
        learning_starts=config.get('learning_starts', 10_000),
        batch_size=config.get('batch_size', 256),
        tau=config.get('tau', 0.005),
        gamma=config.get('gamma', 0.99),
        train_freq=(1, "step"),
        gradient_steps=config.get('update_frequency', 1),
        policy_kwargs=policy_kwargs,
        tensorboard_log=tensorboard_log,
        verbose=1,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )

    return agent


def create_ppo_agent(
    env,
    config: Dict[str, Any],
    tensorboard_log: Optional[str] = None
):
    """
    Create PPO agent for behavior selection (Level 2).

    PPO is chosen for behavior selection because:
    - Stable on-policy learning
    - Good for discrete/hybrid action spaces
    - Robust to hyperparameter choices

    Args:
        env: Gymnasium environment
        config: Agent configuration
        tensorboard_log: Path for tensorboard logs

    Returns:
        Configured PPO agent
    """
    from stable_baselines3 import PPO
    import torch

    policy_kwargs = dict(
        net_arch=config.get('hidden_dims', [128, 128]),
        activation_fn=torch.nn.ReLU
    )

    return PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=config.get('learning_rate', 3e-4),
        n_steps=config.get('n_steps', 2048),
        batch_size=config.get('batch_size', 64),
        n_epochs=config.get('n_epochs', 10),
        gamma=config.get('gamma', 0.99),
        gae_lambda=config.get('gae_lambda', 0.95),
        clip_range=config.get('clip_range', 0.2),
        ent_coef=config.get('ent_coef', 0.01),
        vf_coef=config.get('vf_coef', 0.5),
        max_grad_norm=config.get('max_grad_norm', 0.5),
        policy_kwargs=policy_kwargs,
        tensorboard_log=tensorboard_log,
        verbose=1,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )


def create_td3_agent(
    env,
    config: Dict[str, Any],
    tensorboard_log: Optional[str] = None
):
    """
    Create TD3 agent for trajectory optimization (Level 3).

    TD3 is chosen for trajectory optimization because:
    - High-frequency control requires low-latency inference
    - Deterministic policy suitable for continuous control
    - Twin critics reduce overestimation

    Args:
        env: Gymnasium environment
        config: Agent configuration
        tensorboard_log: Path for tensorboard logs

    Returns:
        Configured TD3 agent
    """
    from stable_baselines3 import TD3
    import torch

    policy_kwargs = dict(
        net_arch=config.get('hidden_dims', [64, 64]),
        activation_fn=torch.nn.ReLU
    )

    return TD3(
        policy="MlpPolicy",
        env=env,
        learning_rate=config.get('learning_rate', 1e-3),
        buffer_size=config.get('buffer_size', 100_000),
        learning_starts=config.get('learning_starts', 1000),
        batch_size=config.get('batch_size', 100),
        tau=config.get('tau', 0.005),
        gamma=config.get('gamma', 0.99),
        train_freq=(1, "step"),
        gradient_steps=config.get('gradient_steps', 1),
        policy_delay=config.get('policy_delay', 2),
        target_policy_noise=config.get('target_noise', 0.2),
        target_noise_clip=config.get('noise_clip', 0.5),
        policy_kwargs=policy_kwargs,
        tensorboard_log=tensorboard_log,
        verbose=1,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )


def load_agent(agent_type: str, env, checkpoint_path: str):
    """
    Load a pre-trained agent from checkpoint.

    Args:
        agent_type: One of 'sac', 'ppo', 'td3'
        env: Gymnasium environment
        checkpoint_path: Path to saved model

    Returns:
        Loaded agent
    """
    from stable_baselines3 import SAC, PPO, TD3

    agent_classes = {
        'sac': SAC,
        'ppo': PPO,
        'td3': TD3
    }

    if agent_type not in agent_classes:
        raise ValueError(f"Unknown agent type: {agent_type}")

    return agent_classes[agent_type].load(checkpoint_path, env=env)
