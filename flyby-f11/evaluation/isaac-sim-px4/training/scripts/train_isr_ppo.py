#!/usr/bin/env python3
"""
ISR Training with PPO - Production Training Script

Trains policies for ISR drone missions using Proximal Policy Optimization
with Stable-Baselines3. Supports three scenarios:
1. area_coverage - Basic navigation and coverage
2. nfz_avoidance - Transit with dynamic no-fly zones
3. multi_objective - Multi-priority ISR with threat zones

Usage:
    # Basic training
    python training/scripts/train_isr_ppo.py --scenario area_coverage

    # With custom settings
    python training/scripts/train_isr_ppo.py --scenario multi_objective --num_envs 512

    # Resume from checkpoint
    python training/scripts/train_isr_ppo.py --scenario area_coverage --checkpoint checkpoints/model.zip

    # Curriculum learning
    python training/scripts/train_isr_ppo.py --curriculum

Requirements:
    - Isaac Sim 5.1 with Isaac Lab
    - stable-baselines3 >= 2.0
    - PyTorch with CUDA

Author: Finley Holt
"""

import argparse
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, Optional

import yaml
import numpy as np
import torch

# Add parent directories to path
SCRIPT_DIR = Path(__file__).parent
TRAINING_DIR = SCRIPT_DIR.parent
ISAAC_SIM_PX4_DIR = TRAINING_DIR.parent
sys.path.insert(0, str(ISAAC_SIM_PX4_DIR))

from training.environments import (
    ISRTrainingEnv,
    ISRTrainingConfig,
    WorldConfig,
    MissionConfig,
    NFZConfig,
    ThreatConfig,
    ScenarioType,
    CameraConfig,
    TargetConfig,
    DroneConfig,
)
from training.wrappers import ISRVecEnvWrapper
from training.callbacks import ISRMetricsCallback, CurriculumCallback


def linear_schedule(initial_value: float) -> Callable[[float], float]:
    """
    Create a linear learning rate schedule.

    Args:
        initial_value: Initial learning rate

    Returns:
        Function that takes progress_remaining (1.0 -> 0.0) and returns lr
    """
    def func(progress_remaining: float) -> float:
        return progress_remaining * initial_value
    return func


def load_scenario_config(scenario: str, config_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load scenario configuration from YAML file.

    Args:
        scenario: Scenario name (area_coverage, nfz_avoidance, multi_objective)
        config_path: Optional path to config file

    Returns:
        Dictionary with scenario configuration
    """
    if config_path is None:
        config_path = TRAINING_DIR / "configs" / "scenario_configs.yaml"

    with open(config_path, 'r') as f:
        all_configs = yaml.safe_load(f)

    if scenario not in all_configs:
        available = [k for k in all_configs.keys() if k != "curriculum"]
        raise ValueError(f"Unknown scenario: {scenario}. Available: {available}")

    return all_configs[scenario]


def build_env_config(
    scenario_config: Dict[str, Any],
    num_envs: int,
    device: str
) -> ISRTrainingConfig:
    """
    Build ISRTrainingConfig from scenario configuration.

    Args:
        scenario_config: Dictionary with scenario settings
        num_envs: Number of parallel environments
        device: Device string (e.g., "cuda:0")

    Returns:
        ISRTrainingConfig instance
    """
    world_cfg = scenario_config.get('world', {})
    mission_cfg = scenario_config.get('mission', {})
    targets_cfg = scenario_config.get('targets', {})
    camera_cfg = scenario_config.get('camera', {})
    nfz_cfg = scenario_config.get('nfz', {})
    threat_cfg = scenario_config.get('threat', {})

    # Parse scenario type
    scenario_str = mission_cfg.get('scenario', 'area_coverage')
    scenario_type = ScenarioType(scenario_str)

    # Build config objects
    world = WorldConfig(
        bounds=tuple(world_cfg.get('bounds', [-250, 250, -250, 250, 0, 120])),
        coverage_resolution=world_cfg.get('coverage_resolution', 10.0),
        launch_position=tuple(world_cfg.get('launch_position', [0, 0, 0])),
    )

    mission = MissionConfig(
        scenario=scenario_type,
        max_episode_steps=mission_cfg.get('max_episode_steps', 37500),
        target_coverage_pct=mission_cfg.get('target_coverage_pct', 85.0),
        initial_battery_pct=mission_cfg.get('initial_battery_pct', 100.0),
        battery_reserve_pct=mission_cfg.get('battery_reserve_pct', 25.0),
        battery_drain_rate=mission_cfg.get('battery_drain_rate', 0.001),
    )

    camera = CameraConfig(
        optimal_altitude=camera_cfg.get('optimal_altitude', 50.0),
        optimal_range=camera_cfg.get('optimal_range', 50.0),
    )

    targets = TargetConfig(
        max_targets=targets_cfg.get('max_targets', 16),
        detection_reward=targets_cfg.get('detection_reward', 10.0),
        tracking_reward=targets_cfg.get('tracking_reward', 0.1),
    )

    nfz = NFZConfig(
        enabled=nfz_cfg.get('enabled', False),
        max_nfz=nfz_cfg.get('max_nfz', 10),
        nfz_violation_reward=nfz_cfg.get('nfz_violation_reward', -100.0),
        nfz_buffer_reward=nfz_cfg.get('nfz_buffer_reward', -0.1),
        nfz_buffer_distance=nfz_cfg.get('nfz_buffer_distance', 50.0),
    )

    threat = ThreatConfig(
        enabled=threat_cfg.get('enabled', False),
        max_threat_zones=threat_cfg.get('max_threat_zones', 5),
        max_medium_exposure=threat_cfg.get('max_medium_exposure', 30.0),
        high_threat_terminates=threat_cfg.get('high_threat_terminates', True),
        medium_threat_rate=threat_cfg.get('medium_threat_rate', -0.1),
        high_threat_reward=threat_cfg.get('high_threat_reward', -200.0),
    )

    return ISRTrainingConfig(
        num_envs=num_envs,
        device=device,
        world=world,
        mission=mission,
        camera=camera,
        targets=targets,
        nfz=nfz,
        threat=threat,
    )


def create_eval_env(
    scenario: str,
    num_envs: int = 16,
    device: str = "cuda:0"
) -> ISRVecEnvWrapper:
    """
    Create evaluation environment with deterministic settings.

    Args:
        scenario: Scenario name
        num_envs: Number of parallel eval environments
        device: Device string

    Returns:
        Wrapped evaluation environment
    """
    config = load_scenario_config(scenario)

    # Reduce noise for evaluation
    config['world']['position_noise_std'] = 0.0
    config['world']['velocity_noise_std'] = 0.0
    config['world']['wind_enabled'] = False

    env_config = build_env_config(config, num_envs, device)
    env_config.base_seed = 12345  # Fixed seed for reproducibility

    eval_env = ISRTrainingEnv(env_config)
    eval_env.setup()

    return ISRVecEnvWrapper(eval_env)


def create_run_name(scenario: str, suffix: str = "") -> str:
    """Create a unique run name for logging."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if suffix:
        return f"{scenario}_{suffix}_{timestamp}"
    return f"{scenario}_{timestamp}"


def train_ppo(
    scenario: str,
    num_envs: int = 256,
    device: str = "cuda:0",
    checkpoint: Optional[str] = None,
    total_timesteps: Optional[int] = None,
    save_path: str = "./checkpoints",
    log_path: str = "./logs",
    seed: int = 42,
    verbose: int = 1,
) -> None:
    """
    Train PPO agent on ISR scenario.

    Args:
        scenario: Scenario name
        num_envs: Number of parallel environments
        device: Training device
        checkpoint: Optional checkpoint to resume from
        total_timesteps: Override total timesteps from config
        save_path: Directory to save checkpoints
        log_path: Directory for TensorBoard logs
        seed: Random seed
        verbose: Verbosity level
    """
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import (
        CallbackList,
        CheckpointCallback,
    )
    from stable_baselines3.common.vec_env import VecNormalize

    # Set random seeds
    torch.manual_seed(seed)
    np.random.seed(seed)

    # Load config
    print(f"[TRAIN] Loading scenario config: {scenario}")
    scenario_config = load_scenario_config(scenario)
    training_cfg = scenario_config.get('training', {})

    # Create environment
    print(f"[TRAIN] Creating environment with {num_envs} parallel instances...")
    env_config = build_env_config(scenario_config, num_envs, device)
    env_config.base_seed = seed

    isr_env = ISRTrainingEnv(env_config)
    isr_env.setup()

    # Wrap for SB3
    vec_env = ISRVecEnvWrapper(isr_env)

    # Optional: Add observation normalization
    # vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=False)

    # Create run name and paths
    run_name = create_run_name(scenario)
    checkpoint_dir = Path(save_path) / scenario
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    # Policy network architecture
    policy_kwargs = dict(
        net_arch=dict(
            pi=[256, 256],  # Actor network
            vf=[256, 256],  # Critic network
        ),
        activation_fn=torch.nn.ReLU,
    )

    # Create or load model
    if checkpoint:
        print(f"[TRAIN] Loading checkpoint: {checkpoint}")
        model = PPO.load(checkpoint, env=vec_env, device=device)
    else:
        print(f"[TRAIN] Creating new PPO model...")

        # Calculate n_steps per env
        # SB3's n_steps is per environment, total batch = n_steps * num_envs
        batch_size = training_cfg.get('batch_size', 8192)
        n_steps = max(batch_size // num_envs, 128)  # At least 128 steps per env

        model = PPO(
            policy="MlpPolicy",
            env=vec_env,
            n_steps=n_steps,
            batch_size=training_cfg.get('minibatch_size', 1024),
            n_epochs=training_cfg.get('n_epochs', 10),
            learning_rate=linear_schedule(training_cfg.get('learning_rate', 3e-4)),
            gamma=training_cfg.get('gamma', 0.99),
            gae_lambda=training_cfg.get('gae_lambda', 0.95),
            clip_range=training_cfg.get('clip_range', 0.2),
            ent_coef=training_cfg.get('entropy_coef', 0.01),
            vf_coef=training_cfg.get('value_loss_coef', 0.5),
            max_grad_norm=training_cfg.get('max_grad_norm', 0.5),
            tensorboard_log=log_path,
            policy_kwargs=policy_kwargs,
            device=device,
            verbose=verbose,
            seed=seed,
        )

    # Setup callbacks
    callbacks = []

    # Checkpoint callback
    save_freq = training_cfg.get('save_freq', 1_000_000) // num_envs
    callbacks.append(
        CheckpointCallback(
            save_freq=save_freq,
            save_path=str(checkpoint_dir),
            name_prefix=scenario,
            save_replay_buffer=False,
            save_vecnormalize=False,
        )
    )

    # ISR metrics callback
    callbacks.append(
        ISRMetricsCallback(
            log_freq=10000,
            success_window=100,
            verbose=verbose,
        )
    )

    # Evaluation callback - DISABLED for Isaac Sim
    # Isaac Sim only supports a single environment instance per process.
    # Creating an eval env would attempt to start a second Isaac Sim instance,
    # which fails. For evaluation, run a separate evaluation script after training.
    # eval_freq = training_cfg.get('eval_freq', 500_000)
    # if eval_freq > 0:
    #     eval_env = create_eval_env(scenario, num_envs=16, device=device)
    #     callbacks.append(EvalCallback(...))
    print("[INFO] Eval callback disabled (Isaac Sim single-instance limitation)")

    # Determine total timesteps
    timesteps = total_timesteps or training_cfg.get('total_timesteps', 50_000_000)

    # Print training configuration
    print(f"\n{'='*60}")
    print(f"ISR PPO Training Configuration")
    print(f"{'='*60}")
    print(f"  Scenario: {scenario}")
    print(f"  Num envs: {num_envs}")
    print(f"  Total timesteps: {timesteps:,}")
    print(f"  Batch size: {n_steps * num_envs:,} (n_steps={n_steps} x num_envs={num_envs})")
    print(f"  Minibatch size: {model.batch_size}")
    print(f"  Learning rate: {training_cfg.get('learning_rate', 3e-4)}")
    print(f"  Gamma: {model.gamma}")
    print(f"  Device: {device}")
    print(f"  Checkpoint dir: {checkpoint_dir}")
    print(f"  Log dir: {log_path}")
    print(f"{'='*60}\n")

    # Train
    start_time = time.time()
    try:
        model.learn(
            total_timesteps=timesteps,
            callback=CallbackList(callbacks),
            progress_bar=True,
            tb_log_name=run_name,
        )
    except KeyboardInterrupt:
        print("\n[TRAIN] Training interrupted by user")
    finally:
        # Save final model
        final_path = checkpoint_dir / f"{scenario}_final.zip"
        model.save(str(final_path))
        print(f"[TRAIN] Final model saved to {final_path}")

        # Training summary
        elapsed = time.time() - start_time
        print(f"\n[TRAIN] Training complete!")
        print(f"  Total time: {elapsed/3600:.1f} hours")
        print(f"  Steps: {model.num_timesteps:,}")
        print(f"  FPS: {model.num_timesteps/elapsed:,.0f}")

    # Clean up
    vec_env.close()


def train_curriculum(
    num_envs: int = 256,
    device: str = "cuda:0",
    checkpoint: Optional[str] = None,
    save_path: str = "./checkpoints",
    log_path: str = "./logs",
    seed: int = 42,
    verbose: int = 1,
) -> None:
    """
    Train with curriculum learning across all scenarios.

    Progresses through stages:
    1. area_coverage (70% success)
    2. nfz_avoidance (70% success)
    3. multi_objective (60% success)

    Args:
        num_envs: Number of parallel environments
        device: Training device
        checkpoint: Optional checkpoint to resume from
        save_path: Directory for checkpoints
        log_path: Directory for logs
        seed: Random seed
        verbose: Verbosity level
    """
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
    from training.callbacks import CurriculumCallback, CurriculumStage

    # Set random seeds
    torch.manual_seed(seed)
    np.random.seed(seed)

    # Define curriculum stages
    stages = [
        CurriculumStage("area_coverage", 0.7, 10000, "Basic navigation and coverage"),
        CurriculumStage("nfz_avoidance", 0.7, 10000, "Transit with dynamic NFZs"),
        CurriculumStage("multi_objective", 0.6, 20000, "Multi-priority ISR with threats"),
    ]

    # Start with first scenario
    current_scenario = stages[0].scenario
    scenario_config = load_scenario_config(current_scenario)
    training_cfg = scenario_config.get('training', {})

    # Create initial environment
    print(f"[CURRICULUM] Starting with scenario: {current_scenario}")
    env_config = build_env_config(scenario_config, num_envs, device)
    env_config.base_seed = seed

    isr_env = ISRTrainingEnv(env_config)
    isr_env.setup()
    vec_env = ISRVecEnvWrapper(isr_env)

    # Setup paths
    checkpoint_dir = Path(save_path) / "curriculum"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    # Policy architecture
    policy_kwargs = dict(
        net_arch=dict(pi=[256, 256], vf=[256, 256]),
        activation_fn=torch.nn.ReLU,
    )

    # Create model
    batch_size = training_cfg.get('batch_size', 8192)
    n_steps = max(batch_size // num_envs, 128)

    if checkpoint:
        model = PPO.load(checkpoint, env=vec_env, device=device)
    else:
        model = PPO(
            policy="MlpPolicy",
            env=vec_env,
            n_steps=n_steps,
            batch_size=training_cfg.get('minibatch_size', 1024),
            n_epochs=10,
            learning_rate=linear_schedule(3e-4),
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            vf_coef=0.5,
            max_grad_norm=0.5,
            tensorboard_log=log_path,
            policy_kwargs=policy_kwargs,
            device=device,
            verbose=verbose,
            seed=seed,
        )

    # Stage change callback
    def on_stage_advance(stage: CurriculumStage):
        """Called when curriculum advances to next stage."""
        nonlocal vec_env, isr_env

        # Save current model
        model.save(str(checkpoint_dir / f"stage_{stage.scenario}_start.zip"))

        # Close old environment
        vec_env.close()

        # Create new environment for new scenario
        new_config = load_scenario_config(stage.scenario)
        env_cfg = build_env_config(new_config, num_envs, device)
        env_cfg.base_seed = seed

        isr_env = ISRTrainingEnv(env_cfg)
        isr_env.setup()
        vec_env = ISRVecEnvWrapper(isr_env)

        # Update model's environment
        model.set_env(vec_env)

    # Setup callbacks
    callbacks = [
        CheckpointCallback(
            save_freq=1_000_000 // num_envs,
            save_path=str(checkpoint_dir),
            name_prefix="curriculum",
        ),
        ISRMetricsCallback(log_freq=10000, verbose=verbose),
        CurriculumCallback(
            stages=stages,
            check_freq=10000,
            save_path=str(checkpoint_dir / "curriculum_progress.json"),
            on_stage_advance=on_stage_advance,
            verbose=verbose,
        ),
    ]

    # Total timesteps across all stages
    total_timesteps = 200_000_000  # 200M steps for full curriculum

    print(f"\n{'='*60}")
    print(f"Curriculum Learning Configuration")
    print(f"{'='*60}")
    print(f"  Stages: {[s.scenario for s in stages]}")
    print(f"  Total timesteps: {total_timesteps:,}")
    print(f"  Num envs: {num_envs}")
    print(f"{'='*60}\n")

    # Train
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=CallbackList(callbacks),
            progress_bar=True,
            tb_log_name="curriculum",
        )
    except KeyboardInterrupt:
        print("\n[CURRICULUM] Training interrupted")
    finally:
        model.save(str(checkpoint_dir / "curriculum_final.zip"))
        vec_env.close()


def main():
    parser = argparse.ArgumentParser(
        description="Train ISR policies with PPO",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Train on area coverage
    python train_isr_ppo.py --scenario area_coverage

    # Train with more environments
    python train_isr_ppo.py --scenario area_coverage --num_envs 512

    # Resume from checkpoint
    python train_isr_ppo.py --scenario area_coverage --checkpoint checkpoints/model.zip

    # Curriculum learning
    python train_isr_ppo.py --curriculum
        """
    )

    parser.add_argument(
        "--scenario",
        type=str,
        default="area_coverage",
        choices=["area_coverage", "nfz_avoidance", "multi_objective"],
        help="Training scenario",
    )
    parser.add_argument(
        "--num_envs",
        type=int,
        default=256,
        help="Number of parallel environments",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda:0",
        help="Training device",
    )
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="Path to checkpoint to resume from",
    )
    parser.add_argument(
        "--total_timesteps",
        type=int,
        default=None,
        help="Override total timesteps from config",
    )
    parser.add_argument(
        "--save_path",
        type=str,
        default="./checkpoints",
        help="Directory for checkpoints",
    )
    parser.add_argument(
        "--log_path",
        type=str,
        default="./logs",
        help="Directory for TensorBoard logs",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed",
    )
    parser.add_argument(
        "--curriculum",
        action="store_true",
        help="Use curriculum learning across all scenarios",
    )
    parser.add_argument(
        "--verbose",
        type=int,
        default=1,
        help="Verbosity level (0=silent, 1=info, 2=debug)",
    )

    args = parser.parse_args()

    # Check for CUDA
    if "cuda" in args.device and not torch.cuda.is_available():
        print("[WARN] CUDA not available, falling back to CPU")
        args.device = "cpu"

    # Run training
    if args.curriculum:
        train_curriculum(
            num_envs=args.num_envs,
            device=args.device,
            checkpoint=args.checkpoint,
            save_path=args.save_path,
            log_path=args.log_path,
            seed=args.seed,
            verbose=args.verbose,
        )
    else:
        train_ppo(
            scenario=args.scenario,
            num_envs=args.num_envs,
            device=args.device,
            checkpoint=args.checkpoint,
            total_timesteps=args.total_timesteps,
            save_path=args.save_path,
            log_path=args.log_path,
            seed=args.seed,
            verbose=args.verbose,
        )


if __name__ == "__main__":
    main()
