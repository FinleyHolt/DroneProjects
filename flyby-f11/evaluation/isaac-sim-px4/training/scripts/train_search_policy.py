#!/usr/bin/env python3
"""
Train SearchPolicy for ISR Area Coverage

Uses PPO with curriculum learning for robust sim-to-real transfer.
Target: 2M steps, ~12-15 hours on RTX 5090 with 1024 parallel envs.

Usage:
    python train_search_policy.py [--num-envs 1024] [--total-steps 2000000]
"""

import sys
import argparse
from datetime import datetime
from pathlib import Path

# =============================================================================
# CRITICAL: Initialize Isaac Sim BEFORE importing torch/numpy/stable-baselines
# =============================================================================
# Isaac Sim's Carbonite/Omniverse plugins must be loaded before any CUDA
# libraries are initialized. Importing torch/numpy before Isaac Sim can cause
# crashes during extension loading (carbOnPluginPreStartup).
# =============================================================================

# Add parent directories to path first (no CUDA imports here)
_script_dir = Path(__file__).parent.absolute()
_training_dir = _script_dir.parent
_eval_dir = _training_dir.parent
sys.path.insert(0, str(_training_dir))
sys.path.insert(0, str(_eval_dir))

# Initialize Isaac Sim FIRST
from isaacsim import SimulationApp
_ISAAC_SIM_CONFIG = {
    "headless": True,
    "width": 1280,
    "height": 720,
}
print("[Init] Creating SimulationApp...", flush=True)
simulation_app = SimulationApp(_ISAAC_SIM_CONFIG)
print("[Init] SimulationApp created successfully", flush=True)

# NOW safe to import CUDA/torch libraries
import torch
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import VecMonitor

from training.environments import SearchTrainingEnv, SearchTrainingConfig
from training.wrappers.vec_env_wrapper import IsaacVecEnvWrapper
from training.callbacks.parameter_curriculum_callback import ParameterCurriculumCallback
from training.callbacks.isr_metrics_callback import ISRMetricsCallback


def parse_args():
    parser = argparse.ArgumentParser(description="Train SearchPolicy")
    parser.add_argument("--num-envs", type=int, default=1024,
                        help="Number of parallel environments")
    parser.add_argument("--total-steps", type=int, default=2_000_000,
                        help="Total training steps")
    parser.add_argument("--learning-rate", type=float, default=3e-4,
                        help="Learning rate")
    parser.add_argument("--batch-size", type=int, default=4096,
                        help="Batch size for PPO updates")
    parser.add_argument("--n-steps", type=int, default=2048,
                        help="Steps per environment per update")
    parser.add_argument("--n-epochs", type=int, default=10,
                        help="Number of PPO epochs per update")
    parser.add_argument("--gamma", type=float, default=0.99,
                        help="Discount factor")
    parser.add_argument("--gae-lambda", type=float, default=0.95,
                        help="GAE lambda")
    parser.add_argument("--clip-range", type=float, default=0.2,
                        help="PPO clip range")
    parser.add_argument("--ent-coef", type=float, default=0.01,
                        help="Entropy coefficient")
    parser.add_argument("--output-dir", type=str, default="./outputs/search_policy",
                        help="Output directory for models and logs")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed")
    parser.add_argument("--device", type=str, default="cuda:0",
                        help="Device for training")
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to checkpoint to resume from")
    return parser.parse_args()


def create_env(args, sim_app):
    """Create SearchTrainingEnv with Isaac Sim."""
    config = SearchTrainingConfig(
        num_envs=args.num_envs,
        device=args.device,
        # Curriculum starts with easier settings
        coverage_target_pct=70.0,  # Start easier, increase via curriculum
        max_episode_steps=15000,
        spawn_area_radius=30.0,    # Start with smaller area
    )

    env = SearchTrainingEnv(config)
    env.set_simulation_app(sim_app)  # Use pre-initialized SimulationApp
    env.setup()

    # Wrap for Stable-Baselines3
    vec_env = IsaacVecEnvWrapper(env)
    vec_env = VecMonitor(vec_env)

    return vec_env, env


def create_eval_env(args, sim_app):
    """Create evaluation environment with harder settings."""
    config = SearchTrainingConfig(
        num_envs=16,  # Smaller for eval
        device=args.device,
        coverage_target_pct=85.0,
        max_episode_steps=20000,
        spawn_area_radius=50.0,
        # Disable domain randomization for consistent eval
        mass_randomization=0.0,
        drag_randomization=0.0,
        wind_enabled=False,
        # Use different prim paths to avoid conflicts with training env
        prim_path_prefix="Eval",
    )

    env = SearchTrainingEnv(config)
    env.set_simulation_app(sim_app)  # Use shared SimulationApp
    env.setup()

    vec_env = IsaacVecEnvWrapper(env)
    vec_env = VecMonitor(vec_env)

    return vec_env


def main():
    args = parse_args()

    # Set seeds
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    # Create output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_dir) / timestamp
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("SearchPolicy Training")
    print("=" * 60)
    print(f"  Environments: {args.num_envs}")
    print(f"  Total steps: {args.total_steps:,}")
    print(f"  Batch size: {args.batch_size}")
    print(f"  Device: {args.device}")
    print(f"  Output: {output_dir}")
    print("=" * 60)

    # Create environments (use global simulation_app created at module load)
    print("\nCreating training environment...")
    train_env, raw_env = create_env(args, simulation_app)

    print("\nCreating evaluation environment...")
    eval_env = create_eval_env(args, simulation_app)

    # Create callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=50000 // args.num_envs,
        save_path=str(output_dir / "checkpoints"),
        name_prefix="search_policy",
    )

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(output_dir / "best_model"),
        log_path=str(output_dir / "eval_logs"),
        eval_freq=100000 // args.num_envs,
        n_eval_episodes=10,
        deterministic=True,
    )

    # Curriculum callback for progressive difficulty
    curriculum_callback = ParameterCurriculumCallback(
        env=raw_env,
        stages=[
            # Stage 1: Easy (0-25% of training)
            {"coverage_target_pct": 70.0, "spawn_area_radius": 30.0,
             "mass_randomization": 0.0, "wind_enabled": False},
            # Stage 2: Medium (25-50% of training)
            {"coverage_target_pct": 75.0, "spawn_area_radius": 40.0,
             "mass_randomization": 0.03, "wind_enabled": False},
            # Stage 3: Hard (50-75% of training)
            {"coverage_target_pct": 80.0, "spawn_area_radius": 50.0,
             "mass_randomization": 0.05, "wind_enabled": True},
            # Stage 4: Full difficulty (75-100% of training)
            {"coverage_target_pct": 85.0, "spawn_area_radius": 50.0,
             "mass_randomization": 0.07, "wind_enabled": True},
        ],
        total_timesteps=args.total_steps,
        save_path=str(output_dir / "curriculum_progress.json"),
    )

    metrics_callback = ISRMetricsCallback(
        log_freq=1000,
        verbose=1,
    )

    callbacks = [checkpoint_callback, eval_callback, curriculum_callback, metrics_callback]

    # Create or load model
    if args.resume:
        print(f"\nResuming from: {args.resume}")
        model = PPO.load(args.resume, env=train_env)
    else:
        print("\nCreating new PPO model...")
        model = PPO(
            "MlpPolicy",
            train_env,
            learning_rate=args.learning_rate,
            n_steps=args.n_steps,
            batch_size=args.batch_size,
            n_epochs=args.n_epochs,
            gamma=args.gamma,
            gae_lambda=args.gae_lambda,
            clip_range=args.clip_range,
            ent_coef=args.ent_coef,
            verbose=1,
            tensorboard_log=str(output_dir / "tensorboard"),
            device=args.device,
            policy_kwargs={
                "net_arch": [256, 256, 128],  # Larger network for complex task
                "activation_fn": torch.nn.ReLU,
            },
        )

    # Train
    print("\nStarting training...")
    try:
        model.learn(
            total_timesteps=args.total_steps,
            callback=callbacks,
            progress_bar=True,
        )
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
    finally:
        # Save final model
        final_path = output_dir / "final_model"
        model.save(str(final_path))
        print(f"\nFinal model saved to: {final_path}")

    # Cleanup
    train_env.close()
    eval_env.close()

    print("\nTraining complete!")
    print(f"Models saved to: {output_dir}")


if __name__ == "__main__":
    main()
