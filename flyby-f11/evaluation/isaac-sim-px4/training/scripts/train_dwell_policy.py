#!/usr/bin/env python3
"""
Train DwellPolicy for ISR Target Tracking with 2-Axis Gimbal

Uses SAC for sample efficiency and exploration in the tracking task.
Target: 1.5M steps, ~10-12 hours on RTX 5090 with 1024 parallel envs.

Usage:
    python train_dwell_policy.py [--num-envs 1024] [--total-steps 1500000]
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

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import VecMonitor

from training.environments import DwellTrainingEnv, DwellTrainingConfig
from training.wrappers.vec_env_wrapper import IsaacVecEnvWrapper
from training.callbacks.parameter_curriculum_callback import ParameterCurriculumCallback
from training.callbacks.isr_metrics_callback import ISRMetricsCallback


def parse_args():
    parser = argparse.ArgumentParser(description="Train DwellPolicy")
    parser.add_argument("--num-envs", type=int, default=1024,
                        help="Number of parallel environments")
    parser.add_argument("--total-steps", type=int, default=1_500_000,
                        help="Total training steps")
    parser.add_argument("--learning-rate", type=float, default=3e-4,
                        help="Learning rate")
    parser.add_argument("--buffer-size", type=int, default=1_000_000,
                        help="Replay buffer size")
    parser.add_argument("--batch-size", type=int, default=256,
                        help="Batch size for SAC updates")
    parser.add_argument("--learning-starts", type=int, default=10000,
                        help="Steps before learning starts")
    parser.add_argument("--tau", type=float, default=0.005,
                        help="Soft update coefficient")
    parser.add_argument("--gamma", type=float, default=0.99,
                        help="Discount factor")
    parser.add_argument("--train-freq", type=int, default=1,
                        help="Update frequency")
    parser.add_argument("--gradient-steps", type=int, default=1,
                        help="Gradient steps per update")
    parser.add_argument("--ent-coef", type=str, default="auto",
                        help="Entropy coefficient (auto for automatic)")
    parser.add_argument("--output-dir", type=str, default="./outputs/dwell_policy",
                        help="Output directory for models and logs")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed")
    parser.add_argument("--device", type=str, default="cuda:0",
                        help="Device for training")
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to checkpoint to resume from")
    return parser.parse_args()


def create_env(args, sim_app):
    """Create DwellTrainingEnv with Isaac Sim."""
    config = DwellTrainingConfig(
        num_envs=args.num_envs,
        device=args.device,
        # Curriculum starts with easier settings
        min_dwell_time=15.0,  # Start with shorter dwell requirement
        track_lost_timeout=10.0,  # More forgiving timeout
        target_stationary_prob=0.8,  # Mostly stationary targets initially
        initial_target_distance_range=(30.0, 60.0),
    )

    env = DwellTrainingEnv(config)
    env.set_simulation_app(sim_app)  # Use pre-initialized SimulationApp
    env.setup()

    # Wrap for Stable-Baselines3
    vec_env = IsaacVecEnvWrapper(env)
    vec_env = VecMonitor(vec_env)

    return vec_env, env


def create_eval_env(args, sim_app):
    """Create evaluation environment with harder settings."""
    config = DwellTrainingConfig(
        num_envs=16,  # Smaller for eval
        device=args.device,
        min_dwell_time=30.0,
        track_lost_timeout=5.0,
        target_stationary_prob=0.5,  # Mix of stationary and moving
        initial_target_distance_range=(30.0, 80.0),
        # Disable domain randomization for consistent eval
        mass_randomization=0.0,
        drag_randomization=0.0,
        gimbal_noise=0.0,
        # Use different prim paths to avoid conflicts with training env
        prim_path_prefix="Eval",
    )

    env = DwellTrainingEnv(config)
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
    print("DwellPolicy Training (2-Axis Gimbal)")
    print("=" * 60)
    print(f"  Environments: {args.num_envs}")
    print(f"  Total steps: {args.total_steps:,}")
    print(f"  Batch size: {args.batch_size}")
    print(f"  Device: {args.device}")
    print(f"  Output: {output_dir}")
    print(f"  Action dims: 6 (4 velocity + 2 gimbal)")
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
        name_prefix="dwell_policy",
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
            # Stage 1: Easy (0-25% of training) - stationary targets, short dwell
            {"min_dwell_time": 15.0, "track_lost_timeout": 10.0,
             "target_stationary_prob": 0.9, "mass_randomization": 0.0},
            # Stage 2: Medium (25-50% of training) - some moving targets
            {"min_dwell_time": 20.0, "track_lost_timeout": 7.0,
             "target_stationary_prob": 0.7, "mass_randomization": 0.02},
            # Stage 3: Hard (50-75% of training) - more moving, longer dwell
            {"min_dwell_time": 25.0, "track_lost_timeout": 5.0,
             "target_stationary_prob": 0.5, "mass_randomization": 0.03},
            # Stage 4: Full difficulty (75-100% of training)
            {"min_dwell_time": 30.0, "track_lost_timeout": 5.0,
             "target_stationary_prob": 0.5, "mass_randomization": 0.03,
             "gimbal_noise": 0.01},
        ],
        total_timesteps=args.total_steps,
        save_path=str(output_dir / "curriculum_progress.json"),
    )

    metrics_callback = ISRMetricsCallback(
        log_freq=1000,
        verbose=1,
    )

    callbacks = [checkpoint_callback, eval_callback, curriculum_callback, metrics_callback]

    # Parse entropy coefficient
    ent_coef = args.ent_coef
    if ent_coef != "auto":
        ent_coef = float(ent_coef)

    # Create or load model
    if args.resume:
        print(f"\nResuming from: {args.resume}")
        model = SAC.load(args.resume, env=train_env)
    else:
        print("\nCreating new SAC model...")
        model = SAC(
            "MlpPolicy",
            train_env,
            learning_rate=args.learning_rate,
            buffer_size=args.buffer_size,
            learning_starts=args.learning_starts,
            batch_size=args.batch_size,
            tau=args.tau,
            gamma=args.gamma,
            train_freq=args.train_freq,
            gradient_steps=args.gradient_steps,
            ent_coef=ent_coef,
            verbose=1,
            tensorboard_log=str(output_dir / "tensorboard"),
            device=args.device,
            policy_kwargs={
                "net_arch": [256, 256, 128],
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
