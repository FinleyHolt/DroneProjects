#!/usr/bin/env python3
"""
Scenario-Based Training Script for ISR Environments

This script trains RL policies on the three canonical ISR scenarios:
1. Comms-Denied Area Surveillance
2. Dynamic NFZ Avoidance
3. Multi-Objective ISR with Threat Avoidance

Usage:
    python training/scripts/train_scenario.py --scenario comms_denied
    python training/scripts/train_scenario.py --scenario nfz_avoidance --num_envs 64
    python training/scripts/train_scenario.py --scenario multi_objective --checkpoint path/to/model.pt

Requirements:
    - Isaac Sim 5.1 with Isaac Lab
    - rl_games or stable-baselines3

Author: Finley Holt
"""

import argparse
import os
import sys
import time
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional

import yaml
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


def load_scenario_config(scenario: str, config_path: Optional[str] = None) -> Dict[str, Any]:
    """
    Load scenario configuration from YAML file.

    Args:
        scenario: Scenario name (comms_denied, nfz_avoidance, multi_objective)
        config_path: Optional path to config file (defaults to configs/scenario_configs.yaml)

    Returns:
        Dictionary with scenario configuration
    """
    if config_path is None:
        config_path = TRAINING_DIR / "configs" / "scenario_configs.yaml"

    with open(config_path, 'r') as f:
        all_configs = yaml.safe_load(f)

    if scenario not in all_configs:
        raise ValueError(f"Unknown scenario: {scenario}. Available: {list(all_configs.keys())}")

    return all_configs[scenario]


def build_env_config(scenario_config: Dict[str, Any], num_envs: int, device: str) -> ISRTrainingConfig:
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
    nfz_cfg = scenario_config.get('nfz', {})
    threat_cfg = scenario_config.get('threat', {})

    # Parse scenario type
    scenario_str = mission_cfg.get('scenario', 'comms_denied')
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
        nfz=nfz,
        threat=threat,
        targets=TargetConfig(
            max_targets=targets_cfg.get('max_targets', 16),
            detection_reward=targets_cfg.get('detection_reward', 10.0),
            tracking_reward=targets_cfg.get('tracking_reward', 0.1),
        ),
    )


def create_run_name(scenario: str) -> str:
    """Create a unique run name for logging."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{scenario}_{timestamp}"


def train_with_rl_games(
    env: ISRTrainingEnv,
    config: Dict[str, Any],
    run_name: str,
    checkpoint: Optional[str] = None,
) -> None:
    """
    Train using rl_games library (Isaac Lab default).

    Args:
        env: Training environment
        config: Training configuration
        run_name: Name for this training run
        checkpoint: Optional checkpoint to resume from
    """
    try:
        from rl_games.algos_torch import torch_ext
        from rl_games.common import env_configurations, vecenv
        from rl_games.torch_runner import Runner
    except ImportError:
        print("ERROR: rl_games not available. Install with: pip install rl-games")
        print("Falling back to dummy training loop...")
        train_dummy(env, config, run_name)
        return

    # TODO: Implement rl_games integration
    # This requires setting up rl_games config format
    print("rl_games training not yet fully implemented")
    print("Using dummy training loop for now...")
    train_dummy(env, config, run_name)


def train_with_sb3(
    env: ISRTrainingEnv,
    config: Dict[str, Any],
    run_name: str,
    checkpoint: Optional[str] = None,
) -> None:
    """
    Train using stable-baselines3 library with PPO.

    Args:
        env: Training environment
        config: Training configuration
        run_name: Name for this training run
        checkpoint: Optional checkpoint to resume from
    """
    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
    except ImportError:
        print("ERROR: stable-baselines3 not available. Install with: pip install stable-baselines3")
        print("Falling back to dummy training loop...")
        train_dummy(env, config, run_name)
        return

    from training.wrappers import ISRVecEnvWrapper
    from training.callbacks import ISRMetricsCallback

    # Wrap environment for SB3 compatibility
    print(f"[SB3] Wrapping environment for SB3 compatibility...")
    vec_env = ISRVecEnvWrapper(env)

    training_cfg = config.get('training', {})

    # Calculate n_steps (SB3's n_steps is per environment)
    batch_size = training_cfg.get('batch_size', 8192)
    n_steps = max(batch_size // env.num_envs, 128)

    # Policy network architecture
    policy_kwargs = dict(
        net_arch=dict(
            pi=[256, 256],  # Actor network
            vf=[256, 256],  # Critic network
        ),
        activation_fn=torch.nn.ReLU,
    )

    # Linear learning rate schedule
    def linear_schedule(initial_value: float):
        def func(progress_remaining: float) -> float:
            return progress_remaining * initial_value
        return func

    # Create or load model
    if checkpoint:
        print(f"[SB3] Loading checkpoint: {checkpoint}")
        model = PPO.load(checkpoint, env=vec_env, device=env.device)
    else:
        print(f"[SB3] Creating new PPO model...")
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
            tensorboard_log=f"./logs",
            policy_kwargs=policy_kwargs,
            device=str(env.device),
            verbose=1,
        )

    # Setup callbacks
    checkpoint_dir = Path(f"./checkpoints/{run_name}")
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    callbacks = [
        CheckpointCallback(
            save_freq=training_cfg.get('save_freq', 1_000_000) // env.num_envs,
            save_path=str(checkpoint_dir),
            name_prefix="model",
        ),
        ISRMetricsCallback(log_freq=10000, verbose=1),
    ]

    total_timesteps = training_cfg.get('total_timesteps', 50_000_000)

    print(f"\n{'='*60}")
    print(f"PPO Training Configuration")
    print(f"{'='*60}")
    print(f"  Run name: {run_name}")
    print(f"  Num envs: {env.num_envs}")
    print(f"  Total timesteps: {total_timesteps:,}")
    print(f"  Batch size: {n_steps * env.num_envs:,}")
    print(f"  Minibatch size: {model.batch_size}")
    print(f"  Device: {env.device}")
    print(f"{'='*60}\n")

    # Train
    start_time = time.perf_counter()
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=CallbackList(callbacks),
            progress_bar=True,
            tb_log_name=run_name,
        )
    except KeyboardInterrupt:
        print("\n[SB3] Training interrupted by user")
    finally:
        # Save final model
        final_path = checkpoint_dir / "final.zip"
        model.save(str(final_path))
        print(f"[SB3] Final model saved to {final_path}")

        elapsed = time.perf_counter() - start_time
        print(f"\n[SB3] Training summary:")
        print(f"  Total time: {elapsed/3600:.1f} hours")
        print(f"  Steps: {model.num_timesteps:,}")
        print(f"  FPS: {model.num_timesteps/elapsed:,.0f}")


def train_dummy(
    env: ISRTrainingEnv,
    config: Dict[str, Any],
    run_name: str,
) -> None:
    """
    Dummy training loop for testing environment setup.

    This validates the environment works before integrating with RL libraries.
    Enhanced with detailed diagnostics to verify drones are flying and learning.

    Args:
        env: Training environment
        config: Training configuration
        run_name: Name for this training run
    """
    training_cfg = config.get('training', {})
    total_steps = training_cfg.get('total_timesteps', 10000)
    log_interval = 10000  # Log every 10k steps for cleaner output
    detailed_log_interval = 50000  # Detailed diagnostics every 50k steps

    print(f"\n{'='*70}")
    print(f"Starting dummy training run: {run_name}")
    print(f"{'='*70}")
    print(f"Total timesteps: {total_steps}")
    print(f"Num environments: {env.num_envs}")
    print(f"Observation dim: {env.observation_dim}")
    print(f"Action dim: {env.action_dim}")
    print(f"{'='*70}\n")

    # Random policy for testing
    obs = env.reset()

    step = 0
    episode_count = 0
    total_reward = 0.0

    # Tracking for diagnostics
    reward_history = []
    episode_lengths = []
    current_episode_steps = torch.zeros((env.num_envs,), device=env.device)

    # Termination tracking
    term_counts = {
        "crashed": 0,
        "out_of_bounds": 0,
        "timeout": 0,
        "battery_depleted": 0,
        "mission_success": 0,
        "diverged": 0,
        "nfz_violation": 0,
        "high_threat": 0,
    }

    # Timing for FPS measurement
    start_time = time.perf_counter()
    last_log_time = start_time
    last_log_step = 0

    print("Starting training loop...")
    print("-" * 70)

    while step < total_steps:
        # Random actions (explore the action space)
        actions = torch.rand((env.num_envs, env.action_dim), device=env.device) * 2 - 1

        obs, rewards, dones, info = env.step(actions)

        step += env.num_envs
        total_reward += rewards.sum().item()
        current_episode_steps += 1

        # Track termination reasons
        if 'term_info' in info:
            for key in term_counts:
                if key in info['term_info']:
                    term_counts[key] += info['term_info'][key].sum().item()

        # Track episode completions
        done_count = dones.sum().item()
        if done_count > 0:
            episode_count += done_count
            # Track episode lengths for done envs
            done_indices = torch.where(dones)[0]
            for idx in done_indices:
                episode_lengths.append(current_episode_steps[idx].item())
            current_episode_steps[dones] = 0

        # Store rewards for statistics
        reward_history.append(rewards.mean().item())
        if len(reward_history) > 1000:
            reward_history = reward_history[-1000:]  # Keep last 1000

        # Regular logging
        if step % log_interval < env.num_envs:
            current_time = time.perf_counter()
            elapsed = current_time - last_log_time
            steps_since_log = step - last_log_step
            fps = steps_since_log / elapsed if elapsed > 0 else 0

            coverage = info['coverage_pct'].mean().item()
            battery = info['battery_pct'].mean().item()
            recent_reward_mean = sum(reward_history[-100:]) / max(len(reward_history[-100:]), 1)

            print(f"Step {step:>8d} | FPS: {fps:>7.0f} | Eps: {episode_count:>4d} | "
                  f"Reward: {recent_reward_mean:>7.3f} | "
                  f"Cov: {coverage:>5.1f}% | Bat: {battery:>5.1f}%")

            last_log_time = current_time
            last_log_step = step

        # Detailed diagnostics
        if step % detailed_log_interval < env.num_envs:
            print(f"\n{'='*70}")
            print(f"DETAILED DIAGNOSTICS at step {step}")
            print(f"{'='*70}")

            # Drone state statistics
            drone_pos = env._drone_pos
            drone_vel = env._drone_vel
            print(f"\nDrone Position Stats (across {env.num_envs} envs):")
            print(f"  X: mean={drone_pos[:, 0].mean().item():>7.1f}, "
                  f"std={drone_pos[:, 0].std().item():>6.1f}, "
                  f"range=[{drone_pos[:, 0].min().item():>7.1f}, {drone_pos[:, 0].max().item():>7.1f}]")
            print(f"  Y: mean={drone_pos[:, 1].mean().item():>7.1f}, "
                  f"std={drone_pos[:, 1].std().item():>6.1f}, "
                  f"range=[{drone_pos[:, 1].min().item():>7.1f}, {drone_pos[:, 1].max().item():>7.1f}]")
            print(f"  Z: mean={drone_pos[:, 2].mean().item():>7.1f}, "
                  f"std={drone_pos[:, 2].std().item():>6.1f}, "
                  f"range=[{drone_pos[:, 2].min().item():>7.1f}, {drone_pos[:, 2].max().item():>7.1f}]")

            vel_mag = torch.norm(drone_vel, dim=-1)
            print(f"\nVelocity Magnitude:")
            print(f"  mean={vel_mag.mean().item():>6.2f} m/s, "
                  f"max={vel_mag.max().item():>6.2f} m/s")

            # Target detection stats
            detected = env._target_detected.sum(dim=-1).float()
            print(f"\nTargets Detected per Env:")
            print(f"  mean={detected.mean().item():>5.2f}, "
                  f"max={detected.max().item():>3.0f}/{env.cfg.targets.max_targets}")

            # Observation statistics (check for normalization issues)
            print(f"\nObservation Stats:")
            print(f"  Shape: {obs.shape}")
            print(f"  Mean: {obs.mean().item():>8.4f}")
            print(f"  Std:  {obs.std().item():>8.4f}")
            print(f"  Min:  {obs.min().item():>8.4f}")
            print(f"  Max:  {obs.max().item():>8.4f}")
            if torch.isnan(obs).any():
                print(f"  WARNING: {torch.isnan(obs).sum().item()} NaN values in observations!")

            # Reward statistics
            print(f"\nReward Stats (last 1000 steps):")
            print(f"  mean={sum(reward_history)/len(reward_history):>8.4f}")
            print(f"  min={min(reward_history):>8.4f}")
            print(f"  max={max(reward_history):>8.4f}")

            # Episode length statistics
            if episode_lengths:
                recent_lengths = episode_lengths[-100:]
                print(f"\nEpisode Length Stats (last 100 episodes):")
                print(f"  mean={sum(recent_lengths)/len(recent_lengths):>7.1f} steps")
                print(f"  min={min(recent_lengths):>5.0f}, max={max(recent_lengths):>5.0f}")

            # Termination reason breakdown
            total_terms = sum(term_counts.values())
            if total_terms > 0:
                print(f"\nTermination Reasons ({total_terms} total):")
                for reason, count in sorted(term_counts.items(), key=lambda x: -x[1]):
                    if count > 0:
                        pct = 100 * count / total_terms
                        print(f"  {reason}: {count:>5d} ({pct:>5.1f}%)")

            print(f"{'='*70}\n")

    total_elapsed = time.perf_counter() - start_time
    avg_fps = step / total_elapsed if total_elapsed > 0 else 0

    print(f"\n{'='*70}")
    print(f"TRAINING COMPLETE")
    print(f"{'='*70}")
    print(f"Total steps: {step:,}")
    print(f"Total episodes: {episode_count:,}")
    print(f"Total time: {total_elapsed:.1f}s")
    print(f"Average FPS: {avg_fps:,.0f}")
    print(f"\nFinal Statistics:")
    print(f"  Avg episode reward: {total_reward / max(episode_count, 1):.2f}")
    if episode_lengths:
        print(f"  Avg episode length: {sum(episode_lengths)/len(episode_lengths):.1f} steps")
    print(f"\nTermination Breakdown:")
    total_terms = sum(term_counts.values())
    for reason, count in sorted(term_counts.items(), key=lambda x: -x[1]):
        if count > 0:
            pct = 100 * count / total_terms
            print(f"  {reason}: {count:,} ({pct:.1f}%)")
    print(f"{'='*70}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Train ISR policies on canonical scenarios",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Train on area coverage scenario with default settings
    python train_scenario.py --scenario area_coverage

    # Train on NFZ avoidance with custom environment count
    python train_scenario.py --scenario nfz_avoidance --num_envs 64

    # Resume training from checkpoint
    python train_scenario.py --scenario multi_objective --checkpoint checkpoints/model.pt

    # Use custom config file
    python train_scenario.py --scenario area_coverage --config my_config.yaml
        """
    )

    parser.add_argument(
        "--scenario",
        type=str,
        required=True,
        choices=["area_coverage", "nfz_avoidance", "multi_objective"],
        help="Training scenario",
    )
    parser.add_argument(
        "--num_envs",
        type=int,
        default=None,
        help="Number of parallel environments (overrides config)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda:0",
        help="Device for training",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to custom config file",
    )
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="Path to checkpoint to resume from",
    )
    parser.add_argument(
        "--trainer",
        type=str,
        default="dummy",
        choices=["rl_games", "sb3", "dummy"],
        help="Training library to use",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=True,
        help="Run in headless mode (default: True)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed",
    )

    args = parser.parse_args()

    # Force stdout/stderr to be unbuffered for real-time logging
    import sys
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)

    print(f"[STARTUP] train_scenario.py starting...", flush=True)

    # Set random seed
    torch.manual_seed(args.seed)

    # Load scenario config
    print(f"[CONFIG] Loading scenario config: {args.scenario}", flush=True)
    scenario_config = load_scenario_config(args.scenario, args.config)
    print(f"[CONFIG] Scenario config loaded successfully", flush=True)

    # Determine num_envs
    if args.num_envs is not None:
        num_envs = args.num_envs
    else:
        num_envs = scenario_config.get('training', {}).get('num_envs', 64)

    # Build environment config
    print(f"[CONFIG] Building environment config for {num_envs} envs...", flush=True)
    env_config = build_env_config(scenario_config, num_envs, args.device)
    env_config.base_seed = args.seed

    # Create environment
    print(f"[ENV] Creating ISRTrainingEnv with {num_envs} parallel instances...", flush=True)
    env = ISRTrainingEnv(env_config)
    print(f"[ENV] ISRTrainingEnv created, calling setup()...", flush=True)
    env.setup()
    print(f"[ENV] Environment setup complete!", flush=True)

    # Create run name
    run_name = create_run_name(args.scenario)
    print(f"[TRAIN] Starting training run: {run_name}", flush=True)

    # Train
    try:
        if args.trainer == "rl_games":
            train_with_rl_games(env, scenario_config, run_name, args.checkpoint)
        elif args.trainer == "sb3":
            train_with_sb3(env, scenario_config, run_name, args.checkpoint)
        else:
            train_dummy(env, scenario_config, run_name)
    finally:
        print(f"[CLEANUP] Closing environment...", flush=True)
        env.close()
        print(f"[CLEANUP] Done!", flush=True)


if __name__ == "__main__":
    main()
