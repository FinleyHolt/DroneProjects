#!/usr/bin/env python3
"""
Flyby F-11 Canonical Problem Training Script

Trains hierarchical RL agents on the three canonical ISR problems:
1. Comms-Denied Area Surveillance
2. Dynamic No-Fly Zone Avoidance
3. Multi-Objective ISR with Threat Avoidance

Features:
- Ontology-constrained safety shielding via Vampire ATP
- Domain randomization for sim-to-real transfer
- Curriculum learning with progressive difficulty
- Hierarchical multi-agent RL (Mission Planner, Behavior Selector, Trajectory Optimizer)
"""

import argparse
import sys
import os
import yaml
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional

# Add project paths
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "environments"))

# Pegasus path
PEGASUS_PATH = "/pegasus/extensions/pegasus.simulator"
if PEGASUS_PATH not in sys.path:
    sys.path.insert(0, PEGASUS_PATH)


def load_config(config_path: str) -> Dict[str, Any]:
    """Load training configuration from YAML file."""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def create_environment(config: Dict[str, Any], problem: str):
    """Create the appropriate environment based on problem type."""
    from environments.comms_denied_env import CommsDeniedSurveillanceEnv, CommsDeniedConfig
    from environments.dynamic_nfz_env import DynamicNFZAvoidanceEnv, DynamicNFZConfig
    from environments.multi_objective_env import MultiObjectiveISREnv, MultiObjectiveConfig

    env_config = config.get('environment', {})
    problem_config = config.get('canonical_problems', {}).get(problem, {})
    dr_config = config.get('domain_randomization', {})

    if problem == 'comms_denied':
        env_specific = CommsDeniedConfig(
            headless=env_config.get('headless', True),
            surveillance_area_size=problem_config.get('surveillance_area_size', 500.0),
            max_mission_time=problem_config.get('max_mission_time', 900.0),
            target_coverage=problem_config.get('target_coverage', 85.0),
            target_poi_count=problem_config.get('target_poi_count', 10),
            comms_denial_time=problem_config.get('comms_denial_time', 30.0),
            randomize_lighting=dr_config.get('lighting', {}).get('enabled', True),
            randomize_weather=dr_config.get('weather', {}).get('enabled', True),
            randomize_sensors=dr_config.get('sensors', {}).get('enabled', True),
        )
        return CommsDeniedSurveillanceEnv(env_specific)

    elif problem == 'dynamic_nfz':
        env_specific = DynamicNFZConfig(
            headless=env_config.get('headless', True),
            destination=np.array([
                problem_config.get('transit_distance', 3000.0),
                500.0,
                80.0
            ]),
            cruise_speed=problem_config.get('cruise_speed', 6.3),
            max_mission_time=problem_config.get('max_mission_time', 600.0),
            nfz_activation_time=problem_config.get('nfz_activation_time', 180.0),
            randomize_lighting=dr_config.get('lighting', {}).get('enabled', True),
            randomize_weather=dr_config.get('weather', {}).get('enabled', True),
        )
        return DynamicNFZAvoidanceEnv(env_specific)

    elif problem == 'multi_objective':
        env_specific = MultiObjectiveConfig(
            headless=env_config.get('headless', True),
            max_mission_time=problem_config.get('max_mission_time', 1200.0),
            initial_battery=problem_config.get('initial_battery', 92.0),
            max_medium_threat_time=problem_config.get('max_medium_threat_time', 30.0),
            randomize_lighting=dr_config.get('lighting', {}).get('enabled', True),
        )
        return MultiObjectiveISREnv(env_specific)

    else:
        raise ValueError(f"Unknown problem type: {problem}")


def create_agent(env, config: Dict[str, Any], agent_type: str, log_dir: str):
    """
    Create RL agent based on type.

    Args:
        env: Gymnasium-wrapped environment
        config: Training configuration
        agent_type: One of 'mission_planner', 'behavior_selector', 'trajectory_optimizer'
        log_dir: Directory for tensorboard logs

    Returns:
        Configured SB3 agent
    """
    from agents.sac_agent import create_sac_agent, create_ppo_agent, create_td3_agent

    rl_config = config.get('rl', {}).get(agent_type, {})

    if agent_type == 'mission_planner':
        return create_sac_agent(env, rl_config, tensorboard_log=log_dir)
    elif agent_type == 'behavior_selector':
        return create_ppo_agent(env, rl_config, tensorboard_log=log_dir)
    elif agent_type == 'trajectory_optimizer':
        return create_td3_agent(env, rl_config, tensorboard_log=log_dir)
    else:
        raise ValueError(f"Unknown agent type: {agent_type}")


class DummyAgent:
    """
    Placeholder agent for testing environment setup.
    Replace with actual RL agent implementations (SAC, PPO, TD3).
    """

    def __init__(self, obs_dim: int, action_dim: int, config: Dict[str, Any]):
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.config = config

    def select_action(self, observation: np.ndarray, evaluate: bool = False) -> np.ndarray:
        """Select action (random for placeholder)."""
        # Random action in [-1, 1] range
        return np.random.uniform(-1, 1, size=self.action_dim).astype(np.float32)

    def update(self, batch: Dict[str, np.ndarray]) -> Dict[str, float]:
        """Update agent (no-op for placeholder)."""
        return {"loss": 0.0}

    def save(self, path: str) -> None:
        """Save agent checkpoint."""
        pass

    def load(self, path: str) -> None:
        """Load agent checkpoint."""
        pass


class TrainingRunner:
    """
    Main training loop runner.

    Manages:
    - Environment creation and reset
    - Agent training loop
    - Curriculum progression
    - Logging and checkpointing
    """

    def __init__(
        self,
        config: Dict[str, Any],
        problem: str,
        checkpoint_dir: str,
        log_dir: str,
        agent_type: str = 'trajectory_optimizer',
        use_sb3: bool = True
    ):
        self.config = config
        self.problem = problem
        self.checkpoint_dir = Path(checkpoint_dir)
        self.log_dir = Path(log_dir)
        self.agent_type = agent_type
        self.use_sb3 = use_sb3

        # Create directories
        self.checkpoint_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Environment (created lazily)
        self.env = None

        # Agent
        self.agent = None

        # Training state
        self.episode = 0
        self.total_steps = 0
        self.curriculum_stage = 0

    def setup(self) -> None:
        """Initialize environment and agent."""
        print("=" * 60)
        print(f"Training Setup: {self.problem}")
        print("=" * 60)

        # Create environment
        print("\nCreating environment...")
        isaac_env = create_environment(self.config, self.problem)
        isaac_env.setup()

        if self.use_sb3:
            # Wrap with Gymnasium wrapper for SB3 compatibility
            from environments.gymnasium_wrapper import IsaacSimGymWrapper

            gym_config = {
                'max_episode_steps': self.config.get('training', {}).get('max_episode_steps', 10000)
            }
            self.env = IsaacSimGymWrapper(isaac_env, gym_config)

            # Create SB3 agent
            print("\nCreating SB3 agent...")
            self.agent = create_agent(
                self.env,
                self.config,
                self.agent_type,
                str(self.log_dir)
            )
            print(f"  Agent type: {type(self.agent).__name__}")
            print(f"  Observation space: {self.env.observation_space}")
            print(f"  Action space: {self.env.action_space}")
        else:
            # Use raw Isaac Sim environment with DummyAgent
            self.env = isaac_env

            # Create dummy agent
            print("\nCreating agent...")
            rl_config = self.config.get('rl', {}).get(self.agent_type, {})
            self.agent = DummyAgent(
                obs_dim=self.env.observation_dim,
                action_dim=self.env.action_dim,
                config=rl_config
            )
            print(f"  Observation dim: {self.env.observation_dim}")
            print(f"  Action dim: {self.env.action_dim}")

    def run_episode(self, evaluate: bool = False, seed: Optional[int] = None) -> Dict[str, Any]:
        """Run a single episode."""
        if self.use_sb3:
            # SB3 agent with Gymnasium wrapper
            obs, info = self.env.reset(seed=seed)
            observation = obs
        else:
            # DummyAgent with raw Isaac Sim environment
            state = self.env.reset(seed=seed)
            observation = self.env.state_to_observation(state)

        episode_reward = 0.0
        episode_steps = 0
        done = False
        truncated = False

        transitions = []
        info = {}

        while not done and not truncated and episode_steps < self.config.get('training', {}).get('max_episode_steps', 10000):
            # Select action
            if self.use_sb3:
                action, _ = self.agent.predict(observation, deterministic=evaluate)
            else:
                action = self.agent.select_action(observation, evaluate=evaluate)

            # Safety shield check (only for non-SB3 mode, SB3 handles this in wrapper)
            if not self.use_sb3 and self.config.get('safety', {}).get('enabled', True):
                is_safe, safe_action = self.env.check_action_safety(action)
                if not is_safe and safe_action is not None:
                    action = safe_action

            # Execute action
            if self.use_sb3:
                next_observation, reward, done, truncated, info = self.env.step(action)
            else:
                next_state, reward, done, info = self.env.step(action)
                next_observation = self.env.state_to_observation(next_state)

            # Store transition
            transitions.append({
                'observation': observation,
                'action': action,
                'reward': reward,
                'next_observation': next_observation,
                'done': done,
            })

            episode_reward += reward
            episode_steps += 1
            self.total_steps += 1

            observation = next_observation
            if not self.use_sb3:
                state = next_state

        # Episode summary
        episode_info = {
            'reward': episode_reward,
            'steps': episode_steps,
            'success': info.get('mission_success', False),
            **info
        }

        return episode_info

    def train(self) -> None:
        """Run full training loop."""
        self.setup()

        training_config = self.config.get('training', {})
        total_episodes = training_config.get('total_episodes', 1000)
        log_frequency = training_config.get('logging', {}).get('log_frequency', 100)
        checkpoint_frequency = training_config.get('checkpoints', {}).get('save_frequency', 100)

        curriculum = training_config.get('curriculum', {})
        curriculum_enabled = curriculum.get('enabled', False)
        curriculum_stages = curriculum.get('stages', [])

        print("\n" + "=" * 60)
        print("Starting Training")
        print("=" * 60)
        print(f"Total episodes: {total_episodes}")
        print(f"Curriculum enabled: {curriculum_enabled}")
        print()

        episode_rewards = []
        episode_successes = []

        for episode in range(total_episodes):
            self.episode = episode

            # Check curriculum progression
            if curriculum_enabled:
                self._update_curriculum(episode, curriculum_stages)

            # Run episode
            seed = episode if training_config.get('deterministic', False) else None
            episode_info = self.run_episode(evaluate=False, seed=seed)

            episode_rewards.append(episode_info['reward'])
            episode_successes.append(episode_info['success'])

            # Logging
            if (episode + 1) % log_frequency == 0:
                avg_reward = np.mean(episode_rewards[-log_frequency:])
                success_rate = np.mean(episode_successes[-log_frequency:])

                print(f"Episode {episode + 1}/{total_episodes}")
                print(f"  Avg Reward: {avg_reward:.2f}")
                print(f"  Success Rate: {success_rate * 100:.1f}%")
                print(f"  Total Steps: {self.total_steps}")

            # Checkpointing
            if (episode + 1) % checkpoint_frequency == 0:
                self._save_checkpoint(episode + 1)

        # Final checkpoint
        self._save_checkpoint(total_episodes, final=True)

        print("\n" + "=" * 60)
        print("Training Complete!")
        print("=" * 60)

    def train_sb3(self) -> None:
        """Run training using SB3's built-in training loop."""
        from stable_baselines3.common.callbacks import (
            CheckpointCallback,
            EvalCallback
        )

        # Setup gymnasium environment
        from environments.gymnasium_wrapper import IsaacSimGymWrapper

        print("=" * 60)
        print(f"Training Setup: {self.problem} with SB3")
        print("=" * 60)

        # Create and wrap environment
        print("\nCreating environment...")
        isaac_env = create_environment(self.config, self.problem)
        isaac_env.setup()

        gym_config = {
            'max_episode_steps': self.config.get('training', {}).get('max_episode_steps', 10000)
        }
        self.env = IsaacSimGymWrapper(isaac_env, gym_config)

        # Create agent
        print("\nCreating SB3 agent...")
        self.agent = create_agent(
            self.env,
            self.config,
            self.agent_type,
            str(self.log_dir)
        )

        print(f"  Agent type: {type(self.agent).__name__}")
        print(f"  Observation space: {self.env.observation_space}")
        print(f"  Action space: {self.env.action_space}")

        # Setup callbacks
        training_config = self.config.get('training', {})
        checkpoint_freq = training_config.get('checkpoints', {}).get('save_frequency', 100)

        checkpoint_callback = CheckpointCallback(
            save_freq=checkpoint_freq * 1000,  # Convert episodes to steps
            save_path=str(self.checkpoint_dir),
            name_prefix=f"{self.problem}_{self.agent_type}"
        )

        callbacks = [checkpoint_callback]

        # Calculate total timesteps
        total_episodes = training_config.get('total_episodes', 1000)
        max_steps = training_config.get('max_episode_steps', 10000)
        total_timesteps = total_episodes * max_steps

        print(f"\nStarting training for {total_timesteps} timesteps...")
        import sys
        sys.stdout.flush()  # Force flush to see prints immediately

        # Train
        print("[DEBUG] About to call model.learn()...")
        sys.stdout.flush()
        self.agent.learn(
            total_timesteps=total_timesteps,
            callback=callbacks,
            progress_bar=False  # Disable progress bar for cleaner log output
        )
        print("[DEBUG] model.learn() returned")

        # Save final model
        final_path = self.checkpoint_dir / f"{self.problem}_{self.agent_type}_final"
        self.agent.save(str(final_path))
        print(f"\nFinal model saved to {final_path}")

    def _update_curriculum(self, episode: int, stages: list) -> None:
        """Update curriculum stage based on episode count."""
        cumulative = 0
        for i, stage in enumerate(stages):
            cumulative += stage.get('episodes', 0)
            if episode < cumulative:
                if i != self.curriculum_stage:
                    self.curriculum_stage = i
                    print(f"\n>>> Curriculum Stage: {stage['name']} (difficulty: {stage['difficulty']})")
                return

    def _save_checkpoint(self, episode: int, final: bool = False) -> None:
        """Save training checkpoint."""
        suffix = "final" if final else f"ep{episode}"
        checkpoint_path = self.checkpoint_dir / f"checkpoint_{self.problem}_{suffix}.pt"

        print(f"  Saving checkpoint: {checkpoint_path}")
        self.agent.save(str(checkpoint_path))

    def evaluate(self, num_episodes: int = 10) -> Dict[str, Any]:
        """Evaluate trained agent."""
        print("\n" + "=" * 60)
        print("Evaluation")
        print("=" * 60)

        results = []

        for i in range(num_episodes):
            episode_info = self.run_episode(evaluate=True, seed=i + 1000)
            results.append(episode_info)
            print(f"  Episode {i + 1}: Reward={episode_info['reward']:.2f}, Success={episode_info['success']}")

        # Aggregate metrics
        avg_reward = np.mean([r['reward'] for r in results])
        success_rate = np.mean([r['success'] for r in results])

        print(f"\nEvaluation Results:")
        print(f"  Average Reward: {avg_reward:.2f}")
        print(f"  Success Rate: {success_rate * 100:.1f}%")

        return {
            'avg_reward': avg_reward,
            'success_rate': success_rate,
            'episodes': results,
        }

    def close(self) -> None:
        """Cleanup."""
        if self.env is not None:
            self.env.close()


def main():
    parser = argparse.ArgumentParser(
        description="Train Flyby F-11 ISR agents on canonical problems"
    )
    parser.add_argument(
        "--problem",
        type=str,
        choices=['comms_denied', 'dynamic_nfz', 'multi_objective'],
        default='comms_denied',
        help="Canonical problem to train on"
    )
    parser.add_argument(
        "--config",
        type=str,
        default=str(PROJECT_ROOT / "config" / "training_config.yaml"),
        help="Path to training configuration file"
    )
    parser.add_argument(
        "--checkpoint-dir",
        type=str,
        default="./checkpoints",
        help="Directory for saving checkpoints"
    )
    parser.add_argument(
        "--log-dir",
        type=str,
        default="./logs",
        help="Directory for logs"
    )
    parser.add_argument(
        "--evaluate",
        action="store_true",
        help="Run evaluation only (requires checkpoint)"
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=None,
        help="Override number of training episodes"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode (no GUI)"
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="Run with GUI (overrides config headless=true)"
    )
    parser.add_argument(
        "--agent-type",
        type=str,
        choices=['mission_planner', 'behavior_selector', 'trajectory_optimizer'],
        default='trajectory_optimizer',
        help="Type of RL agent to train"
    )
    parser.add_argument(
        "--use-dummy-agent",
        action="store_true",
        help="Use DummyAgent instead of SB3 (for testing)"
    )

    args = parser.parse_args()

    # Load configuration
    config = load_config(args.config)

    # Apply overrides
    if args.headless:
        config['environment']['headless'] = True
    elif args.gui:
        config['environment']['headless'] = False
    if args.episodes is not None:
        config['training']['total_episodes'] = args.episodes

    # Check for perception mode override from environment variable
    perception_mode = os.environ.get('FLYBY_PERCEPTION_MODE')
    if perception_mode:
        config['perception']['mode'] = perception_mode
        print(f"[Config] Perception mode overridden to: {perception_mode}")

    # Create runner
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    checkpoint_dir = Path(args.checkpoint_dir) / args.problem / timestamp
    log_dir = Path(args.log_dir) / args.problem / timestamp

    # Determine whether to use SB3 or DummyAgent
    use_sb3 = not args.use_dummy_agent

    runner = TrainingRunner(
        config=config,
        problem=args.problem,
        checkpoint_dir=str(checkpoint_dir),
        log_dir=str(log_dir),
        agent_type=args.agent_type,
        use_sb3=use_sb3
    )

    try:
        if args.evaluate:
            runner.setup()
            runner.evaluate()
        else:
            if use_sb3:
                # Use SB3's built-in training loop
                runner.train_sb3()
            else:
                # Use custom training loop with DummyAgent
                runner.train()
            runner.evaluate()
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
    finally:
        runner.close()


if __name__ == "__main__":
    main()
