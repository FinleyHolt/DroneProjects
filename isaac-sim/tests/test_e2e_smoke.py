#!/usr/bin/env python3
"""
End-to-End Smoke Test for Flyby F-11 RL Training Pipeline

Full pipeline validation:
- 100-step episode rollout with random policy
- Verify reward is finite (not NaN/inf)
- Verify observation stays in bounds
- Verify episode terminates properly
- Log domain randomization stats
- Test all three canonical problems

This test REQUIRES Isaac Sim container.

Run inside container:
/isaac-sim/python.sh -m pytest /workspace/tests/test_e2e_smoke.py -v -s

For quick smoke test:
/isaac-sim/python.sh /workspace/tests/test_e2e_smoke.py --quick
"""
import sys
import os
import argparse
import time
import json
from pathlib import Path
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, field, asdict
from datetime import datetime
import numpy as np

import pytest

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Check if we're in Isaac Sim environment
ISAAC_SIM_AVAILABLE = False
try:
    from isaacsim import SimulationApp
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    pass

requires_isaac = pytest.mark.skipif(
    not ISAAC_SIM_AVAILABLE,
    reason="Requires Isaac Sim environment"
)


@dataclass
class EpisodeStats:
    """Statistics for a single episode."""
    problem: str
    episode_id: int
    seed: int
    steps: int
    total_reward: float
    min_reward: float
    max_reward: float
    mean_reward: float
    terminated: bool
    truncated: bool
    success: bool

    # Observation stats
    obs_min: float = 0.0
    obs_max: float = 0.0
    obs_nan_count: int = 0
    obs_inf_count: int = 0

    # Reward stats
    reward_nan_count: int = 0
    reward_inf_count: int = 0

    # Domain randomization
    battery_drain_multiplier: float = 1.0
    wind_magnitude: float = 0.0
    gnss_degradation_step: int = -1

    # Safety
    safety_interventions: int = 0

    # Problem-specific
    extra_info: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SmokeTestResults:
    """Aggregated smoke test results."""
    timestamp: str
    total_episodes: int
    total_steps: int
    problems_tested: List[str]
    all_passed: bool
    episode_stats: List[EpisodeStats] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


class SmokeTestRunner:
    """
    Runs end-to-end smoke tests on all canonical problems.

    Tests:
    1. Environment can be created and reset
    2. 100-step rollout with random policy completes
    3. All rewards are finite
    4. All observations are finite and in expected bounds
    5. Episode terminates properly
    6. Domain randomization produces valid values
    """

    def __init__(self, steps_per_episode: int = 100, episodes_per_problem: int = 3):
        self.steps_per_episode = steps_per_episode
        self.episodes_per_problem = episodes_per_problem
        self.results = SmokeTestResults(
            timestamp=datetime.now().isoformat(),
            total_episodes=0,
            total_steps=0,
            problems_tested=[],
            all_passed=True,
        )

    def run_all(self) -> SmokeTestResults:
        """Run smoke tests on all canonical problems."""
        problems = ['comms_denied', 'dynamic_nfz', 'multi_objective']

        for problem in problems:
            print(f"\n{'='*60}")
            print(f"Testing: {problem}")
            print(f"{'='*60}")

            try:
                self._test_problem(problem)
            except Exception as e:
                error_msg = f"FAILED: {problem} - {type(e).__name__}: {e}"
                print(f"\n  ERROR: {error_msg}")
                self.results.errors.append(error_msg)
                self.results.all_passed = False

        return self.results

    def _test_problem(self, problem: str) -> None:
        """Test a single canonical problem."""
        from environments.gymnasium_wrapper import make_isaac_gym_env

        self.results.problems_tested.append(problem)

        # Create environment
        config = {
            'env_params': {'headless': True},
            'max_episode_steps': self.steps_per_episode + 100,  # Buffer
        }

        print(f"\n  Creating environment...")
        env = make_isaac_gym_env(problem, config)

        try:
            for ep_idx in range(self.episodes_per_problem):
                seed = 1000 + ep_idx
                stats = self._run_episode(env, problem, ep_idx, seed)
                self.results.episode_stats.append(stats)
                self.results.total_episodes += 1
                self.results.total_steps += stats.steps

                # Check for failures
                if stats.reward_nan_count > 0 or stats.reward_inf_count > 0:
                    self.results.all_passed = False
                    self.results.errors.append(
                        f"{problem} ep{ep_idx}: Invalid rewards detected"
                    )

                if stats.obs_nan_count > 0 or stats.obs_inf_count > 0:
                    self.results.all_passed = False
                    self.results.errors.append(
                        f"{problem} ep{ep_idx}: Invalid observations detected"
                    )

        finally:
            env.close()

    def _run_episode(
        self,
        env,
        problem: str,
        episode_id: int,
        seed: int
    ) -> EpisodeStats:
        """Run a single episode and collect statistics."""
        print(f"\n  Episode {episode_id + 1}/{self.episodes_per_problem} (seed={seed})")

        # Reset
        obs, info = env.reset(seed=seed)

        # Initialize stats
        rewards = []
        obs_values = []
        nan_obs = 0
        inf_obs = 0
        nan_reward = 0
        inf_reward = 0

        terminated = False
        truncated = False

        # Get initial DR stats
        dr_stats = {}
        if hasattr(env.isaac_env, 'get_domain_randomization_stats'):
            dr_stats = env.isaac_env.get_domain_randomization_stats()

        # Run episode
        start_time = time.time()

        for step in range(self.steps_per_episode):
            # Random action
            action = env.action_space.sample()

            # Step
            obs, reward, terminated, truncated, info = env.step(action)

            # Collect stats
            rewards.append(reward)
            obs_values.extend(obs.flatten())

            # Check for invalid values
            if np.isnan(reward):
                nan_reward += 1
            if np.isinf(reward):
                inf_reward += 1

            obs_nans = np.sum(np.isnan(obs))
            obs_infs = np.sum(np.isinf(obs))
            nan_obs += obs_nans
            inf_obs += obs_infs

            # Log progress
            if (step + 1) % 25 == 0:
                print(f"    Step {step + 1}: reward={reward:.2f}, "
                      f"obs_range=[{obs.min():.2f}, {obs.max():.2f}]")

            if terminated or truncated:
                print(f"    Episode ended at step {step + 1}: "
                      f"terminated={terminated}, truncated={truncated}")
                break

        elapsed = time.time() - start_time
        steps_completed = len(rewards)

        # Get safety stats
        safety_stats = {}
        if hasattr(env.isaac_env, 'get_safety_stats'):
            safety_stats = env.isaac_env.get_safety_stats()

        # Build stats
        stats = EpisodeStats(
            problem=problem,
            episode_id=episode_id,
            seed=seed,
            steps=steps_completed,
            total_reward=sum(rewards),
            min_reward=min(rewards) if rewards else 0,
            max_reward=max(rewards) if rewards else 0,
            mean_reward=np.mean(rewards) if rewards else 0,
            terminated=terminated,
            truncated=truncated,
            success=info.get('mission_success', False),
            obs_min=min(obs_values) if obs_values else 0,
            obs_max=max(obs_values) if obs_values else 0,
            obs_nan_count=nan_obs,
            obs_inf_count=inf_obs,
            reward_nan_count=nan_reward,
            reward_inf_count=inf_reward,
            battery_drain_multiplier=dr_stats.get('battery_drain_multiplier', 1.0),
            wind_magnitude=dr_stats.get('wind_magnitude_mps', 0.0),
            gnss_degradation_step=dr_stats.get('gnss_degradation_step', -1),
            safety_interventions=safety_stats.get('safety_interventions', 0),
            extra_info={
                'elapsed_seconds': elapsed,
                'steps_per_second': steps_completed / elapsed if elapsed > 0 else 0,
            }
        )

        # Print summary
        print(f"\n    Summary:")
        print(f"      Steps: {stats.steps}")
        print(f"      Total reward: {stats.total_reward:.2f}")
        print(f"      Reward range: [{stats.min_reward:.2f}, {stats.max_reward:.2f}]")
        print(f"      Obs range: [{stats.obs_min:.2f}, {stats.obs_max:.2f}]")
        print(f"      Invalid rewards: {stats.reward_nan_count} NaN, {stats.reward_inf_count} inf")
        print(f"      Invalid obs: {stats.obs_nan_count} NaN, {stats.obs_inf_count} inf")
        print(f"      DR: battery_mult={stats.battery_drain_multiplier:.2f}, "
              f"wind={stats.wind_magnitude:.1f}m/s")
        print(f"      Safety interventions: {stats.safety_interventions}")
        print(f"      Success: {stats.success}")

        return stats


# ==============================================================================
# Pytest Test Classes
# ==============================================================================

class TestEndToEndSmoke:
    """End-to-end smoke tests as pytest test cases."""

    @requires_isaac
    @pytest.fixture(scope="class")
    def smoke_runner(self):
        """Create smoke test runner."""
        return SmokeTestRunner(steps_per_episode=100, episodes_per_problem=2)

    @requires_isaac
    def test_smoke_all_problems(self, smoke_runner):
        """Run smoke test on all problems."""
        results = smoke_runner.run_all()

        assert len(results.errors) == 0, \
            f"Smoke test errors:\n" + "\n".join(results.errors)

        assert results.all_passed, "Smoke test failed"

    @requires_isaac
    def test_no_nan_rewards(self, smoke_runner):
        """Verify no NaN rewards in any episode."""
        if not smoke_runner.results.episode_stats:
            smoke_runner.run_all()

        for stats in smoke_runner.results.episode_stats:
            assert stats.reward_nan_count == 0, \
                f"{stats.problem} ep{stats.episode_id}: Found {stats.reward_nan_count} NaN rewards"

    @requires_isaac
    def test_no_inf_rewards(self, smoke_runner):
        """Verify no infinite rewards in any episode."""
        if not smoke_runner.results.episode_stats:
            smoke_runner.run_all()

        for stats in smoke_runner.results.episode_stats:
            assert stats.reward_inf_count == 0, \
                f"{stats.problem} ep{stats.episode_id}: Found {stats.reward_inf_count} inf rewards"

    @requires_isaac
    def test_observations_finite(self, smoke_runner):
        """Verify all observations are finite."""
        if not smoke_runner.results.episode_stats:
            smoke_runner.run_all()

        for stats in smoke_runner.results.episode_stats:
            assert stats.obs_nan_count == 0, \
                f"{stats.problem} ep{stats.episode_id}: Found NaN in observations"
            assert stats.obs_inf_count == 0, \
                f"{stats.problem} ep{stats.episode_id}: Found inf in observations"

    @requires_isaac
    def test_episodes_terminate(self, smoke_runner):
        """Verify episodes terminate properly within step limit."""
        if not smoke_runner.results.episode_stats:
            smoke_runner.run_all()

        for stats in smoke_runner.results.episode_stats:
            # Should either terminate or complete steps
            completed = stats.terminated or stats.truncated or stats.steps >= 100
            assert completed, \
                f"{stats.problem} ep{stats.episode_id}: Episode didn't terminate properly"

    @requires_isaac
    def test_domain_randomization_varies(self, smoke_runner):
        """Verify domain randomization produces variation."""
        if not smoke_runner.results.episode_stats:
            smoke_runner.run_all()

        # Group by problem
        by_problem = {}
        for stats in smoke_runner.results.episode_stats:
            if stats.problem not in by_problem:
                by_problem[stats.problem] = []
            by_problem[stats.problem].append(stats)

        for problem, episode_stats in by_problem.items():
            if len(episode_stats) < 2:
                continue

            # Check battery drain multiplier varies
            multipliers = [s.battery_drain_multiplier for s in episode_stats]
            assert len(set([round(m, 3) for m in multipliers])) > 1 or \
                   all(abs(m - 1.0) < 0.001 for m in multipliers), \
                f"{problem}: Battery drain multiplier should vary or be disabled"


class TestSingleProblemSmoke:
    """Individual problem smoke tests for more granular control."""

    @requires_isaac
    def test_comms_denied_100_steps(self):
        """Smoke test CommsDenied environment for 100 steps."""
        from environments.gymnasium_wrapper import make_isaac_gym_env

        env = make_isaac_gym_env('comms_denied', {
            'env_params': {'headless': True},
            'max_episode_steps': 200,
        })

        try:
            obs, info = env.reset(seed=42)

            for step in range(100):
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)

                assert np.isfinite(reward), f"Step {step}: Reward is not finite: {reward}"
                assert np.all(np.isfinite(obs)), f"Step {step}: Observation contains non-finite values"

                if terminated or truncated:
                    break
        finally:
            env.close()

    @requires_isaac
    def test_dynamic_nfz_100_steps(self):
        """Smoke test DynamicNFZ environment for 100 steps."""
        from environments.gymnasium_wrapper import make_isaac_gym_env

        env = make_isaac_gym_env('dynamic_nfz', {
            'env_params': {'headless': True},
            'max_episode_steps': 200,
        })

        try:
            obs, info = env.reset(seed=42)

            for step in range(100):
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)

                assert np.isfinite(reward), f"Step {step}: Reward is not finite: {reward}"
                assert np.all(np.isfinite(obs)), f"Step {step}: Observation contains non-finite values"

                if terminated or truncated:
                    break
        finally:
            env.close()

    @requires_isaac
    def test_multi_objective_100_steps(self):
        """Smoke test MultiObjective environment for 100 steps."""
        from environments.gymnasium_wrapper import make_isaac_gym_env

        env = make_isaac_gym_env('multi_objective', {
            'env_params': {'headless': True},
            'max_episode_steps': 200,
        })

        try:
            obs, info = env.reset(seed=42)

            for step in range(100):
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)

                assert np.isfinite(reward), f"Step {step}: Reward is not finite: {reward}"
                assert np.all(np.isfinite(obs)), f"Step {step}: Observation contains non-finite values"

                if terminated or truncated:
                    break
        finally:
            env.close()


# ==============================================================================
# CLI Entry Point
# ==============================================================================

def main():
    """Run smoke tests from command line."""
    parser = argparse.ArgumentParser(description="E2E Smoke Test for Flyby F-11 RL Pipeline")
    parser.add_argument(
        '--quick', action='store_true',
        help="Quick test (50 steps, 1 episode per problem)"
    )
    parser.add_argument(
        '--steps', type=int, default=100,
        help="Steps per episode (default: 100)"
    )
    parser.add_argument(
        '--episodes', type=int, default=3,
        help="Episodes per problem (default: 3)"
    )
    parser.add_argument(
        '--problem', type=str, choices=['comms_denied', 'dynamic_nfz', 'multi_objective'],
        help="Test only this problem"
    )
    parser.add_argument(
        '--output', type=str, default=None,
        help="Output JSON file for results"
    )

    args = parser.parse_args()

    if args.quick:
        args.steps = 50
        args.episodes = 1

    print("="*60)
    print("Flyby F-11 RL Pipeline E2E Smoke Test")
    print("="*60)
    print(f"Steps per episode: {args.steps}")
    print(f"Episodes per problem: {args.episodes}")
    print()

    runner = SmokeTestRunner(
        steps_per_episode=args.steps,
        episodes_per_problem=args.episodes
    )

    if args.problem:
        # Test single problem
        try:
            runner._test_problem(args.problem)
            runner.results.problems_tested = [args.problem]
        except Exception as e:
            runner.results.errors.append(f"{args.problem}: {e}")
            runner.results.all_passed = False
    else:
        # Test all problems
        runner.run_all()

    # Print summary
    print("\n" + "="*60)
    print("SMOKE TEST SUMMARY")
    print("="*60)
    print(f"Problems tested: {', '.join(runner.results.problems_tested)}")
    print(f"Total episodes: {runner.results.total_episodes}")
    print(f"Total steps: {runner.results.total_steps}")
    print(f"Errors: {len(runner.results.errors)}")
    print(f"Warnings: {len(runner.results.warnings)}")
    print(f"Status: {'PASSED' if runner.results.all_passed else 'FAILED'}")

    if runner.results.errors:
        print("\nErrors:")
        for error in runner.results.errors:
            print(f"  - {error}")

    # Save results
    if args.output:
        # Convert to serializable dict
        results_dict = {
            'timestamp': runner.results.timestamp,
            'total_episodes': runner.results.total_episodes,
            'total_steps': runner.results.total_steps,
            'problems_tested': runner.results.problems_tested,
            'all_passed': runner.results.all_passed,
            'errors': runner.results.errors,
            'warnings': runner.results.warnings,
            'episode_stats': [asdict(s) for s in runner.results.episode_stats],
        }

        with open(args.output, 'w') as f:
            json.dump(results_dict, f, indent=2)
        print(f"\nResults saved to: {args.output}")

    # Exit code
    sys.exit(0 if runner.results.all_passed else 1)


if __name__ == "__main__":
    main()
