"""
ISR-specific metrics callback for Stable-Baselines3.

Logs ISR mission metrics to TensorBoard:
- Coverage percentage
- Battery remaining
- Target detection rates
- Termination reason breakdown
- Episode success rates
"""

from collections import deque
from typing import Any, Dict, Optional

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback


class ISRMetricsCallback(BaseCallback):
    """
    Callback for logging ISR-specific metrics during training.

    Tracks and logs:
    - Coverage percentage (mean across envs)
    - Battery percentage (mean across envs)
    - Targets detected per episode
    - Episode success rate (rolling window)
    - Termination reason breakdown
    - Dwell time statistics
    - Action smoothness metrics

    Example:
        >>> from training.callbacks import ISRMetricsCallback
        >>> callback = ISRMetricsCallback(
        ...     log_freq=1000,
        ...     success_window=100,
        ...     verbose=1
        ... )
        >>> model.learn(total_timesteps=1_000_000, callback=callback)
    """

    def __init__(
        self,
        log_freq: int = 1000,
        success_window: int = 100,
        verbose: int = 0,
    ):
        """
        Initialize the ISR metrics callback.

        Args:
            log_freq: How often to log metrics (in steps)
            success_window: Rolling window size for success rate calculation
            verbose: Verbosity level (0=silent, 1=info, 2=debug)
        """
        super().__init__(verbose)
        self.log_freq = log_freq
        self.success_window = success_window

        # Rolling windows for statistics
        self._episode_successes: deque = deque(maxlen=success_window)
        self._episode_lengths: deque = deque(maxlen=success_window)
        self._episode_returns: deque = deque(maxlen=success_window)
        self._episode_coverages: deque = deque(maxlen=success_window)
        self._episode_batteries: deque = deque(maxlen=success_window)
        self._targets_detected: deque = deque(maxlen=success_window)

        # Termination reason tracking
        self._termination_counts: Dict[str, int] = {
            "mission_success": 0,
            "crashed": 0,
            "out_of_bounds": 0,
            "timeout": 0,
            "battery_depleted": 0,
            "nfz_violation": 0,
            "high_threat": 0,
            "diverged": 0,
        }
        self._total_episodes = 0

        # Per-step metrics accumulation
        self._step_coverages: list = []
        self._step_batteries: list = []
        self._step_rewards: list = []

    def _on_training_start(self) -> None:
        """Called at the start of training."""
        if self.verbose >= 1:
            print(f"[ISRMetrics] Starting training with {self.training_env.num_envs} envs")

    def _on_step(self) -> bool:
        """
        Called after each environment step.

        Returns:
            True to continue training, False to stop
        """
        # Get info dicts from the rollout
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", np.zeros(self.training_env.num_envs, dtype=bool))
        rewards = self.locals.get("rewards", np.zeros(self.training_env.num_envs))

        # Accumulate per-step metrics
        for i, info in enumerate(infos):
            if isinstance(info, dict):
                if "coverage_pct" in info:
                    self._step_coverages.append(info["coverage_pct"])
                if "battery_pct" in info:
                    self._step_batteries.append(info["battery_pct"])

        self._step_rewards.extend(rewards.tolist() if hasattr(rewards, "tolist") else [rewards])

        # Track episode completions
        for i, done in enumerate(dones):
            if done:
                self._total_episodes += 1
                info = infos[i] if i < len(infos) else {}

                # Track success
                term_reason = info.get("termination_reason", "unknown")
                is_success = term_reason == "mission_success"
                self._episode_successes.append(1 if is_success else 0)

                # Track termination reason
                if term_reason in self._termination_counts:
                    self._termination_counts[term_reason] += 1

                # Track final metrics
                if "coverage_pct" in info:
                    self._episode_coverages.append(info["coverage_pct"])
                if "battery_pct" in info:
                    self._episode_batteries.append(info["battery_pct"])
                if "targets_detected" in info:
                    self._targets_detected.append(info["targets_detected"])

        # Log metrics at specified frequency
        if self.n_calls % self.log_freq == 0:
            self._log_metrics()

        return True

    def _log_metrics(self) -> None:
        """Log accumulated metrics to TensorBoard."""
        # ISR-specific metrics
        if self._step_coverages:
            self.logger.record("isr/coverage_pct_mean", np.mean(self._step_coverages))
            self._step_coverages = []

        if self._step_batteries:
            self.logger.record("isr/battery_pct_mean", np.mean(self._step_batteries))
            self._step_batteries = []

        if self._step_rewards:
            self.logger.record("isr/step_reward_mean", np.mean(self._step_rewards))
            self._step_rewards = []

        # Episode-level metrics (rolling window)
        if self._episode_successes:
            success_rate = np.mean(self._episode_successes)
            self.logger.record("isr/success_rate", success_rate)

        if self._episode_coverages:
            self.logger.record("isr/final_coverage_mean", np.mean(self._episode_coverages))
            self.logger.record("isr/final_coverage_max", np.max(self._episode_coverages))

        if self._episode_batteries:
            self.logger.record("isr/final_battery_mean", np.mean(self._episode_batteries))

        if self._targets_detected:
            self.logger.record("isr/targets_detected_mean", np.mean(self._targets_detected))

        # Termination breakdown (percentages)
        if self._total_episodes > 0:
            for reason, count in self._termination_counts.items():
                pct = 100 * count / self._total_episodes
                self.logger.record(f"isr/term_{reason}_pct", pct)

        # Total episodes
        self.logger.record("isr/total_episodes", self._total_episodes)

    def _on_training_end(self) -> None:
        """Called at the end of training."""
        if self.verbose >= 1:
            print(f"\n[ISRMetrics] Training complete!")
            print(f"  Total episodes: {self._total_episodes}")

            if self._episode_successes:
                print(f"  Final success rate: {np.mean(self._episode_successes):.1%}")

            print(f"\n  Termination breakdown:")
            for reason, count in sorted(
                self._termination_counts.items(), key=lambda x: -x[1]
            ):
                if count > 0:
                    pct = 100 * count / self._total_episodes
                    print(f"    {reason}: {count} ({pct:.1f}%)")

    def get_success_rate(self) -> float:
        """
        Get the current rolling success rate.

        Returns:
            Success rate as a float between 0 and 1
        """
        if not self._episode_successes:
            return 0.0
        return float(np.mean(self._episode_successes))

    def get_termination_breakdown(self) -> Dict[str, float]:
        """
        Get termination reason percentages.

        Returns:
            Dict mapping termination reason to percentage
        """
        if self._total_episodes == 0:
            return {k: 0.0 for k in self._termination_counts}

        return {
            reason: 100 * count / self._total_episodes
            for reason, count in self._termination_counts.items()
        }
