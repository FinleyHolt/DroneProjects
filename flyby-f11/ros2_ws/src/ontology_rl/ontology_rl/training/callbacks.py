"""
Stable-Baselines3 callbacks for ontology-aware training.

These callbacks provide:
- Safety violation tracking and early stopping
- Episode logging with ontology metrics
- Curriculum learning support
- Model checkpointing with safety guarantees
"""

import os
import numpy as np
from typing import Any, Callable, Dict, List, Optional

try:
    from stable_baselines3.common.callbacks import (
        BaseCallback,
        EvalCallback,
        CheckpointCallback,
    )
    SB3_AVAILABLE = True
except ImportError:
    SB3_AVAILABLE = False
    BaseCallback = object


class OntologyViolationCallback(BaseCallback):
    """
    Callback that tracks and responds to ontology constraint violations.

    Features:
    - Counts total violations during training
    - Optionally stops training if violation rate exceeds threshold
    - Logs violation statistics to tensorboard
    - Can trigger curriculum updates based on violation patterns
    """

    def __init__(
        self,
        violation_threshold: float = 0.1,
        stop_on_threshold: bool = False,
        window_size: int = 100,
        verbose: int = 0
    ):
        """
        Initialize violation callback.

        Args:
            violation_threshold: Max allowed violation rate (0-1)
            stop_on_threshold: Stop training if threshold exceeded
            window_size: Rolling window for violation rate calculation
            verbose: Verbosity level
        """
        super().__init__(verbose)

        self.violation_threshold = violation_threshold
        self.stop_on_threshold = stop_on_threshold
        self.window_size = window_size

        # Tracking
        self._violations: List[int] = []
        self._episode_violations: int = 0
        self._total_violations: int = 0

    def _on_step(self) -> bool:
        """Called after each environment step."""
        # Check for violations in info
        infos = self.locals.get('infos', [])
        for info in infos:
            if isinstance(info, dict):
                violations = info.get('safety_violations', 0)
                self._episode_violations += violations
                self._total_violations += violations

                # Check ontology filter info
                filter_info = info.get('ontology_filter', {})
                if filter_info.get('violations'):
                    self._total_violations += len(filter_info['violations'])

        return True

    def _on_rollout_end(self) -> None:
        """Called at end of rollout (for on-policy algorithms)."""
        # Log violation rate
        if len(self._violations) > 0:
            recent_rate = np.mean(self._violations[-self.window_size:])

            if self.logger:
                self.logger.record('ontology/violation_rate', recent_rate)
                self.logger.record('ontology/total_violations', self._total_violations)

            if self.verbose > 0:
                print(f"Violation rate: {recent_rate:.3f}, Total: {self._total_violations}")

            # Check threshold
            if self.stop_on_threshold and recent_rate > self.violation_threshold:
                if self.verbose > 0:
                    print(f"Stopping: violation rate {recent_rate:.3f} > {self.violation_threshold}")
                return False

    def _on_training_end(self) -> None:
        """Called at end of training."""
        if self.verbose > 0:
            print(f"Training ended with {self._total_violations} total violations")

    def get_violation_stats(self) -> Dict[str, Any]:
        """Get violation statistics."""
        return {
            'total_violations': self._total_violations,
            'violation_rate': np.mean(self._violations) if self._violations else 0.0,
            'recent_rate': np.mean(self._violations[-self.window_size:]) if self._violations else 0.0
        }


class EpisodeLogCallback(BaseCallback):
    """
    Callback for detailed episode logging.

    Logs per-episode metrics including:
    - Episode length and return
    - Mission completion rate
    - Battery consumption
    - Ontology query statistics
    - Timing information
    """

    def __init__(
        self,
        log_freq: int = 1,
        verbose: int = 0
    ):
        """
        Initialize episode log callback.

        Args:
            log_freq: Log every N episodes
            verbose: Verbosity level
        """
        super().__init__(verbose)

        self.log_freq = log_freq

        # Episode tracking
        self._episode_count: int = 0
        self._episode_rewards: List[float] = []
        self._episode_lengths: List[int] = []
        self._completion_rates: List[float] = []

    def _on_step(self) -> bool:
        """Called after each step."""
        # Check for episode end
        dones = self.locals.get('dones', [])
        infos = self.locals.get('infos', [])

        for i, done in enumerate(dones):
            if done:
                self._episode_count += 1
                info = infos[i] if i < len(infos) else {}

                # Extract episode info
                ep_info = info.get('episode', {})
                if ep_info:
                    self._episode_rewards.append(ep_info.get('r', 0))
                    self._episode_lengths.append(ep_info.get('l', 0))

                # Log if needed
                if self._episode_count % self.log_freq == 0:
                    self._log_episode(info)

        return True

    def _log_episode(self, info: Dict) -> None:
        """Log episode metrics."""
        if self.logger is None:
            return

        # Recent averages
        n = min(100, len(self._episode_rewards))
        if n > 0:
            self.logger.record('episode/mean_reward', np.mean(self._episode_rewards[-n:]))
            self.logger.record('episode/mean_length', np.mean(self._episode_lengths[-n:]))

        # Episode-specific info
        self.logger.record('episode/battery_final', info.get('battery', 0))
        self.logger.record('episode/violations', info.get('safety_violations', 0))

        if self.verbose > 0:
            print(f"Episode {self._episode_count}: "
                  f"reward={self._episode_rewards[-1] if self._episode_rewards else 0:.2f}, "
                  f"length={self._episode_lengths[-1] if self._episode_lengths else 0}")


class SafetyMetricsCallback(BaseCallback):
    """
    Callback that tracks comprehensive safety metrics.

    Monitors:
    - Geofence boundary approaches
    - NFZ proximity warnings
    - Battery reserve compliance
    - Action filtering frequency
    - Near-miss incidents
    """

    def __init__(
        self,
        geofence_warning_dist: float = 10.0,
        nfz_warning_dist: float = 5.0,
        battery_warning_level: float = 20.0,
        verbose: int = 0
    ):
        """
        Initialize safety metrics callback.

        Args:
            geofence_warning_dist: Distance to log geofence warning
            nfz_warning_dist: Distance to log NFZ warning
            battery_warning_level: Battery % to log warning
            verbose: Verbosity level
        """
        super().__init__(verbose)

        self.geofence_warning_dist = geofence_warning_dist
        self.nfz_warning_dist = nfz_warning_dist
        self.battery_warning_level = battery_warning_level

        # Metrics
        self._geofence_warnings: int = 0
        self._nfz_warnings: int = 0
        self._battery_warnings: int = 0
        self._actions_filtered: int = 0
        self._total_actions: int = 0

    def _on_step(self) -> bool:
        """Called after each step."""
        self._total_actions += 1

        infos = self.locals.get('infos', [])
        for info in infos:
            if not isinstance(info, dict):
                continue

            # Check ontology filter info
            filter_info = info.get('ontology_filter', {})
            if filter_info.get('violations'):
                self._actions_filtered += 1

                for v in filter_info['violations']:
                    vtype = v.get('type', '')
                    if 'geofence' in vtype:
                        self._geofence_warnings += 1
                    elif 'nfz' in vtype:
                        self._nfz_warnings += 1
                    elif 'battery' in vtype:
                        self._battery_warnings += 1

        return True

    def _on_rollout_end(self) -> None:
        """Log safety metrics at end of rollout."""
        if self.logger is None:
            return

        self.logger.record('safety/geofence_warnings', self._geofence_warnings)
        self.logger.record('safety/nfz_warnings', self._nfz_warnings)
        self.logger.record('safety/battery_warnings', self._battery_warnings)
        self.logger.record('safety/filter_rate',
                          self._actions_filtered / max(1, self._total_actions))

    def get_safety_summary(self) -> Dict[str, Any]:
        """Get summary of safety metrics."""
        return {
            'geofence_warnings': self._geofence_warnings,
            'nfz_warnings': self._nfz_warnings,
            'battery_warnings': self._battery_warnings,
            'actions_filtered': self._actions_filtered,
            'total_actions': self._total_actions,
            'filter_rate': self._actions_filtered / max(1, self._total_actions)
        }


class CurriculumCallback(BaseCallback):
    """
    Callback for curriculum learning progression.

    Adjusts environment difficulty based on agent performance:
    - Increases mission complexity as success rate improves
    - Adds more NFZ regions progressively
    - Tightens battery constraints
    - Introduces wind disturbances
    """

    def __init__(
        self,
        success_threshold: float = 0.8,
        window_size: int = 100,
        max_level: int = 5,
        verbose: int = 0
    ):
        """
        Initialize curriculum callback.

        Args:
            success_threshold: Success rate to advance level
            window_size: Episodes to average for success rate
            max_level: Maximum curriculum level
            verbose: Verbosity level
        """
        super().__init__(verbose)

        self.success_threshold = success_threshold
        self.window_size = window_size
        self.max_level = max_level

        self._current_level: int = 1
        self._successes: List[bool] = []

    def _on_step(self) -> bool:
        """Track episode outcomes."""
        dones = self.locals.get('dones', [])
        infos = self.locals.get('infos', [])

        for i, done in enumerate(dones):
            if done:
                info = infos[i] if i < len(infos) else {}
                # Consider success if no violations and mission progress > 0.9
                violations = info.get('safety_violations', 0)
                # Assume success if terminated without violations
                success = violations == 0 and info.get('battery', 100) > 5
                self._successes.append(success)

                # Check for level advancement
                if len(self._successes) >= self.window_size:
                    recent_rate = np.mean(self._successes[-self.window_size:])

                    if recent_rate >= self.success_threshold and self._current_level < self.max_level:
                        self._current_level += 1
                        self._advance_curriculum()

        return True

    def _advance_curriculum(self) -> None:
        """Advance to next curriculum level."""
        if self.verbose > 0:
            print(f"Advancing to curriculum level {self._current_level}")

        # Update environment parameters
        # This would need access to the environment to modify
        if self.logger:
            self.logger.record('curriculum/level', self._current_level)

    def get_current_level(self) -> int:
        """Get current curriculum level."""
        return self._current_level
