"""
Domain Randomization Manager - Sampling and applying DR effects.

Single responsibility: Sample and apply domain randomization for each episode.
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class DomainRandomizationConfig:
    """Domain randomization configuration for sensor noise and disturbances."""
    # Master enable/disable
    enabled: bool = True

    # Battery variation (±percentage of base drain rate)
    battery_variation_enabled: bool = True
    battery_drain_rate_variation: float = 0.1  # ±10% variation

    # GNSS degradation
    gnss_degradation_enabled: bool = True
    gnss_degradation_prob: float = 0.1  # 10% chance per episode

    # VIO noise
    vio_noise_enabled: bool = True
    vio_noise_scale: float = 0.0  # Additional position drift (meters)

    # Wind disturbance
    wind_enabled: bool = True
    wind_speed_range: Tuple[float, float] = (0.0, 5.0)  # m/s magnitude range


@dataclass
class EpisodeDRState:
    """Domain randomization state sampled for an episode."""
    battery_drain_multiplier: float = 1.0
    gnss_degradation_step: int = -1  # Step at which GNSS degrades (-1 = never)
    wind_vector: np.ndarray = None
    vio_noise_scale: float = 0.0

    def __post_init__(self):
        if self.wind_vector is None:
            self.wind_vector = np.zeros(3)


class DomainRandomizationManager:
    """
    Manages domain randomization sampling and effects.

    Responsibilities:
    - Sample DR parameters at episode start
    - Apply DR effects during state updates
    - Provide DR stats for logging
    """

    def __init__(self, config: DomainRandomizationConfig, max_mission_time: float = 900.0):
        """
        Initialize DomainRandomizationManager.

        Args:
            config: DR configuration
            max_mission_time: Maximum mission time in seconds
        """
        self.config = config
        self.max_mission_time = max_mission_time
        self._rng = np.random.default_rng()
        self._episode_state = EpisodeDRState()

    def sample_episode(self, seed: int = None) -> EpisodeDRState:
        """
        Sample domain randomization values for a new episode.

        Args:
            seed: Optional random seed for reproducibility

        Returns:
            Episode DR state
        """
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        state = EpisodeDRState()

        if not self.config.enabled:
            self._episode_state = state
            return state

        # Battery drain rate variation
        if self.config.battery_variation_enabled:
            var = self.config.battery_drain_rate_variation
            state.battery_drain_multiplier = self._rng.uniform(1.0 - var, 1.0 + var)

        # GNSS degradation
        if self.config.gnss_degradation_enabled:
            if self._rng.random() < self.config.gnss_degradation_prob:
                # Degradation at random point in first 80% of episode
                max_steps = int(self.max_mission_time * 60 * 0.8)
                state.gnss_degradation_step = self._rng.integers(0, max(1, max_steps))

        # Wind disturbance
        if self.config.wind_enabled:
            min_speed, max_speed = self.config.wind_speed_range
            wind_magnitude = self._rng.uniform(min_speed, max_speed)
            wind_direction = self._rng.uniform(0, 2 * np.pi)
            state.wind_vector = np.array([
                wind_magnitude * np.cos(wind_direction),
                wind_magnitude * np.sin(wind_direction),
                0.0  # Horizontal wind only
            ])

        # VIO noise
        if self.config.vio_noise_enabled:
            state.vio_noise_scale = self.config.vio_noise_scale

        self._episode_state = state
        return state

    def apply_vio_drift(
        self,
        current_drift: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Apply VIO position drift accumulation.

        Args:
            current_drift: Current accumulated drift
            dt: Time delta

        Returns:
            Updated drift vector
        """
        if self._episode_state.vio_noise_scale <= 0:
            return current_drift

        # Random walk drift
        drift_increment = self._rng.normal(
            0, self._episode_state.vio_noise_scale * dt, 3
        )
        new_drift = current_drift + drift_increment

        # Cap at realistic bounds
        max_drift = 10.0  # meters
        drift_magnitude = np.linalg.norm(new_drift)
        if drift_magnitude > max_drift:
            new_drift *= (max_drift / drift_magnitude)

        return new_drift

    def should_trigger_gnss_degradation(self, step_count: int) -> bool:
        """Check if GNSS degradation should trigger at this step."""
        return (
            self._episode_state.gnss_degradation_step >= 0 and
            step_count >= self._episode_state.gnss_degradation_step
        )

    def get_stats(self) -> dict:
        """Get DR statistics for logging."""
        wind_magnitude = np.linalg.norm(self._episode_state.wind_vector)
        wind_direction_deg = np.degrees(np.arctan2(
            self._episode_state.wind_vector[1],
            self._episode_state.wind_vector[0]
        )) if wind_magnitude > 0 else 0.0

        return {
            'dr_enabled': self.config.enabled,
            'battery_drain_multiplier': self._episode_state.battery_drain_multiplier,
            'gnss_degradation_step': self._episode_state.gnss_degradation_step,
            'wind_magnitude_mps': wind_magnitude,
            'wind_direction_deg': wind_direction_deg,
            'vio_noise_scale': self._episode_state.vio_noise_scale,
        }

    @property
    def episode_state(self) -> EpisodeDRState:
        """Current episode DR state."""
        return self._episode_state
