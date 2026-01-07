"""
Observation Space Manager - Compute observation bounds for Gymnasium.

Single responsibility: Define and compute observation space bounds.
"""

from typing import Tuple

import numpy as np

from .base_isr_env import BaseISREnvironment


class ObservationSpaceManager:
    """
    Manages observation space bounds computation for Gymnasium.

    Responsibilities:
    - Compute proper bounds for state observations
    - Compute bounds for perception observations
    - Handle varying observation dimensions
    """

    # Base state dimensions breakdown:
    # position (3) + velocity (3) + orientation (4) + wind (3) + scalars (15) = 28

    def __init__(self, env: BaseISREnvironment):
        """
        Initialize ObservationSpaceManager.

        Args:
            env: Base ISR environment
        """
        self.env = env
        self.obs_dim = env.observation_dim

    def compute_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute observation space bounds.

        Returns:
            Tuple of (low, high) bounds arrays
        """
        obs_low = np.zeros(self.obs_dim, dtype=np.float32)
        obs_high = np.ones(self.obs_dim, dtype=np.float32)

        # Position (0-2): geofence bounds
        gf = self.env.config.geofence
        obs_low[0:3] = [gf.min_x, gf.min_y, gf.min_z]
        obs_high[0:3] = [gf.max_x, gf.max_y, gf.max_z]

        # Velocity (3-5): ±10 m/s
        obs_low[3:6] = -10.0
        obs_high[3:6] = 10.0

        # Orientation quaternion (6-9): [-1, 1]
        obs_low[6:10] = -1.0
        obs_high[6:10] = 1.0

        # Wind (10-12): [-1, 1] normalized
        obs_low[10:13] = -1.0
        obs_high[10:13] = 1.0

        # Scalars (13-27)
        # battery_pct/100, mission_time/max, coverage_pct/100: [0, 1]
        obs_low[13:16] = 0.0
        obs_high[13:16] = 1.0

        # in_geofence, in_nfz: [0, 1]
        obs_low[16:18] = 0.0
        obs_high[16:18] = 1.0

        # nfz_distance/100: [0, inf] but typically [0, 2]
        obs_low[18] = 0.0
        obs_high[18] = 2.0

        # in_threat_zone, threat_level/3, threat_exposure/max: [0, 1]
        obs_low[19:22] = 0.0
        obs_high[19:22] = 1.0

        # autonomous_mode, comms_status/2, gnss_status/2, vio_valid: [0, 1]
        obs_low[22:26] = 0.0
        obs_high[22:26] = 1.0

        # gnss_degradation_triggered: [0, 1]
        obs_low[26] = 0.0
        obs_high[26] = 1.0

        # battery_drain_multiplier: [0.5, 1.5]
        obs_low[27] = 0.5
        obs_high[27] = 1.5

        # Perception observations (516 dims if enabled)
        if self.obs_dim > 28:
            self._set_perception_bounds(obs_low, obs_high)

        return obs_low, obs_high

    def _set_perception_bounds(
        self,
        obs_low: np.ndarray,
        obs_high: np.ndarray,
    ) -> None:
        """Set bounds for perception observations."""
        if hasattr(self.env, 'perception') and self.env.perception is not None:
            if hasattr(self.env.perception, 'get_observation_space_bounds'):
                p_low, p_high = self.env.perception.get_observation_space_bounds()
                obs_low[28:] = p_low
                obs_high[28:] = p_high
            else:
                # Default perception bounds
                obs_low[28:] = -1.0
                obs_high[28:] = 1.0
        else:
            obs_low[28:] = -1.0
            obs_high[28:] = 1.0
