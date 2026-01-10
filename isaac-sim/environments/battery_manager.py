"""
Battery Manager - Battery state and reserve calculations.

Single responsibility: Track battery state and compute required reserves.
"""

import numpy as np

from .base_isr_env import UAVState, CommsStatus


class BatteryManager:
    """
    Manages battery state and reserve calculations.

    Responsibilities:
    - Track battery percentage
    - Apply drain rate with domain randomization
    - Calculate required reserve based on distance to home
    - Handle comms-denied reserve multiplier
    """

    # F-11 flight parameters
    CRUISE_SPEED = 6.3      # m/s
    BASE_DRAIN_RATE = 0.1   # % per second at cruise
    SAFETY_FACTOR = 1.5     # Buffer for wind, altitude, maneuvering
    MIN_RESERVE = 10.0      # % hard floor for landing

    def __init__(self, home_position: np.ndarray):
        """
        Initialize BatteryManager.

        Args:
            home_position: Home/spawn position for reserve calculations
        """
        self.home_position = home_position.copy()
        self._drain_multiplier = 1.0

    def set_drain_multiplier(self, multiplier: float) -> None:
        """Set battery drain rate multiplier for domain randomization."""
        self._drain_multiplier = multiplier

    def update(self, state: UAVState, dt: float) -> float:
        """
        Update battery state with drain.

        Args:
            state: Current UAV state
            dt: Time delta in seconds

        Returns:
            New battery percentage
        """
        actual_drain_rate = self.BASE_DRAIN_RATE * self._drain_multiplier
        new_battery = state.battery_pct - actual_drain_rate * dt
        return max(0.0, new_battery)

    def compute_required_reserve(
        self,
        position: np.ndarray,
        comms_status: CommsStatus,
    ) -> float:
        """
        Compute required battery reserve based on distance to home.

        Uses F-11 flight characteristics to calculate minimum battery needed
        to safely return to launch point.

        Formula: reserve = (distance / cruise_speed) * power_rate * safety_factor

        Args:
            position: Current UAV position
            comms_status: Current communications status

        Returns:
            Required battery reserve percentage
        """
        # Calculate 3D distance to home
        distance_to_home = np.linalg.norm(position - self.home_position)

        # Time to fly home at cruise speed
        time_to_home = distance_to_home / self.CRUISE_SPEED

        # Battery needed to return
        required = time_to_home * self.BASE_DRAIN_RATE * self.SAFETY_FACTOR

        # Apply drain multiplier (if drain is faster, need more reserve)
        required *= self._drain_multiplier

        # Apply comms-denied multiplier (more conservative when autonomous)
        if comms_status == CommsStatus.DENIED:
            required *= 1.2  # 20% extra buffer

        return max(self.MIN_RESERVE, required)

    def is_rtl_required(
        self,
        battery_pct: float,
        required_reserve: float,
    ) -> bool:
        """
        Check if RTL is required due to battery.

        Args:
            battery_pct: Current battery percentage
            required_reserve: Required reserve percentage

        Returns:
            True if RTL should be triggered
        """
        return battery_pct <= required_reserve

    def is_emergency_required(self, battery_pct: float) -> bool:
        """
        Check if emergency landing is required.

        Args:
            battery_pct: Current battery percentage

        Returns:
            True if emergency landing should be triggered
        """
        # Emergency at 50% of minimum reserve
        emergency_threshold = self.MIN_RESERVE * 0.5
        return battery_pct <= emergency_threshold
