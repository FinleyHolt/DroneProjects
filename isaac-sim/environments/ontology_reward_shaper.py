"""
Ontology Reward Shaper - Reward shaping for ontology-mandated behaviors.

Single responsibility: Compute shaped rewards when ontology preempts RL.
"""

from typing import Optional

import numpy as np

from .base_isr_env import UAVState
from .ontology_behavior_controller import OntologyBehavior, BehaviorCommand


class OntologyRewardShaper:
    """
    Applies reward shaping when ontology behaviors are active.

    The goal is to:
    1. Not penalize agent for ontology-mandated behaviors
    2. Give credit for safe behavior execution
    3. Penalize behaviors that led to ontology intervention

    Responsibilities:
    - Shape rewards based on active ontology behavior
    - Track behavior completion criteria
    - Integrate with ontology controller state
    """

    def __init__(
        self,
        home_position: np.ndarray,
        takeoff_altitude: float = 15.0,
        takeoff_tolerance: float = 1.0,
    ):
        """
        Initialize OntologyRewardShaper.

        Args:
            home_position: Home position for RTL progress calculation
            takeoff_altitude: Target takeoff altitude
            takeoff_tolerance: Takeoff altitude tolerance
        """
        self.home_position = home_position.copy()
        self.takeoff_altitude = takeoff_altitude
        self.takeoff_tolerance = takeoff_tolerance

    def shape_reward(
        self,
        base_reward: float,
        state: UAVState,
        command: Optional[BehaviorCommand],
    ) -> float:
        """
        Apply reward shaping for ontology behaviors.

        Args:
            base_reward: Reward from environment
            state: Current UAV state
            command: Active ontology behavior command

        Returns:
            Shaped reward
        """
        if command is None:
            return base_reward

        behavior = command.behavior

        if behavior == OntologyBehavior.TAKEOFF:
            return self._shape_takeoff_reward(state)

        elif behavior == OntologyBehavior.RTL:
            return self._shape_rtl_reward(state)

        elif behavior == OntologyBehavior.EMERGENCY_LAND:
            return self._shape_emergency_reward(state)

        elif behavior == OntologyBehavior.HOVER:
            return -0.01  # Small negative for being stuck

        elif behavior == OntologyBehavior.GEOFENCE_RECOVERY:
            return self._shape_geofence_recovery_reward(state)

        elif behavior == OntologyBehavior.NFZ_AVOIDANCE:
            return self._shape_nfz_avoidance_reward(state)

        elif behavior == OntologyBehavior.PERSON_AVOIDANCE:
            return -0.05  # Small negative during avoidance

        return base_reward

    def _shape_takeoff_reward(self, state: UAVState) -> float:
        """Shape reward during takeoff."""
        progress = min(1.0, state.position[2] / self.takeoff_altitude)
        return progress * 0.1

    def _shape_rtl_reward(self, state: UAVState) -> float:
        """Shape reward during RTL."""
        distance_to_home = np.linalg.norm(state.position - self.home_position)
        # Normalize by max mission distance (~300m typical)
        progress_reward = (1.0 - distance_to_home / 300.0) * 0.1
        return progress_reward

    def _shape_emergency_reward(self, state: UAVState) -> float:
        """Shape reward during emergency landing."""
        if state.position[2] < 0.5:
            return 1.0  # Successfully landed
        return 0.0  # Neutral during descent

    def _shape_geofence_recovery_reward(self, state: UAVState) -> float:
        """Shape reward during geofence recovery."""
        if state.in_geofence:
            return 0.5  # Successfully recovered
        return -0.1  # Still outside

    def _shape_nfz_avoidance_reward(self, state: UAVState) -> float:
        """Shape reward during NFZ avoidance."""
        if not state.in_nfz:
            return 0.5  # Successfully exited
        return -0.5  # Still inside (bad)

    def check_behavior_complete(
        self,
        state: UAVState,
        command: Optional[BehaviorCommand],
        person_avoidance_active: bool = False,
    ) -> bool:
        """
        Check if the active ontology behavior has completed.

        Args:
            state: Current UAV state
            command: Active behavior command
            person_avoidance_active: Whether person avoidance is active

        Returns:
            True if behavior is complete
        """
        if command is None:
            return False

        behavior = command.behavior

        if behavior == OntologyBehavior.TAKEOFF:
            target_alt = self.takeoff_altitude
            tolerance = self.takeoff_tolerance
            return state.position[2] >= target_alt - tolerance

        elif behavior == OntologyBehavior.RTL:
            distance_to_home = np.linalg.norm(state.position - self.home_position)
            return distance_to_home < 5.0 and state.position[2] < 2.0

        elif behavior == OntologyBehavior.EMERGENCY_LAND:
            return state.position[2] < 0.5

        elif behavior == OntologyBehavior.GEOFENCE_RECOVERY:
            return state.in_geofence

        elif behavior == OntologyBehavior.NFZ_AVOIDANCE:
            return not state.in_nfz and state.nfz_distance > 20.0

        elif behavior == OntologyBehavior.PERSON_AVOIDANCE:
            return not person_avoidance_active

        return False
