"""
Mission Planner RL environment (Level 1, 10s decision horizon).

This environment handles strategic mission planning decisions:
- Waypoint selection and sequencing
- Mission adaptation based on conditions
- High-level goal prioritization

The agent selects which waypoint to navigate to next, considering:
- Current position and battery state
- Mission progress and remaining waypoints
- No-fly zone avoidance
- Time constraints

Ontology integration:
- Queries Vampire for waypoint validity
- Publishes mission state facts
- Respects geofence and NFZ constraints
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Any, Dict, List, Optional, Tuple

from ontology_rl.envs.base_env import FlybyBaseEnv


class MissionPlannerEnv(FlybyBaseEnv):
    """
    Level 1 Mission Planner environment.

    Observation Space (8-dimensional):
        - UAV position (x, y, z) - meters
        - Battery level (0-100) - percentage
        - Mission progress (0-1) - fraction complete
        - Current waypoint index
        - Remaining waypoints count
        - Time elapsed - seconds

    Action Space (Discrete, 4 options):
        - 0: Navigate to next waypoint
        - 1: Skip next waypoint
        - 2: Return to previous waypoint
        - 3: Abort mission (return to home)

    Reward Structure:
        - Progress reward: +10 * progress_delta
        - Efficiency penalty: -0.01 per step
        - Safety violation penalty: -100 per violation
        - Mission completion bonus: +100
        - Battery-aware return bonus: +50 (if RTH when battery < 30%)
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize mission planner environment.

        Args:
            config: Configuration dict with optional keys:
                - max_steps: Maximum steps per episode (default: 1000)
                - num_waypoints: Number of waypoints to generate (default: 5)
                - world_size: Size of world in meters (default: 100)
                - use_ros: Enable ROS 2 integration (default: True)
        """
        config = config or {}
        config.setdefault('max_steps', 1000)
        config.setdefault('num_waypoints', 5)
        config.setdefault('world_size', 100.0)

        super().__init__(config=config, node_name='mission_planner_env')

        # Mission state
        self._waypoints: List[np.ndarray] = []
        self._current_wp_idx: int = 0
        self._visited_waypoints: set = set()
        self._mission_start_time: float = 0.0

        # NFZ regions (from ontology - list of [center_x, center_y, radius])
        self._nfz_regions = [
            np.array([-50.0, 50.0, 30.0]),  # NFZ at (-50, 50) with 30m radius
        ]

        # Previous state for reward calculation
        self._prev_wp_idx: int = 0
        self._prev_battery: float = 100.0

    def _define_spaces(self):
        """Define observation and action spaces."""
        # Observation: [x, y, z, battery, progress, wp_idx, remaining, time]
        world_size = self.config.get('world_size', 100.0)

        self.observation_space = spaces.Box(
            low=np.array([
                -world_size,  # x
                -world_size,  # y
                0.0,          # z
                0.0,          # battery
                0.0,          # progress
                0.0,          # wp_idx
                0.0,          # remaining
                0.0           # time
            ], dtype=np.float32),
            high=np.array([
                world_size,   # x
                world_size,   # y
                120.0,        # z (max altitude)
                100.0,        # battery
                1.0,          # progress
                100.0,        # wp_idx
                100.0,        # remaining
                3600.0        # time (1 hour max)
            ], dtype=np.float32),
            dtype=np.float32
        )

        # Action: discrete waypoint selection
        # 0=next, 1=skip, 2=previous, 3=abort
        self.action_space = spaces.Discrete(4)

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Reset environment and generate new mission."""
        obs, info = super().reset(seed=seed, options=options)

        # Generate new mission waypoints
        self._waypoints = self._generate_mission()
        self._current_wp_idx = 0
        self._visited_waypoints = set()
        self._mission_start_time = self._episode_start_time

        # Reset tracking
        self._prev_wp_idx = 0
        self._prev_battery = self._current_battery

        # Publish initial mission facts to ontology
        self._publish_mission_facts()

        # Update observation with mission state
        obs = self._get_observation()

        info.update({
            'num_waypoints': len(self._waypoints),
            'waypoints': [wp.tolist() for wp in self._waypoints]
        })

        return obs, info

    def _generate_mission(self) -> List[np.ndarray]:
        """
        Generate random mission waypoints avoiding NFZ regions.

        Returns:
            List of waypoint positions [x, y, z]
        """
        num_waypoints = self.config.get('num_waypoints', 5)
        world_size = self.config.get('world_size', 100.0)
        np_random = self.np_random

        waypoints = []
        attempts = 0
        max_attempts = 100

        while len(waypoints) < num_waypoints and attempts < max_attempts:
            attempts += 1

            # Generate random waypoint
            x = np_random.uniform(-world_size * 0.8, world_size * 0.8)
            y = np_random.uniform(-world_size * 0.8, world_size * 0.8)
            z = np_random.uniform(30.0, 80.0)  # Safe altitude range

            candidate = np.array([x, y, z])

            # Check against NFZ regions
            valid = True
            for nfz in self._nfz_regions:
                dist = np.linalg.norm(candidate[:2] - nfz[:2])
                if dist < nfz[2]:  # Inside NFZ
                    valid = False
                    break

            if valid:
                waypoints.append(candidate)

        # If we couldn't generate enough, use a simple pattern
        if len(waypoints) < num_waypoints:
            default_wps = [
                np.array([0.0, 0.0, 50.0]),
                np.array([50.0, 0.0, 50.0]),
                np.array([50.0, 50.0, 50.0]),
                np.array([0.0, 50.0, 50.0]),
                np.array([0.0, 0.0, 50.0]),
            ]
            waypoints = default_wps[:num_waypoints]

        return waypoints

    def _publish_mission_facts(self):
        """Publish mission state as TPTP facts."""
        # Mission waypoints
        for i, wp in enumerate(self._waypoints):
            fact = f"fof(wp_{i}, axiom, waypoint(wp{i}, {wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}))."
            self.publish_fact(fact)

        # Current mission state
        fact = f"fof(mission_state, axiom, missionActive(true))."
        self.publish_fact(fact)

    def _execute_action(self, action: int):
        """
        Execute waypoint selection action.

        Args:
            action: Discrete action (0=next, 1=skip, 2=previous, 3=abort)
        """
        self._prev_wp_idx = self._current_wp_idx
        self._prev_battery = self._current_battery

        if action == 0:  # Navigate to next waypoint
            if self._current_wp_idx < len(self._waypoints) - 1:
                self._visited_waypoints.add(self._current_wp_idx)
                self._current_wp_idx += 1

        elif action == 1:  # Skip next waypoint
            if self._current_wp_idx < len(self._waypoints) - 2:
                self._visited_waypoints.add(self._current_wp_idx)
                self._current_wp_idx += 2
            elif self._current_wp_idx < len(self._waypoints) - 1:
                self._visited_waypoints.add(self._current_wp_idx)
                self._current_wp_idx = len(self._waypoints) - 1

        elif action == 2:  # Return to previous waypoint
            if self._current_wp_idx > 0:
                self._current_wp_idx -= 1

        elif action == 3:  # Abort mission (return to home)
            self._current_wp_idx = len(self._waypoints)  # Signal mission end

        # Simulate battery consumption based on distance
        if self._current_wp_idx < len(self._waypoints) and self._prev_wp_idx < len(self._waypoints):
            if self._current_wp_idx != self._prev_wp_idx:
                distance = np.linalg.norm(
                    self._waypoints[min(self._current_wp_idx, len(self._waypoints)-1)] -
                    self._waypoints[self._prev_wp_idx]
                )
                # ~0.5% battery per 10m of travel
                self._current_battery -= distance * 0.05
                self._current_battery = max(0.0, self._current_battery)

        # Publish updated state
        if self.use_ros and self._action_pub is not None:
            from std_msgs.msg import String
            msg = String()
            msg.data = f"mission_action:{action}:wp_idx:{self._current_wp_idx}"
            self._action_pub.publish(msg)

    def _get_observation(self) -> np.ndarray:
        """Build observation vector."""
        pos = self.get_safe_position()

        # Calculate progress
        total_wps = max(1, len(self._waypoints))
        progress = len(self._visited_waypoints) / total_wps

        # Time elapsed
        elapsed = self._episode_step  # Use step count as time proxy

        remaining = max(0, len(self._waypoints) - self._current_wp_idx)

        return np.array([
            pos[0],                    # x
            pos[1],                    # y
            pos[2],                    # z
            self._current_battery,     # battery
            progress,                  # progress
            float(self._current_wp_idx),  # wp_idx
            float(remaining),          # remaining
            float(elapsed)             # time
        ], dtype=np.float32)

    def _compute_reward(self) -> float:
        """
        Compute reward for mission planning decisions.

        Reward structure:
        - Progress: +10 for each new waypoint visited
        - Efficiency: -0.01 per step (encourage speed)
        - Safety violations: -100 per violation
        - Mission complete: +100 bonus
        - Smart battery management: +50 for RTH when battery < 30%
        """
        reward = 0.0

        # Progress reward (new waypoints visited)
        new_visits = len(self._visited_waypoints) - (
            len(self._visited_waypoints) - 1 if self._prev_wp_idx in self._visited_waypoints else len(self._visited_waypoints)
        )
        if self._prev_wp_idx != self._current_wp_idx and self._current_wp_idx < len(self._waypoints):
            reward += 10.0

        # Efficiency penalty
        reward -= 0.01

        # Safety violation penalty
        with self._ros_lock:
            new_violations = len(self._safety_violations)
        reward -= new_violations * 100.0

        # Mission completion bonus
        if self._current_wp_idx >= len(self._waypoints) - 1:
            if len(self._visited_waypoints) >= len(self._waypoints) - 1:
                reward += 100.0

        # Battery management reward
        if self._current_battery < 30.0 and self._current_wp_idx >= len(self._waypoints):
            # Returned home with low battery - smart decision
            reward += 50.0
        elif self._current_battery < 10.0:
            # Very low battery without returning - penalty
            reward -= 50.0

        return reward

    def _is_terminated(self) -> bool:
        """Check if episode should terminate."""
        # Mission complete (all waypoints visited or aborted)
        if self._current_wp_idx >= len(self._waypoints):
            return True

        # Safety violation (critical)
        with self._ros_lock:
            if len(self._safety_violations) > 0:
                return True

        # Battery depleted
        if self._current_battery <= 5.0:
            return True

        # Geofence violation
        pos = self.get_safe_position()
        if not self.check_geofence(pos):
            return True

        return False

    def filter_action(self, action: int) -> int:
        """
        Filter action through ontology constraints.

        Prevents actions that would violate:
        - Geofence boundaries
        - NFZ regions
        - Battery constraints

        Args:
            action: Raw action from agent

        Returns:
            Filtered action (may be modified to prevent violations)
        """
        # If battery is critically low, force return to home
        if self._current_battery < 15.0:
            return 3  # Force abort/RTH

        # Check if target waypoint is in NFZ
        if action in [0, 1]:  # Next or skip
            target_idx = min(
                self._current_wp_idx + (1 if action == 0 else 2),
                len(self._waypoints) - 1
            )
            if target_idx < len(self._waypoints):
                target_wp = self._waypoints[target_idx]

                # Check NFZ
                for nfz in self._nfz_regions:
                    dist = np.linalg.norm(target_wp[:2] - nfz[:2])
                    if dist < nfz[2]:
                        # Target in NFZ - skip it instead
                        return 1 if action == 0 else 3

                # Check geofence
                if not self.check_geofence(target_wp):
                    return 2  # Go back instead

        return action

    def get_current_waypoint(self) -> Optional[np.ndarray]:
        """Get current target waypoint position."""
        if 0 <= self._current_wp_idx < len(self._waypoints):
            return self._waypoints[self._current_wp_idx].copy()
        return None

    def get_waypoints(self) -> List[np.ndarray]:
        """Get all mission waypoints."""
        return [wp.copy() for wp in self._waypoints]


# Register environment with Gymnasium
gym.register(
    id='FlybyMissionPlanner-v0',
    entry_point='ontology_rl.envs:MissionPlannerEnv',
    max_episode_steps=1000,
)
