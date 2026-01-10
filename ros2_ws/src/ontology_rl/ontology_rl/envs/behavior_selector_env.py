"""
Behavior Selector RL environment (Level 2, 1s decision horizon).

This environment handles tactical behavior selection:
- Choosing between flight behaviors (hover, cruise, orbit, etc.)
- Reactive obstacle avoidance decisions
- Sensor mode selection

The agent selects which behavior primitive to execute, considering:
- Current waypoint and approach angle
- Obstacle proximity
- Wind conditions
- Gimbal/sensor requirements

Ontology integration:
- Queries Vampire for behavior compatibility
- Publishes behavior state facts
- Respects platform capability constraints
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Any, Dict, Optional, Tuple

from ontology_rl.envs.base_env import FlybyBaseEnv


class BehaviorSelectorEnv(FlybyBaseEnv):
    """
    Level 2 Behavior Selector environment.

    Observation Space (12-dimensional):
        - UAV position (x, y, z) - meters
        - UAV velocity (vx, vy, vz) - m/s
        - Target waypoint relative (dx, dy, dz) - meters
        - Obstacle distance - meters
        - Current behavior index
        - Battery level - percentage

    Action Space (Discrete, 6 options):
        - 0: Hover (hold position)
        - 1: Cruise (direct flight to waypoint)
        - 2: Orbit (circle around point)
        - 3: Approach (slow approach to waypoint)
        - 4: Evade (obstacle avoidance maneuver)
        - 5: Return (return to previous safe point)

    Reward Structure:
        - Progress toward waypoint: +1 per meter closer
        - Smooth transitions: +5 for compatible behavior changes
        - Obstacle avoidance: +10 for evading when necessary
        - Collision penalty: -100 for obstacle proximity < 2m
        - Efficiency: -0.1 per step
    """

    # Behavior IDs
    HOVER = 0
    CRUISE = 1
    ORBIT = 2
    APPROACH = 3
    EVADE = 4
    RETURN = 5

    BEHAVIOR_NAMES = ['hover', 'cruise', 'orbit', 'approach', 'evade', 'return']

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize behavior selector environment.

        Args:
            config: Configuration dict with optional keys:
                - max_steps: Maximum steps per episode (default: 500)
                - use_ros: Enable ROS 2 integration (default: True)
        """
        config = config or {}
        config.setdefault('max_steps', 500)

        super().__init__(config=config, node_name='behavior_selector_env')

        # Behavior state
        self._current_behavior: int = self.HOVER
        self._target_waypoint: np.ndarray = np.array([50.0, 0.0, 50.0])
        self._obstacle_distance: float = 100.0

        # Previous state for reward calculation
        self._prev_distance_to_wp: float = 0.0
        self._prev_behavior: int = self.HOVER

    def _define_spaces(self):
        """Define observation and action spaces."""
        self.observation_space = spaces.Box(
            low=np.array([
                -100.0, -100.0, 0.0,       # position
                -30.0, -30.0, -10.0,       # velocity
                -200.0, -200.0, -120.0,    # target relative
                0.0,                       # obstacle distance
                0.0,                       # current behavior
                0.0                        # battery
            ], dtype=np.float32),
            high=np.array([
                100.0, 100.0, 120.0,       # position
                30.0, 30.0, 10.0,          # velocity
                200.0, 200.0, 120.0,       # target relative
                200.0,                     # obstacle distance
                5.0,                       # current behavior
                100.0                      # battery
            ], dtype=np.float32),
            dtype=np.float32
        )

        # Action: discrete behavior selection
        self.action_space = spaces.Discrete(6)

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Reset environment for new episode."""
        obs, info = super().reset(seed=seed, options=options)

        # Reset behavior state
        self._current_behavior = self.HOVER

        # Generate random target waypoint
        self._target_waypoint = np.array([
            self.np_random.uniform(20.0, 80.0),
            self.np_random.uniform(-50.0, 50.0),
            self.np_random.uniform(30.0, 70.0)
        ])

        # Initialize obstacle distance
        self._obstacle_distance = self.np_random.uniform(10.0, 100.0)

        # Initialize tracking
        pos = self.get_safe_position()
        self._prev_distance_to_wp = np.linalg.norm(self._target_waypoint - pos)
        self._prev_behavior = self.HOVER

        obs = self._get_observation()

        info.update({
            'target_waypoint': self._target_waypoint.tolist(),
            'initial_behavior': self.BEHAVIOR_NAMES[self._current_behavior]
        })

        return obs, info

    def _execute_action(self, action: int):
        """Execute behavior selection action."""
        self._prev_behavior = self._current_behavior
        self._prev_distance_to_wp = np.linalg.norm(
            self._target_waypoint - self.get_safe_position()
        )

        # Update behavior
        self._current_behavior = action

        # Simulate behavior effects
        self._simulate_behavior_step()

    def _simulate_behavior_step(self):
        """Simulate one step of the current behavior."""
        pos = self.get_safe_position()
        vel = self.get_safe_velocity()

        # Simulate position change based on behavior
        if self._current_behavior == self.CRUISE:
            # Move toward waypoint
            direction = self._target_waypoint - pos
            direction = direction / (np.linalg.norm(direction) + 1e-6)
            new_pos = pos + direction * 2.0  # 2 m/s cruise
        elif self._current_behavior == self.APPROACH:
            # Slow approach
            direction = self._target_waypoint - pos
            direction = direction / (np.linalg.norm(direction) + 1e-6)
            new_pos = pos + direction * 0.5  # 0.5 m/s approach
        elif self._current_behavior == self.EVADE:
            # Move away from obstacle (assume obstacle ahead)
            new_pos = pos + np.array([0.0, 2.0, 0.0])  # Lateral evade
        elif self._current_behavior == self.RETURN:
            # Return to origin
            direction = -pos
            direction = direction / (np.linalg.norm(direction) + 1e-6)
            new_pos = pos + direction * 1.5
        else:
            # HOVER or ORBIT - minimal movement
            new_pos = pos

        # Update simulated pose (in standalone mode)
        if not self.use_ros:
            self._current_pose = np.array([
                new_pos[0], new_pos[1], new_pos[2],
                0.0, 0.0, 0.0, 1.0
            ])

        # Update obstacle distance (simulate dynamic obstacle)
        self._obstacle_distance += self.np_random.uniform(-5.0, 5.0)
        self._obstacle_distance = np.clip(self._obstacle_distance, 1.0, 200.0)

        # Battery consumption
        self._current_battery -= 0.05
        self._current_battery = max(0.0, self._current_battery)

    def _get_observation(self) -> np.ndarray:
        """Build observation vector."""
        pos = self.get_safe_position()
        vel = self.get_safe_velocity()

        # Target relative position
        target_rel = self._target_waypoint - pos

        return np.array([
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            target_rel[0], target_rel[1], target_rel[2],
            self._obstacle_distance,
            float(self._current_behavior),
            self._current_battery
        ], dtype=np.float32)

    def _compute_reward(self) -> float:
        """Compute reward for behavior selection."""
        reward = 0.0

        # Distance progress toward waypoint
        pos = self.get_safe_position()
        current_distance = np.linalg.norm(self._target_waypoint - pos)
        distance_delta = self._prev_distance_to_wp - current_distance
        reward += distance_delta * 1.0

        # Smooth behavior transitions
        compatible_transitions = {
            (self.HOVER, self.CRUISE),
            (self.HOVER, self.APPROACH),
            (self.CRUISE, self.APPROACH),
            (self.CRUISE, self.HOVER),
            (self.APPROACH, self.HOVER),
            (self.EVADE, self.HOVER),
        }
        if (self._prev_behavior, self._current_behavior) in compatible_transitions:
            reward += 5.0
        elif self._prev_behavior != self._current_behavior:
            reward -= 2.0  # Penalty for jarring transitions

        # Obstacle avoidance
        if self._obstacle_distance < 10.0:
            if self._current_behavior == self.EVADE:
                reward += 10.0  # Good - evading when needed
            else:
                reward -= 20.0  # Should be evading

        if self._obstacle_distance < 2.0:
            reward -= 100.0  # Collision

        # Efficiency penalty
        reward -= 0.1

        return reward

    def _is_terminated(self) -> bool:
        """Check if episode should terminate."""
        # Reached waypoint
        pos = self.get_safe_position()
        distance = np.linalg.norm(self._target_waypoint - pos)
        if distance < 2.0:
            return True

        # Collision
        if self._obstacle_distance < 1.0:
            return True

        # Battery depleted
        if self._current_battery <= 5.0:
            return True

        # Safety violations
        with self._ros_lock:
            if len(self._safety_violations) > 0:
                return True

        return False


# Register environment with Gymnasium
gym.register(
    id='FlybyBehaviorSelector-v0',
    entry_point='ontology_rl.envs:BehaviorSelectorEnv',
    max_episode_steps=500,
)
