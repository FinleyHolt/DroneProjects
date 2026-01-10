"""
Trajectory Optimizer RL environment (Level 3, 0.1s decision horizon).

This environment handles low-level trajectory optimization:
- Velocity command generation
- Attitude rate adjustments
- Smooth trajectory following

The agent outputs velocity setpoints that are sent to the flight controller,
considering:
- Current velocity and acceleration limits
- Waypoint tracking error
- Wind disturbance compensation
- Gimbal stabilization requirements

Ontology integration:
- Respects velocity/acceleration limits from platform ontology
- Publishes trajectory state facts
- Filters commands through safety constraints
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Any, Dict, Optional, Tuple

from ontology_rl.envs.base_env import FlybyBaseEnv


class TrajectoryOptimizerEnv(FlybyBaseEnv):
    """
    Level 3 Trajectory Optimizer environment.

    Observation Space (15-dimensional):
        - UAV position (x, y, z) - meters
        - UAV velocity (vx, vy, vz) - m/s
        - Target position relative (dx, dy, dz) - meters
        - Target velocity (desired vx, vy, vz) - m/s
        - Heading error - radians
        - Pitch error - radians
        - Roll rate - rad/s

    Action Space (Continuous, 4-dimensional):
        - Velocity x command (-10 to 10 m/s)
        - Velocity y command (-10 to 10 m/s)
        - Velocity z command (-5 to 5 m/s)
        - Yaw rate command (-1 to 1 rad/s)

    Reward Structure:
        - Position tracking: -distance_error^2
        - Velocity smoothness: -jerk penalty
        - Command effort: -0.01 * ||action||^2
        - Constraint violation: -50 per violation
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize trajectory optimizer environment.

        Args:
            config: Configuration dict with optional keys:
                - max_steps: Maximum steps per episode (default: 200)
                - max_velocity: Maximum velocity magnitude (default: 10.0)
                - max_acceleration: Maximum acceleration (default: 5.0)
                - use_ros: Enable ROS 2 integration (default: True)
        """
        config = config or {}
        config.setdefault('max_steps', 200)
        config.setdefault('max_velocity', 10.0)
        config.setdefault('max_acceleration', 5.0)

        super().__init__(config=config, node_name='trajectory_optimizer_env')

        # Trajectory state
        self._target_position: np.ndarray = np.array([50.0, 0.0, 50.0])
        self._target_velocity: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._heading: float = 0.0
        self._pitch: float = 0.0
        self._roll_rate: float = 0.0

        # Previous state for reward calculation
        self._prev_velocity: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._prev_action: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0])

        # Limits from ontology
        self._max_velocity = config.get('max_velocity', 10.0)
        self._max_acceleration = config.get('max_acceleration', 5.0)

    def _define_spaces(self):
        """Define observation and action spaces."""
        self.observation_space = spaces.Box(
            low=np.array([
                -100.0, -100.0, 0.0,       # position
                -15.0, -15.0, -10.0,       # velocity
                -100.0, -100.0, -100.0,    # target relative
                -15.0, -15.0, -10.0,       # target velocity
                -np.pi,                    # heading error
                -np.pi/2,                  # pitch error
                -5.0                       # roll rate
            ], dtype=np.float32),
            high=np.array([
                100.0, 100.0, 120.0,       # position
                15.0, 15.0, 10.0,          # velocity
                100.0, 100.0, 100.0,       # target relative
                15.0, 15.0, 10.0,          # target velocity
                np.pi,                     # heading error
                np.pi/2,                   # pitch error
                5.0                        # roll rate
            ], dtype=np.float32),
            dtype=np.float32
        )

        # Action: continuous velocity commands
        self.action_space = spaces.Box(
            low=np.array([-10.0, -10.0, -5.0, -1.0], dtype=np.float32),
            high=np.array([10.0, 10.0, 5.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Reset environment for new episode."""
        obs, info = super().reset(seed=seed, options=options)

        # Generate random target position
        self._target_position = np.array([
            self.np_random.uniform(20.0, 80.0),
            self.np_random.uniform(-30.0, 30.0),
            self.np_random.uniform(30.0, 70.0)
        ])

        # Target velocity (usually toward waypoint)
        direction = self._target_position - self.get_safe_position()
        direction = direction / (np.linalg.norm(direction) + 1e-6)
        self._target_velocity = direction * 5.0  # 5 m/s target speed

        # Reset attitude
        self._heading = self.np_random.uniform(-0.5, 0.5)
        self._pitch = 0.0
        self._roll_rate = 0.0

        # Reset tracking
        self._prev_velocity = self.get_safe_velocity()
        self._prev_action = np.zeros(4)

        obs = self._get_observation()

        info.update({
            'target_position': self._target_position.tolist(),
            'target_velocity': self._target_velocity.tolist()
        })

        return obs, info

    def _execute_action(self, action: np.ndarray):
        """Execute velocity command action."""
        self._prev_velocity = self.get_safe_velocity()
        self._prev_action = action.copy()

        # Apply velocity command (simulate in standalone mode)
        if not self.use_ros:
            # Simple kinematic simulation
            pos = self.get_safe_position()
            vel_cmd = action[:3]

            # Apply acceleration limits
            current_vel = self.get_safe_velocity()
            vel_delta = vel_cmd - current_vel
            vel_delta_mag = np.linalg.norm(vel_delta)

            dt = 0.1  # 10 Hz control
            max_delta = self._max_acceleration * dt

            if vel_delta_mag > max_delta:
                vel_delta = vel_delta / vel_delta_mag * max_delta

            new_vel = current_vel + vel_delta
            new_pos = pos + new_vel * dt

            # Apply velocity limits
            vel_mag = np.linalg.norm(new_vel)
            if vel_mag > self._max_velocity:
                new_vel = new_vel / vel_mag * self._max_velocity

            # Update state
            self._current_pose = np.array([
                new_pos[0], new_pos[1], new_pos[2],
                0.0, 0.0, 0.0, 1.0
            ])
            self._current_velocity = new_vel

            # Update heading
            self._heading += action[3] * dt

        # Battery consumption (proportional to effort)
        effort = np.linalg.norm(action[:3])
        self._current_battery -= 0.01 * effort
        self._current_battery = max(0.0, self._current_battery)

    def filter_action(self, action: np.ndarray) -> np.ndarray:
        """Filter action through velocity and acceleration constraints."""
        filtered = action.copy()

        # Apply velocity limits
        vel_cmd = filtered[:3]
        vel_mag = np.linalg.norm(vel_cmd)
        if vel_mag > self._max_velocity:
            filtered[:3] = vel_cmd / vel_mag * self._max_velocity

        # Apply acceleration limits
        current_vel = self.get_safe_velocity()
        vel_delta = filtered[:3] - current_vel
        vel_delta_mag = np.linalg.norm(vel_delta)

        dt = 0.1
        max_delta = self._max_acceleration * dt

        if vel_delta_mag > max_delta:
            vel_delta = vel_delta / vel_delta_mag * max_delta
            filtered[:3] = current_vel + vel_delta

        return filtered

    def _get_observation(self) -> np.ndarray:
        """Build observation vector."""
        pos = self.get_safe_position()
        vel = self.get_safe_velocity()

        # Target relative position
        target_rel = self._target_position - pos

        # Heading error (angle to target)
        target_heading = np.arctan2(target_rel[1], target_rel[0])
        heading_error = target_heading - self._heading
        # Wrap to [-pi, pi]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        return np.array([
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            target_rel[0], target_rel[1], target_rel[2],
            self._target_velocity[0], self._target_velocity[1], self._target_velocity[2],
            heading_error,
            self._pitch,
            self._roll_rate
        ], dtype=np.float32)

    def _compute_reward(self) -> float:
        """Compute reward for trajectory tracking."""
        reward = 0.0

        pos = self.get_safe_position()
        vel = self.get_safe_velocity()

        # Position tracking error (squared)
        position_error = np.linalg.norm(self._target_position - pos)
        reward -= position_error ** 2 * 0.01

        # Velocity tracking
        velocity_error = np.linalg.norm(self._target_velocity - vel)
        reward -= velocity_error ** 2 * 0.05

        # Jerk penalty (change in acceleration)
        jerk = np.linalg.norm(vel - self._prev_velocity) / 0.1  # Approximation
        reward -= jerk * 0.01

        # Command effort
        reward -= 0.01 * np.linalg.norm(self._prev_action) ** 2

        # Bonus for reaching target
        if position_error < 1.0:
            reward += 10.0

        # Constraint violation penalty
        with self._ros_lock:
            reward -= len(self._safety_violations) * 50.0

        return reward

    def _is_terminated(self) -> bool:
        """Check if episode should terminate."""
        # Reached target
        pos = self.get_safe_position()
        if np.linalg.norm(self._target_position - pos) < 0.5:
            return True

        # Geofence violation
        if not self.check_geofence(pos):
            return True

        # Safety violations
        with self._ros_lock:
            if len(self._safety_violations) > 0:
                return True

        # Battery depleted
        if self._current_battery <= 5.0:
            return True

        return False


# Register environment with Gymnasium
gym.register(
    id='FlybyTrajectoryOptimizer-v0',
    entry_point='ontology_rl.envs:TrajectoryOptimizerEnv',
    max_episode_steps=200,
)
