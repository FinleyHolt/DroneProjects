#!/usr/bin/env python3
"""
Flyby F-11 Gymnasium Environment for Reinforcement Learning

A Gymnasium-compatible environment wrapper for UAV navigation training
using ArduPilot SITL and pymavlink.

Features:
- Configurable observation space (position, velocity, attitude, sensors)
- Configurable action space (velocity commands or waypoints)
- Reward function with task completion, safety, and efficiency components
- Episode reset mechanism with vehicle respawn
- MAVLink communication via pymavlink
- YAML-configurable parameters

Requirements:
- gymnasium
- pymavlink
- numpy
- pyyaml

Usage:
    from rl_gym_env import FlybyUAVEnv
    env = FlybyUAVEnv(config_path='configs/rl_training.yaml')
    obs, info = env.reset()
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
"""

import json
import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import gymnasium as gym
import numpy as np
import yaml
from gymnasium import spaces
from pymavlink import mavutil

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FlightMode(Enum):
    """ArduPilot flight modes."""
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    LAND = 9
    POSHOLD = 16


class TaskType(Enum):
    """Supported task types."""
    TRAJECTORY_TRACKING = "trajectory_tracking"
    WAYPOINT_NAVIGATION = "waypoint_navigation"
    HOVER = "hover"
    VELOCITY_TRACKING = "velocity_tracking"


@dataclass
class VehicleState:
    """Current vehicle state from MAVLink."""
    timestamp: float = 0.0
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    attitude: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll, pitch, yaw
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    battery_voltage: float = 0.0
    battery_remaining: float = 1.0
    armed: bool = False
    mode: str = ""
    connected: bool = False


@dataclass
class EpisodeStats:
    """Statistics for current episode."""
    total_reward: float = 0.0
    num_steps: int = 0
    start_time: float = 0.0
    collisions: int = 0
    geofence_violations: int = 0
    goal_reached: bool = False
    termination_reason: str = ""


class FlybyUAVEnv(gym.Env):
    """
    Gymnasium environment for Flyby F-11 UAV training.

    This environment wraps ArduPilot SITL via pymavlink for RL training.
    """

    metadata = {
        "render_modes": ["human", "rgb_array", "ansi"],
        "render_fps": 30,
    }

    def __init__(
        self,
        config_path: Optional[str] = None,
        config_dict: Optional[Dict] = None,
        render_mode: Optional[str] = None,
        instance_id: int = 0,
    ):
        """
        Initialize the UAV environment.

        Args:
            config_path: Path to YAML configuration file
            config_dict: Configuration dictionary (overrides config_path)
            render_mode: Rendering mode ('human', 'rgb_array', 'ansi')
            instance_id: SITL instance ID for parallel environments
        """
        super().__init__()

        self.instance_id = instance_id
        self.render_mode = render_mode

        # Load configuration
        self.config = self._load_config(config_path, config_dict)

        # Connection parameters
        self.connection_string = self.config.get(
            'connection_string', f'tcp:localhost:{5760 + instance_id}'
        )
        self.connection: Optional[mavutil.mavlink_connection] = None

        # Environment parameters
        self.dt = self.config.get('dt', 0.1)  # Control timestep (10 Hz default)
        self.max_episode_steps = self.config.get('max_episode_steps', 1000)
        self.task_type = TaskType(self.config.get('task_type', 'velocity_tracking'))

        # Vehicle constraints
        self.max_velocity = self.config.get('max_velocity', 10.0)
        self.max_altitude = self.config.get('max_altitude', 100.0)
        self.min_altitude = self.config.get('min_altitude', 2.0)
        self.takeoff_altitude = self.config.get('takeoff_altitude', 10.0)

        # Geofence (from world config)
        self.geofence_bounds = self.config.get('geofence_bounds', {
            'min': {'x': -125, 'y': -125, 'z': 0},
            'max': {'x': 125, 'y': 125, 'z': 100}
        })

        # No-fly zones
        self.no_fly_zones = self.config.get('no_fly_zones', [])

        # Spawn positions
        self.spawn_positions = self.config.get('spawn_positions', [
            {'x': 0, 'y': 0, 'z': 0, 'heading_deg': 0}
        ])

        # Reward weights
        self.reward_weights = self.config.get('reward_weights', {
            'position_tracking': 1.0,
            'velocity_tracking': 0.5,
            'smoothness': 0.1,
            'energy': 0.05,
            'collision': -100.0,
            'geofence': -50.0,
            'goal_reached': 100.0,
            'step_penalty': -0.01,
        })

        # Target for tracking tasks
        self.target_position = np.zeros(3)
        self.target_velocity = np.zeros(3)

        # State
        self.vehicle_state = VehicleState()
        self.prev_vehicle_state = VehicleState()
        self.episode_stats = EpisodeStats()
        self.current_step = 0

        # Define observation and action spaces
        self._setup_spaces()

        # Domain randomization (will be set externally if enabled)
        self.domain_randomization = None

        logger.info(f"FlybyUAVEnv initialized (instance {instance_id})")

    def _load_config(
        self,
        config_path: Optional[str],
        config_dict: Optional[Dict]
    ) -> Dict:
        """Load configuration from file or dictionary."""
        config = {}

        if config_path and Path(config_path).exists():
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            logger.info(f"Loaded config from {config_path}")

        if config_dict:
            config.update(config_dict)

        return config

    def _setup_spaces(self):
        """Define observation and action spaces."""
        # Observation space configuration
        obs_config = self.config.get('observation_space', {})

        # Build observation space dictionary
        obs_spaces = {}

        # Position (NED coordinates, in meters)
        if obs_config.get('include_position', True):
            obs_spaces['position'] = spaces.Box(
                low=np.array([-500, -500, -200]),
                high=np.array([500, 500, 0]),
                dtype=np.float32
            )

        # Velocity (NED, m/s)
        if obs_config.get('include_velocity', True):
            obs_spaces['velocity'] = spaces.Box(
                low=-self.max_velocity * np.ones(3),
                high=self.max_velocity * np.ones(3),
                dtype=np.float32
            )

        # Attitude (roll, pitch, yaw in radians)
        if obs_config.get('include_attitude', True):
            obs_spaces['attitude'] = spaces.Box(
                low=np.array([-np.pi, -np.pi/2, -np.pi]),
                high=np.array([np.pi, np.pi/2, np.pi]),
                dtype=np.float32
            )

        # Angular velocity (rad/s)
        if obs_config.get('include_angular_velocity', False):
            obs_spaces['angular_velocity'] = spaces.Box(
                low=-2*np.pi * np.ones(3),
                high=2*np.pi * np.ones(3),
                dtype=np.float32
            )

        # Target position (for tracking tasks)
        if obs_config.get('include_target_position', True):
            obs_spaces['target_position'] = spaces.Box(
                low=np.array([-500, -500, -200]),
                high=np.array([500, 500, 0]),
                dtype=np.float32
            )

        # Target velocity (for velocity tracking)
        if obs_config.get('include_target_velocity', False):
            obs_spaces['target_velocity'] = spaces.Box(
                low=-self.max_velocity * np.ones(3),
                high=self.max_velocity * np.ones(3),
                dtype=np.float32
            )

        # Position error (relative to target)
        if obs_config.get('include_position_error', True):
            obs_spaces['position_error'] = spaces.Box(
                low=-100 * np.ones(3),
                high=100 * np.ones(3),
                dtype=np.float32
            )

        # Battery state
        if obs_config.get('include_battery', False):
            obs_spaces['battery_remaining'] = spaces.Box(
                low=0.0, high=1.0, shape=(1,), dtype=np.float32
            )

        # Previous action (for smoothness)
        if obs_config.get('include_previous_action', True):
            obs_spaces['previous_action'] = spaces.Box(
                low=-self.max_velocity * np.ones(3),
                high=self.max_velocity * np.ones(3),
                dtype=np.float32
            )

        self.observation_space = spaces.Dict(obs_spaces)

        # Action space configuration
        action_config = self.config.get('action_space', {})
        action_type = action_config.get('type', 'velocity')

        if action_type == 'velocity':
            # Velocity commands [vx, vy, vz] in m/s (NED frame)
            max_vel = action_config.get('max_velocity', self.max_velocity)
            max_vz = action_config.get('max_vertical_velocity', 5.0)
            self.action_space = spaces.Box(
                low=np.array([-max_vel, -max_vel, -max_vz]),
                high=np.array([max_vel, max_vel, max_vz]),
                dtype=np.float32
            )
        elif action_type == 'position':
            # Position setpoint [x, y, z] relative to current position
            max_offset = action_config.get('max_position_offset', 10.0)
            self.action_space = spaces.Box(
                low=-max_offset * np.ones(3),
                high=max_offset * np.ones(3),
                dtype=np.float32
            )
        elif action_type == 'discrete':
            # Discrete actions for behavior selection
            self.action_space = spaces.Discrete(
                action_config.get('num_actions', 8)
            )

        # Store previous action for smoothness computation
        self.previous_action = np.zeros(3)

        logger.info(f"Observation space: {self.observation_space}")
        logger.info(f"Action space: {self.action_space}")

    def _connect_mavlink(self) -> bool:
        """Establish MAVLink connection to ArduPilot SITL."""
        try:
            logger.info(f"Connecting to {self.connection_string}...")
            self.connection = mavutil.mavlink_connection(
                self.connection_string,
                autoreconnect=True,
                source_system=255,
                source_component=0
            )

            # Wait for heartbeat
            msg = self.connection.wait_heartbeat(timeout=10)
            if msg is None:
                logger.error("No heartbeat received")
                return False

            logger.info(f"Connected! Heartbeat from system {self.connection.target_system}")
            self.vehicle_state.connected = True
            return True

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False

    def _disconnect_mavlink(self):
        """Close MAVLink connection."""
        if self.connection:
            self.connection.close()
            self.connection = None
            self.vehicle_state.connected = False

    def _update_vehicle_state(self, timeout: float = 0.1) -> bool:
        """
        Update vehicle state from MAVLink messages.

        Returns:
            True if state updated successfully
        """
        if not self.connection:
            return False

        # Store previous state
        self.prev_vehicle_state = VehicleState(
            timestamp=self.vehicle_state.timestamp,
            position=self.vehicle_state.position.copy(),
            velocity=self.vehicle_state.velocity.copy(),
            attitude=self.vehicle_state.attitude.copy(),
            angular_velocity=self.vehicle_state.angular_velocity.copy(),
            acceleration=self.vehicle_state.acceleration.copy(),
            battery_voltage=self.vehicle_state.battery_voltage,
            battery_remaining=self.vehicle_state.battery_remaining,
            armed=self.vehicle_state.armed,
            mode=self.vehicle_state.mode,
            connected=self.vehicle_state.connected,
        )

        self.vehicle_state.timestamp = time.time()

        # Request data streams if not already streaming
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            50,  # 50 Hz
            1    # Start streaming
        )

        # Read messages with timeout
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.connection.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.001)
                continue

            msg_type = msg.get_type()

            if msg_type == 'LOCAL_POSITION_NED':
                self.vehicle_state.position = np.array([msg.x, msg.y, msg.z])
                self.vehicle_state.velocity = np.array([msg.vx, msg.vy, msg.vz])

            elif msg_type == 'ATTITUDE':
                self.vehicle_state.attitude = np.array([
                    msg.roll, msg.pitch, msg.yaw
                ])
                self.vehicle_state.angular_velocity = np.array([
                    msg.rollspeed, msg.pitchspeed, msg.yawspeed
                ])

            elif msg_type == 'HEARTBEAT':
                self.vehicle_state.armed = (msg.base_mode &
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                self.vehicle_state.mode = mavutil.mode_string_v10(msg)

            elif msg_type == 'SYS_STATUS':
                self.vehicle_state.battery_voltage = msg.voltage_battery / 1000.0
                self.vehicle_state.battery_remaining = msg.battery_remaining / 100.0

            elif msg_type == 'SCALED_IMU':
                # Convert from mg to m/s^2 and mrad/s to rad/s
                self.vehicle_state.acceleration = np.array([
                    msg.xacc / 1000.0 * 9.81,
                    msg.yacc / 1000.0 * 9.81,
                    msg.zacc / 1000.0 * 9.81
                ])

        return True

    def _send_velocity_command(self, velocity: np.ndarray, yaw_rate: float = 0.0):
        """
        Send velocity setpoint command via MAVLink.

        Args:
            velocity: [vx, vy, vz] in NED frame (m/s)
            yaw_rate: Yaw rate (rad/s)
        """
        if not self.connection:
            return

        # Type mask: only use velocity
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

        self.connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,  # position (ignored)
            velocity[0], velocity[1], velocity[2],  # velocity
            0, 0, 0,  # acceleration (ignored)
            0,  # yaw (ignored)
            yaw_rate  # yaw_rate
        )

    def _send_position_command(self, position: np.ndarray, yaw: float = 0.0):
        """
        Send position setpoint command via MAVLink.

        Args:
            position: [x, y, z] in NED frame (m)
            yaw: Yaw angle (rad)
        """
        if not self.connection:
            return

        # Type mask: only use position
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            position[0], position[1], position[2],  # position
            0, 0, 0,  # velocity (ignored)
            0, 0, 0,  # acceleration (ignored)
            yaw,  # yaw
            0  # yaw_rate (ignored)
        )

    def _set_mode(self, mode: str) -> bool:
        """Set flight mode."""
        if not self.connection:
            return False

        mode_mapping = self.connection.mode_mapping()
        if mode not in mode_mapping:
            logger.error(f"Unknown mode: {mode}")
            return False

        mode_id = mode_mapping[mode]
        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        # Wait for mode change confirmation
        start_time = time.time()
        while time.time() - start_time < 5.0:
            self._update_vehicle_state(timeout=0.1)
            if self.vehicle_state.mode == mode:
                return True

        return False

    def _arm(self) -> bool:
        """Arm the vehicle."""
        if not self.connection:
            return False

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

        # Wait for arming
        start_time = time.time()
        while time.time() - start_time < 5.0:
            self._update_vehicle_state(timeout=0.1)
            if self.vehicle_state.armed:
                return True

        return False

    def _disarm(self) -> bool:
        """Disarm the vehicle."""
        if not self.connection:
            return False

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            21196,  # force disarm
            0, 0, 0, 0, 0
        )

        time.sleep(0.5)
        self._update_vehicle_state(timeout=0.1)
        return not self.vehicle_state.armed

    def _takeoff(self, altitude: float) -> bool:
        """Command takeoff to specified altitude."""
        if not self.connection:
            return False

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude
        )

        # Wait for takeoff (altitude in NED is negative)
        target_z = -altitude
        start_time = time.time()
        while time.time() - start_time < 30.0:
            self._update_vehicle_state(timeout=0.1)
            if self.vehicle_state.position[2] <= target_z * 0.9:
                return True

        return False

    def _check_geofence_violation(self) -> bool:
        """Check if vehicle is outside geofence."""
        pos = self.vehicle_state.position
        bounds = self.geofence_bounds

        if (pos[0] < bounds['min']['x'] or pos[0] > bounds['max']['x'] or
            pos[1] < bounds['min']['y'] or pos[1] > bounds['max']['y'] or
            -pos[2] < bounds['min']['z'] or -pos[2] > bounds['max']['z']):
            return True

        return False

    def _check_nfz_violation(self) -> bool:
        """Check if vehicle is inside any no-fly zone."""
        pos = self.vehicle_state.position

        for nfz in self.no_fly_zones:
            if nfz.get('type') == 'cylinder':
                center = nfz.get('center', {})
                radius = nfz.get('radius_m', 0)
                height = nfz.get('height_m', 0)

                dx = pos[0] - center.get('x', 0)
                dy = pos[1] - center.get('y', 0)
                dist_horizontal = math.sqrt(dx*dx + dy*dy)

                if dist_horizontal < radius and -pos[2] < height:
                    return True

            elif nfz.get('type') == 'box':
                bounds = nfz.get('bounds', {})
                min_b = bounds.get('min', {})
                max_b = bounds.get('max', {})

                if (min_b.get('x', float('inf')) <= pos[0] <= max_b.get('x', float('-inf')) and
                    min_b.get('y', float('inf')) <= pos[1] <= max_b.get('y', float('-inf')) and
                    min_b.get('z', float('inf')) <= -pos[2] <= max_b.get('z', float('-inf'))):
                    return True

        return False

    def _check_collision(self) -> bool:
        """
        Check for collision (ground or obstacle).
        Simple implementation - can be extended with sensor data.
        """
        # Ground collision (NED: z is down, so positive z means below ground)
        if self.vehicle_state.position[2] > -0.5:  # Less than 0.5m altitude
            if self.vehicle_state.armed and np.linalg.norm(self.vehicle_state.velocity) > 0.5:
                return True

        return False

    def _get_observation(self) -> Dict[str, np.ndarray]:
        """Build observation dictionary from current state."""
        obs = {}

        if 'position' in self.observation_space.spaces:
            obs['position'] = self.vehicle_state.position.astype(np.float32)

        if 'velocity' in self.observation_space.spaces:
            obs['velocity'] = self.vehicle_state.velocity.astype(np.float32)

        if 'attitude' in self.observation_space.spaces:
            obs['attitude'] = self.vehicle_state.attitude.astype(np.float32)

        if 'angular_velocity' in self.observation_space.spaces:
            obs['angular_velocity'] = self.vehicle_state.angular_velocity.astype(np.float32)

        if 'target_position' in self.observation_space.spaces:
            obs['target_position'] = self.target_position.astype(np.float32)

        if 'target_velocity' in self.observation_space.spaces:
            obs['target_velocity'] = self.target_velocity.astype(np.float32)

        if 'position_error' in self.observation_space.spaces:
            error = self.target_position - self.vehicle_state.position
            obs['position_error'] = error.astype(np.float32)

        if 'battery_remaining' in self.observation_space.spaces:
            obs['battery_remaining'] = np.array(
                [self.vehicle_state.battery_remaining], dtype=np.float32
            )

        if 'previous_action' in self.observation_space.spaces:
            obs['previous_action'] = self.previous_action.astype(np.float32)

        return obs

    def _compute_reward(
        self,
        obs: Dict[str, np.ndarray],
        action: np.ndarray,
        info: Dict[str, Any]
    ) -> float:
        """
        Compute reward based on task performance.

        Reward components:
        - Position tracking error (negative)
        - Velocity tracking error (negative)
        - Smoothness (action difference penalty)
        - Energy efficiency (control effort penalty)
        - Collision penalty
        - Geofence violation penalty
        - Goal reached bonus
        - Step penalty (time pressure)
        """
        reward = 0.0
        weights = self.reward_weights

        # Position tracking
        if 'position_error' in obs:
            position_error = np.linalg.norm(obs['position_error'])
            reward += weights.get('position_tracking', 1.0) * (-position_error / 10.0)

            # Goal reached bonus
            if position_error < 1.0:  # Within 1 meter
                reward += weights.get('goal_reached', 100.0)
                info['goal_reached'] = True

        # Velocity tracking
        if 'target_velocity' in obs and 'velocity' in obs:
            velocity_error = np.linalg.norm(
                obs['target_velocity'] - obs['velocity']
            )
            reward += weights.get('velocity_tracking', 0.5) * (-velocity_error / 5.0)

        # Smoothness (action difference from previous)
        if isinstance(action, np.ndarray):
            action_diff = np.linalg.norm(action - self.previous_action)
            reward += weights.get('smoothness', 0.1) * (-action_diff / 5.0)

        # Energy efficiency
        if isinstance(action, np.ndarray):
            control_effort = np.linalg.norm(action)
            reward += weights.get('energy', 0.05) * (-control_effort / 10.0)

        # Collision penalty
        if info.get('collision', False):
            reward += weights.get('collision', -100.0)

        # Geofence violation penalty
        if info.get('geofence_violation', False):
            reward += weights.get('geofence', -50.0)

        # NFZ violation penalty
        if info.get('nfz_violation', False):
            reward += weights.get('geofence', -50.0)

        # Step penalty (encourage faster completion)
        reward += weights.get('step_penalty', -0.01)

        return reward

    def _select_spawn_position(self) -> Dict:
        """Select spawn position for episode reset."""
        if self.config.get('random_spawn', False) and self.spawn_positions:
            return np.random.choice(self.spawn_positions)
        elif self.spawn_positions:
            return self.spawn_positions[0]
        else:
            return {'x': 0, 'y': 0, 'z': 0, 'heading_deg': 0}

    def _generate_target(self) -> np.ndarray:
        """Generate target position/velocity for the task."""
        if self.task_type == TaskType.HOVER:
            # Hover at spawn position
            spawn = self._select_spawn_position()
            return np.array([
                spawn.get('x', 0),
                spawn.get('y', 0),
                -self.takeoff_altitude
            ])

        elif self.task_type == TaskType.WAYPOINT_NAVIGATION:
            # Random waypoint within geofence
            bounds = self.geofence_bounds
            target = np.array([
                np.random.uniform(bounds['min']['x'] * 0.8, bounds['max']['x'] * 0.8),
                np.random.uniform(bounds['min']['y'] * 0.8, bounds['max']['y'] * 0.8),
                -np.random.uniform(self.min_altitude + 5, self.max_altitude * 0.5)
            ])
            return target

        elif self.task_type == TaskType.VELOCITY_TRACKING:
            # Random target velocity
            max_vel = self.max_velocity * 0.5
            return np.array([
                np.random.uniform(-max_vel, max_vel),
                np.random.uniform(-max_vel, max_vel),
                np.random.uniform(-2, 2)
            ])

        else:
            # Default: hold current position
            return self.vehicle_state.position.copy()

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        """
        Reset the environment for a new episode.

        Args:
            seed: Random seed for reproducibility
            options: Additional reset options

        Returns:
            Initial observation and info dictionary
        """
        super().reset(seed=seed)

        # Reset episode stats
        self.episode_stats = EpisodeStats()
        self.episode_stats.start_time = time.time()
        self.current_step = 0
        self.previous_action = np.zeros(3)

        # Connect if not connected
        if not self.vehicle_state.connected:
            if not self._connect_mavlink():
                logger.error("Failed to connect to SITL")
                return self._get_observation(), {'error': 'connection_failed'}

        # Apply domain randomization if enabled
        if self.domain_randomization:
            self.domain_randomization.apply_randomization(self)

        # Reset procedure
        logger.info("Resetting environment...")

        # 1. Disarm if armed
        if self.vehicle_state.armed:
            self._disarm()
            time.sleep(0.5)

        # 2. Set GUIDED mode
        if not self._set_mode('GUIDED'):
            logger.warning("Failed to set GUIDED mode, retrying...")
            time.sleep(1.0)
            self._set_mode('GUIDED')

        # 3. Arm
        if not self._arm():
            logger.error("Failed to arm vehicle")
            return self._get_observation(), {'error': 'arm_failed'}

        # 4. Takeoff
        if not self._takeoff(self.takeoff_altitude):
            logger.error("Failed to takeoff")
            return self._get_observation(), {'error': 'takeoff_failed'}

        # 5. Generate target
        self.target_position = self._generate_target()
        if self.task_type == TaskType.VELOCITY_TRACKING:
            self.target_velocity = self._generate_target()

        # 6. Update state
        self._update_vehicle_state(timeout=0.5)

        # Build initial observation
        obs = self._get_observation()
        info = {
            'spawn_position': self._select_spawn_position(),
            'target_position': self.target_position.tolist(),
            'connected': self.vehicle_state.connected,
            'armed': self.vehicle_state.armed,
        }

        logger.info(f"Episode reset complete. Target: {self.target_position}")

        return obs, info

    def step(
        self,
        action: Union[np.ndarray, int]
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        """
        Execute one environment step.

        Args:
            action: Action to execute (velocity command or discrete action)

        Returns:
            observation, reward, terminated, truncated, info
        """
        self.current_step += 1
        info = {}

        # Clip action to valid range
        if isinstance(action, np.ndarray):
            action = np.clip(action, self.action_space.low, self.action_space.high)

        # Execute action
        action_config = self.config.get('action_space', {})
        action_type = action_config.get('type', 'velocity')

        if action_type == 'velocity':
            self._send_velocity_command(action)
        elif action_type == 'position':
            target_pos = self.vehicle_state.position + action
            self._send_position_command(target_pos)
        elif action_type == 'discrete':
            # Map discrete action to velocity command
            velocity = self._discrete_action_to_velocity(action)
            self._send_velocity_command(velocity)

        # Wait for control timestep
        time.sleep(self.dt)

        # Update vehicle state
        self._update_vehicle_state(timeout=self.dt)

        # Check termination conditions
        terminated = False
        truncated = False

        # Collision check
        if self._check_collision():
            info['collision'] = True
            self.episode_stats.collisions += 1
            terminated = True
            self.episode_stats.termination_reason = 'collision'

        # Geofence check
        if self._check_geofence_violation():
            info['geofence_violation'] = True
            self.episode_stats.geofence_violations += 1
            terminated = True
            self.episode_stats.termination_reason = 'geofence_violation'

        # NFZ check
        if self._check_nfz_violation():
            info['nfz_violation'] = True
            terminated = True
            self.episode_stats.termination_reason = 'nfz_violation'

        # Timeout check
        if self.current_step >= self.max_episode_steps:
            truncated = True
            self.episode_stats.termination_reason = 'timeout'

        # Build observation
        obs = self._get_observation()

        # Compute reward
        reward = self._compute_reward(obs, action, info)
        self.episode_stats.total_reward += reward
        self.episode_stats.num_steps = self.current_step

        # Check if goal reached
        if info.get('goal_reached', False):
            self.episode_stats.goal_reached = True
            # Optionally terminate on goal
            if self.config.get('terminate_on_goal', False):
                terminated = True
                self.episode_stats.termination_reason = 'goal_reached'

        # Update previous action
        if isinstance(action, np.ndarray):
            self.previous_action = action.copy()

        # Add episode stats to info
        info['episode_stats'] = {
            'total_reward': self.episode_stats.total_reward,
            'num_steps': self.episode_stats.num_steps,
            'goal_reached': self.episode_stats.goal_reached,
            'termination_reason': self.episode_stats.termination_reason,
        }

        return obs, reward, terminated, truncated, info

    def _discrete_action_to_velocity(self, action: int) -> np.ndarray:
        """Map discrete action to velocity command."""
        speed = 3.0  # m/s

        # 8 discrete actions: 4 cardinal directions + 4 diagonals + up/down
        action_map = {
            0: np.array([speed, 0, 0]),      # Forward
            1: np.array([-speed, 0, 0]),     # Backward
            2: np.array([0, speed, 0]),      # Right
            3: np.array([0, -speed, 0]),     # Left
            4: np.array([0, 0, speed]),      # Down (NED)
            5: np.array([0, 0, -speed]),     # Up (NED)
            6: np.array([speed/1.41, speed/1.41, 0]),   # Forward-right
            7: np.array([speed/1.41, -speed/1.41, 0]),  # Forward-left
        }

        return action_map.get(action, np.zeros(3))

    def render(self):
        """Render the environment."""
        if self.render_mode == 'ansi':
            return self._render_ansi()
        elif self.render_mode == 'human':
            self._render_human()
        elif self.render_mode == 'rgb_array':
            return self._render_rgb_array()

    def _render_ansi(self) -> str:
        """Return ASCII representation of state."""
        pos = self.vehicle_state.position
        vel = self.vehicle_state.velocity
        att = np.degrees(self.vehicle_state.attitude)

        return (
            f"Step: {self.current_step} | "
            f"Pos: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}] | "
            f"Vel: [{vel[0]:.1f}, {vel[1]:.1f}, {vel[2]:.1f}] | "
            f"Att: [{att[0]:.1f}, {att[1]:.1f}, {att[2]:.1f}] | "
            f"Target: [{self.target_position[0]:.1f}, {self.target_position[1]:.1f}, "
            f"{self.target_position[2]:.1f}] | "
            f"Reward: {self.episode_stats.total_reward:.2f}"
        )

    def _render_human(self):
        """Print state to console."""
        print(self._render_ansi())

    def _render_rgb_array(self) -> np.ndarray:
        """Return RGB array (placeholder for future visualization)."""
        # Return a simple placeholder image
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def close(self):
        """Clean up resources."""
        logger.info("Closing environment...")

        if self.vehicle_state.armed:
            self._disarm()

        self._disconnect_mavlink()

    def set_domain_randomization(self, domain_randomization):
        """Set domain randomization module."""
        self.domain_randomization = domain_randomization

    def set_target(self, position: np.ndarray = None, velocity: np.ndarray = None):
        """Manually set target position/velocity."""
        if position is not None:
            self.target_position = np.array(position)
        if velocity is not None:
            self.target_velocity = np.array(velocity)


def make_flyby_env(
    config_path: str = None,
    instance_id: int = 0,
    render_mode: str = None,
    **kwargs
) -> FlybyUAVEnv:
    """
    Factory function for creating Flyby UAV environments.

    Args:
        config_path: Path to configuration YAML
        instance_id: SITL instance ID for parallel environments
        render_mode: Rendering mode
        **kwargs: Additional configuration overrides

    Returns:
        Configured FlybyUAVEnv instance
    """
    return FlybyUAVEnv(
        config_path=config_path,
        config_dict=kwargs,
        render_mode=render_mode,
        instance_id=instance_id,
    )


# Register environment with Gymnasium
def register_flyby_envs():
    """Register Flyby environments with Gymnasium."""
    try:
        gym.register(
            id='FlybyUAV-v0',
            entry_point='rl_gym_env:FlybyUAVEnv',
            max_episode_steps=1000,
        )
        logger.info("Registered FlybyUAV-v0 environment")
    except gym.error.Error:
        pass  # Already registered


if __name__ == '__main__':
    # Test environment
    register_flyby_envs()

    # Create environment
    env = FlybyUAVEnv(
        config_dict={
            'connection_string': 'tcp:localhost:5760',
            'max_episode_steps': 100,
            'task_type': 'velocity_tracking',
        },
        render_mode='ansi'
    )

    print("Testing FlybyUAVEnv...")
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")

    # Test reset and step
    obs, info = env.reset()
    print(f"Initial observation: {obs}")
    print(f"Initial info: {info}")

    # Run a few steps
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        env.render()

        if terminated or truncated:
            print(f"Episode ended: {info.get('episode_stats', {})}")
            break

    env.close()
    print("Test complete!")
