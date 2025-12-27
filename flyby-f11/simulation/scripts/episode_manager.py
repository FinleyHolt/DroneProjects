#!/usr/bin/env python3
"""
Flyby F-11 Episode Manager

Manages RL training episodes:
- Resets simulation state between episodes
- Spawns vehicle at configurable positions
- Handles episode timeout
- Publishes episode info (reward signals, done flags)

This module can be used standalone or as a ROS 2 node.
"""

import json
import math
import random
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from mavros_msgs.msg import State as FCUState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger, Empty


class EpisodeState(Enum):
    """Episode state machine states."""
    IDLE = "idle"
    RESETTING = "resetting"
    ARMING = "arming"
    TAKING_OFF = "taking_off"
    RUNNING = "running"
    LANDING = "landing"
    DONE = "done"
    ERROR = "error"


@dataclass
class SpawnPosition:
    """Spawn position configuration."""
    name: str
    x: float
    y: float
    z: float
    heading_deg: float = 0.0

    def to_dict(self) -> Dict:
        return {
            'name': self.name,
            'position': {'x': self.x, 'y': self.y, 'z': self.z},
            'heading_deg': self.heading_deg,
        }


@dataclass
class EpisodeInfo:
    """Information about current episode."""
    episode_id: int = 0
    state: EpisodeState = EpisodeState.IDLE
    start_time: float = 0.0
    duration: float = 0.0
    spawn_position: Optional[SpawnPosition] = None
    done: bool = False
    truncated: bool = False
    success: bool = False
    failure_reason: str = ""


@dataclass
class WorldConfig:
    """Training world configuration."""
    geofence_bounds: Dict = field(default_factory=dict)
    no_fly_zones: List[Dict] = field(default_factory=list)
    spawn_positions: List[SpawnPosition] = field(default_factory=list)
    landing_zones: List[Dict] = field(default_factory=list)

    @classmethod
    def from_json(cls, path: str) -> 'WorldConfig':
        """Load world config from JSON file."""
        with open(path, 'r') as f:
            data = json.load(f)

        spawn_positions = []
        for sp in data.get('spawn_positions', {}).get('positions', []):
            pos = sp.get('position', {})
            spawn_positions.append(SpawnPosition(
                name=sp.get('name', 'unnamed'),
                x=pos.get('x', 0),
                y=pos.get('y', 0),
                z=pos.get('z', 0),
                heading_deg=sp.get('heading_deg', 0),
            ))

        return cls(
            geofence_bounds=data.get('geofence', {}).get('bounds', {}),
            no_fly_zones=data.get('no_fly_zones', []),
            spawn_positions=spawn_positions,
            landing_zones=data.get('landing_zones', []),
        )


class EpisodeManager(Node):
    """ROS 2 node for managing RL training episodes."""

    def __init__(self):
        super().__init__('episode_manager')

        # Declare parameters
        self.declare_parameter('world_config', '/simulation/worlds/training_world_config.json')
        self.declare_parameter('reset_timeout_s', 10.0)
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('max_episode_duration_s', 300.0)
        self.declare_parameter('random_spawn', False)

        # Load parameters
        world_config_path = self.get_parameter('world_config').value
        self.reset_timeout = self.get_parameter('reset_timeout_s').value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude_m').value
        self.max_duration = self.get_parameter('max_episode_duration_s').value
        self.random_spawn = self.get_parameter('random_spawn').value

        # Load world config
        try:
            self.world_config = WorldConfig.from_json(world_config_path)
            self.get_logger().info(f'Loaded world config from {world_config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load world config: {e}')
            self.world_config = WorldConfig()

        # State
        self.episode_info = EpisodeInfo()
        self.current_pose: Optional[PoseStamped] = None
        self.current_velocity: Optional[TwistStamped] = None
        self.current_battery: Optional[BatteryState] = None
        self.fcu_state: Optional[FCUState] = None

        # QoS for reliable communication
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/flyby/state/pose',
            self._pose_callback, qos)
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/flyby/state/velocity',
            self._velocity_callback, qos)
        self.battery_sub = self.create_subscription(
            BatteryState, '/flyby/state/battery',
            self._battery_callback, qos)
        self.fcu_state_sub = self.create_subscription(
            FCUState, '/flyby/state/fcu_state',
            self._fcu_state_callback, qos)

        # Publishers
        self.episode_done_pub = self.create_publisher(Bool, '/flyby/training/episode_done', qos)
        self.episode_reward_pub = self.create_publisher(Float32, '/flyby/training/episode_reward', qos)

        # Services (clients)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        # Services (provided)
        self.reset_srv = self.create_service(
            Trigger, '/flyby/training/reset_episode',
            self._reset_episode_callback)

        # Timer for episode monitoring
        self.monitor_timer = self.create_timer(0.1, self._monitor_episode)

        self.get_logger().info('Episode Manager initialized')

    def _pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def _velocity_callback(self, msg: TwistStamped):
        self.current_velocity = msg

    def _battery_callback(self, msg: BatteryState):
        self.current_battery = msg

    def _fcu_state_callback(self, msg: FCUState):
        self.fcu_state = msg

    def _reset_episode_callback(self, request, response):
        """Handle reset episode service request."""
        self.get_logger().info('Reset episode requested')

        try:
            success = self.reset_episode()
            response.success = success
            response.message = 'Episode reset successful' if success else 'Reset failed'
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def reset_episode(self, spawn_name: Optional[str] = None) -> bool:
        """
        Reset the simulation for a new episode.

        Args:
            spawn_name: Name of spawn position, or None for random/default

        Returns:
            True if reset successful
        """
        self.episode_info.state = EpisodeState.RESETTING
        self.episode_info.done = False
        self.episode_info.truncated = False
        self.episode_info.success = False
        self.episode_info.failure_reason = ""

        # Select spawn position
        if spawn_name:
            spawn_pos = next(
                (sp for sp in self.world_config.spawn_positions if sp.name == spawn_name),
                None
            )
            if not spawn_pos:
                self.get_logger().warn(f'Spawn position {spawn_name} not found, using default')
                spawn_pos = self._get_default_spawn()
        elif self.random_spawn and self.world_config.spawn_positions:
            spawn_pos = random.choice(self.world_config.spawn_positions)
        else:
            spawn_pos = self._get_default_spawn()

        self.episode_info.spawn_position = spawn_pos
        self.get_logger().info(f'Spawning at: {spawn_pos.name} ({spawn_pos.x}, {spawn_pos.y})')

        # Step 1: Disarm if armed
        if self.fcu_state and self.fcu_state.armed:
            self._disarm()
            time.sleep(1.0)

        # Step 2: Reset position (via SITL)
        # In real implementation, this would call ArduPilot SITL's position reset
        # For now, we rely on the vehicle respawning at origin after landing

        # Step 3: Set mode to GUIDED
        if not self._set_mode('GUIDED'):
            self.get_logger().error('Failed to set GUIDED mode')
            self.episode_info.state = EpisodeState.ERROR
            return False

        time.sleep(0.5)

        # Step 4: Arm
        self.episode_info.state = EpisodeState.ARMING
        if not self._arm():
            self.get_logger().error('Failed to arm')
            self.episode_info.state = EpisodeState.ERROR
            return False

        time.sleep(0.5)

        # Step 5: Takeoff
        self.episode_info.state = EpisodeState.TAKING_OFF
        if not self._takeoff(self.takeoff_altitude):
            self.get_logger().error('Failed to takeoff')
            self.episode_info.state = EpisodeState.ERROR
            return False

        # Wait for takeoff completion
        start_time = time.time()
        while time.time() - start_time < self.reset_timeout:
            if self.current_pose:
                altitude = self.current_pose.pose.position.z
                if altitude >= self.takeoff_altitude * 0.9:
                    break
            time.sleep(0.1)

        # Episode is now running
        self.episode_info.episode_id += 1
        self.episode_info.state = EpisodeState.RUNNING
        self.episode_info.start_time = time.time()

        self.get_logger().info(f'Episode {self.episode_info.episode_id} started')
        return True

    def end_episode(self, success: bool = False, reason: str = ""):
        """
        End the current episode.

        Args:
            success: Whether episode completed successfully
            reason: Reason for ending (if failure)
        """
        self.episode_info.state = EpisodeState.LANDING
        self.episode_info.done = True
        self.episode_info.success = success
        self.episode_info.failure_reason = reason
        self.episode_info.duration = time.time() - self.episode_info.start_time

        # Land the vehicle
        self._land()

        # Publish done signal
        done_msg = Bool()
        done_msg.data = True
        self.episode_done_pub.publish(done_msg)

        self.episode_info.state = EpisodeState.DONE
        self.get_logger().info(
            f'Episode {self.episode_info.episode_id} ended: '
            f'success={success}, duration={self.episode_info.duration:.1f}s'
        )

    def _monitor_episode(self):
        """Monitor running episode for termination conditions."""
        if self.episode_info.state != EpisodeState.RUNNING:
            return

        # Check timeout
        elapsed = time.time() - self.episode_info.start_time
        if elapsed > self.max_duration:
            self.episode_info.truncated = True
            self.end_episode(success=False, reason="timeout")
            return

        # Check geofence violation
        if self.current_pose and self._check_geofence_violation():
            self.end_episode(success=False, reason="geofence_violation")
            return

        # Check NFZ violation
        if self.current_pose and self._check_nfz_violation():
            self.end_episode(success=False, reason="nfz_violation")
            return

        # Check battery
        if self.current_battery and self.current_battery.percentage < 0.1:
            self.end_episode(success=False, reason="battery_critical")
            return

    def _check_geofence_violation(self) -> bool:
        """Check if vehicle is outside geofence."""
        if not self.current_pose or not self.world_config.geofence_bounds:
            return False

        pos = self.current_pose.pose.position
        bounds = self.world_config.geofence_bounds

        min_b = bounds.get('min', {})
        max_b = bounds.get('max', {})

        if (pos.x < min_b.get('x', float('-inf')) or
            pos.x > max_b.get('x', float('inf')) or
            pos.y < min_b.get('y', float('-inf')) or
            pos.y > max_b.get('y', float('inf')) or
            pos.z < min_b.get('z', 0) or
            pos.z > max_b.get('z', float('inf'))):
            return True

        return False

    def _check_nfz_violation(self) -> bool:
        """Check if vehicle is inside any NFZ."""
        if not self.current_pose or not self.world_config.no_fly_zones:
            return False

        pos = self.current_pose.pose.position

        for nfz in self.world_config.no_fly_zones:
            if not nfz.get('active', True):
                continue

            nfz_type = nfz.get('type', 'box')

            if nfz_type == 'cylinder':
                center = nfz.get('center', {})
                radius = nfz.get('radius_m', 0)
                height = nfz.get('height_m', 0)

                dx = pos.x - center.get('x', 0)
                dy = pos.y - center.get('y', 0)
                dist_horizontal = math.sqrt(dx*dx + dy*dy)

                if dist_horizontal < radius and pos.z < height:
                    return True

            elif nfz_type == 'box':
                bounds = nfz.get('bounds', {})
                min_b = bounds.get('min', {})
                max_b = bounds.get('max', {})

                if (min_b.get('x', float('inf')) <= pos.x <= max_b.get('x', float('-inf')) and
                    min_b.get('y', float('inf')) <= pos.y <= max_b.get('y', float('-inf')) and
                    min_b.get('z', float('inf')) <= pos.z <= max_b.get('z', float('-inf'))):
                    return True

        return False

    def _get_default_spawn(self) -> SpawnPosition:
        """Get default spawn position."""
        if self.world_config.spawn_positions:
            # Try to find 'center' or first position
            for sp in self.world_config.spawn_positions:
                if sp.name == 'center':
                    return sp
            return self.world_config.spawn_positions[0]
        return SpawnPosition(name='origin', x=0, y=0, z=0)

    def _arm(self) -> bool:
        """Arm the vehicle."""
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            return False
        request = CommandBool.Request()
        request.value = True
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() and future.result().success

    def _disarm(self) -> bool:
        """Disarm the vehicle."""
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            return False
        request = CommandBool.Request()
        request.value = False
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() and future.result().success

    def _set_mode(self, mode: str) -> bool:
        """Set flight mode."""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            return False
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() and future.result().mode_sent

    def _takeoff(self, altitude: float) -> bool:
        """Command takeoff."""
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            return False
        request = CommandTOL.Request()
        request.altitude = altitude
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() and future.result().success

    def _land(self) -> bool:
        """Command landing."""
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            return False
        request = CommandTOL.Request()
        future = self.land_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result() and future.result().success


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
