"""
ROS 2 node that exposes Gymnasium environments via services and topics.

This node creates a Gymnasium environment and provides:
- Reset service to initialize episodes
- Step service to execute actions
- Observation topic for latest state
- Action topic for receiving commands

This allows external RL training frameworks or manual testing
to interact with the ontology-constrained environments.
"""

import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger

import gymnasium as gym
import numpy as np

# Import environments to register them
import ontology_rl.envs


class GymEnvNode(Node):
    """
    ROS 2 node that wraps a Gymnasium environment.

    Parameters:
        env_id (str): Gymnasium environment ID (default: FlybyMissionPlanner-v0)
        max_steps (int): Maximum steps per episode (default: 1000)
        auto_reset (bool): Automatically reset on episode end (default: True)
        publish_rate (float): Observation publish rate in Hz (default: 10.0)

    Published Topics:
        /gym/observation (Float32MultiArray): Latest observation vector
        /gym/reward (Float32MultiArray): Latest reward [reward, cumulative]
        /gym/info (String): JSON-encoded info dictionary
        /gym/done (String): Episode termination status

    Subscribed Topics:
        /gym/action (Float32MultiArray): Action to execute

    Services:
        /gym/reset (Trigger): Reset environment
        /gym/step (Trigger): Execute step with last action
        /gym/get_spaces (Trigger): Get observation/action space info
    """

    def __init__(self):
        super().__init__('gym_env_node')

        # Declare parameters
        self.declare_parameter('env_id', 'FlybyMissionPlanner-v0')
        self.declare_parameter('max_steps', 1000)
        self.declare_parameter('auto_reset', True)
        self.declare_parameter('publish_rate', 10.0)

        # Get parameters
        self._env_id = self.get_parameter('env_id').value
        self._max_steps = self.get_parameter('max_steps').value
        self._auto_reset = self.get_parameter('auto_reset').value
        self._publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f'Creating environment: {self._env_id}')

        # Create environment with use_ros=False to avoid conflicts
        # (this node IS the ROS interface)
        env_config = {
            'max_steps': self._max_steps,
            'use_ros': False  # Disable internal ROS to prevent double-spin
        }

        try:
            self._env = gym.make(self._env_id, config=env_config)
            self.get_logger().info(f'Environment created: {self._env_id}')
            self.get_logger().info(f'Observation space: {self._env.observation_space}')
            self.get_logger().info(f'Action space: {self._env.action_space}')
        except Exception as e:
            self.get_logger().error(f'Failed to create environment: {e}')
            raise

        # State tracking
        self._current_obs: Optional[np.ndarray] = None
        self._current_reward: float = 0.0
        self._cumulative_reward: float = 0.0
        self._current_info: dict = {}
        self._episode_done: bool = True
        self._pending_action: Optional[np.ndarray] = None
        self._step_count: int = 0

        # Threading
        self._lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()

        # Publishers
        self._obs_pub = self.create_publisher(
            Float32MultiArray, '/gym/observation', 10
        )
        self._reward_pub = self.create_publisher(
            Float32MultiArray, '/gym/reward', 10
        )
        self._info_pub = self.create_publisher(
            String, '/gym/info', 10
        )
        self._done_pub = self.create_publisher(
            String, '/gym/done', 10
        )

        # Subscribers
        self.create_subscription(
            Float32MultiArray,
            '/gym/action',
            self._action_callback,
            10,
            callback_group=self._callback_group
        )

        # Services
        self.create_service(
            Trigger,
            '/gym/reset',
            self._reset_service,
            callback_group=self._callback_group
        )
        self.create_service(
            Trigger,
            '/gym/step',
            self._step_service,
            callback_group=self._callback_group
        )
        self.create_service(
            Trigger,
            '/gym/get_spaces',
            self._get_spaces_service,
            callback_group=self._callback_group
        )

        # Timer for periodic observation publishing
        timer_period = 1.0 / self._publish_rate
        self.create_timer(
            timer_period,
            self._publish_state,
            callback_group=self._callback_group
        )

        # Auto-reset on startup
        if self._auto_reset:
            self._do_reset()

        self.get_logger().info('GymEnvNode initialized and ready')

    def _action_callback(self, msg: Float32MultiArray):
        """Handle incoming action messages."""
        with self._lock:
            action_data = np.array(msg.data, dtype=np.float32)

            # Handle discrete vs continuous actions
            if hasattr(self._env.action_space, 'n'):
                # Discrete action space - take first element as int
                self._pending_action = int(action_data[0])
            else:
                self._pending_action = action_data

        self.get_logger().debug(f'Received action: {self._pending_action}')

    def _reset_service(self, request, response):
        """Handle reset service calls."""
        try:
            self._do_reset()
            response.success = True
            response.message = f'Environment reset. Observation shape: {self._current_obs.shape}'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Reset failed: {e}')

        return response

    def _step_service(self, request, response):
        """Handle step service calls."""
        with self._lock:
            if self._pending_action is None:
                # Use random action if none pending
                action = self._env.action_space.sample()
            else:
                action = self._pending_action
                self._pending_action = None

        try:
            self._do_step(action)
            response.success = True
            response.message = (
                f'Step {self._step_count}: '
                f'reward={self._current_reward:.4f}, '
                f'done={self._episode_done}'
            )
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Step failed: {e}')

        return response

    def _get_spaces_service(self, request, response):
        """Return observation and action space information."""
        spaces_info = {
            'observation_space': str(self._env.observation_space),
            'action_space': str(self._env.action_space),
            'obs_shape': list(self._env.observation_space.shape) if hasattr(
                self._env.observation_space, 'shape') else None,
            'obs_low': self._env.observation_space.low.tolist() if hasattr(
                self._env.observation_space, 'low') else None,
            'obs_high': self._env.observation_space.high.tolist() if hasattr(
                self._env.observation_space, 'high') else None,
            'action_n': self._env.action_space.n if hasattr(
                self._env.action_space, 'n') else None,
            'action_low': self._env.action_space.low.tolist() if hasattr(
                self._env.action_space, 'low') else None,
            'action_high': self._env.action_space.high.tolist() if hasattr(
                self._env.action_space, 'high') else None,
        }

        response.success = True
        response.message = json.dumps(spaces_info)
        return response

    def _do_reset(self):
        """Perform environment reset."""
        with self._lock:
            obs, info = self._env.reset()
            self._current_obs = obs
            self._current_info = info
            self._current_reward = 0.0
            self._cumulative_reward = 0.0
            self._episode_done = False
            self._step_count = 0
            self._pending_action = None

        self.get_logger().info('Environment reset')
        self._publish_state()

    def _do_step(self, action):
        """Perform environment step."""
        with self._lock:
            if self._episode_done:
                if self._auto_reset:
                    self._do_reset()
                else:
                    self.get_logger().warn('Episode done, call reset first')
                    return

            obs, reward, terminated, truncated, info = self._env.step(action)

            self._current_obs = obs
            self._current_reward = reward
            self._cumulative_reward += reward
            self._current_info = info
            self._episode_done = terminated or truncated
            self._step_count += 1

        self._publish_state()

        if self._episode_done:
            self.get_logger().info(
                f'Episode ended after {self._step_count} steps. '
                f'Cumulative reward: {self._cumulative_reward:.2f}'
            )

            if self._auto_reset:
                self._do_reset()

    def _publish_state(self):
        """Publish current state to topics."""
        with self._lock:
            if self._current_obs is None:
                return

            # Observation
            obs_msg = Float32MultiArray()
            obs_msg.data = self._current_obs.flatten().tolist()
            self._obs_pub.publish(obs_msg)

            # Reward
            reward_msg = Float32MultiArray()
            reward_msg.data = [self._current_reward, self._cumulative_reward]
            self._reward_pub.publish(reward_msg)

            # Info
            info_msg = String()
            info_msg.data = json.dumps(self._current_info, default=str)
            self._info_pub.publish(info_msg)

            # Done status
            done_msg = String()
            done_msg.data = json.dumps({
                'done': self._episode_done,
                'step': self._step_count,
                'cumulative_reward': self._cumulative_reward
            })
            self._done_pub.publish(done_msg)

    def destroy_node(self):
        """Clean up resources."""
        if hasattr(self, '_env') and self._env is not None:
            self._env.close()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = GymEnvNode()

    # Use multi-threaded executor for service calls during timer callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
