"""
ROS 2 to Gymnasium bridge wrapper.

This wrapper provides a clean interface between ROS 2 topics/services
and Gymnasium environments. It handles:
- Observation buffering from ROS topics
- Action publishing to ROS topics
- Synchronization between ROS time and Gym steps
- Reset coordination with simulation
"""

import gymnasium as gym
from gymnasium import Wrapper
import numpy as np
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Float32MultiArray, String
    from std_srvs.srv import Trigger, Empty
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class ROS2GymBridge(Wrapper):
    """
    Gymnasium wrapper that bridges ROS 2 communication.

    This wrapper can be applied to any Gymnasium environment to add
    ROS 2 topic-based observation and action interfaces, allowing
    external ROS nodes to interact with the environment.

    Example:
        env = gym.make('FlybyMissionPlanner-v0')
        env = ROS2GymBridge(env, node_name='gym_bridge')

    Topics Published:
        - /gym/observation (Float32MultiArray): Latest observation
        - /gym/reward (Float32): Latest reward
        - /gym/info (String): JSON-encoded info dict

    Topics Subscribed:
        - /gym/action (Float32MultiArray): Action to execute

    Services:
        - /gym/reset (Trigger): Reset environment
        - /gym/step (Trigger): Execute pending action
    """

    def __init__(
        self,
        env: gym.Env,
        node_name: str = 'gym_bridge',
        action_timeout: float = 1.0,
        auto_step: bool = False
    ):
        """
        Initialize ROS 2 bridge wrapper.

        Args:
            env: Gymnasium environment to wrap
            node_name: Name for the ROS 2 node
            action_timeout: Timeout for waiting for actions (seconds)
            auto_step: If True, step immediately when action received
        """
        super().__init__(env)

        self._node_name = node_name
        self._action_timeout = action_timeout
        self._auto_step = auto_step

        # ROS 2 state
        self._node: Optional[Node] = None
        self._executor = None
        self._ros_thread: Optional[threading.Thread] = None
        self._shutdown_flag = threading.Event()

        # Action buffering
        self._pending_action: Optional[np.ndarray] = None
        self._action_event = threading.Event()
        self._action_lock = threading.Lock()

        # Latest step result
        self._latest_obs: Optional[np.ndarray] = None
        self._latest_reward: float = 0.0
        self._latest_info: Dict = {}

        if ROS2_AVAILABLE:
            self._init_ros2()

    def _init_ros2(self):
        """Initialize ROS 2 node and interfaces."""
        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(self._node_name)

        # Publishers
        self._obs_pub = self._node.create_publisher(
            Float32MultiArray, '/gym/observation', 10
        )
        self._reward_pub = self._node.create_publisher(
            Float32MultiArray, '/gym/reward', 10
        )
        self._info_pub = self._node.create_publisher(
            String, '/gym/info', 10
        )

        # Subscribers
        self._node.create_subscription(
            Float32MultiArray,
            '/gym/action',
            self._action_callback,
            10
        )

        # Services
        self._node.create_service(
            Trigger,
            '/gym/reset',
            self._reset_service
        )
        self._node.create_service(
            Trigger,
            '/gym/step',
            self._step_service
        )

        # Spin in background
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._ros_thread = threading.Thread(
            target=self._ros_spin_loop,
            daemon=True
        )
        self._ros_thread.start()

        self._node.get_logger().info(f'ROS2GymBridge initialized: {self._node_name}')

    def _ros_spin_loop(self):
        """Background ROS 2 spin loop."""
        while not self._shutdown_flag.is_set() and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.01)

    def _action_callback(self, msg: 'Float32MultiArray'):
        """Handle incoming action messages."""
        with self._action_lock:
            self._pending_action = np.array(msg.data, dtype=np.float32)
            self._action_event.set()

        if self._auto_step and self._pending_action is not None:
            self._execute_step()

    def _reset_service(self, request, response):
        """Handle reset service calls."""
        try:
            obs, info = self.reset()
            self._publish_observation(obs)
            response.success = True
            response.message = 'Environment reset successfully'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _step_service(self, request, response):
        """Handle step service calls."""
        try:
            self._execute_step()
            response.success = True
            response.message = f'Step executed, reward: {self._latest_reward:.4f}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _execute_step(self):
        """Execute a step with the pending action."""
        with self._action_lock:
            if self._pending_action is None:
                return

            action = self._pending_action
            self._pending_action = None
            self._action_event.clear()

        obs, reward, terminated, truncated, info = self.env.step(action)

        self._latest_obs = obs
        self._latest_reward = reward
        self._latest_info = info

        self._publish_observation(obs)
        self._publish_reward(reward)
        self._publish_info(info)

    def _publish_observation(self, obs: np.ndarray):
        """Publish observation to ROS topic."""
        if self._obs_pub is None:
            return

        msg = Float32MultiArray()
        msg.data = obs.flatten().tolist()
        self._obs_pub.publish(msg)

    def _publish_reward(self, reward: float):
        """Publish reward to ROS topic."""
        if self._reward_pub is None:
            return

        msg = Float32MultiArray()
        msg.data = [float(reward)]
        self._reward_pub.publish(msg)

    def _publish_info(self, info: Dict):
        """Publish info dict to ROS topic."""
        if self._info_pub is None:
            return

        import json
        msg = String()
        msg.data = json.dumps(info, default=str)
        self._info_pub.publish(msg)

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None
    ) -> Tuple[np.ndarray, Dict]:
        """Reset environment and publish initial observation."""
        obs, info = self.env.reset(seed=seed, options=options)

        self._latest_obs = obs
        self._latest_info = info

        # Clear pending actions
        with self._action_lock:
            self._pending_action = None
            self._action_event.clear()

        self._publish_observation(obs)

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Execute environment step.

        If action is provided directly (not via ROS), execute immediately.
        Otherwise, wait for action from ROS topic.
        """
        obs, reward, terminated, truncated, info = self.env.step(action)

        self._latest_obs = obs
        self._latest_reward = reward
        self._latest_info = info

        self._publish_observation(obs)
        self._publish_reward(reward)
        self._publish_info(info)

        return obs, reward, terminated, truncated, info

    def wait_for_action(self, timeout: Optional[float] = None) -> Optional[np.ndarray]:
        """
        Wait for action from ROS topic.

        Args:
            timeout: Maximum wait time (uses default if None)

        Returns:
            Action array or None if timeout
        """
        timeout = timeout or self._action_timeout

        if self._action_event.wait(timeout=timeout):
            with self._action_lock:
                action = self._pending_action
                self._pending_action = None
                self._action_event.clear()
                return action

        return None

    def close(self):
        """Clean up ROS 2 resources."""
        self._shutdown_flag.set()

        if self._executor:
            self._executor.shutdown()

        if self._node:
            self._node.destroy_node()

        self.env.close()

    def get_latest_observation(self) -> Optional[np.ndarray]:
        """Get the most recent observation."""
        return self._latest_obs

    def get_latest_reward(self) -> float:
        """Get the most recent reward."""
        return self._latest_reward

    def get_latest_info(self) -> Dict:
        """Get the most recent info dict."""
        return self._latest_info
