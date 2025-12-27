"""
Base Gymnasium environment for F-11 ontology-constrained RL.

Provides common interfaces for ROS 2 communication and Vampire theorem prover
integration. All hierarchical environments (mission planner, behavior selector,
trajectory optimizer) inherit from this base class.

The base environment handles:
- ROS 2 node initialization and threading
- Vampire bridge communication for ontology queries
- Common state tracking (pose, battery, violations)
- Action filtering through ontology constraints
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import threading
import time
from typing import Any, Dict, Optional, Tuple
from abc import abstractmethod

# ROS 2 imports (optional - allows standalone testing without ROS)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String, Float32
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import BatteryState
    from nav_msgs.msg import Odometry
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object


class FlybyBaseEnv(gym.Env):
    """
    Base Gymnasium environment with ROS 2 and Vampire integration.

    This abstract base class provides:
    - Thread-safe ROS 2 node management
    - Vampire theorem prover query interface
    - Common observation/action space patterns
    - Safety violation tracking

    Subclasses must implement:
    - _define_spaces(): Define observation_space and action_space
    - _execute_action(): Execute action in simulation
    - _get_observation(): Build observation vector
    - _compute_reward(): Calculate step reward
    - _is_terminated(): Check episode termination
    """

    metadata = {
        'render_modes': ['human', 'rgb_array'],
        'render_fps': 30
    }

    def __init__(
        self,
        config: Optional[Dict[str, Any]] = None,
        use_ros: bool = True,
        node_name: str = 'gym_env'
    ):
        """
        Initialize base environment.

        Args:
            config: Environment configuration dictionary
            use_ros: Whether to initialize ROS 2 (False for standalone testing)
            node_name: Name for the ROS 2 node
        """
        super().__init__()

        self.config = config or {}
        self.use_ros = use_ros and ROS2_AVAILABLE
        self._node_name = node_name

        # ROS 2 state
        self._node: Optional[Node] = None
        self._executor = None
        self._ros_thread: Optional[threading.Thread] = None
        self._ros_lock = threading.Lock()
        self._shutdown_flag = threading.Event()

        # UAV state (updated by ROS callbacks)
        self._current_pose: Optional[np.ndarray] = None  # [x, y, z, qx, qy, qz, qw]
        self._current_velocity: Optional[np.ndarray] = None  # [vx, vy, vz]
        self._current_battery: float = 100.0
        self._battery_voltage: float = 16.8  # 4S LiPo nominal

        # Episode tracking
        self._episode_step: int = 0
        self._max_steps: int = self.config.get('max_steps', 1000)
        self._episode_start_time: float = 0.0

        # Ontology/Vampire integration
        self._pending_queries: list = []
        self._query_results: Dict[str, bool] = {}
        self._safety_violations: list = []
        self._ontology_facts: list = []

        # Geofence limits (from ontology - meters from origin)
        self._geofence_min = np.array(
            self.config.get('geofence_min', [-100.0, -100.0, 0.0])
        )
        self._geofence_max = np.array(
            self.config.get('geofence_max', [100.0, 100.0, 120.0])
        )

        # Battery return threshold (from ontology)
        self._battery_return_threshold = self.config.get(
            'battery_return_threshold', 20.0
        )

        # Initialize ROS 2 if enabled
        if self.use_ros:
            self._init_ros2()

        # Define spaces (implemented by subclass)
        self._define_spaces()

    def _init_ros2(self):
        """Initialize ROS 2 node in background thread."""
        if not ROS2_AVAILABLE:
            return

        # Initialize rclpy if not already done
        if not rclpy.ok():
            rclpy.init()

        # Create node
        self._node = rclpy.create_node(self._node_name)

        # Create subscriptions
        self._node.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            10
        )

        self._node.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._velocity_callback,
            10
        )

        self._node.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery_callback,
            10
        )

        self._node.create_subscription(
            String,
            '/vampire/violations',
            self._violation_callback,
            10
        )

        self._node.create_subscription(
            String,
            '/vampire/query_result',
            self._query_result_callback,
            10
        )

        # Create publishers
        self._fact_pub = self._node.create_publisher(
            String, '/perception/tptp_facts', 10
        )

        self._query_pub = self._node.create_publisher(
            String, '/vampire/query', 10
        )

        self._action_pub = self._node.create_publisher(
            String, '/ontology_rl/action', 10
        )

        # Create executor and spin in background
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        self._ros_thread = threading.Thread(
            target=self._ros_spin_loop,
            daemon=True
        )
        self._ros_thread.start()

        self._node.get_logger().info(
            f'FlybyBaseEnv ROS 2 node initialized: {self._node_name}'
        )

    def _ros_spin_loop(self):
        """Background thread for ROS 2 spinning."""
        while not self._shutdown_flag.is_set() and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.01)

    def _pose_callback(self, msg: 'PoseStamped'):
        """Handle pose updates from MAVROS."""
        with self._ros_lock:
            self._current_pose = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ])

    def _velocity_callback(self, msg: 'TwistStamped'):
        """Handle velocity updates from MAVROS."""
        with self._ros_lock:
            self._current_velocity = np.array([
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z
            ])

    def _battery_callback(self, msg: 'BatteryState'):
        """Handle battery state updates."""
        with self._ros_lock:
            self._current_battery = msg.percentage * 100.0
            self._battery_voltage = msg.voltage

    def _violation_callback(self, msg: 'String'):
        """Handle safety violations from Vampire bridge."""
        with self._ros_lock:
            self._safety_violations.append({
                'type': msg.data,
                'step': self._episode_step,
                'time': time.time()
            })

    def _query_result_callback(self, msg: 'String'):
        """Handle query results from Vampire bridge."""
        # Expected format: "query_id:result" where result is "true" or "false"
        try:
            query_id, result = msg.data.split(':', 1)
            with self._ros_lock:
                self._query_results[query_id] = result.lower() == 'true'
        except ValueError:
            pass

    def query_ontology(self, query: str, timeout: float = 1.0) -> Optional[bool]:
        """
        Query Vampire theorem prover for ontology constraint check.

        Args:
            query: TPTP-format query string
            timeout: Maximum wait time for result (seconds)

        Returns:
            True if constraint satisfied, False if violated, None if timeout
        """
        if not self.use_ros or self._query_pub is None:
            # Standalone mode - assume constraints satisfied
            return True

        # Generate unique query ID
        query_id = f"q_{self._episode_step}_{time.time()}"
        full_query = f"{query_id}:{query}"

        # Publish query
        msg = String()
        msg.data = full_query
        self._query_pub.publish(msg)

        # Wait for result
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self._ros_lock:
                if query_id in self._query_results:
                    result = self._query_results.pop(query_id)
                    return result
            time.sleep(0.01)

        return None  # Timeout

    def publish_fact(self, fact: str):
        """
        Publish a TPTP fact to the perception pipeline.

        Args:
            fact: TPTP-format fact string
        """
        if not self.use_ros or self._fact_pub is None:
            self._ontology_facts.append(fact)
            return

        msg = String()
        msg.data = fact
        self._fact_pub.publish(msg)
        self._ontology_facts.append(fact)

    def filter_action(self, action: np.ndarray) -> np.ndarray:
        """
        Filter action through ontology constraints.

        Default implementation applies geofence limits. Subclasses can
        override for more sophisticated constraint checking.

        Args:
            action: Raw action from RL agent

        Returns:
            Filtered action that respects ontology constraints
        """
        # Default: no filtering (subclasses should override)
        return action

    def check_geofence(self, position: np.ndarray) -> bool:
        """
        Check if position is within geofence limits.

        Args:
            position: [x, y, z] position to check

        Returns:
            True if within geofence, False if violation
        """
        return np.all(position >= self._geofence_min[:3]) and \
               np.all(position <= self._geofence_max[:3])

    def check_battery_return(self) -> bool:
        """
        Check if battery level requires return to home.

        Returns:
            True if should return, False otherwise
        """
        return self._current_battery <= self._battery_return_threshold

    @abstractmethod
    def _define_spaces(self):
        """Define observation_space and action_space. Must be implemented by subclass."""
        raise NotImplementedError

    @abstractmethod
    def _execute_action(self, action: np.ndarray):
        """Execute filtered action in simulation. Must be implemented by subclass."""
        raise NotImplementedError

    @abstractmethod
    def _get_observation(self) -> np.ndarray:
        """Build observation vector. Must be implemented by subclass."""
        raise NotImplementedError

    @abstractmethod
    def _compute_reward(self) -> float:
        """Calculate step reward. Must be implemented by subclass."""
        raise NotImplementedError

    @abstractmethod
    def _is_terminated(self) -> bool:
        """Check episode termination condition. Must be implemented by subclass."""
        raise NotImplementedError

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Reset environment for new episode.

        Args:
            seed: Random seed for reproducibility
            options: Additional reset options

        Returns:
            Tuple of (observation, info dict)
        """
        super().reset(seed=seed)

        # Reset episode tracking
        self._episode_step = 0
        self._episode_start_time = time.time()

        # Clear violations and facts
        with self._ros_lock:
            self._safety_violations = []
            self._ontology_facts = []
            self._query_results = {}

        # Reset state to defaults if no ROS data
        if self._current_pose is None:
            self._current_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        if self._current_velocity is None:
            self._current_velocity = np.array([0.0, 0.0, 0.0])

        # Get initial observation
        obs = self._get_observation()

        info = {
            'episode_start_time': self._episode_start_time,
            'max_steps': self._max_steps,
            'use_ros': self.use_ros
        }

        return obs, info

    def step(
        self,
        action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """
        Execute one environment step.

        Args:
            action: Action from RL agent

        Returns:
            Tuple of (observation, reward, terminated, truncated, info)
        """
        self._episode_step += 1

        # Filter action through ontology constraints
        filtered_action = self.filter_action(action)

        # Execute action in simulation
        self._execute_action(filtered_action)

        # Small delay for simulation to update
        if self.use_ros:
            time.sleep(0.01)

        # Get observation
        obs = self._get_observation()

        # Compute reward
        reward = self._compute_reward()

        # Check termination
        terminated = self._is_terminated()
        truncated = self._episode_step >= self._max_steps

        # Build info dict
        with self._ros_lock:
            info = {
                'step': self._episode_step,
                'safety_violations': len(self._safety_violations),
                'violation_details': list(self._safety_violations),
                'battery': self._current_battery,
                'action_filtered': not np.allclose(action, filtered_action),
                'elapsed_time': time.time() - self._episode_start_time
            }

        return obs, reward, terminated, truncated, info

    def render(self):
        """Render environment (optional, mainly for debugging)."""
        if self.render_mode == 'human':
            print(f"Step {self._episode_step}: "
                  f"pos={self._current_pose[:3] if self._current_pose is not None else 'N/A'}, "
                  f"battery={self._current_battery:.1f}%")
        return None

    def close(self):
        """Clean up ROS 2 resources."""
        self._shutdown_flag.set()

        if self._executor:
            self._executor.shutdown()

        if self._node:
            self._node.destroy_node()
            self._node = None

        # Don't shutdown rclpy globally - other envs may be using it

    def get_safe_position(self) -> np.ndarray:
        """Get current position, defaulting to origin if unavailable."""
        if self._current_pose is not None:
            return self._current_pose[:3].copy()
        return np.zeros(3)

    def get_safe_velocity(self) -> np.ndarray:
        """Get current velocity, defaulting to zero if unavailable."""
        if self._current_velocity is not None:
            return self._current_velocity.copy()
        return np.zeros(3)
