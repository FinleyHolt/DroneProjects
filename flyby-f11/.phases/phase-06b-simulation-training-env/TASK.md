# Phase 6b: Simulation & Training Environment

## Overview

Create the Gymnasium-based RL training environment that connects to the F-11 Gazebo simulation. This phase establishes the `ontology_rl` package with environments, training infrastructure, and the simulation-training loop required for Phase 7's RL agent development.

## Human Description

Before RL training can begin, we need:

1. **ontology_rl** - ROS 2 package with Gymnasium environments
2. **Training world** - Gazebo world with training scenarios
3. **Simulation-training bridge** - Connect Gym env to ROS 2/Gazebo
4. **Verification tests** - Ensure episodes run without errors

This phase creates the scaffolding. Phase 7+ will implement the actual RL agents.

## AI Agent Instructions

### Prerequisites
- Phase 6a completed (sensor packages, launch files)
- Understanding of Gymnasium API
- Familiarity with ROS 2 action servers
- Knowledge of Stable-Baselines3

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

#### 1. Create ontology_rl Package

**Location:** `ros2_ws/src/ontology_rl/`

```bash
cd ros2_ws/src
ros2 pkg create ontology_rl --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs vampire_bridge mission_orchestrator
```

**Package Structure:**
```
ontology_rl/
├── package.xml
├── setup.py
├── setup.cfg
├── ontology_rl/
│   ├── __init__.py
│   ├── envs/
│   │   ├── __init__.py
│   │   ├── base_env.py
│   │   ├── mission_planner_env.py
│   │   ├── behavior_selector_env.py
│   │   └── trajectory_optimizer_env.py
│   ├── wrappers/
│   │   ├── __init__.py
│   │   ├── ros2_gym_bridge.py
│   │   └── ontology_filter.py
│   ├── training/
│   │   ├── __init__.py
│   │   ├── config.py
│   │   └── callbacks.py
│   └── nodes/
│       ├── __init__.py
│       └── gym_env_node.py
├── config/
│   ├── mission_planner.yaml
│   ├── behavior_selector.yaml
│   └── trajectory_optimizer.yaml
├── launch/
│   └── training.launch.py
└── test/
    ├── test_env_creation.py
    └── test_episode_run.py
```

#### 2. Implement Base Environment

```python
# ontology_rl/envs/base_env.py
"""
Base Gymnasium environment for F-11 ontology-constrained RL.
Provides common interfaces for ROS 2 and Vampire integration.
"""
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import threading

class FlybyBaseEnv(gym.Env):
    """Base environment with ROS 2 and Vampire integration."""

    metadata = {'render_modes': ['human', 'rgb_array']}

    def __init__(self, config: dict = None):
        super().__init__()

        self.config = config or {}

        # Initialize ROS 2 in separate thread
        self._ros_thread = None
        self._node = None
        self._init_ros2()

        # State tracking
        self._current_pose = None
        self._current_battery = 100.0
        self._episode_step = 0
        self._max_steps = self.config.get('max_steps', 1000)

        # Vampire integration
        self._pending_facts = []
        self._safety_violations = []

    def _init_ros2(self):
        """Initialize ROS 2 node in background thread."""
        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node('gym_env')

        # Subscriptions
        self._node.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            10
        )

        self._node.create_subscription(
            String,
            '/vampire/violations',
            self._violation_callback,
            10
        )

        # Publishers
        self._fact_pub = self._node.create_publisher(
            String,
            '/perception/tptp_facts',
            10
        )

        # Spin in background
        self._ros_thread = threading.Thread(
            target=lambda: rclpy.spin(self._node),
            daemon=True
        )
        self._ros_thread.start()

    def _pose_callback(self, msg):
        self._current_pose = msg

    def _violation_callback(self, msg):
        self._safety_violations.append(msg.data)

    def query_ontology(self, query: str) -> bool:
        """Query Vampire for ontology constraint check."""
        # Publish query to vampire_bridge
        # Wait for response
        # Return True if constraint satisfied
        pass  # Implemented by subclasses

    def filter_action(self, action: np.ndarray) -> np.ndarray:
        """Filter action through ontology constraints."""
        # Default: no filtering
        return action

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._episode_step = 0
        self._safety_violations = []
        return self._get_observation(), {}

    def step(self, action):
        self._episode_step += 1

        # Filter action through ontology
        filtered_action = self.filter_action(action)

        # Execute action (implemented by subclass)
        self._execute_action(filtered_action)

        # Get observation
        obs = self._get_observation()

        # Compute reward
        reward = self._compute_reward()

        # Check termination
        terminated = self._is_terminated()
        truncated = self._episode_step >= self._max_steps

        info = {
            'safety_violations': len(self._safety_violations),
            'step': self._episode_step
        }

        return obs, reward, terminated, truncated, info

    def _execute_action(self, action):
        raise NotImplementedError

    def _get_observation(self):
        raise NotImplementedError

    def _compute_reward(self):
        raise NotImplementedError

    def _is_terminated(self):
        raise NotImplementedError

    def close(self):
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

#### 3. Implement Mission Planner Environment Stub

```python
# ontology_rl/envs/mission_planner_env.py
"""
Mission Planner RL environment (Level 1, 10s horizon).
Selects waypoints and adapts mission plans.
"""
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from .base_env import FlybyBaseEnv

class MissionPlannerEnv(FlybyBaseEnv):
    """
    Level 1 Mission Planner environment.

    Observation Space:
    - UAV position (x, y, z)
    - Battery level (0-100)
    - Mission progress (0-1)
    - Current waypoint index
    - Remaining waypoints count
    - Time elapsed

    Action Space:
    - Waypoint selection (discrete: next, skip, insert, abort)
    - Or continuous: waypoint offset (dx, dy, dz)
    """

    def __init__(self, config: dict = None):
        super().__init__(config)

        # Observation: [x, y, z, battery, progress, wp_idx, remaining, time]
        self.observation_space = spaces.Box(
            low=np.array([-1000, -1000, 0, 0, 0, 0, 0, 0]),
            high=np.array([1000, 1000, 500, 100, 1, 100, 100, 3600]),
            dtype=np.float32
        )

        # Action: discrete waypoint selection
        # 0=next, 1=skip, 2=previous, 3=abort
        self.action_space = spaces.Discrete(4)

        # Mission state
        self._waypoints = []
        self._current_wp_idx = 0
        self._mission_start_time = 0

    def reset(self, seed=None, options=None):
        obs, info = super().reset(seed=seed, options=options)

        # Reset mission state
        self._waypoints = self._generate_mission()
        self._current_wp_idx = 0

        return self._get_observation(), info

    def _generate_mission(self):
        """Generate random mission waypoints."""
        # Simple mission: 5 waypoints in a pattern
        return [
            (0, 0, 50),
            (100, 0, 50),
            (100, 100, 50),
            (0, 100, 50),
            (0, 0, 50),
        ]

    def _execute_action(self, action):
        """Execute waypoint selection action."""
        if action == 0:  # Next waypoint
            self._current_wp_idx = min(
                self._current_wp_idx + 1,
                len(self._waypoints) - 1
            )
        elif action == 1:  # Skip waypoint
            self._current_wp_idx = min(
                self._current_wp_idx + 2,
                len(self._waypoints) - 1
            )
        elif action == 2:  # Previous waypoint
            self._current_wp_idx = max(0, self._current_wp_idx - 1)
        elif action == 3:  # Abort mission
            self._current_wp_idx = len(self._waypoints)  # End mission

    def _get_observation(self):
        """Build observation vector."""
        if self._current_pose:
            x = self._current_pose.pose.position.x
            y = self._current_pose.pose.position.y
            z = self._current_pose.pose.position.z
        else:
            x, y, z = 0, 0, 0

        progress = self._current_wp_idx / max(1, len(self._waypoints))
        remaining = len(self._waypoints) - self._current_wp_idx

        return np.array([
            x, y, z,
            self._current_battery,
            progress,
            self._current_wp_idx,
            remaining,
            self._episode_step
        ], dtype=np.float32)

    def _compute_reward(self):
        """Compute reward for mission planning."""
        reward = 0.0

        # Progress reward
        progress = self._current_wp_idx / max(1, len(self._waypoints))
        reward += progress * 10.0

        # Efficiency penalty (time)
        reward -= self._episode_step * 0.01

        # Safety violation penalty
        reward -= len(self._safety_violations) * 100.0

        return reward

    def _is_terminated(self):
        """Check if episode should terminate."""
        # Mission complete
        if self._current_wp_idx >= len(self._waypoints):
            return True

        # Safety violation (critical)
        if len(self._safety_violations) > 0:
            return True

        # Battery depleted
        if self._current_battery <= 10.0:
            return True

        return False


# Register environment
gym.register(
    id='FlybyMissionPlanner-v0',
    entry_point='ontology_rl.envs:MissionPlannerEnv',
    max_episode_steps=1000,
)
```

#### 4. Create Training World

**Location:** `simulation/worlds/training_arena.world`

Create a Gazebo world with:
- Open flat area (200m x 200m)
- Static obstacles (buildings, trees)
- NFZ regions (marked areas)
- Landing zones
- Waypoint markers (visual)

```xml
<!-- simulation/worlds/training_arena.world -->
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="flyby_training">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.5 0.3 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle: Building 1 -->
    <model name="building_1">
      <static>true</static>
      <pose>50 50 10 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>20 20 20</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>20 20 20</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- NFZ marker (red zone) -->
    <model name="nfz_marker">
      <static>true</static>
      <pose>-50 50 0.1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>30</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1.0 0.0 0.0 0.5</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

#### 5. Create Training Launch File

```python
# launch/training.launch.py
"""Launch training environment with simulation."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('flyby_f11_bringup')

    return LaunchDescription([
        # Include simulation launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'simulation.launch.py')
            )
        ),

        # Vampire bridge for ontology queries
        Node(
            package='vampire_bridge',
            executable='vampire_node',
            name='vampire_bridge',
            output='screen'
        ),

        # Mission orchestrator
        Node(
            package='mission_orchestrator',
            executable='mission_manager_node',
            name='mission_manager',
            output='screen'
        ),

        # Gym environment node (exposes env via ROS service)
        Node(
            package='ontology_rl',
            executable='gym_env_node',
            name='gym_env',
            output='screen',
            parameters=[{
                'env_id': 'FlybyMissionPlanner-v0',
                'max_steps': 1000
            }]
        ),
    ])
```

#### 6. Create Environment Tests

```python
# test/test_env_creation.py
"""Test Gymnasium environment creation."""
import pytest
import gymnasium as gym

def test_env_registration():
    """Verify environment is registered."""
    env_ids = [spec.id for spec in gym.envs.registry.values()]
    assert 'FlybyMissionPlanner-v0' in env_ids

def test_env_creation():
    """Test environment can be created."""
    env = gym.make('FlybyMissionPlanner-v0')
    assert env is not None
    assert env.observation_space is not None
    assert env.action_space is not None
    env.close()

def test_env_reset():
    """Test environment reset."""
    env = gym.make('FlybyMissionPlanner-v0')
    obs, info = env.reset()
    assert obs.shape == env.observation_space.shape
    env.close()

def test_env_step():
    """Test environment step."""
    env = gym.make('FlybyMissionPlanner-v0')
    env.reset()

    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)

    assert obs.shape == env.observation_space.shape
    assert isinstance(reward, (int, float))
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)

    env.close()
```

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `ontology_rl` package builds successfully
- [ ] `FlybyMissionPlanner-v0` environment registered
- [ ] Environment reset returns valid observation
- [ ] Environment step executes without error
- [ ] Training world loads in Gazebo
- [ ] training.launch.py runs without errors
- [ ] At least 100 episode steps complete

### Verification

Run automated verification:
```bash
bash .phases/phase-06b-simulation-training-env/verification.sh
```

### Common Pitfalls

- **ROS 2 threading**: Gym env and ROS 2 spin must be in separate threads
- **Observation normalization**: Scale observations for stable training
- **Reset reliability**: Simulation reset must be deterministic
- **Reward scale**: Keep rewards in reasonable range (-100 to +100)

### References

- [Gymnasium API](https://gymnasium.farama.org/)
- [Stable-Baselines3](https://stable-baselines3.readthedocs.io/)
- [ROS 2 Threading](https://docs.ros.org/en/humble/Concepts/About-Executors.html)

### Dependencies
See `dependencies.json` - requires Phase 6a completion.

### Next Phase
After completion, proceed to Phase 7: Mission Planner RL Agent
