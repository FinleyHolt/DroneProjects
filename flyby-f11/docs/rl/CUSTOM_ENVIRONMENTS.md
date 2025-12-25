# Custom Gymnasium Environments for UAV Navigation

## Overview

This guide details how to implement custom Gymnasium environments for training the three hierarchical RL agents in the flyby-f11 autonomy stack. Each agent requires a specialized environment tailored to its temporal horizon and decision-making scope.

## Environment Architecture

### Base UAV Environment

All three environments inherit from a common base class that provides:
- PX4 SITL connection via MAVSDK
- Gazebo sensor data access (cameras, depth, IMU, GPS)
- Vehicle state management (position, velocity, attitude)
- Safety constraint checking (geofencing, velocity limits)
- Rendering for visualization

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from mavsdk import System
import asyncio

class BaseUAVEnv(gym.Env):
    """Base environment for UAV navigation tasks."""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(self, render_mode=None, px4_connection="udp://:14540"):
        super().__init__()

        # MAVSDK connection
        self.drone = System()
        self.px4_connection = px4_connection
        self.render_mode = render_mode

        # Safety constraints (from SYSTEM_CONSTRAINTS.qmd)
        self.max_velocity = 15.0  # m/s
        self.max_altitude = 120.0  # m AGL
        self.min_altitude = 2.0    # m AGL
        self.geofence_radius = 1000.0  # m

        # Simulation timestep
        self.dt = 0.1  # 10 Hz control loop

    async def _connect_px4(self):
        """Establish connection to PX4 SITL."""
        await self.drone.connect(system_address=self.px4_connection)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break

    def _check_safety_constraints(self, position, velocity):
        """Verify state satisfies safety constraints."""
        # Altitude limits
        if position[2] < -self.max_altitude or position[2] > -self.min_altitude:
            return False

        # Geofence (horizontal)
        horizontal_dist = np.linalg.norm(position[:2])
        if horizontal_dist > self.geofence_radius:
            return False

        # Velocity limits
        if np.linalg.norm(velocity) > self.max_velocity:
            return False

        return True

    def close(self):
        """Clean up resources."""
        # Disconnect from PX4
        asyncio.run(self.drone.action.kill())
```

## 1. Mission Planner Environment

### Purpose
Train high-level mission planning: waypoint sequencing, mission phase transitions, strategic decision-making.

### Observation Space
```python
class MissionPlannerEnv(BaseUAVEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Observation: mission context and strategic state
        self.observation_space = spaces.Dict({
            # Current mission phase
            "mission_phase": spaces.Discrete(5),  # [TAKEOFF, TRANSIT, SEARCH, ENGAGE, RTL]

            # Vehicle state (high-level)
            "position": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            "remaining_battery": spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32),

            # Mission objectives
            "target_position": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            "target_found": spaces.Discrete(2),  # Binary: found or not

            # Environmental context
            "wind_velocity": spaces.Box(low=-20.0, high=20.0, shape=(3,), dtype=np.float32),
            "time_of_day": spaces.Box(low=0.0, high=24.0, shape=(1,), dtype=np.float32),

            # Communication status
            "comms_available": spaces.Discrete(2),  # Communications-denied scenario
        })

        # Action: high-level mission decisions
        self.action_space = spaces.Dict({
            # Mission phase transition
            "next_phase": spaces.Discrete(5),

            # Strategic waypoint selection (relative to current position)
            "waypoint_offset": spaces.Box(low=-500.0, high=500.0, shape=(3,), dtype=np.float32),

            # Behavior mode for next phase
            "behavior_mode": spaces.Discrete(4),  # [DIRECT, CAUTIOUS, AGGRESSIVE, SEARCH]
        })
```

### Reward Shaping
```python
def _compute_mission_reward(self, obs, action, next_obs, info):
    """Reward function for mission-level decisions."""
    reward = 0.0

    # Mission progress (sparse reward)
    if next_obs["target_found"] and not obs["target_found"]:
        reward += 1000.0  # Large reward for achieving objective

    # Efficiency penalties
    distance_traveled = np.linalg.norm(next_obs["position"] - obs["position"])
    reward -= 0.1 * distance_traveled  # Encourage efficient paths

    # Battery management
    battery_consumed = obs["remaining_battery"] - next_obs["remaining_battery"]
    reward -= 10.0 * battery_consumed  # Penalize battery waste

    # Safety violations (terminal penalty)
    if info["constraint_violated"]:
        reward -= 500.0

    # Time penalty (encourage mission completion)
    reward -= 1.0  # Per timestep

    return reward
```

### Episode Termination
```python
def _check_termination(self, obs, info):
    """Determine if episode should terminate."""
    # Success: target found and returned to base
    if obs["target_found"] and obs["mission_phase"] == 4:  # RTL complete
        return True, False  # terminated, not truncated

    # Failure: battery depleted
    if obs["remaining_battery"] < 0.1:
        return True, False

    # Failure: safety constraint violated
    if info["constraint_violated"]:
        return True, False

    # Timeout: max mission duration exceeded
    if info["step_count"] > 10000:  # ~16 minutes at 10 Hz
        return False, True  # not terminated, truncated

    return False, False
```

## 2. Behavior Selector Environment

### Purpose
Train mid-level behavior selection: choosing appropriate navigation behaviors given mission context and sensor data.

### Observation Space
```python
class BehaviorSelectorEnv(BaseUAVEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.observation_space = spaces.Dict({
            # Vehicle state (detailed)
            "position": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            "velocity": spaces.Box(low=-20.0, high=20.0, shape=(3,), dtype=np.float32),
            "attitude": spaces.Box(low=-np.pi, high=np.pi, shape=(3,), dtype=np.float32),

            # Sensor data (processed)
            "depth_image": spaces.Box(low=0.0, high=255.0, shape=(64, 64, 1), dtype=np.uint8),
            "obstacle_distances": spaces.Box(low=0.0, high=50.0, shape=(8,), dtype=np.float32),  # 8 directions

            # Mission context (from high-level agent)
            "current_phase": spaces.Discrete(5),
            "target_waypoint": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            "behavior_mode": spaces.Discrete(4),

            # Local map (occupancy grid)
            "local_map": spaces.Box(low=0.0, high=1.0, shape=(32, 32), dtype=np.float32),
        })

        # Action: behavior primitive selection
        self.action_space = spaces.Discrete(8)
        # [FOLLOW_WAYPOINT, AVOID_OBSTACLE, HOVER, SEARCH_PATTERN,
        #  ORBIT_POINT, LAND, EMERGENCY_STOP, CLIMB]
```

### Reward Shaping
```python
def _compute_behavior_reward(self, obs, action, next_obs, info):
    """Reward function for behavior-level decisions."""
    reward = 0.0

    # Progress toward waypoint
    dist_before = np.linalg.norm(obs["position"] - obs["target_waypoint"])
    dist_after = np.linalg.norm(next_obs["position"] - next_obs["target_waypoint"])
    progress = dist_before - dist_after
    reward += 10.0 * progress  # Reward progress

    # Obstacle avoidance
    min_obstacle_dist = np.min(next_obs["obstacle_distances"])
    if min_obstacle_dist < 2.0:
        reward -= 50.0 * (2.0 - min_obstacle_dist)  # Heavy penalty for close obstacles

    # Smoothness (discourage erratic behavior)
    velocity_change = np.linalg.norm(next_obs["velocity"] - obs["velocity"])
    reward -= 1.0 * velocity_change

    # Waypoint reached
    if dist_after < 1.0:  # Within 1m of waypoint
        reward += 100.0

    # Collision (terminal penalty)
    if info["collision"]:
        reward -= 1000.0

    return reward
```

## 3. Trajectory Optimizer Environment

### Purpose
Train low-level trajectory optimization: generating smooth, dynamically feasible velocity commands.

### Observation Space
```python
class TrajectoryOptimizerEnv(BaseUAVEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.observation_space = spaces.Dict({
            # Precise vehicle state
            "position": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            "velocity": spaces.Box(low=-20.0, high=20.0, shape=(3,), dtype=np.float32),
            "acceleration": spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),
            "attitude": spaces.Box(low=-np.pi, high=np.pi, shape=(3,), dtype=np.float32),
            "angular_velocity": spaces.Box(low=-2*np.pi, high=2*np.pi, shape=(3,), dtype=np.float32),

            # Immediate sensor data
            "imu_accel": spaces.Box(low=-50.0, high=50.0, shape=(3,), dtype=np.float32),
            "imu_gyro": spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32),

            # Desired behavior (from mid-level agent)
            "desired_behavior": spaces.Discrete(8),
            "target_position": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            "target_velocity": spaces.Box(low=-20.0, high=20.0, shape=(3,), dtype=np.float32),

            # Immediate obstacles
            "nearest_obstacles": spaces.Box(low=0.0, high=10.0, shape=(6,), dtype=np.float32),  # +X, -X, +Y, -Y, +Z, -Z
        })

        # Action: velocity commands (continuous control)
        self.action_space = spaces.Box(
            low=np.array([-10.0, -10.0, -5.0]),   # [vx, vy, vz] in m/s
            high=np.array([10.0, 10.0, 5.0]),
            dtype=np.float32
        )
```

### Reward Shaping
```python
def _compute_trajectory_reward(self, obs, action, next_obs, info):
    """Reward function for trajectory-level control."""
    reward = 0.0

    # Track desired velocity
    velocity_error = np.linalg.norm(next_obs["velocity"] - obs["target_velocity"])
    reward -= 5.0 * velocity_error

    # Track desired position
    position_error = np.linalg.norm(next_obs["position"] - obs["target_position"])
    reward -= 2.0 * position_error

    # Smoothness (minimize jerk)
    jerk = np.linalg.norm(next_obs["acceleration"] - obs["acceleration"]) / self.dt
    reward -= 0.1 * jerk

    # Energy efficiency (minimize control effort)
    control_effort = np.linalg.norm(action)
    reward -= 0.01 * control_effort

    # Obstacle clearance (immediate safety)
    min_clearance = np.min(next_obs["nearest_obstacles"])
    if min_clearance < 1.0:
        reward -= 100.0 * (1.0 - min_clearance)

    # Collision (terminal)
    if info["collision"]:
        reward -= 500.0

    # Dynamic feasibility (penalize commands exceeding vehicle limits)
    if info["command_saturated"]:
        reward -= 10.0

    return reward
```

## Environment Integration with PX4 SITL

### Asynchronous MAVSDK Interface
```python
class UAVEnvMAVSDKWrapper:
    """Wrapper to handle async MAVSDK calls in synchronous Gym environment."""

    def __init__(self, env):
        self.env = env
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

    def reset(self, **kwargs):
        """Reset environment (synchronous wrapper)."""
        obs, info = self.loop.run_until_complete(self._async_reset(**kwargs))
        return obs, info

    async def _async_reset(self, **kwargs):
        """Reset PX4 SITL to initial state."""
        # Disarm and kill motors
        await self.env.drone.action.kill()

        # Reset to initial position
        await self.env.drone.action.goto_location(
            latitude_deg=47.397606,
            longitude_deg=8.545594,
            absolute_altitude_m=488.0,
            yaw_deg=0.0
        )

        # Arm and takeoff
        await self.env.drone.action.arm()
        await self.env.drone.action.takeoff()

        # Get initial observation
        obs = await self._get_observation()
        info = {}

        return obs, info

    def step(self, action):
        """Execute action in environment (synchronous wrapper)."""
        obs, reward, terminated, truncated, info = self.loop.run_until_complete(
            self._async_step(action)
        )
        return obs, reward, terminated, truncated, info

    async def _async_step(self, action):
        """Execute action via MAVSDK."""
        # Send velocity command to PX4
        await self.env.drone.offboard.set_velocity_ned(
            VelocityNedYaw(action[0], action[1], action[2], 0.0)
        )

        # Wait for simulation timestep
        await asyncio.sleep(self.env.dt)

        # Get new observation
        obs = await self._get_observation()

        # Compute reward
        reward = self.env._compute_reward(obs)

        # Check termination
        terminated, truncated = self.env._check_termination(obs)

        info = {}

        return obs, reward, terminated, truncated, info
```

## Vectorized Environments for Parallel Training

### Multi-Process Training
```python
from gymnasium.vector import AsyncVectorEnv

def make_mission_planner_env(rank):
    """Factory function for parallel environments."""
    def _init():
        # Each environment connects to different PX4 instance
        px4_port = 14540 + rank
        env = MissionPlannerEnv(px4_connection=f"udp://:{px4_port}")
        env = UAVEnvMAVSDKWrapper(env)
        return env
    return _init

# Create vectorized environment with 8 parallel instances
num_envs = 8
env = AsyncVectorEnv([make_mission_planner_env(i) for i in range(num_envs)])
```

## Environment Validation

### Check Environment Compatibility
```python
from stable_baselines3.common.env_checker import check_env

# Validate environment before training
env = MissionPlannerEnv()
env = UAVEnvMAVSDKWrapper(env)
check_env(env, warn=True)
```

## Ontology-Constrained Action Spaces

### Action Masking
Implement action masking to enforce ontological constraints:

```python
def get_valid_actions(self, obs):
    """Return mask of valid actions given current state."""
    valid_actions = np.ones(self.action_space.n, dtype=bool)

    # Example: Cannot transition to SEARCH phase if battery < 30%
    if obs["remaining_battery"] < 0.3 and obs["current_phase"] != 4:  # Not already RTL
        valid_actions[3] = False  # Disable SEARCH phase

    # Example: Cannot select LAND behavior if altitude < 5m
    if obs["position"][2] > -5.0:  # NED frame: negative is up
        valid_actions[5] = False  # Disable LAND

    return valid_actions
```

## Next Steps

1. **Implement Base Environment**: Start with `BaseUAVEnv` class
2. **Test MAVSDK Connection**: Verify PX4 SITL communication
3. **Build Mission Planner Env**: Implement high-level environment first
4. **Validate with Random Policy**: Test environment with random actions
5. **Tune Reward Functions**: Iterate on reward shaping based on training results
6. **Create Vectorized Setup**: Scale to parallel training

## References

- Custom Environment Creation: `gymnasium/custom_environments.md`
- Vectorized Environments: `gymnasium/vectorized_environments.md`
- SB3 Integration: `stable-baselines3/custom_env_integration.md`
- MAVSDK Python: `https://github.com/mavlink/MAVSDK-Python`
