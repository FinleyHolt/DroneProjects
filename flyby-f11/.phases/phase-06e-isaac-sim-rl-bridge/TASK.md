# Phase 6e: Isaac Sim RL Training Bridge

## Overview

Complete the Isaac Sim to RL training pipeline by implementing the remaining integration components: Gymnasium wrapper, Stable-Baselines3 agents, action-to-MAVLink bridge, and training infrastructure. This phase bridges the gap between the working Isaac Sim simulation and actual RL training.

## Human Description

The Isaac Sim evaluation environment has:
- Working drone simulation with PX4 SITL
- Procedural world generation (terrain, trees, lighting)
- Sensor suite (GPS, IMU, cameras, depth)
- Environment base classes with `step()`, `reset()`, `compute_reward()`
- Domain randomization infrastructure

What's missing to start training:
1. **Gymnasium compatibility wrapper** - Convert `UAVState` to numpy observations
2. **Stable-Baselines3 agents** - Replace `DummyAgent` with SAC/PPO/TD3
3. **Action → MAVLink bridge** - Connect agent actions to PX4 commands
4. **Training loop integration** - Experience collection, learning, checkpointing
5. **Safety action filtering** - Vampire queries during training

This phase does NOT require a custom F-11 3D model - the generic Iris model with correct physical parameters (mass, thrust, drag) provides functional accuracy for sim-to-real transfer.

## AI Agent Instructions

### Prerequisites
- Phase 6b completed (ontology_rl package exists)
- Isaac Sim container builds and runs (`fly_f11_camera_test.py` works)
- PX4 SITL connects ("Ready for takeoff!" message)
- Understanding of Stable-Baselines3 API

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

#### 1. Add Gymnasium Wrapper to Isaac Sim Environments

The Isaac Sim environments return `UAVState` objects but Gymnasium expects numpy arrays.

**File:** `evaluation/isaac-sim-px4/environments/gymnasium_wrapper.py`

```python
"""
Gymnasium wrapper for Isaac Sim ISR environments.
Converts UAVState <-> numpy observations for RL training.
"""
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Any, Dict, Optional, Tuple

from .base_isr_env import BaseISREnv, UAVState


class IsaacSimGymWrapper(gym.Env):
    """
    Wraps Isaac Sim environment for Gymnasium compatibility.

    Handles:
    - UAVState to numpy observation conversion
    - Action space definition (continuous position commands)
    - Proper reset/step signatures
    - Truncation vs termination semantics
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, isaac_env: BaseISREnv, config: Optional[Dict] = None):
        super().__init__()
        self.isaac_env = isaac_env
        self.config = config or {}

        # Define observation space based on UAVState fields
        # [x, y, z, vx, vy, vz, roll, pitch, yaw, battery,
        #  in_geofence, in_nfz, gps_valid, mission_progress]
        self.observation_space = spaces.Box(
            low=np.array([
                -1000, -1000, 0,      # position
                -20, -20, -10,        # velocity
                -np.pi, -np.pi/2, -np.pi,  # orientation
                0,                     # battery
                0, 0, 0,              # flags
                0                      # progress
            ], dtype=np.float32),
            high=np.array([
                1000, 1000, 500,
                20, 20, 10,
                np.pi, np.pi/2, np.pi,
                100,
                1, 1, 1,
                1
            ], dtype=np.float32),
            dtype=np.float32
        )

        # Continuous action: [vx, vy, vz, yaw_rate] velocity commands
        self.action_space = spaces.Box(
            low=np.array([-5, -5, -3, -1], dtype=np.float32),
            high=np.array([5, 5, 3, 1], dtype=np.float32),
            dtype=np.float32
        )

        self._max_episode_steps = self.config.get('max_episode_steps', 10000)
        self._step_count = 0

    def _state_to_obs(self, state: UAVState) -> np.ndarray:
        """Convert UAVState to numpy observation."""
        return np.array([
            state.position[0],
            state.position[1],
            state.position[2],
            state.velocity[0] if state.velocity is not None else 0,
            state.velocity[1] if state.velocity is not None else 0,
            state.velocity[2] if state.velocity is not None else 0,
            state.orientation[0] if state.orientation is not None else 0,
            state.orientation[1] if state.orientation is not None else 0,
            state.orientation[2] if state.orientation is not None else 0,
            state.battery_pct,
            float(state.in_geofence),
            float(state.in_nfz),
            float(state.gps_valid),
            state.mission_progress if hasattr(state, 'mission_progress') else 0,
        ], dtype=np.float32)

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        self._step_count = 0

        state = self.isaac_env.reset(seed=seed)
        obs = self._state_to_obs(state)

        info = {
            "raw_state": state,
            "step": 0
        }
        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        self._step_count += 1

        # Execute action in Isaac Sim
        state, reward, terminated, info = self.isaac_env.step(action)

        obs = self._state_to_obs(state)
        truncated = self._step_count >= self._max_episode_steps

        info.update({
            "raw_state": state,
            "step": self._step_count
        })

        return obs, reward, terminated, truncated, info

    def render(self):
        """Return RGB array from Isaac Sim camera."""
        if hasattr(self.isaac_env, 'get_camera_image'):
            return self.isaac_env.get_camera_image()
        return None

    def close(self):
        self.isaac_env.close()


def make_isaac_gym_env(problem: str, config: Dict) -> IsaacSimGymWrapper:
    """Factory function to create wrapped Isaac Sim environment."""
    from .comms_denied_env import CommsDeniedSurveillanceEnv, CommsDeniedConfig
    from .dynamic_nfz_env import DynamicNFZAvoidanceEnv, DynamicNFZConfig
    from .multi_objective_env import MultiObjectiveISREnv, MultiObjectiveConfig

    if problem == 'comms_denied':
        env_config = CommsDeniedConfig(**config.get('env_params', {}))
        isaac_env = CommsDeniedSurveillanceEnv(env_config)
    elif problem == 'dynamic_nfz':
        env_config = DynamicNFZConfig(**config.get('env_params', {}))
        isaac_env = DynamicNFZAvoidanceEnv(env_config)
    elif problem == 'multi_objective':
        env_config = MultiObjectiveConfig(**config.get('env_params', {}))
        isaac_env = MultiObjectiveISREnv(env_config)
    else:
        raise ValueError(f"Unknown problem: {problem}")

    return IsaacSimGymWrapper(isaac_env, config)
```

#### 2. Implement Action → MAVLink Bridge

Connect agent velocity commands to PX4 through Pegasus.

**File:** `evaluation/isaac-sim-px4/environments/action_bridge.py`

```python
"""
Action bridge: Convert RL agent actions to PX4 MAVLink commands.
"""
import numpy as np
from typing import Optional
from pymavlink import mavutil


class PX4ActionBridge:
    """
    Bridges RL agent actions to PX4 velocity commands.

    Supports:
    - Velocity setpoints (vx, vy, vz, yaw_rate)
    - Position setpoints (x, y, z, yaw)
    - Safety limiting (max velocity, geofence)
    """

    def __init__(self, mavlink_connection: Optional[mavutil.mavlink_connection] = None):
        self.mav = mavlink_connection

        # Safety limits
        self.max_horizontal_vel = 6.3  # m/s (F-11 cruise speed)
        self.max_vertical_vel = 3.0    # m/s
        self.max_yaw_rate = 1.0        # rad/s

    def set_mavlink_connection(self, mav):
        """Set MAVLink connection (called after PX4 connects)."""
        self.mav = mav

    def send_velocity_command(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float
    ) -> bool:
        """
        Send velocity setpoint to PX4.

        Args:
            vx: Forward velocity (m/s, body frame)
            vy: Right velocity (m/s, body frame)
            vz: Down velocity (m/s, NED frame, positive = descend)
            yaw_rate: Yaw rate (rad/s)

        Returns:
            True if command sent successfully
        """
        if self.mav is None:
            return False

        # Apply safety limits
        vx = np.clip(vx, -self.max_horizontal_vel, self.max_horizontal_vel)
        vy = np.clip(vy, -self.max_horizontal_vel, self.max_horizontal_vel)
        vz = np.clip(vz, -self.max_vertical_vel, self.max_vertical_vel)
        yaw_rate = np.clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)

        # Send SET_POSITION_TARGET_LOCAL_NED with velocity mask
        # type_mask: ignore position, use velocity
        type_mask = 0b0000011111000111

        self.mav.mav.set_position_target_local_ned_send(
            0,                      # time_boot_ms
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,               # position (ignored)
            vx, vy, vz,            # velocity
            0, 0, 0,               # acceleration (ignored)
            0,                      # yaw (ignored)
            yaw_rate                # yaw_rate
        )
        return True

    def action_to_velocity(self, action: np.ndarray) -> tuple:
        """
        Convert normalized action to velocity command.

        Args:
            action: [vx, vy, vz, yaw_rate] in [-1, 1] range

        Returns:
            (vx, vy, vz, yaw_rate) in physical units
        """
        vx = action[0] * self.max_horizontal_vel
        vy = action[1] * self.max_horizontal_vel
        vz = action[2] * self.max_vertical_vel
        yaw_rate = action[3] * self.max_yaw_rate

        return vx, vy, vz, yaw_rate
```

#### 3. Integrate _apply_action in Base Environment

Update `base_isr_env.py` to use the action bridge.

**Edit:** `evaluation/isaac-sim-px4/environments/base_isr_env.py`

```python
def _apply_action(self, action: np.ndarray):
    """
    Apply action to drone through PX4 MAVLink.

    Args:
        action: [vx, vy, vz, yaw_rate] velocity command
    """
    if self.action_bridge is None:
        # Initialize action bridge with MAVLink connection from backend
        if hasattr(self, 'drone') and hasattr(self.drone, '_backends'):
            for backend in self.drone._backends:
                if hasattr(backend, '_connection'):
                    self.action_bridge = PX4ActionBridge(backend._connection)
                    break

    if self.action_bridge:
        vx, vy, vz, yaw_rate = self.action_bridge.action_to_velocity(action)
        self.action_bridge.send_velocity_command(vx, vy, vz, yaw_rate)
```

#### 4. Create Stable-Baselines3 Agent Implementations

**File:** `evaluation/isaac-sim-px4/scripts/training/agents/sac_agent.py`

```python
"""
SAC Agent for Mission Planner (Level 1).
"""
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
import torch
from pathlib import Path
from typing import Dict, Any, Optional


def create_sac_agent(
    env,
    config: Dict[str, Any],
    tensorboard_log: Optional[str] = None
) -> SAC:
    """
    Create SAC agent for mission planning.

    Args:
        env: Gymnasium environment
        config: Agent configuration from training_config.yaml
        tensorboard_log: Path for tensorboard logs

    Returns:
        Configured SAC agent
    """
    policy_kwargs = dict(
        net_arch=config.get('hidden_dims', [256, 256, 128]),
        activation_fn=torch.nn.ReLU
    )

    agent = SAC(
        policy="MlpPolicy",
        env=env,
        learning_rate=config.get('learning_rate', 3e-4),
        buffer_size=config.get('buffer_size', 1_000_000),
        learning_starts=config.get('learning_starts', 10_000),
        batch_size=config.get('batch_size', 256),
        tau=config.get('tau', 0.005),
        gamma=config.get('gamma', 0.99),
        train_freq=(1, "step"),
        gradient_steps=config.get('update_frequency', 1),
        policy_kwargs=policy_kwargs,
        tensorboard_log=tensorboard_log,
        verbose=1,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )

    return agent


def create_ppo_agent(env, config: Dict[str, Any], tensorboard_log: Optional[str] = None):
    """Create PPO agent for behavior selection."""
    from stable_baselines3 import PPO

    policy_kwargs = dict(
        net_arch=config.get('hidden_dims', [128, 128]),
        activation_fn=torch.nn.ReLU
    )

    return PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=config.get('learning_rate', 3e-4),
        n_steps=2048,
        batch_size=config.get('batch_size', 64),
        n_epochs=config.get('n_epochs', 10),
        gamma=config.get('gamma', 0.99),
        clip_range=config.get('clip_range', 0.2),
        ent_coef=config.get('ent_coef', 0.01),
        policy_kwargs=policy_kwargs,
        tensorboard_log=tensorboard_log,
        verbose=1,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )


def create_td3_agent(env, config: Dict[str, Any], tensorboard_log: Optional[str] = None):
    """Create TD3 agent for trajectory optimization."""
    from stable_baselines3 import TD3

    policy_kwargs = dict(
        net_arch=config.get('hidden_dims', [64, 64]),
        activation_fn=torch.nn.ReLU
    )

    return TD3(
        policy="MlpPolicy",
        env=env,
        learning_rate=config.get('learning_rate', 1e-3),
        buffer_size=config.get('buffer_size', 100_000),
        learning_starts=config.get('learning_starts', 1000),
        batch_size=config.get('batch_size', 100),
        tau=config.get('tau', 0.005),
        gamma=config.get('gamma', 0.99),
        policy_delay=config.get('policy_delay', 2),
        policy_kwargs=policy_kwargs,
        tensorboard_log=tensorboard_log,
        verbose=1,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )
```

#### 5. Update Training Script

**Edit:** `evaluation/isaac-sim-px4/scripts/training/train_canonical.py`

Replace `DummyAgent` with actual SB3 agents:

```python
def create_agent(env, config: Dict[str, Any], agent_type: str, log_dir: str):
    """Create RL agent based on type."""
    from agents.sac_agent import create_sac_agent, create_ppo_agent, create_td3_agent

    rl_config = config.get('rl', {}).get(agent_type, {})

    if agent_type == 'mission_planner':
        return create_sac_agent(env, rl_config, tensorboard_log=log_dir)
    elif agent_type == 'behavior_selector':
        return create_ppo_agent(env, rl_config, tensorboard_log=log_dir)
    elif agent_type == 'trajectory_optimizer':
        return create_td3_agent(env, rl_config, tensorboard_log=log_dir)
    else:
        raise ValueError(f"Unknown agent type: {agent_type}")


def train(config: Dict[str, Any], problem: str, agent_type: str = 'mission_planner'):
    """Main training loop."""
    from stable_baselines3.common.callbacks import CheckpointCallback
    from gymnasium_wrapper import make_isaac_gym_env

    # Create environment
    env = make_isaac_gym_env(problem, config)

    # Create agent
    log_dir = f"./logs/{problem}_{agent_type}"
    agent = create_agent(env, config, agent_type, log_dir)

    # Setup callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=config['training']['checkpoints']['save_frequency'] * 1000,
        save_path=config['training']['checkpoints']['path'],
        name_prefix=f"{problem}_{agent_type}"
    )

    # Train
    total_timesteps = (
        config['training']['total_episodes'] *
        config['training']['max_episode_steps']
    )

    agent.learn(
        total_timesteps=total_timesteps,
        callback=checkpoint_callback,
        progress_bar=True
    )

    # Save final model
    agent.save(f"{log_dir}/final_model")
    env.close()
```

#### 6. Add Stable-Baselines3 to Container

**Edit:** `evaluation/isaac-sim-px4/Containerfile.sitl`

Add to pip install section:
```dockerfile
RUN /isaac-sim/python.sh -m pip install \
    stable-baselines3[extra] \
    tensorboard \
    gymnasium
```

#### 7. Create Safety Action Filter (Vampire Integration)

**File:** `evaluation/isaac-sim-px4/environments/safety_filter.py`

```python
"""
Safety filter using Vampire theorem prover.
Checks actions against ontology constraints before execution.
"""
import subprocess
import tempfile
from pathlib import Path
from typing import Optional, Tuple
import numpy as np


class VampireSafetyFilter:
    """
    Filters actions through Vampire theorem prover for safety.

    Hard constraints (blocked):
    - geofenceViolation
    - noFlyZoneViolation
    - highThreatExposure

    Soft constraints (penalized but allowed):
    - nfz_buffer_penetration
    - medium_threat_exposure
    """

    def __init__(
        self,
        ontology_path: str = "/workspace/ontology/planning_mode",
        timeout_ms: int = 50
    ):
        self.ontology_path = Path(ontology_path)
        self.timeout_s = timeout_ms / 1000.0
        self.hard_constraints = [
            'geofenceViolation',
            'noFlyZoneViolation',
            'highThreatExposure',
            'mustLand',
            'mustReturnToLaunch'
        ]

    def check_action_safety(
        self,
        current_state: dict,
        proposed_action: np.ndarray,
        predicted_next_state: dict
    ) -> Tuple[bool, Optional[str]]:
        """
        Check if proposed action is safe.

        Args:
            current_state: Current UAV state as dict
            proposed_action: Action agent wants to take
            predicted_next_state: Predicted state after action

        Returns:
            (is_safe, violation_type) - True if safe, else violation name
        """
        # Generate TPTP query
        query = self._generate_safety_query(current_state, predicted_next_state)

        # Run Vampire
        result = self._run_vampire(query)

        # Parse result for violations
        for constraint in self.hard_constraints:
            if constraint in result:
                return False, constraint

        return True, None

    def _generate_safety_query(self, current: dict, next_state: dict) -> str:
        """Generate TPTP safety query."""
        query = f"""
% Current state
fof(current_pos, axiom, position(uav, {current['x']:.2f}, {current['y']:.2f}, {current['z']:.2f})).
fof(current_battery, axiom, batteryLevel(uav, {current['battery']:.1f})).

% Predicted next state
fof(next_pos, axiom, nextPosition(uav, {next_state['x']:.2f}, {next_state['y']:.2f}, {next_state['z']:.2f})).

% Query: Is there any safety violation?
fof(safety_check, conjecture,
    ~(geofenceViolation(uav) | noFlyZoneViolation(uav) | highThreatExposure(uav))
).
"""
        return query

    def _run_vampire(self, query: str) -> str:
        """Run Vampire on query and return result."""
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.tptp', delete=False) as f:
                f.write(query)
                query_file = f.name

            result = subprocess.run(
                ['vampire', '--input_syntax', 'tptp', '--time_limit', str(int(self.timeout_s))],
                input=query,
                capture_output=True,
                text=True,
                timeout=self.timeout_s + 1
            )
            return result.stdout
        except subprocess.TimeoutExpired:
            return ""
        except Exception as e:
            print(f"Vampire error: {e}")
            return ""

    def get_safe_fallback_action(self, current_state: dict) -> np.ndarray:
        """Return safe fallback action (hover in place)."""
        return np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
```

#### 8. Create Integration Test

**File:** `evaluation/isaac-sim-px4/scripts/training/test_training_loop.py`

```python
#!/usr/bin/env python3
"""
Integration test for training pipeline.
Verifies:
1. Environment creates and resets
2. Agent can select actions
3. Actions execute in Isaac Sim
4. Rewards computed correctly
5. Episodes complete without crash
"""
import sys
sys.path.insert(0, '/workspace/environments')
sys.path.insert(0, '/workspace/scripts/training')

def test_basic_episode():
    """Run 100 steps in environment."""
    from gymnasium_wrapper import make_isaac_gym_env

    config = {
        'env_params': {
            'headless': True,
            'max_mission_time': 60.0
        },
        'max_episode_steps': 100
    }

    env = make_isaac_gym_env('comms_denied', config)

    obs, info = env.reset()
    print(f"Initial observation shape: {obs.shape}")

    total_reward = 0
    for step in range(100):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward

        if step % 20 == 0:
            print(f"Step {step}: reward={reward:.2f}, total={total_reward:.2f}")

        if terminated or truncated:
            print(f"Episode ended at step {step}")
            break

    env.close()
    print(f"Test passed! Total reward: {total_reward:.2f}")


def test_agent_creation():
    """Test SB3 agent creation."""
    from gymnasium_wrapper import make_isaac_gym_env
    from agents.sac_agent import create_sac_agent

    config = {
        'env_params': {'headless': True},
        'max_episode_steps': 100
    }

    env = make_isaac_gym_env('comms_denied', config)

    agent_config = {
        'hidden_dims': [64, 64],
        'learning_rate': 3e-4,
        'buffer_size': 1000
    }

    agent = create_sac_agent(env, agent_config)
    print(f"Agent created: {type(agent)}")

    # Take a few learning steps
    agent.learn(total_timesteps=100)
    print("Agent learning test passed!")

    env.close()


if __name__ == "__main__":
    print("=" * 60)
    print("Training Pipeline Integration Test")
    print("=" * 60)

    print("\n1. Testing basic episode...")
    test_basic_episode()

    print("\n2. Testing agent creation...")
    test_agent_creation()

    print("\n" + "=" * 60)
    print("All tests passed!")
    print("=" * 60)
```

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `gymnasium_wrapper.py` converts UAVState to numpy observations
- [ ] `action_bridge.py` sends velocity commands to PX4
- [ ] SAC/PPO/TD3 agents create without errors
- [ ] `train_canonical.py` runs with real agents (not DummyAgent)
- [ ] 100 episode steps complete without crash
- [ ] Tensorboard logs generated during training
- [ ] Checkpoints saved at specified intervals
- [ ] Safety filter blocks obviously unsafe actions

### Verification

Run automated verification:
```bash
bash .phases/phase-06e-isaac-sim-rl-bridge/verification.sh
```

Manual verification:
```bash
# Start container
./scripts/run_isaac_gui.sh isaac-training

# Run integration test
podman exec isaac-training /isaac-sim/python.sh \
    /workspace/scripts/training/test_training_loop.py

# Run short training (100 episodes)
podman exec isaac-training /isaac-sim/python.sh \
    /workspace/scripts/training/train_canonical.py \
    --config /workspace/config/training_config.yaml \
    --problem comms_denied \
    --episodes 100
```

### Common Pitfalls

1. **Observation space mismatch**: Ensure UAVState fields match observation_space bounds
2. **Action scaling**: SB3 expects actions in action_space bounds, may need normalization
3. **MAVLink timing**: PX4 needs continuous heartbeat, don't block too long
4. **CUDA memory**: SAC uses replay buffer, may need to reduce buffer_size on low VRAM
5. **Vampire timeout**: 50ms target may not be achievable under load, use fallback

### References

- [Stable-Baselines3 Docs](https://stable-baselines3.readthedocs.io/)
- [Gymnasium API](https://gymnasium.farama.org/)
- [PX4 MAVLink Offboard Control](https://docs.px4.io/main/en/ros/mavros_offboard.html)
- [training_config.yaml](../../evaluation/isaac-sim-px4/config/training_config.yaml)

### Dependencies
See `dependencies.json` - requires Phase 6b completion.

### Next Phase
After completion, proceed to Phase 7: Mission Planner RL Agent training.
