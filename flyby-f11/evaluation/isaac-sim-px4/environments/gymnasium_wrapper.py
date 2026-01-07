"""
Gymnasium wrapper for Isaac Sim ISR environments.

Converts UAVState <-> numpy observations for RL training.

Architecture:
    IsaacSimGymWrapper (orchestrator) composes:
    - ObservationSpaceManager: Observation bounds computation
    - OntologyRewardShaper: Reward shaping for ontology behaviors
    - OntologyBehaviorController: Safety behavior preemption
    - OntologyBehaviorExecutor: Behavior command execution

The wrapper delegates observation conversion to the underlying environment,
allowing each canonical problem to define its own observation space.
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Any, Dict, Optional, Tuple

from .base_isr_env import BaseISREnvironment, UAVState
from .observation_space_manager import ObservationSpaceManager
from .ontology_reward_shaper import OntologyRewardShaper
from .ontology_behavior_controller import (
    OntologyBehaviorController,
    OntologyBehaviorExecutor,
    OntologyControllerConfig,
    OntologyBehavior,
    BehaviorCommand,
)


class IsaacSimGymWrapper(gym.Env):
    """
    Wraps Isaac Sim environment for Gymnasium compatibility.

    Handles:
    - Dynamic observation space based on underlying environment
    - Normalized action space [-1, 1] for RL compatibility
    - Proper reset/step signatures (Gymnasium v26+ API)
    - Truncation vs termination semantics
    - Ontology-based behavior preemption (RTL, emergency landing, etc.)
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, isaac_env: BaseISREnvironment, config: Optional[Dict] = None):
        super().__init__()
        self.isaac_env = isaac_env
        self.config = config or {}

        # Observation space manager
        obs_manager = ObservationSpaceManager(isaac_env)
        obs_low, obs_high = obs_manager.compute_bounds()
        self.observation_space = spaces.Box(
            low=obs_low, high=obs_high, dtype=np.float32
        )

        # Normalized action space: [vx, vy, vz, yaw_rate] in [-1, 1]
        self.action_space = spaces.Box(
            low=-np.ones(4, dtype=np.float32),
            high=np.ones(4, dtype=np.float32),
            dtype=np.float32
        )

        self._max_episode_steps = self.config.get('max_episode_steps', 10000)
        self._step_count = 0

        # Ontology controller
        ontology_config = OntologyControllerConfig(
            ontology_path=self.config.get('ontology_path', '/workspace/ontology/planning_mode'),
            battery_rtl_threshold=self.config.get('battery_rtl_threshold', 25.0),
            battery_rtl_comms_denied_threshold=self.config.get('battery_rtl_comms_denied', 30.0),
            battery_emergency_threshold=self.config.get('battery_emergency_threshold', 15.0),
            use_vampire=self.config.get('use_vampire', True),
        )
        self.ontology_controller = OntologyBehaviorController(ontology_config)
        self.behavior_executor = OntologyBehaviorExecutor()

        # Reward shaper (initialized in reset when home position is known)
        self.reward_shaper: Optional[OntologyRewardShaper] = None

        # Preemption state
        self._ontology_preempting = False
        self._active_behavior: Optional[BehaviorCommand] = None
        self._ontology_preemption_count = 0

    def _state_to_obs(self, state: UAVState) -> np.ndarray:
        """Convert UAVState to numpy observation."""
        return self.isaac_env.state_to_observation(state)

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Reset environment."""
        super().reset(seed=seed)
        self._step_count = 0

        state = self.isaac_env.reset(seed=seed)
        obs = self._state_to_obs(state)

        # Reset ontology controller
        self.ontology_controller.reset()
        self.ontology_controller.set_home_position(self.isaac_env.config.spawn_position)
        self._ontology_preempting = False
        self._active_behavior = None

        # Initialize reward shaper with home position
        self.reward_shaper = OntologyRewardShaper(
            home_position=self.isaac_env.config.spawn_position,
            takeoff_altitude=self.ontology_controller.config.takeoff_altitude,
            takeoff_tolerance=self.ontology_controller.config.takeoff_altitude_tolerance,
        )

        # Pass mission tasking if available
        if hasattr(self.isaac_env, 'mission_tasking') and self.isaac_env.mission_tasking is not None:
            self.ontology_controller.receive_mission_tasking(self.isaac_env.mission_tasking)

        info = {
            "raw_state": state,
            "step": 0,
            "ontology_preempting": False,
            "ontology_behavior": None,
            "mission_progress": self.ontology_controller.get_mission_progress(),
        }
        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one step with ontology behavior preemption."""
        self._step_count += 1

        current_state = self.isaac_env.uav_state

        # Ontology check: Does ontology need to preempt RL?
        behavior_command = self.ontology_controller.update(current_state)

        if behavior_command is not None and behavior_command.preempts_rl:
            # Ontology preempting
            self._ontology_preempting = True
            self._active_behavior = behavior_command
            self._ontology_preemption_count += 1

            ontology_action = self.behavior_executor.execute(behavior_command, current_state)
            effective_action = ontology_action
        else:
            # RL has control
            self._ontology_preempting = False
            self._active_behavior = None
            effective_action = action

        # Execute action
        state, reward, terminated, info = self.isaac_env.step(effective_action)

        obs = self._state_to_obs(state)
        truncated = self._step_count >= self._max_episode_steps

        # Reward shaping for ontology behaviors
        if self._ontology_preempting and self.reward_shaper is not None:
            reward = self.reward_shaper.shape_reward(reward, state, behavior_command)

            # Check behavior completion
            person_avoidance_active = getattr(
                self.ontology_controller, '_person_avoidance_active', False
            )
            if self.reward_shaper.check_behavior_complete(
                state, behavior_command, person_avoidance_active
            ):
                info['ontology_behavior_complete'] = True

        # Termination from ontology behaviors
        if behavior_command is not None:
            if behavior_command.behavior == OntologyBehavior.EMERGENCY_LAND:
                if state.position[2] < 0.5:
                    terminated = True
                    info['termination_reason'] = 'emergency_landing_complete'

        info.update({
            "raw_state": state,
            "step": self._step_count,
            "ontology_preempting": self._ontology_preempting,
            "ontology_behavior": behavior_command.behavior.name if behavior_command else None,
            "ontology_violations": [v.axiom_name for v in self.ontology_controller.active_violations],
            "ontology_rtl_triggered": self.ontology_controller.rtl_triggered,
            "ontology_emergency_triggered": self.ontology_controller.emergency_triggered,
            "ontology_stats": self.ontology_controller.get_stats(),
            "perception_stats": self.isaac_env.get_perception_stats(),
        })

        return obs, reward, terminated, truncated, info

    def render(self):
        """Return RGB array from Isaac Sim camera."""
        if hasattr(self.isaac_env, 'get_camera_image'):
            return self.isaac_env.get_camera_image()
        return None

    def close(self):
        """Close environment."""
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
