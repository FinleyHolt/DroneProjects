"""
Gymnasium wrapper for Isaac Sim ISR environments.
Converts UAVState <-> numpy observations for RL training.

The wrapper delegates observation conversion to the underlying environment,
allowing each canonical problem to define its own observation space.

Integrates OntologyBehaviorController for safety-critical behavior preemption.
When ontology axioms are violated, the controller takes over from RL.
"""
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Any, Dict, Optional, Tuple

from .base_isr_env import BaseISREnvironment, UAVState, GNSSStatus
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

    Ontology Integration:
    The wrapper monitors UAV state via OntologyBehaviorController. When safety-
    critical axioms are violated (low battery, geofence breach, etc.), the
    ontology controller preempts RL and executes the appropriate behavior.

    This ensures the ontology is the "source of truth" for safety-critical
    behaviors while allowing RL to learn mission execution within constraints.
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, isaac_env: BaseISREnvironment, config: Optional[Dict] = None):
        super().__init__()
        self.isaac_env = isaac_env
        self.config = config or {}

        # Get observation dimension from the actual environment
        # This allows each canonical problem to define its own observation space
        obs_dim = isaac_env.observation_dim

        # Define observation space with proper bounds
        # State observations have varying ranges, see base_isr_env.state_to_observation()
        # pos(3) + vel(3) + quat(4) + wind(3) + scalars(15) = 28 base dims
        # + perception(516) if enabled = 544 total
        obs_low, obs_high = self._compute_observation_bounds(isaac_env, obs_dim)
        self.observation_space = spaces.Box(
            low=obs_low,
            high=obs_high,
            dtype=np.float32
        )

        # Normalized action space: [vx, vy, vz, yaw_rate] in [-1, 1]
        # The action_bridge scales these to physical units (m/s, rad/s)
        self.action_space = spaces.Box(
            low=-np.ones(4, dtype=np.float32),
            high=np.ones(4, dtype=np.float32),
            dtype=np.float32
        )

        self._max_episode_steps = self.config.get('max_episode_steps', 10000)
        self._step_count = 0

        # === Ontology Behavior Controller ===
        # Monitors state and triggers safety-critical behaviors
        ontology_config = OntologyControllerConfig(
            ontology_path=self.config.get('ontology_path', '/workspace/ontology/planning_mode'),
            battery_rtl_threshold=self.config.get('battery_rtl_threshold', 25.0),
            battery_rtl_comms_denied_threshold=self.config.get('battery_rtl_comms_denied', 30.0),
            battery_emergency_threshold=self.config.get('battery_emergency_threshold', 15.0),
            use_vampire=self.config.get('use_vampire', True),
        )
        self.ontology_controller = OntologyBehaviorController(ontology_config)
        self.behavior_executor = OntologyBehaviorExecutor()

        # Track ontology preemption state
        self._ontology_preempting = False
        self._active_behavior: Optional[BehaviorCommand] = None
        self._ontology_preemption_count = 0

    def _compute_observation_bounds(
        self,
        env: BaseISREnvironment,
        obs_dim: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute proper observation space bounds.

        The observation vector from state_to_observation() contains:
        - position: 3D world position (unbounded, but geofence limits to ±150m)
        - velocity: 3D velocity (±10 m/s max)
        - orientation: quaternion (unit length, components in [-1, 1])
        - wind: wind vector normalized by 10 m/s → [-1, 1]
        - battery_pct / 100: [0, 1]
        - mission_time / max_time: [0, 1]
        - coverage_pct / 100: [0, 1]
        - in_geofence: [0, 1]
        - in_nfz: [0, 1]
        - nfz_distance / 100: [0, inf] clipped to [0, 1]
        - in_threat_zone: [0, 1]
        - threat_level / 3: [0, 1]
        - threat_exposure / max: [0, 1]
        - autonomous_mode: [0, 1]
        - comms_status / 2: [0, 1]
        - gnss_status / 2: [0, 1]
        - vio_valid: [0, 1]
        - gnss_degradation_triggered: [0, 1]
        - battery_drain_multiplier: [0.9, 1.1] typically

        + perception observations: 516 dims with varying bounds
        """
        obs_low = np.zeros(obs_dim, dtype=np.float32)
        obs_high = np.ones(obs_dim, dtype=np.float32)

        # Base state dimensions: 28
        # position (3): geofence bounds
        gf = env.config.geofence
        obs_low[0:3] = [gf.min_x, gf.min_y, gf.min_z]
        obs_high[0:3] = [gf.max_x, gf.max_y, gf.max_z]

        # velocity (3): ±10 m/s
        obs_low[3:6] = -10.0
        obs_high[3:6] = 10.0

        # orientation quaternion (4): [-1, 1]
        obs_low[6:10] = -1.0
        obs_high[6:10] = 1.0

        # wind (3): [-1, 1] (normalized)
        obs_low[10:13] = -1.0
        obs_high[10:13] = 1.0

        # scalars (indices 13-27): mostly [0, 1]
        # battery_pct/100, mission_time/max, coverage_pct/100: [0, 1]
        obs_low[13:16] = 0.0
        obs_high[13:16] = 1.0

        # in_geofence, in_nfz: [0, 1]
        obs_low[16:18] = 0.0
        obs_high[16:18] = 1.0

        # nfz_distance/100: [0, inf] but typically [0, 1]
        obs_low[18] = 0.0
        obs_high[18] = 2.0  # Allow some overflow

        # in_threat_zone, threat_level/3, threat_exposure/max: [0, 1]
        obs_low[19:22] = 0.0
        obs_high[19:22] = 1.0

        # autonomous_mode, comms_status/2, gnss_status/2, vio_valid: [0, 1]
        obs_low[22:26] = 0.0
        obs_high[22:26] = 1.0

        # gnss_degradation_triggered: [0, 1]
        obs_low[26] = 0.0
        obs_high[26] = 1.0

        # battery_drain_multiplier: typically [0.8, 1.2]
        obs_low[27] = 0.5
        obs_high[27] = 1.5

        # Perception observations (516 dims if enabled)
        if obs_dim > 28:
            # Get bounds from perception encoder if available
            if hasattr(env, 'perception') and env.perception is not None:
                if hasattr(env.perception, 'get_observation_space_bounds'):
                    p_low, p_high = env.perception.get_observation_space_bounds()
                    obs_low[28:] = p_low
                    obs_high[28:] = p_high
                else:
                    # Default perception bounds
                    # Priority detections can have negative values (rel pos, bearing, velocity)
                    obs_low[28:] = -1.0
                    obs_high[28:] = 1.0
            else:
                # Default perception bounds
                obs_low[28:] = -1.0
                obs_high[28:] = 1.0

        return obs_low, obs_high

    def _state_to_obs(self, state: UAVState) -> np.ndarray:
        """
        Convert UAVState to numpy observation.

        Delegates to the underlying environment's state_to_observation method,
        which handles environment-specific observation features.
        """
        return self.isaac_env.state_to_observation(state)

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        self._step_count = 0

        print("[Gymnasium] reset() called, calling isaac_env.reset()...")
        state = self.isaac_env.reset(seed=seed)
        print(f"[Gymnasium] isaac_env.reset() done, state.position = {state.position}")
        obs = self._state_to_obs(state)
        print(f"[Gymnasium] obs shape = {obs.shape}, has NaN = {np.any(np.isnan(obs))}, has Inf = {np.any(np.isinf(obs))}")

        # Reset ontology controller for new episode
        self.ontology_controller.reset()
        self.ontology_controller.set_home_position(self.isaac_env.config.spawn_position)
        self._ontology_preempting = False
        self._active_behavior = None

        # Pass mission tasking to ontology controller if available
        # This enables the ontology to generate a flight plan
        if hasattr(self.isaac_env, 'mission_tasking') and self.isaac_env.mission_tasking is not None:
            self.ontology_controller.receive_mission_tasking(self.isaac_env.mission_tasking)
            print("[Gymnasium] Mission tasking passed to ontology controller")

        info = {
            "raw_state": state,
            "step": 0,
            "ontology_preempting": False,
            "ontology_behavior": None,
            "mission_progress": self.ontology_controller.get_mission_progress(),
        }
        print("[Gymnasium] reset() complete, returning observation")
        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Execute one environment step with ontology behavior preemption.

        The ontology controller is checked BEFORE executing the RL action.
        If a safety-critical axiom is violated, the ontology controller
        preempts RL and executes its own behavior (RTL, emergency land, etc.).

        This ensures:
        1. Ontology is the "source of truth" for safety-critical behaviors
        2. RL only controls when all safety axioms are satisfied
        3. Smooth handoff between RL and ontology behaviors
        """
        import sys
        self._step_count += 1
        if self._step_count <= 5 or self._step_count % 100 == 0:
            print(f"[Gymnasium] step {self._step_count}, action = {action}")
            sys.stdout.flush()

        # Get current state for ontology check
        current_state = self.isaac_env.uav_state

        # === ONTOLOGY CHECK: Does ontology need to preempt RL? ===
        behavior_command = self.ontology_controller.update(current_state)

        # Debug: Log ontology check results for first few steps
        if self._step_count <= 10 or self._step_count % 500 == 0:
            alt = current_state.position[2]
            print(f"  [Ontology] step={self._step_count}, alt={alt:.2f}m, command={behavior_command.behavior.name if behavior_command else 'None'}")
            sys.stdout.flush()

        if behavior_command is not None and behavior_command.preempts_rl:
            # Ontology is preempting RL
            self._ontology_preempting = True
            self._active_behavior = behavior_command
            self._ontology_preemption_count += 1

            # Execute ontology-commanded behavior instead of RL action
            ontology_action = self.behavior_executor.execute(behavior_command, current_state)
            effective_action = ontology_action

            # Debug: Show ontology override
            if self._step_count <= 10:
                print(f"  [Ontology OVERRIDE] Using ontology action: {ontology_action}")
                sys.stdout.flush()
        else:
            # RL has control (within safety constraints)
            self._ontology_preempting = False
            self._active_behavior = None
            effective_action = action

        # Execute effective action in Isaac Sim
        state, reward, terminated, info = self.isaac_env.step(effective_action)

        obs = self._state_to_obs(state)
        truncated = self._step_count >= self._max_episode_steps

        # === REWARD SHAPING FOR ONTOLOGY BEHAVIORS ===
        # When ontology preempts, apply appropriate reward signals
        if self._ontology_preempting:
            reward = self._apply_ontology_reward_shaping(reward, state, behavior_command)

            # Check if ontology behavior completed (e.g., landed safely)
            if self._check_ontology_behavior_complete(state, behavior_command):
                # Behavior completed successfully
                info['ontology_behavior_complete'] = True

        # Check for termination due to ontology behaviors
        if behavior_command is not None:
            if behavior_command.behavior == OntologyBehavior.EMERGENCY_LAND:
                # Emergency landing terminates episode
                if state.position[2] < 0.5:  # On ground
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
            # Perception stats for monitoring
            "perception_stats": self.isaac_env.get_perception_stats(),
        })

        return obs, reward, terminated, truncated, info

    def _apply_ontology_reward_shaping(
        self,
        base_reward: float,
        state: UAVState,
        command: Optional[BehaviorCommand]
    ) -> float:
        """
        Apply reward shaping when ontology behaviors are active.

        The goal is to:
        1. Not penalize agent for ontology-mandated behaviors
        2. Give credit for safe behavior execution
        3. Penalize behaviors that led to ontology intervention

        Args:
            base_reward: Reward from environment
            state: Current UAV state
            command: Active ontology behavior command

        Returns:
            Shaped reward
        """
        if command is None:
            return base_reward

        shaped_reward = base_reward

        if command.behavior == OntologyBehavior.TAKEOFF:
            # Neutral reward during takeoff (prerequisite, not a penalty)
            # Small reward for altitude gain
            target_alt = self.ontology_controller.config.takeoff_altitude
            progress = min(1.0, state.position[2] / target_alt)
            shaped_reward = progress * 0.1  # Small positive for progress
        elif command.behavior == OntologyBehavior.RTL:
            # Neutral reward during RTL (don't penalize safety behavior)
            # Small reward for making progress toward home
            distance_to_home = np.linalg.norm(
                state.position - self.ontology_controller._home_position
            )
            # Normalize by max mission distance (~300m typical)
            progress_reward = (1.0 - distance_to_home / 300.0) * 0.1
            shaped_reward = progress_reward

        elif command.behavior == OntologyBehavior.EMERGENCY_LAND:
            # Reward for safe descent
            if state.position[2] < 0.5:
                shaped_reward = 1.0  # Successfully landed
            else:
                shaped_reward = 0.0  # Neutral during descent

        elif command.behavior == OntologyBehavior.HOVER:
            # Small negative for being stuck in hover (encourage mission progress)
            shaped_reward = -0.01

        elif command.behavior == OntologyBehavior.GEOFENCE_RECOVERY:
            # Reward for returning to geofence
            if state.in_geofence:
                shaped_reward = 0.5  # Successfully recovered
            else:
                shaped_reward = -0.1  # Still outside

        elif command.behavior == OntologyBehavior.NFZ_AVOIDANCE:
            # Reward for leaving NFZ
            if not state.in_nfz:
                shaped_reward = 0.5  # Successfully exited
            else:
                shaped_reward = -0.5  # Still inside (bad)

        elif command.behavior == OntologyBehavior.PERSON_AVOIDANCE:
            # Neutral reward during person avoidance (safety behavior)
            # Small negative to encourage maintaining safe distance proactively
            shaped_reward = -0.05

        return shaped_reward

    def _check_ontology_behavior_complete(
        self,
        state: UAVState,
        command: Optional[BehaviorCommand]
    ) -> bool:
        """Check if the active ontology behavior has completed."""
        if command is None:
            return False

        if command.behavior == OntologyBehavior.TAKEOFF:
            # Takeoff complete when at mission altitude
            target_alt = self.ontology_controller.config.takeoff_altitude
            tolerance = self.ontology_controller.config.takeoff_altitude_tolerance
            return state.position[2] >= target_alt - tolerance

        elif command.behavior == OntologyBehavior.RTL:
            # RTL complete when close to home and low altitude
            distance_to_home = np.linalg.norm(
                state.position - self.ontology_controller._home_position
            )
            return distance_to_home < 5.0 and state.position[2] < 2.0

        elif command.behavior == OntologyBehavior.EMERGENCY_LAND:
            return state.position[2] < 0.5

        elif command.behavior == OntologyBehavior.GEOFENCE_RECOVERY:
            return state.in_geofence

        elif command.behavior == OntologyBehavior.NFZ_AVOIDANCE:
            return not state.in_nfz and state.nfz_distance > 20.0

        elif command.behavior == OntologyBehavior.PERSON_AVOIDANCE:
            # Person avoidance completes when ontology controller clears it
            # (no person detections within warning distance)
            return not self.ontology_controller._person_avoidance_active

        return False

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
