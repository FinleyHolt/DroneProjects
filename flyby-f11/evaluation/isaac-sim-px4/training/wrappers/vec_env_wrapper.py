"""
Stable-Baselines3 VecEnv wrapper for Isaac Sim training environments.

The training environments already batch internally (256-1024 parallel envs in a single
env object using GPU tensor operations). This wrapper exposes that batching to
SB3 without creating additional subprocess workers.

Supports:
- ISRTrainingEnv (full ISR training)
- SearchTrainingEnv (search policy training)
- DwellTrainingEnv (dwell policy training)

Key Design Decisions:
- Does NOT use SubprocVecEnv - internal GPU batching handles parallelization
- GPU->CPU transfer happens once per step (unavoidable for SB3)
- Auto-reset is handled internally by the environments
- Info dicts are converted from batched tensors to list of dicts for SB3
"""

from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
import torch
from gymnasium import spaces
from stable_baselines3.common.vec_env.base_vec_env import VecEnv, VecEnvObs, VecEnvStepReturn

# Type hint for training environments (avoid circular import)
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from training.environments import ISRTrainingEnv, SearchTrainingEnv, DwellTrainingEnv


class IsaacVecEnvWrapper(VecEnv):
    """
    Generic VecEnv wrapper for Isaac Sim-based training environments.

    Works with ISRTrainingEnv, SearchTrainingEnv, and DwellTrainingEnv.
    Automatically detects observation and action dimensions from the environment.
    """

    def __init__(self, env):
        """
        Initialize the VecEnv wrapper.

        Args:
            env: Training environment instance (ISRTrainingEnv, SearchTrainingEnv, or DwellTrainingEnv)
        """
        self.env = env
        self._num_envs = env.num_envs
        self._device = env.device

        # Detect observation dimension from env
        if hasattr(env, 'OBS_DIM'):
            obs_dim = env.OBS_DIM
        elif hasattr(env, 'observation_dim'):
            obs_dim = env.observation_dim
        elif hasattr(env, 'observation_space_shape'):
            obs_dim = env.observation_space_shape[0]
        else:
            raise ValueError("Cannot determine observation dimension from environment")

        # Detect action dimension from env
        if hasattr(env, 'ACTION_DIM'):
            action_dim = env.ACTION_DIM
        elif hasattr(env, 'action_dim'):
            action_dim = env.action_dim
        elif hasattr(env, 'action_space_shape'):
            action_dim = env.action_space_shape[0]
        else:
            raise ValueError("Cannot determine action dimension from environment")

        # Build spaces
        observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32,
        )

        action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(action_dim,),
            dtype=np.float32,
        )

        self._actions: Optional[torch.Tensor] = None

        super().__init__(
            num_envs=self._num_envs,
            observation_space=observation_space,
            action_space=action_space,
        )

    def step_async(self, actions: np.ndarray) -> None:
        self._actions = torch.from_numpy(actions).float().to(self._device)

    def step_wait(self) -> VecEnvStepReturn:
        if self._actions is None:
            raise RuntimeError("step_async must be called before step_wait")

        obs, rewards, dones, info = self.env.step(self._actions)

        obs_np = obs.cpu().numpy()
        rewards_np = rewards.cpu().numpy()
        dones_np = dones.cpu().numpy()

        infos = self._convert_info_to_list(info, dones_np)
        self._actions = None

        return obs_np, rewards_np, dones_np, infos

    def reset(self) -> VecEnvObs:
        obs = self.env.reset()
        return obs.cpu().numpy()

    def close(self) -> None:
        self.env.close()

    def seed(self, seed: Optional[int] = None) -> List[Optional[int]]:
        if seed is not None and hasattr(self.env, 'cfg'):
            self.env.cfg.base_seed = seed
        return [seed] * self._num_envs if seed else [None] * self._num_envs

    def get_attr(self, attr_name: str, indices: Optional[Sequence[int]] = None) -> List[Any]:
        value = getattr(self.env, attr_name)
        n = len(indices) if indices is not None else self._num_envs
        return [value] * n

    def set_attr(self, attr_name: str, value: Any, indices: Optional[Sequence[int]] = None) -> None:
        setattr(self.env, attr_name, value)

    def env_method(self, method_name: str, *method_args,
                   indices: Optional[Sequence[int]] = None, **method_kwargs) -> List[Any]:
        method = getattr(self.env, method_name)
        result = method(*method_args, **method_kwargs)
        n = len(indices) if indices is not None else self._num_envs
        return [result] * n

    def env_is_wrapped(self, wrapper_class: type, indices: Optional[Sequence[int]] = None) -> List[bool]:
        n = len(indices) if indices is not None else self._num_envs
        return [False] * n

    def _convert_info_to_list(self, info: Dict[str, Any], dones: np.ndarray) -> List[Dict[str, Any]]:
        """Convert batched info dict to list of dicts for SB3."""
        infos: List[Dict[str, Any]] = []

        for i in range(self._num_envs):
            env_info: Dict[str, Any] = {}

            for key, value in info.items():
                if isinstance(value, torch.Tensor):
                    if value.dim() == 0:
                        env_info[key] = value.item()
                    elif value.shape[0] == self._num_envs:
                        env_info[key] = value[i].cpu().numpy() if value[i].dim() > 0 else value[i].item()
                elif isinstance(value, np.ndarray) and value.shape[0] == self._num_envs:
                    env_info[key] = value[i]
                elif isinstance(value, dict):
                    # Nested dict (like term_info)
                    for sub_key, sub_value in value.items():
                        if isinstance(sub_value, torch.Tensor) and sub_value.shape[0] == self._num_envs:
                            if sub_value[i]:
                                env_info["termination_reason"] = sub_key

            if dones[i]:
                env_info["TimeLimit.truncated"] = False
                env_info["terminal_observation"] = True

            infos.append(env_info)

        return infos

    def get_images(self) -> np.ndarray:
        return np.array([])

    def render(self, mode: str = "human") -> Optional[np.ndarray]:
        return None


# Legacy alias for backwards compatibility
class ISRVecEnvWrapper(VecEnv):
    """
    Wraps ISRTrainingEnv (GPU-batched internally) as a VecEnv for Stable-Baselines3.

    This wrapper:
    1. Exposes num_envs from ISRTrainingEnv's internal batching
    2. Converts between numpy (SB3) and torch tensors (ISRTrainingEnv)
    3. Translates batched info dict to list of dicts
    4. Handles the gymnasium spaces interface

    Example:
        >>> from training.environments import ISRTrainingEnv, ISRTrainingConfig
        >>> from training.wrappers import ISRVecEnvWrapper
        >>>
        >>> config = ISRTrainingConfig(num_envs=256, device="cuda:0")
        >>> isr_env = ISRTrainingEnv(config)
        >>> isr_env.setup()
        >>>
        >>> vec_env = ISRVecEnvWrapper(isr_env)
        >>> model = PPO("MlpPolicy", vec_env)
        >>> model.learn(total_timesteps=1_000_000)
    """

    def __init__(self, env: "ISRTrainingEnv"):
        """
        Initialize the VecEnv wrapper.

        Args:
            env: ISRTrainingEnv instance (must be already setup)
        """
        self.env = env
        self._num_envs = env.num_envs
        self._device = env.device

        # Build observation space from env properties
        obs_dim = env.observation_dim
        observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32,
        )

        # Build action space from env properties
        action_dim = env.action_dim
        action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(action_dim,),
            dtype=np.float32,
        )

        # Storage for async step
        self._actions: Optional[torch.Tensor] = None

        # Initialize VecEnv base class
        # Note: VecEnv sets observation_space and action_space as attributes
        super().__init__(
            num_envs=self._num_envs,
            observation_space=observation_space,
            action_space=action_space,
        )

    def step_async(self, actions: np.ndarray) -> None:
        """
        Begin stepping the environments with the given actions.

        Args:
            actions: Array of shape (num_envs, action_dim) with normalized actions
        """
        # Convert numpy array to torch tensor on GPU
        self._actions = torch.from_numpy(actions).float().to(self._device)

    def step_wait(self) -> VecEnvStepReturn:
        """
        Wait for the step to complete and return results.

        Returns:
            Tuple of (observations, rewards, dones, infos)
        """
        if self._actions is None:
            raise RuntimeError("step_async must be called before step_wait")

        # Step the environment (all envs at once)
        obs, rewards, dones, info = self.env.step(self._actions)

        # Convert torch tensors to numpy arrays for SB3
        obs_np = obs.cpu().numpy()
        rewards_np = rewards.cpu().numpy()
        dones_np = dones.cpu().numpy()

        # Convert batched info dict to list of dicts for SB3
        infos = self._convert_info_to_list(info, dones_np)

        # Clear stored actions
        self._actions = None

        return obs_np, rewards_np, dones_np, infos

    def reset(self) -> VecEnvObs:
        """
        Reset all environments and return initial observations.

        Returns:
            Initial observations array of shape (num_envs, obs_dim)
        """
        obs = self.env.reset()
        return obs.cpu().numpy()

    def close(self) -> None:
        """Clean up environment resources."""
        self.env.close()

    def seed(self, seed: Optional[int] = None) -> List[Optional[int]]:
        """
        Set the random seed for all environments.

        Args:
            seed: Base seed (each env gets seed + env_idx)

        Returns:
            List of seeds used for each environment
        """
        if seed is not None:
            # ISRTrainingEnv handles seeding internally
            self.env.cfg.base_seed = seed
        return [seed] * self._num_envs if seed else [None] * self._num_envs

    def get_attr(self, attr_name: str, indices: Optional[Sequence[int]] = None) -> List[Any]:
        """
        Get attribute from the environment.

        Args:
            attr_name: Name of the attribute
            indices: Indices of environments (ignored, all envs share same attribute)

        Returns:
            List of attribute values (same value repeated for all envs)
        """
        value = getattr(self.env, attr_name)
        n = len(indices) if indices is not None else self._num_envs
        return [value] * n

    def set_attr(
        self, attr_name: str, value: Any, indices: Optional[Sequence[int]] = None
    ) -> None:
        """
        Set attribute on the environment.

        Args:
            attr_name: Name of the attribute
            value: Value to set
            indices: Ignored (all envs share same attribute)
        """
        setattr(self.env, attr_name, value)

    def env_method(
        self,
        method_name: str,
        *method_args,
        indices: Optional[Sequence[int]] = None,
        **method_kwargs,
    ) -> List[Any]:
        """
        Call a method on the environment.

        Args:
            method_name: Name of the method
            method_args: Positional arguments
            indices: Indices of environments (ignored)
            method_kwargs: Keyword arguments

        Returns:
            List with the method result (repeated for each env)
        """
        method = getattr(self.env, method_name)
        result = method(*method_args, **method_kwargs)
        n = len(indices) if indices is not None else self._num_envs
        return [result] * n

    def env_is_wrapped(
        self, wrapper_class: type, indices: Optional[Sequence[int]] = None
    ) -> List[bool]:
        """
        Check if environment is wrapped by a specific wrapper.

        Args:
            wrapper_class: Wrapper class to check
            indices: Indices of environments

        Returns:
            List of booleans (always False since ISRTrainingEnv is not wrapped)
        """
        n = len(indices) if indices is not None else self._num_envs
        return [False] * n

    def _convert_info_to_list(
        self, info: Dict[str, torch.Tensor], dones: np.ndarray
    ) -> List[Dict[str, Any]]:
        """
        Convert batched info dict to list of dicts for SB3.

        SB3 expects a list with one dict per environment. We convert the
        batched tensors from ISRTrainingEnv to this format.

        Args:
            info: Batched info dict with tensors of shape (num_envs,)
            dones: Boolean array indicating which envs are done

        Returns:
            List of info dicts, one per environment
        """
        infos: List[Dict[str, Any]] = []

        for i in range(self._num_envs):
            env_info: Dict[str, Any] = {}

            # Extract scalar values for this environment
            if "coverage_pct" in info:
                env_info["coverage_pct"] = info["coverage_pct"][i].item()

            if "battery_pct" in info:
                env_info["battery_pct"] = info["battery_pct"][i].item()

            if "targets_detected" in info:
                if isinstance(info["targets_detected"], torch.Tensor):
                    env_info["targets_detected"] = info["targets_detected"][i].item()
                else:
                    env_info["targets_detected"] = info["targets_detected"]

            if "mission_time" in info:
                env_info["mission_time"] = info["mission_time"][i].item()

            # Extract termination info if present
            if "term_info" in info:
                term_info = info["term_info"]
                env_info["terminal_observation"] = dones[i]

                # Track termination reason
                for reason in ["crashed", "out_of_bounds", "timeout", "battery_depleted",
                               "mission_success", "diverged", "nfz_violation", "high_threat"]:
                    if reason in term_info:
                        if term_info[reason][i]:
                            env_info["termination_reason"] = reason
                            break

            # For done environments, SB3 expects terminal_observation
            # ISRTrainingEnv auto-resets, so the returned obs is already the new obs
            # We need to flag this for SB3's VecEnvWrapper handling
            if dones[i]:
                env_info["TimeLimit.truncated"] = False  # Not a time limit truncation
                env_info["terminal_observation"] = True

            infos.append(env_info)

        return infos

    def get_images(self) -> np.ndarray:
        """
        Get rendered images from all environments.

        Note: ISRTrainingEnv runs in headless mode for training.
        This returns empty array.

        Returns:
            Empty array (rendering not supported during training)
        """
        return np.array([])

    def render(self, mode: str = "human") -> Optional[np.ndarray]:
        """
        Render the environments.

        Note: ISRTrainingEnv uses headless mode for training.

        Args:
            mode: Render mode (ignored)

        Returns:
            None (rendering not supported during training)
        """
        return None


class ISRVecEnvWrapperWithNormalization(ISRVecEnvWrapper):
    """
    VecEnv wrapper with observation normalization for improved training stability.

    This wrapper adds running statistics for observation normalization,
    which can help with training stability and convergence.

    Note: Use VecNormalize from SB3 instead for most cases. This class
    is provided for cases where you need custom normalization logic.
    """

    def __init__(
        self,
        env: "ISRTrainingEnv",
        normalize_obs: bool = True,
        clip_obs: float = 10.0,
    ):
        """
        Initialize wrapper with normalization.

        Args:
            env: ISRTrainingEnv instance
            normalize_obs: Whether to normalize observations
            clip_obs: Clip normalized observations to this range
        """
        super().__init__(env)
        self.normalize_obs = normalize_obs
        self.clip_obs = clip_obs

        # Running statistics for normalization
        obs_dim = env.observation_dim
        self._obs_mean = np.zeros(obs_dim, dtype=np.float32)
        self._obs_var = np.ones(obs_dim, dtype=np.float32)
        self._obs_count = 0

    def step_wait(self) -> VecEnvStepReturn:
        """Step with observation normalization."""
        obs_np, rewards_np, dones_np, infos = super().step_wait()

        if self.normalize_obs:
            obs_np = self._normalize_obs(obs_np)

        return obs_np, rewards_np, dones_np, infos

    def reset(self) -> VecEnvObs:
        """Reset with observation normalization."""
        obs_np = super().reset()

        if self.normalize_obs:
            obs_np = self._normalize_obs(obs_np)

        return obs_np

    def _normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        """
        Normalize observations using running statistics.

        Args:
            obs: Raw observations of shape (num_envs, obs_dim)

        Returns:
            Normalized observations
        """
        # Update running statistics
        batch_mean = obs.mean(axis=0)
        batch_var = obs.var(axis=0)
        batch_count = obs.shape[0]

        # Welford's online algorithm for mean/variance
        delta = batch_mean - self._obs_mean
        tot_count = self._obs_count + batch_count
        new_mean = self._obs_mean + delta * batch_count / tot_count

        m_a = self._obs_var * self._obs_count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + delta**2 * self._obs_count * batch_count / tot_count
        new_var = m2 / tot_count

        self._obs_mean = new_mean
        self._obs_var = new_var
        self._obs_count = tot_count

        # Normalize
        normalized = (obs - self._obs_mean) / np.sqrt(self._obs_var + 1e-8)

        # Clip
        return np.clip(normalized, -self.clip_obs, self.clip_obs)
