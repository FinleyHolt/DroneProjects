"""
VecEnv wrappers for ISR training environments.

Provides Stable-Baselines3 compatible wrappers for the GPU-batched
ISRTrainingEnv without double-batching.
"""

from .vec_env_wrapper import ISRVecEnvWrapper

__all__ = ["ISRVecEnvWrapper"]
