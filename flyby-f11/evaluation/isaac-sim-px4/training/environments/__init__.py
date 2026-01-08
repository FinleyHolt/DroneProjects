"""
Fast Training Environments

Phase 2+ environments without PX4 SITL for faster training.

Key environments:
- ISRTrainingEnv: Main training env with frustum-filtered ground truth perception
- SearchTrainingEnv: Specialized env for SearchPolicy (40-dim obs, 4-dim action)
- DwellTrainingEnv: Specialized env for DwellPolicy (25-dim obs, 6-dim action with 2-axis gimbal)
- DirectDynamicsEnv: Legacy direct dynamics env (deprecated)

Scenario Support:
- ScenarioType.AREA_COVERAGE: Area surveillance for coverage targets
- ScenarioType.NFZ_AVOIDANCE: Dynamic no-fly zone avoidance
- ScenarioType.MULTI_OBJECTIVE: Multi-objective ISR with threat avoidance

Wrappers:
- ISRVecEnvWrapper: Stable-Baselines3 VecEnv wrapper for ISRTrainingEnv
"""

# Phase 2: ISR Training Environment (recommended)
from .isr_training_env import (
    ISRTrainingEnv,
    ISRTrainingConfig,
    CameraConfig,
    TargetConfig,
    DroneConfig,
    # Scenario support (Phase 1+)
    ScenarioType,
    WorldConfig,
    MissionConfig,
    NFZConfig,
    ThreatConfig,
)

# Specialized policy training environments (for BT integration)
from .search_training_env import (
    SearchTrainingEnv,
    SearchTrainingConfig,
    SearchRewardConfig,
)

from .dwell_training_env import (
    DwellTrainingEnv,
    DwellTrainingConfig,
    DwellRewardConfig,
)

# Legacy direct dynamics (for reference)
from .direct_dynamics_env import (
    DirectDynamicsEnv,
    DirectDynamicsConfig,
    ActionMode,
    QuadrotorDynamicsParams,
)

__all__ = [
    # Primary training env
    "ISRTrainingEnv",
    "ISRTrainingConfig",
    "CameraConfig",
    "TargetConfig",
    "DroneConfig",
    # Specialized policy envs
    "SearchTrainingEnv",
    "SearchTrainingConfig",
    "SearchRewardConfig",
    "DwellTrainingEnv",
    "DwellTrainingConfig",
    "DwellRewardConfig",
    # Scenario support
    "ScenarioType",
    "WorldConfig",
    "MissionConfig",
    "NFZConfig",
    "ThreatConfig",
    # Legacy
    "DirectDynamicsEnv",
    "DirectDynamicsConfig",
    "ActionMode",
    "QuadrotorDynamicsParams",
]
