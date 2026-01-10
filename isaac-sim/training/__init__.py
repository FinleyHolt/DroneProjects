"""
Fast Training Package for Flyby F-11 ISR RL

This package provides optimized environments for faster-than-realtime RL training.
Key optimizations:
- Render frame skipping (Phase 1)
- Direct dynamics without PX4 SITL (Phase 2)
- Multi-drone vectorized environments (Phase 3)

Supported Scenarios:
- Area Coverage (ScenarioType.AREA_COVERAGE)
- Dynamic NFZ Avoidance (ScenarioType.NFZ_AVOIDANCE)
- Multi-Objective ISR with Threats (ScenarioType.MULTI_OBJECTIVE)

Use evaluation/ package for full-fidelity PX4 SITL validation.
"""

from .environments import (
    ISRTrainingEnv,
    ISRTrainingConfig,
    ScenarioType,
    WorldConfig,
    MissionConfig,
    NFZConfig,
    ThreatConfig,
    CameraConfig,
    TargetConfig,
    DroneConfig,
)

__version__ = "0.2.0"

__all__ = [
    "ISRTrainingEnv",
    "ISRTrainingConfig",
    "ScenarioType",
    "WorldConfig",
    "MissionConfig",
    "NFZConfig",
    "ThreatConfig",
    "CameraConfig",
    "TargetConfig",
    "DroneConfig",
]
