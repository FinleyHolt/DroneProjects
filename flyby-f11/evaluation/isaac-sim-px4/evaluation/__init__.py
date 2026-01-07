"""
Full-Fidelity Evaluation Package

This package contains the PX4 SITL-integrated environments for:
- Policy evaluation
- Sim-to-real validation
- Video recording with full perception

Use training/ package for fast RL training.
"""

# Re-export from environments for backwards compatibility
from environments import (
    BaseISREnvironment,
    EnvironmentConfig,
    PerceptionConfig,
    DomainRandomizationConfig,
    UAVState,
    FlightPhase,
    CommsStatus,
    GNSSStatus,
    ThreatLevel,
    IsaacSimGymWrapper,
    make_isaac_gym_env,
)

__all__ = [
    'BaseISREnvironment',
    'EnvironmentConfig',
    'PerceptionConfig',
    'DomainRandomizationConfig',
    'UAVState',
    'FlightPhase',
    'CommsStatus',
    'GNSSStatus',
    'ThreatLevel',
    'IsaacSimGymWrapper',
    'make_isaac_gym_env',
]
