"""
Flyby F-11 Isaac Sim Training Environments

This package provides photorealistic training environments for the three canonical
ISR problems defined in ONTOLOGY_FOUNDATION.qmd:

1. Comms-Denied Area Surveillance
2. Dynamic No-Fly Zone Avoidance
3. Multi-Objective ISR with Threat Avoidance

Each environment integrates:
- Domain randomization for sim-to-real transfer
- Ontology state queries (NFZs, threats, comms status as symbolic facts)
- Vampire theorem prover verification for safety-constrained learning
- Hierarchical RL training with defined reward structures
- Live perception (YOLO detection) for realistic sim-to-real transfer (Phase 6f)
"""

from .base_isr_env import (
    BaseISREnvironment,
    DomainRandomizationConfig,
    UAVState,
    EnvironmentConfig,
    FlightPhase,
    CommsStatus,
    GNSSStatus,
    ThreatLevel,
    PerceptionConfig,
)
from .comms_denied_env import CommsDeniedSurveillanceEnv
from .dynamic_nfz_env import DynamicNFZAvoidanceEnv
from .multi_objective_env import MultiObjectiveISREnv
from .domain_randomizer import DomainRandomizer
from .ontology_bridge import OntologyStateBridge
from .gymnasium_wrapper import IsaacSimGymWrapper, make_isaac_gym_env
from .action_bridge import PX4ActionBridge
from .safety_filter import VampireSafetyFilter
from .ontology_behavior_controller import (
    OntologyBehaviorController,
    OntologyBehaviorExecutor,
    OntologyControllerConfig,
    OntologyBehavior,
    AxiomPriority,
    AxiomViolation,
    BehaviorCommand,
)
from .perception_integration import (
    PerceptionIntegration,
    PerceptionIntegrationConfig,
)
from .perception_manager import (
    PerceptionManager,
    PerceptionConfig,
    Detection,
    GroundTruthConfig,
    MILITARY_CLASS_MAP,
    CLASS_ID_TO_NAME,
)
from .object_tracker import (
    ObjectTracker,
    TrackerConfig,
    Track,
)
from .byte_tracker import (
    ByteTracker,
    ByteTrackerConfig,
    STrack,
    TrackState,
    KalmanFilter,
)
from .debug_overlay import (
    DebugOverlay,
    OverlayConfig,
)

__all__ = [
    # Base environment and configs
    'BaseISREnvironment',
    'DomainRandomizationConfig',
    'UAVState',
    'EnvironmentConfig',
    'FlightPhase',
    'CommsStatus',
    'GNSSStatus',
    'ThreatLevel',
    # Canonical problem environments
    'CommsDeniedSurveillanceEnv',
    'DynamicNFZAvoidanceEnv',
    'MultiObjectiveISREnv',
    'DomainRandomizer',
    'OntologyStateBridge',
    # RL Training Bridge (Phase 6e)
    'IsaacSimGymWrapper',
    'make_isaac_gym_env',
    'PX4ActionBridge',
    'VampireSafetyFilter',
    # Ontology Behavior Controller
    'OntologyBehaviorController',
    'OntologyBehaviorExecutor',
    'OntologyControllerConfig',
    'OntologyBehavior',
    'AxiomPriority',
    'AxiomViolation',
    'BehaviorCommand',
    # Perception Integration (Phase 6f)
    'PerceptionIntegration',
    'PerceptionIntegrationConfig',
    'PerceptionConfig',
    # Ground Truth & Military Detection (Phase 6g)
    'PerceptionManager',
    'Detection',
    'GroundTruthConfig',
    'MILITARY_CLASS_MAP',
    'CLASS_ID_TO_NAME',
    # Object Tracking (Simple)
    'ObjectTracker',
    'TrackerConfig',
    'Track',
    # ByteTrack (Advanced Temporal Coherence)
    'ByteTracker',
    'ByteTrackerConfig',
    'STrack',
    'TrackState',
    'KalmanFilter',
    # Debug Overlay
    'DebugOverlay',
    'OverlayConfig',
]
