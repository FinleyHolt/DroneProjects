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
"""

from .base_isr_env import BaseISREnvironment
from .comms_denied_env import CommsDeniedSurveillanceEnv
from .dynamic_nfz_env import DynamicNFZAvoidanceEnv
from .multi_objective_env import MultiObjectiveISREnv
from .domain_randomizer import DomainRandomizer
from .ontology_bridge import OntologyStateBridge

__all__ = [
    'BaseISREnvironment',
    'CommsDeniedSurveillanceEnv',
    'DynamicNFZAvoidanceEnv',
    'MultiObjectiveISREnv',
    'DomainRandomizer',
    'OntologyStateBridge',
]
