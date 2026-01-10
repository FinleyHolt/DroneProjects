"""
ontology_rl - Gymnasium-based RL training for F-11 ontology-constrained autonomy

This package provides Gymnasium environments that integrate with the Flyby F-11
UAV ontology system for training RL agents that respect logical constraints.

Components:
    - envs: Gymnasium environments for mission planning, behavior selection, trajectory
    - wrappers: ROS 2 bridge and ontology constraint filtering
    - training: SB3 training configuration and callbacks
    - nodes: ROS 2 nodes for environment exposure

Environments:
    - FlybyMissionPlanner-v0: Level 1 mission planning (10s horizon)
    - FlybyBehaviorSelector-v0: Level 2 behavior selection (1s horizon)
    - FlybyTrajectoryOptimizer-v0: Level 3 trajectory optimization (0.1s horizon)
"""

__version__ = "0.1.0"

# Import environments for registration
from ontology_rl import envs
