"""
vampire_bridge - ROS 2 bridge to the Vampire theorem prover

This package provides a ROS 2 interface to the Vampire theorem prover
for UAV ontological reasoning in the Flyby F-11 autonomy system.

Components:
    - VampireNode: Main ROS 2 node handling query service
    - QueryBuilder: Constructs TPTP queries from ROS messages
    - ResultParser: Parses Vampire SZS output to ROS messages
    - QueryCache: LRU cache with TTL for query memoization
    - HotFactBuffer: Thread-safe buffer for volatile state facts (Phase 5)
    - ColdFactCache: Cache for static/rare-update facts (Phase 5)
    - StateGroundingNode: ROS 2 node for sensor-to-TPTP grounding (Phase 5)
"""

__version__ = "0.2.0"
