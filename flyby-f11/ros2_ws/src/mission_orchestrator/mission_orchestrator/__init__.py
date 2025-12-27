"""
mission_orchestrator - ROS 2 package for UAV mission lifecycle management

Coordinates mission planning and execution using the Vampire reasoning engine
for pre-flight verification and runtime safety monitoring.

Components:
    - MissionStateMachine: State machine for mission lifecycle management
    - MissionManagerNode: Main ROS 2 node with services and action server
    - SafetyMonitor: 20Hz safety monitoring using Vampire tactical queries

Architecture:
    With the single-reasoner architecture (ADR-001), Vampire handles both
    planning queries (5s timeout) for pre-flight verification and tactical
    queries (100ms timeout) for runtime safety monitoring.

See: .phases/phase-06-phase-transition/TASK.md for full specification
"""

from mission_orchestrator.mission_state_machine import MissionState, MissionStateMachine

__all__ = ['MissionState', 'MissionStateMachine']
