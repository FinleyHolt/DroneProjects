# Ontology Reasoning Guide

This file provides guidance for working with the ontology-based reasoning system.

## Purpose

Dual-mode ontology system for mission planning (heavyweight formal reasoning) and runtime safety checking (lightweight Prolog queries). Provides formal safety guarantees for autonomous flight.

**Current Status**: Defined but NOT rigorously tested. Needs significant validation work.

## Architecture

### Planning Mode (Offline)
- **Tool**: SUMO + Vampire ATP
- **Purpose**: Mission feasibility, safety verification, constraint analysis
- **Performance**: Seconds to minutes
- **When**: Pre-flight planning

### Execution Mode (Online)
- **Tool**: SWI-Prolog
- **Purpose**: Real-time constraint checking (<10ms)
- **Performance**: Microseconds
- **When**: During flight, always running

## Directory Structure

```
ontology/
├── planning_mode/          # SUMO + Vampire ATP
│   ├── uav_domain.kif      # UAV ontology (KIF format)
│   └── test_scenarios/     # Mission test cases
├── execution_mode/         # SWI-Prolog runtime
│   ├── uav_rules.pl        # Compiled mission rules
│   └── templates/          # Prolog templates
├── translation/            # SUMO to Prolog translator
│   └── sumo_to_prolog.py   # Translation script
└── evaluation/             # Benchmarking
```

## Key Concepts

### Safety Axioms
- `batteryReserveReturn`: RTL when battery < reserve
- `criticalBattery`: Emergency land when battery < 15%
- `geofenceViolation`: Recovery when outside bounds
- `noFlyZoneViolation`: Hover when inside NFZ
- `minPersonDistance`: Hover when person < 10m

### Domain Knowledge
- UAV class hierarchy (Multirotor → Quadcopter → FlybyF11)
- Sensor types (GNSS, vision, depth)
- Mission types (waypoint, survey, ISR)
- Regulatory constraints (FAA Part 107)

## Integration Points

### Isaac Sim
- `environments/safety_filter.py`: Uses Vampire for action validation
- `environments/ontology_behavior_controller.py`: Preempts unsafe RL actions
- Graceful degradation if Vampire unavailable (geometric checks only)

### ROS 2
- `vampire_bridge` package: ROS 2 service interface
- `OntologyQuery.srv`: Query types for safety checks
- `mission_orchestrator`: Uses ontology for planning validation

## Graceful Degradation

If Vampire ATP is unavailable, the safety filter falls back to:
1. Geometric boundary checks
2. Battery threshold checks
3. No formal verification of complex constraints

This allows development without the heavyweight reasoner but provides weaker safety guarantees.

## Development Status

| Component | Status | Notes |
|-----------|--------|-------|
| UAV domain ontology | Defined | KIF format, needs validation |
| Prolog translation | Partial | Basic translation works |
| Isaac Sim integration | Working | Via safety_filter.py |
| ROS 2 bridge | Scaffolded | vampire_bridge package |
| Test suite | Missing | Priority work item |

## Priority Work

1. **Create rigorous test suite** for safety axioms
2. **Validate** Prolog translation correctness
3. **Test** ROS 2 vampire_bridge integration
4. **Document** edge cases and failure modes

## Quick Start

```bash
# Planning mode (inside container)
cd /workspace/ontology/planning_mode
# Load and query SUMO ontology...

# Execution mode
swipl
?- consult('/workspace/ontology/execution_mode/uav_rules.pl').
?- mission_valid(test_mission).
```

## Related Documentation

- `README.md`: Architecture overview
- `translation/TRANSLATION.md`: KIF to Prolog translation
- `../isaac-sim/environments/safety_filter.py`: Integration code
