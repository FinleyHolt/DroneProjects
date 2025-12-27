# ADR-003: RL Agent Integration Architecture

**Status:** Proposed
**Date:** 2024-12-26
**Decision Makers:** Finley Holt

## Context

Phases 7-9 implement a hierarchical reinforcement learning system with three agents:
- **Mission Planner (Level 1)**: 10s horizon, waypoint selection (SAC algorithm)
- **Behavior Selector (Level 2)**: 1s horizon, behavior mode selection (PPO algorithm)
- **Trajectory Optimizer (Level 3)**: 100ms horizon, velocity commands (TD3 algorithm)

These agents need to integrate with the ontology reasoning infrastructure implemented in Phases 1-6, specifically:
- Vampire bridge for constraint validation and reward signals
- Mission state machine for state observations
- Safety monitor for safety-constrained action filtering

## Decision

**RL agents will integrate with the ontology system via the Vampire query service for:**
1. **Reward signal generation** - Query Vampire to evaluate action quality against formal constraints
2. **Action filtering** - Validate proposed actions against ontology safety axioms before execution
3. **State observation** - Use HotFactBuffer snapshots for ontology-grounded state representation

## Integration Points

### 1. Vampire Query Interface for RL Rewards

The Vampire bridge provides a stable service interface for RL reward computation:

```python
# Reward query example
query = ReasoningQuery()
query.query_type = 'planning'
query.query_name = 'mission_progress_score'
query.parameters = ['waypoint_3_reached', 'battery_level_85']
query.timeout_ms = 100  # Tactical timeout
```

**Latency Budget:**
- Training: 100ms per query acceptable (vectorized environments compensate)
- Inference: Use cache (TTL 10s for safety, 60s for operational)

### 2. Observation Space Data Sources

| Agent | Observation Source | Data Path |
|-------|-------------------|-----------|
| Mission Planner | Mission state machine | `MissionStateMachine.state`, `context` |
| Behavior Selector | HotFactBuffer | `get_snapshot_dict()` |
| Trajectory Optimizer | MAVROS topics | Direct sensor access |

### 3. Action Filtering via Ontology

Before executing any RL action, validate against safety constraints:

```python
def filter_action(action, hot_facts):
    # Build validation query with current state
    query = QueryBuilder.build_query(
        'safety', 'validate_action',
        [f'proposed_waypoint_{action}'] + hot_facts
    )
    result = vampire_client.call(query)

    if result.szs_status == 'CounterSatisfiable':
        return action  # Safe to execute
    else:
        return fallback_action  # Violation detected
```

## Reward Shaping Strategy

### Mission Planner Rewards
- **Mission completion**: +1000 for mission success (Vampire verifies all objectives met)
- **Safety compliance**: +10 per timestep no violations detected
- **Resource efficiency**: +5 for battery-optimal waypoint selection
- **Penalty**: -500 for abort (Vampire detects critical violation)

### Behavior Selector Rewards
- **Behavior completion**: +100 for successful behavior execution
- **Smooth transitions**: -10 for frequent behavior switches
- **Safety compliance**: +5 per timestep in compliant behavior

### Trajectory Optimizer Rewards
- **Tracking accuracy**: +1 for position error < 0.5m
- **Smoothness**: +0.5 for jerk < threshold
- **Velocity bounds**: -10 for velocity limit violation

## Training Prerequisites Checklist

| Requirement | Status | Notes |
|-------------|--------|-------|
| Vampire query service stable | Ready | 119 tests passing, <100ms latency |
| Mission state machine | Ready | 41 tests passing, thread-safe |
| Safety monitor at 20Hz | Ready | Configurable, fail-open design |
| Hot/Cold fact injection | Ready | QueryBuilder supports state injection |
| Simulation environment | Not Ready | Need Gazebo worlds + ArduPilot SITL |
| Gymnasium wrapper | Not Started | Phase 7 deliverable |
| Training infrastructure | Not Started | Phase 7 deliverable |

## Blocking Issues for Phases 7-9

### Critical (Must Fix)
1. **Simulation environment missing**: No Gazebo worlds configured for training
2. **ArduPilot SITL integration**: Not yet tested with vampire_bridge

### Non-Critical (Can Proceed)
1. ARM64 performance: Deferred to Phase 12 (hardware validation)
2. Prolog bridge: Reference in inputs.json is outdated (using Vampire, not Prolog)

## Consequences

### Positive
1. **Single reasoning backend** - RL agents use same Vampire interface as safety monitor
2. **Formal safety guarantees** - Actions validated against proven axioms
3. **Consistent state representation** - HotFactBuffer provides unified state view
4. **Caching reduces latency** - Training can use longer TTL for repeated queries

### Negative
1. **Training speed** - Vampire queries add overhead (mitigated by caching)
2. **Complexity** - RL rewards depend on reasoning system availability
3. **Debugging difficulty** - Multi-level hierarchy harder to diagnose

## References

- [TRAINING_GUIDE.md](../../docs/rl/TRAINING_GUIDE.md) - RL training procedures
- [README.md](../../docs/rl/README.md) - RL architecture overview
- [ADR-001-vampire-only-reasoner.md](./ADR-001-vampire-only-reasoner.md) - Reasoner selection
- [ADR_STATE_PERSISTENCE.md](./ADR_STATE_PERSISTENCE.md) - Hot/cold fact design
