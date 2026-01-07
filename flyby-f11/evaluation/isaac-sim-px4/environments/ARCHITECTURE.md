# Environments Architecture

This document describes the modular architecture of the Isaac Sim ISR training environments.

## Design Principles

1. **Single Responsibility**: Each class handles one concern (~300 lines max)
2. **Composition over Inheritance**: Complex behavior via composed components
3. **Orchestrator Pattern**: High-level classes delegate to specialized managers
4. **Explicit Dependencies**: Components receive dependencies via constructor

## Component Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        IsaacSimGymWrapper                           │
│                    (Gymnasium RL Interface)                         │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐  ┌───────────────────┐  ┌──────────────────┐ │
│  │ObservationSpace  │  │OntologyReward     │  │OntologyBehavior  │ │
│  │Manager           │  │Shaper             │  │Controller        │ │
│  └──────────────────┘  └───────────────────┘  └──────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       BaseISREnvironment                            │
│                    (Core Environment Logic)                         │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐  ┌───────────────────┐  ┌──────────────────┐ │
│  │SimulationCore    │  │PerceptionManager  │  │ZoneManager       │ │
│  │(Isaac+PX4)       │  │(Camera+Detection) │  │(NFZ+Geofence)    │ │
│  └──────────────────┘  └───────────────────┘  └──────────────────┘ │
│  ┌──────────────────┐  ┌───────────────────┐                       │
│  │BatteryManager    │  │DomainRandomization│                       │
│  │(Power+Reserves)  │  │Manager (DR)       │                       │
│  └──────────────────┘  └───────────────────┘                       │
└─────────────────────────────────────────────────────────────────────┘
```

## Layer Architecture

### Layer 1: Gymnasium Interface

**IsaacSimGymWrapper** (`gymnasium_wrapper.py`)
- Converts Isaac Sim environment to Gymnasium API
- Handles observation/action space normalization
- Manages ontology behavior preemption
- ~220 lines

Composed of:
- **ObservationSpaceManager** (`observation_space_manager.py`): Computes observation bounds (~80 lines)
- **OntologyRewardShaper** (`ontology_reward_shaper.py`): Shapes rewards for ontology behaviors (~130 lines)
- **OntologyBehaviorController** (`ontology_behavior_controller.py`): Safety behavior preemption

### Layer 2: Core Environment

**BaseISREnvironment** (`base_isr_env.py`)
- Core environment state machine
- Episode lifecycle management
- Reward computation
- ~1000 lines (orchestrator)

Composed of:
- **SimulationCore** (`simulation_core.py`): Isaac Sim + Pegasus + PX4 initialization (~280 lines)
- **PerceptionManager** (`perception_manager.py`): Camera and detection integration (~200 lines)
- **ZoneManager** (`zone_manager.py`): NFZ, geofence, threat zone tracking (~180 lines)
- **BatteryManager** (`battery_manager.py`): Battery state and reserve calculation (~90 lines)
- **DomainRandomizationManager** (`domain_randomization_manager.py`): DR sampling and effects (~140 lines)

### Layer 3: Specialized Environments

Canonical problem environments extend BaseISREnvironment:

- **CommsDeniedSurveillanceEnv** (`comms_denied_env.py`): Comms-denied area surveillance
- **DynamicNFZAvoidanceEnv** (`dynamic_nfz_env.py`): Dynamic NFZ avoidance
- **MultiObjectiveISREnv** (`multi_objective_env.py`): Multi-objective ISR with threats

### Layer 4: Support Components

- **OntologyStateBridge** (`ontology_bridge.py`): Ontology fact generation
- **VampireSafetyFilter** (`safety_filter.py`): Vampire theorem prover integration
- **PX4ActionBridge** (`action_bridge.py`): Normalized actions to PX4 commands
- **PerceptionIntegration** (`perception_integration.py`): YOLO detection pipeline
- **MissionTasking** (`mission_tasking.py`): Mission waypoint management

## Component Responsibilities

### SimulationCore
```
Responsibilities:
├── Isaac Sim application lifecycle
├── Pegasus interface setup
├── PX4 SITL launch with lockstep
├── Vehicle spawning
└── MAVLink connection management
```

### PerceptionManager
```
Responsibilities:
├── Isaac Sim camera initialization
├── Ground truth detection (fast mode)
├── Full inference detection (realistic mode)
└── Observation vector encoding
```

### ZoneManager
```
Responsibilities:
├── Geofence boundary checking
├── NFZ proximity calculation
├── NFZ violation detection
├── Threat zone exposure tracking
└── Communications zone status
```

### BatteryManager
```
Responsibilities:
├── Battery drain rate (with DR)
├── Required reserve calculation
├── RTL threshold monitoring
└── Emergency threshold monitoring
```

### DomainRandomizationManager
```
Responsibilities:
├── Per-episode DR sampling
├── Battery capacity randomization
├── GNSS accuracy degradation
├── Wind perturbation
└── VIO drift injection
```

### OntologyBehaviorController
```
Responsibilities:
├── Axiom violation detection
├── Behavior priority resolution
├── RL preemption decisions
├── RTL/emergency triggering
└── Mission progress tracking
```

### OntologyRewardShaper
```
Responsibilities:
├── Takeoff progress reward
├── RTL distance reward
├── Emergency landing reward
├── NFZ avoidance penalty
└── Behavior completion detection
```

## Data Flow

```
┌────────────┐    action     ┌────────────────┐
│ RL Policy  │──────────────▶│ GymWrapper     │
└────────────┘               │                │
      ▲                      │ ┌────────────┐ │
      │                      │ │ Ontology   │ │
      │                      │ │ Controller │─┼──▶ May preempt action
      │                      │ └────────────┘ │
      │                      └───────┬────────┘
      │                              │
      │   obs, reward, done          ▼
      │                      ┌────────────────┐
      └──────────────────────│ BaseISREnv     │
                             │                │
                             │ ┌────────────┐ │
                             │ │ Perception │─┼──▶ Detections
                             │ └────────────┘ │
                             │ ┌────────────┐ │
                             │ │ Zone Mgr   │─┼──▶ NFZ/Geofence status
                             │ └────────────┘ │
                             │ ┌────────────┐ │
                             │ │ Battery    │─┼──▶ Power state
                             │ └────────────┘ │
                             └───────┬────────┘
                                     │
                                     ▼
                             ┌────────────────┐
                             │ SimulationCore │
                             │ (Isaac+PX4)    │
                             └────────────────┘
```

## Adding New Features

### New Manager Component

1. Create new file (e.g., `weather_manager.py`)
2. Single responsibility, ~100-300 lines
3. Inject into BaseISREnvironment constructor
4. Update `__init__.py` exports

```python
# weather_manager.py
class WeatherManager:
    """Manages dynamic weather effects."""

    def __init__(self, world):
        self.world = world

    def update(self, timestep: float) -> WeatherState:
        """Update weather state."""
        ...
```

### New Reward Shaper

1. Add behavior case to `OntologyRewardShaper.shape_reward()`
2. Add completion check to `check_behavior_complete()`
3. Keep reward values in [-1, 1] range

### New Ontology Behavior

1. Add to `OntologyBehavior` enum
2. Add detection logic in `OntologyBehaviorController.update()`
3. Add execution logic in `OntologyBehaviorExecutor.execute()`
4. Add reward shaping in `OntologyRewardShaper`

## Size Guidelines

| Component Type | Target Lines | Max Lines |
|----------------|-------------|-----------|
| Manager        | 100-200     | 300       |
| Orchestrator   | 300-500     | 800       |
| Environment    | 200-400     | 600       |
| Helper/Utility | 50-100      | 150       |

If a file exceeds max lines, split into focused components.

## Testing Strategy

Each component should be independently testable:

```python
# Test ZoneManager in isolation
zone_mgr = ZoneManager(config)
zone_mgr.set_nfz_zones([...])
result = zone_mgr.update(position)
assert result.in_geofence == True
```

Integration tests validate component composition:

```python
# Test full environment
env = IsaacSimGymWrapper(BaseISREnvironment(config))
obs, info = env.reset()
obs, reward, done, truncated, info = env.step(action)
```

## File Index

| File | Lines | Responsibility |
|------|-------|----------------|
| `simulation_core.py` | ~280 | Isaac Sim + PX4 initialization |
| `battery_manager.py` | ~90 | Battery state and reserves |
| `zone_manager.py` | ~180 | NFZ, geofence, threat tracking |
| `domain_randomization_manager.py` | ~140 | DR sampling and effects |
| `perception_manager.py` | ~200 | Camera and detection |
| `observation_space_manager.py` | ~80 | Observation bounds computation |
| `ontology_reward_shaper.py` | ~130 | Ontology behavior rewards |
| `ontology_behavior_controller.py` | ~400 | Safety behavior preemption |
| `base_isr_env.py` | ~1000 | Core environment orchestrator |
| `gymnasium_wrapper.py` | ~220 | Gymnasium RL interface |
