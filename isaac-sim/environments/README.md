# Flyby F-11 Isaac Sim Training Environments

Photorealistic training environments for ontology-constrained hierarchical RL on the three canonical ISR problems.

## Overview

This package provides Isaac Sim + Pegasus Simulator + PX4 environments for training autonomous ISR agents on the Flyby F-11 platform. Each environment implements a canonical problem from `ONTOLOGY_FOUNDATION.qmd` with:

- **Domain randomization** for sim-to-real transfer
- **Ontology state extraction** for symbolic reasoning
- **Vampire ATP integration** for safety shielding
- **Hierarchical RL support** for multi-level decision making

## Canonical Problems

### 1. Comms-Denied Area Surveillance (`CommsDeniedSurveillanceEnv`)

**Mission**: Conduct persistent surveillance of a 500m × 500m area for 15 minutes without ground station communication, then autonomously return to launch.

**Key Features**:
- Communications denial triggers at T+30s
- Enhanced battery reserve (30% vs 25%) in autonomous mode
- GPS-degraded navigation with VIO fallback
- Area coverage maximization with POI capture
- ISR Payload: Gremsy VIO (640p thermal + 4K RGB + 20x zoom)

**Success Criteria**:
| Criterion | Threshold |
|-----------|-----------|
| Area Coverage | ≥ 85% |
| POI Captures | ≥ 10 |
| Return Accuracy | ≤ 3m |
| Battery at Landing | ≥ 15% |
| Safety Violations | = 0 |

**Usage**:
```python
from environments import CommsDeniedSurveillanceEnv, CommsDeniedConfig

config = CommsDeniedConfig(
    headless=True,
    surveillance_area_size=500.0,
    target_coverage=85.0,
    comms_denial_time=30.0,
)
env = CommsDeniedSurveillanceEnv(config)
env.setup()

state = env.reset()
for _ in range(10000):
    action = agent.select_action(env.state_to_observation(state))
    state, reward, done, info = env.step(action)
    if done:
        break
```

### 2. Dynamic No-Fly Zone Avoidance (`DynamicNFZAvoidanceEnv`)

**Mission**: Complete a 3km point-to-point reconnaissance transit while dynamically avoiding a No-Fly Zone that appears mid-mission (aircraft corridor activation at T+3min).

**Key Features**:
- Dynamic NFZ activation during flight
- Real-time path replanning (target: ≤2s)
- Timeline adherence under constraints
- NFZ buffer zone proximity warnings
- ISR Payload: NextVision Raptor (1280×720p thermal)

**Success Criteria**:
| Criterion | Threshold |
|-----------|-----------|
| Destination Accuracy | ≤ 5m |
| NFZ Penetrations | = 0 |
| Replan Latency | ≤ 2 seconds |
| Timeline Adherence | ≤ 120% |
| Battery at Destination | ≥ 20% |

**Usage**:
```python
from environments import DynamicNFZAvoidanceEnv, DynamicNFZConfig
import numpy as np

config = DynamicNFZConfig(
    headless=True,
    destination=np.array([3000.0, 500.0, 80.0]),
    nfz_activation_time=180.0,
)
env = DynamicNFZAvoidanceEnv(config)
```

### 3. Multi-Objective ISR with Threat Avoidance (`MultiObjectiveISREnv`)

**Mission**: Collect ISR on 8 targets with varying priorities while avoiding threat zones. High-threat zones are prohibited; medium-threat zones have cumulative exposure limits.

**Key Features**:
- 8 targets with priorities (Critical, High, Medium, Low)
- 3 threat zones (2 High, 1 Medium)
- Battery budget optimization
- Risk-reward tradeoffs
- ISR Payload: Gremsy VIO (thermal + 20x zoom)

**Target Distribution**:
| Target | Priority | Value | Threat Level | Battery Cost |
|--------|----------|-------|--------------|--------------|
| TOI-1 | Critical | 100 | Medium | 8% |
| TOI-2 | Critical | 100 | Low | 6% |
| TOI-3 | High | 75 | **HIGH** | 15% |
| TOI-4 | High | 75 | Low | 5% |
| TOI-5 | Medium | 50 | Medium | 9% |
| TOI-6 | Medium | 50 | **HIGH** | 18% |
| TOI-7 | Low | 25 | Low | 4% |
| TOI-8 | Low | 25 | Medium | 10% |

**Theoretical Max**: 500 points | **Achievable (avoid HIGH)**: 400 points

**Success Criteria**:
| Criterion | Threshold |
|-----------|-----------|
| Mission Value Score | ≥ 80% of achievable |
| High Threat Entry | = 0 seconds |
| Medium Threat Exposure | ≤ 30 seconds |
| Critical Targets | 100% if reachable |
| Battery at Landing | ≥ 15% |

## Domain Randomization

The `DomainRandomizer` class provides comprehensive variation for sim-to-real transfer:

### Lighting
- Presets: DAY, DAWN, DUSK, OVERCAST, NIGHT
- Automatic intensity and direction variation
- Shadow control

### Weather
- Conditions: CLEAR, HAZY, FOGGY, DUSTY
- Wind velocity and turbulence
- Visibility range

### Sensors
- GPS denial/degradation (HDOP variation)
- Camera noise and exposure
- IMU noise scaling
- Compass interference
- VIO availability

### Obstacles
- Position noise
- Rotation variation
- Scale variation

**Usage**:
```python
from environments import DomainRandomizer, RandomizationConfig

config = RandomizationConfig(
    seed=42,
    randomize_lighting=True,
    randomize_weather=True,
    randomize_sensors=True,
)
randomizer = DomainRandomizer(config)

# Apply randomization
params = randomizer.randomize(seed=123)
print(params)  # {'lighting': {...}, 'weather': {...}, 'sensors': {...}}
```

## Ontology Integration

The `OntologyStateBridge` connects simulation state to the SUMO-based ontology:

### State-to-Facts Conversion
```python
from environments import OntologyStateBridge

bridge = OntologyStateBridge(
    ontology_path="/workspace/ontology/planning_mode",
    vampire_path="/usr/local/bin/vampire",
)

# Convert state to ontology facts
facts = bridge.state_to_facts(state, nfz_zones, threat_zones, targets)

# Get TPTP format for Vampire
tptp = bridge.facts_to_tptp(facts)
```

### Safety Shielding
```python
# Verify action safety before execution
is_safe, safe_alternative = bridge.verify_action_safety(
    action, state, nfz_zones, threat_zones
)

if not is_safe:
    action = safe_alternative  # Use safe alternative
```

### Symbolic State for Hierarchical RL
```python
# Get symbolic state for high-level planning
symbolic = bridge.get_symbolic_state(state, nfz_zones, threat_zones, targets)
# Returns: flight_phase, battery_adequate, localization_valid, in_threat_zone, etc.
```

## Reward Structures

All reward structures follow ONTOLOGY_FOUNDATION.qmd specifications:

### Comms-Denied Surveillance
```python
# Hard constraints
if geofence_violation: reward -= 100
if nfz_violation: reward -= 500

# Battery reserve (30% for autonomous mode)
if battery < 30 and not returning:
    reward -= 10 * (30 - battery)

# Mission objectives
reward += 2 * coverage_delta  # +2 per %
reward += 10 * poi_delta      # +10 per POI

# Success bonus
if coverage >= 85 and poi >= 10:
    reward += 100
```

### Dynamic NFZ Avoidance
```python
# NFZ violation is terminal
if in_nfz: return -1000

# Buffer zone penalty
if nfz_distance < buffer:
    penetration = 1 - (nfz_distance / buffer)
    reward -= 50 * penetration

# Replan performance
if replan_duration <= 500ms: reward += 30
elif replan_duration <= 2000ms: reward += 10
else: reward -= 20

# Progress
reward += 0.5 * progress_to_destination
```

### Multi-Objective ISR
```python
# High threat is catastrophic
if in_high_threat: return -500

# Medium threat penalty
reward -= risk_rate * dt

# Target observation
for target in newly_observed:
    multiplier = {1: 2.0, 2: 1.5, 3: 1.0, 4: 0.75}[priority]
    reward += value * multiplier
    if priority == 1: reward += 50  # Critical bonus

# Efficiency
if battery_used > 0.1:
    reward += 0.5 * min(value_gained / battery_used, 20)

# Success threshold (80%)
if collected_value >= 0.8 * achievable:
    reward += 100
```

## Training

### Quick Start
```bash
# Build container with environments
cd flyby-f11/evaluation/isaac-sim-px4
podman build -t flyby-f11-training .

# Run training
podman run --device nvidia.com/gpu=all \
    -v $(pwd):/workspace:z \
    flyby-f11-training \
    python scripts/training/train_canonical.py \
    --problem comms_denied \
    --headless
```

### Configuration
Edit `config/training_config.yaml` for:
- Environment parameters
- Domain randomization settings
- RL algorithm hyperparameters
- Curriculum learning stages
- Safety shield configuration

### Curriculum Learning
Training progresses through stages:
1. **Basic** (difficulty=0.0): No randomization, learn fundamental control
2. **Intermediate** (difficulty=0.5): Light randomization, develop robustness
3. **Advanced** (difficulty=1.0): Full randomization, master all conditions
4. **Expert** (difficulty=1.0): Sensor degradation, GPS denial scenarios

## File Structure

```
environments/
├── __init__.py              # Package exports
├── base_isr_env.py          # Base environment class
├── comms_denied_env.py      # Canonical Problem 1
├── dynamic_nfz_env.py       # Canonical Problem 2
├── multi_objective_env.py   # Canonical Problem 3
├── domain_randomizer.py     # Domain randomization system
├── ontology_bridge.py       # Ontology state bridge
└── README.md                # This file

config/
└── training_config.yaml     # Training configuration

scripts/training/
└── train_canonical.py       # Training script
```

## Integration with Flyby F-11 Autonomy Stack

These environments are designed to train agents that deploy on the Flyby F-11:

1. **Ontology Compatibility**: State extraction matches `uav_domain.kif` and `isr_extensions.kif`
2. **ROS 2 Bridge**: Observation/action spaces compatible with `flyby_f11_bringup`
3. **Safety Verification**: Vampire ATP queries match canonical test scenarios
4. **Sensor Payloads**: Matches Gremsy VIO, RESEPI LiDAR, NextVision Raptor specs

## References

- [ONTOLOGY_FOUNDATION.qmd](../../ONTOLOGY_FOUNDATION.qmd) - Canonical problem definitions
- [APPROACH.qmd](../../APPROACH.qmd) - Full system architecture
- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/)
- [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/)
