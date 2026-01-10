# UAV Domain Ontology Documentation

## Overview

The UAV Domain Ontology (`uav_domain.kif`) provides a comprehensive formal knowledge representation for autonomous drone operations. It extends the SUMO (Suggested Upper Merged Ontology) with drone-specific concepts, enabling formal reasoning about mission safety, constraint validation, and autonomous decision-making.

**Version**: 1.0
**Author**: Finley Holt
**Date**: 2024-12-25
**Format**: SUO-KIF (Knowledge Interchange Format)

## Purpose

This ontology serves as the formal foundation for the Flyby F-11 autonomy architecture:

1. **Mission Planning**: Define and validate mission objectives, waypoints, and constraints
2. **Safety Verification**: Formal axioms that detect safety violations before/during flight
3. **Constraint Enforcement**: Encode regulatory (FAA Part 107), physical, and operational limits
4. **Runtime Reasoning**: Support real-time decision-making through lightweight compiled rules
5. **Explainability**: Provide semantic traceability for all autonomous decisions

## Architecture

The ontology follows a hierarchical structure aligned with SUMO:

```
SUMO Upper Level
    └── Device
        └── TransportationDevice
            └── Vehicle
                └── Aircraft (Mid-level-ontology.kif)
                    └── UAV (this ontology)
                        ├── Multirotor
                        │   ├── Quadcopter
                        │   │   └── FlybyF11
                        │   ├── Hexacopter
                        │   └── Octocopter
                        ├── FixedWingUAV
                        └── HybridVTOL
```

## Key Concepts

### 1. UAV Class Hierarchy (Section 1)

Defines the taxonomy of unmanned aerial vehicles:

| Class | Parent | Description |
|-------|--------|-------------|
| `UAV` | Aircraft | Base class for all unmanned aerial vehicles |
| `Multirotor` | UAV | Rotary-wing UAVs with multiple rotors |
| `FixedWingUAV` | UAV | Fixed-wing aircraft without vertical takeoff |
| `HybridVTOL` | UAV | Combines fixed-wing efficiency with VTOL |
| `Quadcopter` | Multirotor | 4-rotor configuration |
| `FlybyF11` | Quadcopter | Flyby Robotics F-11 platform (NDAA-compliant) |

### 2. UAV Components (Section 2)

Sensor and subsystem classes:

- **Positioning**: `GNSSSensor`, `VisualOdometrySensor`
- **Perception**: `DepthCamera`, `RGBCamera`, `LiDARSensor`
- **Inertial**: `InertialMeasurementUnit`
- **Compute**: `UAVComputer`, `JetsonModule`
- **Power**: `UAVBattery`, `LiPoBattery`
- **Propulsion**: `BrushlessMotor`, `Propeller`

### 3. Physical Properties (Section 3)

Key predicates for UAV specifications:

| Predicate | Domain | Range | Description |
|-----------|--------|-------|-------------|
| `hasMaxTakeoffMass` | UAV | MassMeasure | Maximum takeoff weight |
| `hasPayloadCapacity` | UAV | MassMeasure | Maximum payload |
| `hasMaxAltitude` | UAV | LengthMeasure | Maximum operating altitude |
| `hasMaxSpeed` | UAV | SpeedMeasure | Maximum horizontal speed |
| `hasMaxFlightTime` | UAV | TimeDuration | Maximum endurance |
| `hasSensor` | UAV × Sensor × Position | - | Sensor mounting |
| `canHover` | UAV | Boolean | Hover capability |

### 4. Mission Types (Section 4)

Mission class hierarchy:

```
UAVMission (Process)
├── WaypointMission    - Navigate through waypoint sequence
├── SurveyMission      - Systematic area coverage
├── InspectionMission  - Close examination of targets
├── SearchAndRescueMission
├── DeliveryMission
└── ReconnaissanceMission
```

**Mission Status Attributes**:
- `MissionPending` - Not yet started
- `MissionActive` - Currently executing
- `MissionPaused` - Temporarily suspended
- `MissionCompleted` - Successfully finished
- `MissionAborted` - Terminated early
- `MissionFailed` - Could not complete

### 5. Flight Phases (Section 5)

State machine for flight operations:

```
Preflight → Armed → Takeoff → Hover ←→ Transit ←→ Loiter
                      ↓         ↓         ↓
                      └─────────┴─────────┴─→ MissionExecution
                                              ↓
                                        ReturnToLaunch → Landing → Disarmed

[Any flight phase] → EmergencyLanding → Disarmed
```

### 6. Spatial Concepts (Section 6)

Region and obstacle classes:

- **Airspace**: `Geofence`, `NoFlyZone`, `RestrictedAirspace`, `ControlledAirspace`
- **Ground**: `LandingZone`, `LaunchPoint`
- **Obstacles**: `StaticObstacle`, `DynamicObstacle`

**Key Spatial Relations**:
- `isWithin(Object, Region)` - Containment
- `distanceTo(Object1, Object2, Distance)` - Separation
- `altitudeAGL(Object, Altitude)` - Height above ground
- `clearanceDistance(UAV, Direction, Distance)` - Obstacle clearance

### 7. Environmental Conditions (Section 7)

Weather and environmental factors:

- `WindSpeed(Region, Speed)` - Wind velocity
- `VisibilityDistance(Region, Distance)` - Visibility
- `Precipitation(Region, Type)` - Rain/snow/hail
- `GNSSAvailability(Region, Status)` - GPS quality
- `LightingCondition(Region, Condition)` - Day/twilight/night

### 8. Regulatory Constraints (Section 8)

FAA Part 107 and NDAA compliance:

- `FAAPart107Compliant(Mission)` - Part 107 compliance
- `MaxRegulatoryAltitude(Region, Altitude)` - Legal altitude limits
- `RequiresVisualLineOfSight(Mission)` - VLOS requirement
- `NDAACompliant(UAV)` - NDAA compliance (FlybyF11 is compliant)
- `hasAirspaceAuthorization(Mission, Airspace)` - LAANC authorization

### 9. Operational Constraints (Section 9)

Battery and safety margins:

| Predicate | Description | FlybyF11 Default |
|-----------|-------------|------------------|
| `MinimumBatteryLevel` | Minimum battery % for flight | 20% |
| `BatteryReserveForReturn` | Reserve % for RTL | 25% |
| `MinimumObstacleDistance` | Obstacle separation | 3.0m |
| `MinimumPersonDistance` | Person separation | 15.0m |

## Safety Axioms (Section 10)

Critical safety rules encoded as logical implications:

### Collision Avoidance
```kif
(=>
  (and
    (instance ?UAV UAV)
    (instance ?OBS Obstacle)
    (distanceTo ?UAV ?OBS ?DIST)
    (MinimumObstacleDistance ?UAV ?MIN_DIST)
    (lessThan ?DIST ?MIN_DIST))
  (not (safeState ?UAV)))
```

### Geofence Containment
```kif
(=>
  (and (isOutside ?UAV ?GEOFENCE)
       (hasMissionArea ?MISSION ?GEOFENCE))
  (geofenceViolation ?UAV))
```

### No-Fly Zone
```kif
(=>
  (isWithin ?UAV ?NFZ)
  (and (not (safeState ?UAV))
       (noFlyZoneViolation ?UAV)))
```

### Battery Reserve
```kif
(=>
  (and (CurrentBatteryLevel ?UAV ?CURRENT)
       (BatteryReserveForReturn ?UAV ?RESERVE)
       (lessThan ?CURRENT ?RESERVE))
  (mustReturnToLaunch ?UAV))
```

### Lost Localization
```kif
(=>
  (not (hasValidLocalization ?UAV))
  (mustLand ?UAV))
```

## Test Scenarios

Located in `test_scenarios/`:

| Scenario | File | Expected Result |
|----------|------|-----------------|
| Valid waypoint mission | `valid_waypoint_mission.kif` | Mission valid, all constraints satisfied |
| Altitude violation | `invalid_altitude_mission.kif` | `altitudeViolation` triggered |
| Battery constraint | `battery_constraint_mission.kif` | `mustReturnToLaunch` triggered |
| Geofence violation | `geofence_violation.kif` | `geofenceViolation` triggered |
| No-fly zone entry | `no_fly_zone_violation.kif` | `noFlyZoneViolation` triggered |

## Usage

### Loading the Ontology

In the planning container:
```bash
podman run --rm -v ./ontology:/workspace:z flyby-f11-planning:latest \
  vampire --input_syntax tptp /workspace/planning_mode/uav_domain.tptp
```

### Consistency Check

Verify the ontology has no logical contradictions:
```bash
# Convert KIF to TPTP format
python3 /workspace/planning_mode/kif_to_tptp.py uav_domain.kif

# Check consistency with Vampire
vampire --mode casc --input_syntax tptp uav_domain.tptp
```

### Query Examples

Sample queries for the reasoner:

1. **Is the mission valid?**
   ```kif
   (isValidMission ?MISSION)
   ```

2. **Does the UAV have a safety violation?**
   ```kif
   (or (geofenceViolation ?UAV)
       (altitudeViolation ?UAV)
       (noFlyZoneViolation ?UAV))
   ```

3. **Must the UAV return to launch?**
   ```kif
   (mustReturnToLaunch ?UAV)
   ```

## Integration with RL

This ontology supports the ontology-constrained RL architecture:

### State Abstraction
Ontology concepts map to RL state representation:
- Flight phase → discrete state component
- Battery level → continuous state component
- Position quality → categorical state component

### Action Space Filtering
Safety axioms filter RL action space:
```python
def filter_actions(action, state):
    # Check action preconditions against ontology
    if not ontology.query(f"canExecute({action})"):
        return get_safe_alternative(action)
    return action
```

### Reward Shaping
Constraint violations contribute to reward penalties:
- Geofence violation: -100 reward
- Altitude violation: -50 reward
- Battery reserve breach: -75 reward

## Flyby F-11 Specific

Default parameters for Flyby F-11 platform:

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max Altitude | 120m | FAA Part 107 limit |
| Payload Capacity | 3.0 kg | Platform specification |
| Obstacle Distance | 3.0m | Minimum clearance |
| Person Distance | 15.0m | Minimum from non-participants |
| Battery Minimum | 20% | Critical threshold |
| Battery Reserve | 25% | RTL trigger |
| NDAA Compliant | Yes | Government use authorized |

## Future Extensions

Planned ontology enhancements:

1. **Sensor Fusion Rules**: Combine multiple sensor readings for robust perception
2. **Weather Impact Modeling**: Wind effects on trajectory planning
3. **Multi-UAV Coordination**: Shared airspace deconfliction
4. **Communications-Denied Operations**: Rules for autonomous operation without link
5. **Probabilistic Extensions**: Uncertainty handling in sensor readings

## References

- SUMO Ontology: https://github.com/ontologyportal/sumo
- IEEE 1872.2-2021: Standard for Autonomous Robotics Ontology
- FAA Part 107: https://www.faa.gov/uas/commercial_operators/part_107
- Flyby F-11 Specifications: [Platform documentation]

## License

This ontology is released under the MIT License for open research and development.
