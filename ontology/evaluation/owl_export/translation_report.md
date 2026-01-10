# KIF to OWL Translation Report

**Source**: `ontology/planning_mode/uav_domain.kif` (KIF/SUMO format)
**Target**: `ontology/evaluation/owl_export/uav_domain.owl` (OWL 2 Turtle format)
**Date**: 2024-12-25
**Author**: Finley Holt

## Executive Summary

This report documents the translation of the UAV Domain Ontology from KIF/SUMO format to OWL 2. The translation preserves the class hierarchy, object/data properties, and simple facts, but **loses the first-order logic (FOL) axioms** that form the core safety reasoning capabilities of the original ontology.

**Key Finding**: OWL 2 (even with DL profile) cannot express the universal quantification with implications that define the safety axioms. The ELK reasoner, which supports only OWL 2 EL, has even more limited expressivity.

---

## What Was Preserved

### 1. Class Hierarchy (Fully Preserved)

All class hierarchies from the KIF ontology are preserved in OWL:

| KIF Construct | OWL Construct | Status |
|---------------|---------------|--------|
| `(subclass UAV Aircraft)` | `:UAV rdfs:subClassOf :Aircraft` | PRESERVED |
| `(subclass Multirotor UAV)` | `:Multirotor rdfs:subClassOf :UAV` | PRESERVED |
| `(subclass Quadcopter Multirotor)` | `:Quadcopter rdfs:subClassOf :Multirotor` | PRESERVED |
| `(subclass FlybyF11 Quadcopter)` | `:FlybyF11 rdfs:subClassOf :Quadcopter` | PRESERVED |

**Complete class hierarchy preserved**:
- UAV types: UAV, Multirotor, FixedWingUAV, HybridVTOL, Quadcopter, Hexacopter, Octocopter, FlybyF11
- Components: UAVSensor, PositioningSensor, GNSSSensor, VisualOdometrySensor, PerceptionSensor, DepthCamera, RGBCamera, LiDARSensor, IMU
- Compute: UAVComputer, JetsonModule
- Propulsion: UAVPropulsionSystem, BrushlessMotor, Propeller
- Power: UAVBattery, LiPoBattery
- Missions: UAVMission, WaypointMission, SurveyMission, InspectionMission, SearchAndRescueMission, DeliveryMission, ReconnaissanceMission
- Regions: AirspaceRegion, Geofence, NoFlyZone, RestrictedAirspace, ControlledAirspace, UncontrolledAirspace, LandingZone, LaunchPoint
- Obstacles: Obstacle, StaticObstacle, DynamicObstacle
- Actions: UAVAction and all subclasses

### 2. Binary Predicates as Object/Data Properties (Preserved with Limitations)

| KIF Predicate | OWL Property | Status |
|---------------|--------------|--------|
| `(hasComputer ?UAV ?COMP)` | `:hasComputer` ObjectProperty | PRESERVED |
| `(hasSensor ?UAV ?SENSOR ?POS)` | `:hasSensor` ObjectProperty | PARTIAL (lost position arg) |
| `(assignedUAV ?MISSION ?UAV)` | `:assignedUAV` ObjectProperty | PRESERVED |
| `(hasMissionStatus ?M ?S)` | `:hasMissionStatus` ObjectProperty | PRESERVED |
| `(hasFlightPhase ?UAV ?PHASE)` | `:hasFlightPhase` ObjectProperty | PRESERVED |
| `(CurrentBatteryLevel ?UAV ?LEVEL)` | `:currentBatteryLevel` DatatypeProperty | PRESERVED |

### 3. Named Individuals (Preserved)

All enumerated values from KIF are represented as OWL NamedIndividuals:

- **Mission Status**: MissionPending, MissionActive, MissionPaused, MissionCompleted, MissionAborted, MissionFailed
- **Flight Phases**: Preflight, Armed, Takeoff, Hover, Transit, Loiter, MissionExecution, ReturnToLaunch, Landing, EmergencyLanding, Disarmed
- **Sensor States**: SensorOperational, SensorDegraded, SensorFailed
- **Data Quality**: HighQuality, MediumQuality, LowQuality, NoData
- **Emergency Conditions**: LowBatteryEmergency, LostLinkEmergency, LocalizationLostEmergency, ObstacleCollisionImminent, WeatherEmergency

### 4. Simple Facts (Preserved)

Ground facts like flight phase transitions are preserved:

```turtle
:Preflight :canTransitionTo :Armed .
:Armed :canTransitionTo :Takeoff , :Disarmed .
:Hover :canTransitionTo :Transit , :Loiter , :MissionExecution , :Landing .
```

### 5. Disjointness Constraints (Preserved)

OWL AllDisjointClasses axioms capture mutual exclusion:

```turtle
[ rdf:type owl:AllDisjointClasses ;
  owl:members ( :Multirotor :FixedWingUAV :HybridVTOL ) ] .

[ rdf:type owl:AllDisjointClasses ;
  owl:members ( :SafeState :UnsafeState ) ] .
```

---

## What Was Lost in Translation

### 1. First-Order Logic Safety Axioms (LOST)

The most critical loss is the FOL axioms with universal quantification and implications. OWL 2 does not support arbitrary FOL formulas.

#### Collision Avoidance Axiom (LOST)

**Original KIF**:
```lisp
(=>
  (and
    (instance ?UAV UAV)
    (instance ?OBS Obstacle)
    (distanceTo ?UAV ?OBS ?DIST)
    (MinimumObstacleDistance ?UAV ?MIN_DIST)
    (lessThan ?DIST ?MIN_DIST))
  (not (safeState ?UAV)))
```

**Why Lost**: OWL cannot express:
- Variable binding across predicates (`?UAV`, `?OBS`, `?DIST`)
- Numeric comparison (`lessThan ?DIST ?MIN_DIST`)
- Negation of a derived property (`not (safeState ?UAV)`)

**OWL Workaround**: Created a `CollisionRiskState` class that users must manually assert.

#### Geofence Violation Axiom (LOST)

**Original KIF**:
```lisp
(=>
  (and
    (instance ?UAV UAV)
    (instance ?MISSION UAVMission)
    (assignedUAV ?MISSION ?UAV)
    (hasMissionArea ?MISSION ?GEOFENCE)
    (instance ?GEOFENCE Geofence)
    (isOutside ?UAV ?GEOFENCE))
  (geofenceViolation ?UAV))
```

**Why Lost**: OWL cannot express:
- Multi-step path from UAV to MISSION to GEOFENCE
- Automatic inference of `geofenceViolation` from spatial relationship

**OWL Workaround**: Created `GeofenceViolationState` class for manual assertion.

#### No-Fly Zone Axiom (LOST)

**Original KIF**:
```lisp
(=>
  (and
    (instance ?UAV UAV)
    (instance ?NFZ NoFlyZone)
    (isWithin ?UAV ?NFZ))
  (and
    (not (safeState ?UAV))
    (noFlyZoneViolation ?UAV)))
```

**Why Lost**: Same reasons as above - variable binding and automatic inference.

**OWL Workaround**: Created `NoFlyZoneViolationState` class.

#### Battery Reserve Axiom (LOST)

**Original KIF**:
```lisp
(=>
  (and
    (CurrentBatteryLevel ?UAV ?CURRENT)
    (BatteryReserveForReturn ?UAV ?RESERVE)
    (lessThan ?CURRENT ?RESERVE))
  (mustReturnToLaunch ?UAV))
```

**Why Lost**: OWL cannot perform numeric comparisons or bind numeric values.

**OWL Workaround**: Created `LowBatteryState` class.

### 2. Ternary Predicates (Partially Lost)

**Original KIF**:
```lisp
(hasSensor ?UAV ?SENSOR ?POSITION)
(distanceTo ?OBJ1 ?OBJ2 ?DIST)
(hasWaypointSequence ?MISSION ?WP ?INDEX)
```

**Why Lost**: OWL properties are binary. Ternary predicates require reification.

**OWL Workaround**: Simplified to binary properties, losing the third argument.

### 3. Class Inference from Properties (LOST)

**Original KIF**:
```lisp
(=>
  (instance ?UAV Multirotor)
  (canHover ?UAV))

(=>
  (instance ?UAV FlybyF11)
  (NDAACompliant ?UAV))
```

**Partial Preservation**: Expressed as `rdfs:subClassOf` relationships:
```turtle
:Multirotor rdfs:subClassOf :HoverCapable .
:FlybyF11 rdfs:subClassOf :NDAACompliant .
```

This preserves the class membership but loses the predicate-based assertion.

### 4. Numeric Constraints and Calculations (LOST)

**Original KIF**:
```lisp
(hasMaxAltitude ?UAV (MeasureFn 120 Meter))
(MinimumObstacleDistance ?UAV (MeasureFn 3.0 Meter))
```

**Why Lost**: OWL cannot express:
- Units of measure (QUDT ontology could help but adds complexity)
- Numeric calculations or comparisons in axioms

**OWL Workaround**: Plain decimal values without units.

### 5. Action Preconditions (LOST)

**Original KIF**:
```lisp
(actionPrecondition NavigateToWaypoint hasValidLocalization)
```

**Why Lost**: OWL cannot express that an action type requires a precondition to be met.

### 6. Mutual Exclusion Logic (Partially Preserved)

**Original KIF**:
```lisp
(=>
  (isWithin ?OBJ ?REGION)
  (not (isOutside ?OBJ ?REGION)))
```

**OWL Workaround**: `owl:propertyDisjointWith` approximates this but doesn't provide full logical semantics.

---

## Safety Axiom Summary

| Safety Axiom | KIF Status | OWL Status | Reasoning Impact |
|--------------|------------|------------|------------------|
| Collision Avoidance | Full FOL rule | Class-based workaround | Cannot auto-detect violations |
| Geofence Violation | Full FOL rule | Class-based workaround | Cannot auto-detect violations |
| No-Fly Zone Violation | Full FOL rule | Class-based workaround | Cannot auto-detect violations |
| Battery Reserve Check | Full FOL rule | Class-based workaround | Cannot auto-detect low battery |
| Altitude Violation | Full FOL rule | Class-based workaround | Cannot auto-detect violations |
| Lost Localization | Full FOL rule | Class-based workaround | Cannot auto-detect |
| Weather Constraint | Full FOL rule | Class-based workaround | Cannot auto-detect |

**Critical Insight**: All safety axioms that would enable **automatic safety violation detection** are lost. The OWL ontology can classify instances and check class hierarchies, but cannot perform the conditional reasoning required for runtime safety monitoring.

---

## ELK Reasoner Considerations

The ELK reasoner supports only OWL 2 EL profile, which is even more restrictive:

### Supported by ELK:
- Class hierarchies (`rdfs:subClassOf`)
- Object property domains and ranges
- Object property chains
- Existential quantification (`owl:someValuesFrom`)

### NOT Supported by ELK:
- Universal quantification (`owl:allValuesFrom`)
- Negation (`owl:complementOf`)
- Disjointness axioms (limited support)
- Cardinality restrictions
- Property disjointness
- Data properties (limited)

**Recommendation**: The OWL ontology should work with ELK for classification tasks, but the safety axioms cannot be expressed in any OWL profile.

---

## Recommendations

1. **For Safety Reasoning**: Continue using KIF/SUMO with a theorem prover (Vampire) for safety verification
2. **For ELK Benchmarking**: Use the OWL ontology for classification and subsumption queries
3. **Hybrid Approach**: Use OWL for visualization and ontology exploration, FOL for actual safety reasoning
4. **Future Work**: Consider SWRL rules or OWL 2 RL with rule engines for some conditional logic

---

## Conclusion

The OWL translation preserves the taxonomic structure of the UAV ontology but loses the first-order logic axioms that define safety constraints. This is a fundamental limitation of Description Logic (OWL's foundation) compared to full first-order logic.

For the Flyby F-11 autonomous UAV platform, the **KIF/SUMO ontology with Vampire theorem prover remains essential** for safety reasoning. The OWL version serves as a complementary representation for ontology exploration, visualization, and interoperability with OWL-based tools.
