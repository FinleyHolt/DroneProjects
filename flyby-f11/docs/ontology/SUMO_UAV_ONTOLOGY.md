# SUMO UAV Ontology Extension

## Overview

This guide explains how to extend the Suggested Upper Merged Ontology (SUMO) for the UAV domain. SUMO provides a foundational ontology with concepts like `Device`, `Process`, `AutonomousAgent`, and spatial/temporal relations that can be specialized for drone operations.

## Installation

SUMO is already cloned in `repos/sumo/`. The key files are:

- `Merge.kif` - Core upper ontology
- `Mid-level-ontology.kif` - Mid-level concepts
- `ComputingBrands.kif` - Computer hardware/software
- `Transportation.kif` - Transportation domain (useful for UAVs)

## SUO-KIF Syntax Basics

SUMO uses SUO-KIF (Standard Upper Ontology Knowledge Interchange Format):

```lisp
; Comments start with semicolon

; Subclass relationship
(subclass UnmannedAerialVehicle Aircraft)

; Instance declaration
(instance Phantom4 UnmannedAerialVehicle)

; Documentation
(documentation UnmannedAerialVehicle EnglishLanguage
  "A UAV is an aircraft operated without a human pilot aboard")

; Domain/range constraints for relations
(domain altitude 1 UnmannedAerialVehicle)
(domain altitude 2 LengthMeasure)

; Axioms (logical rules)
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (altitude ?UAV ?ALT)
    (greaterThan ?ALT (MeasureFn 400 Foot)))
  (violatesRegulation ?UAV FAAPart107))
```

## UAV Domain Concepts

### Core Classes

See `examples/uav_domain.kif` for the full ontology. Key classes:

```lisp
; Top-level UAV class
(subclass UnmannedAerialVehicle Aircraft)
(subclass UnmannedAerialVehicle AutonomousAgent)

; UAV types
(subclass Multirotor UnmannedAerialVehicle)
(subclass Quadcopter Multirotor)
(subclass Hexacopter Multirotor)
(subclass FixedWing UnmannedAerialVehicle)

; Flight states
(subclass FlightPhase Process)
(subclass TakeoffPhase FlightPhase)
(subclass CruisePhase FlightPhase)
(subclass LandingPhase FlightPhase)
(subclass HoverPhase FlightPhase)

; Sensors
(subclass UAVSensor Device)
(subclass RGBCamera UAVSensor)
(subclass DepthCamera UAVSensor)
(subclass VisualOdometrySensor UAVSensor)
(subclass IMU UAVSensor)
(subclass GPS UAVSensor)
```

### Spatial Relations

SUMO has built-in spatial predicates that can be used for UAVs:

```lisp
; Existing SUMO predicates (already defined):
; (orientation ?OBJ1 ?OBJ2 ?DIRECTION)
; (between ?OBJ1 ?OBJ2 ?OBJ3)
; (distance ?OBJ1 ?OBJ2 ?DISTANCE)

; UAV-specific extensions
(instance altitude BinaryPredicate)
(domain altitude 1 UnmannedAerialVehicle)
(domain altitude 2 LengthMeasure)
(documentation altitude EnglishLanguage
  "Vertical distance above ground level or sea level")

(instance groundSpeed BinaryPredicate)
(domain groundSpeed 1 UnmannedAerialVehicle)
(domain groundSpeed 2 Quantity)

; Safety zones
(subclass NoFlyZone GeographicArea)
(instance isInNoFlyZone BinaryPredicate)
(domain isInNoFlyZone 1 UnmannedAerialVehicle)
(domain isInNoFlyZone 2 NoFlyZone)
```

### Mission Concepts

```lisp
; Mission planning
(subclass UAVMission Plan)
(subclass SurveyMission UAVMission)
(subclass InspectionMission UAVMission)
(subclass DeliveryMission UAVMission)

; Waypoints
(subclass Waypoint GeographicArea)
(instance waypointSequence TernaryPredicate)
(domain waypointSequence 1 UAVMission)
(domain waypointSequence 2 Waypoint)
(domain waypointSequence 3 Integer)  ; sequence number

; Tasks
(subclass UAVTask IntentionalProcess)
(subclass ImageCapture UAVTask)
(subclass ObstacleAvoidance UAVTask)
(subclass ReturnToHome UAVTask)
```

## Safety Axioms

Critical for verification (see `examples/safety_axioms.kif`):

```lisp
; Altitude constraints
(=>
  (and
    (instance ?UAV Quadcopter)
    (altitude ?UAV ?ALT)
    (greaterThan ?ALT (MeasureFn 120 Meter)))
  (violatesSafetyConstraint ?UAV AltitudeLimit))

; No-fly zone enforcement
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (isInNoFlyZone ?UAV ?ZONE))
  (mustExecute ?UAV ReturnToHome))

; Battery safety
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (batteryLevel ?UAV ?LEVEL)
    (lessThan ?LEVEL (MeasureFn 20 Percent)))
  (mustExecute ?UAV ReturnToHome))

; Obstacle proximity
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (instance ?OBS Object)
    (distance ?UAV ?OBS ?DIST)
    (lessThan ?DIST (MeasureFn 2 Meter)))
  (mustExecute ?UAV ObstacleAvoidance))

; GPS loss handling
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (hasSensor ?UAV ?GPS)
    (instance ?GPS GPS)
    (not (operational ?GPS)))
  (mustExecute ?UAV HoverPhase))
```

## Sensor Modeling

```lisp
; Sensor capabilities
(instance hasSensor BinaryPredicate)
(domain hasSensor 1 UnmannedAerialVehicle)
(domain hasSensor 2 UAVSensor)

(instance sensorRange BinaryPredicate)
(domain sensorRange 1 UAVSensor)
(domain sensorRange 2 LengthMeasure)

(instance detectionConfidence BinaryPredicate)
(domain detectionConfidence 1 UAVSensor)
(domain detectionConfidence 2 RealNumber)

; Sensor fusion axiom
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (hasSensor ?UAV ?CAM)
    (instance ?CAM DepthCamera)
    (hasSensor ?UAV ?VO)
    (instance ?VO VisualOdometrySensor)
    (operational ?CAM)
    (operational ?VO))
  (capableOf ?UAV IndoorNavigation))
```

## Flight Phase Transitions

```lisp
; State machine rules
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (currentPhase ?UAV TakeoffPhase)
    (altitude ?UAV ?ALT)
    (greaterThan ?ALT (MeasureFn 5 Meter)))
  (transitionsTo ?UAV CruisePhase))

(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (currentPhase ?UAV CruisePhase)
    (distanceToWaypoint ?UAV ?NEXT ?DIST)
    (lessThan ?DIST (MeasureFn 1 Meter)))
  (or
    (transitionsTo ?UAV HoverPhase)
    (transitionsTo ?UAV LandingPhase)))

; Preconditions for landing
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (currentPhase ?UAV LandingPhase))
  (and
    (exists (?SPEED)
      (and
        (groundSpeed ?UAV ?SPEED)
        (lessThan ?SPEED (MeasureFn 2 MeterPerSecond))))
    (not (exists (?OBS ?DIST)
      (and
        (instance ?OBS Object)
        (distance ?UAV ?OBS ?DIST)
        (lessThan ?DIST (MeasureFn 5 Meter)))))))
```

## Integration with SUMO Tools

### Using Sigma (SUMO IDE)

Sigma provides a web interface for browsing and editing SUMO:

```bash
cd repos/sumo
# Follow README for Sigma setup
# Load custom UAV ontology files
```

### Exporting to Other Formats

SUMO can be exported to:
- **OWL** - For use with Protégé, description logic reasoners
- **TPTP** - For automated theorem provers (Vampire, E-Prover)
- **Prolog** - For runtime reasoning (see SUMO_TO_PROLOG.md)

### Validation

Check ontology consistency:

```bash
# Use Sigma's consistency checker
# Or export to TPTP and verify with Vampire:
vampire --mode casc uav_domain.tptp
```

## Best Practices

1. **Reuse SUMO Concepts**: Don't redefine what SUMO already has (e.g., `Device`, `Process`, `GeographicArea`)
2. **Document Everything**: Use `documentation` predicates for all classes and relations
3. **Axiom Coverage**: Write axioms that connect your new concepts to SUMO's upper ontology
4. **Safety First**: Model all safety constraints as axioms for verification
5. **Modularity**: Keep UAV extensions in separate `.kif` files that import SUMO

## File Organization

```
flyby-f11/docs/ontology/
├── repos/sumo/              # SUMO base ontology
├── examples/
│   ├── uav_domain.kif       # UAV class hierarchy
│   ├── safety_axioms.kif    # Safety constraints
│   └── mission_planning.kif # Mission concepts
```

## Next Steps

1. **Extend the ontology**: Add domain-specific concepts (payload types, weather conditions, etc.)
2. **Verification**: Use Vampire to prove safety properties (see VAMPIRE_VERIFICATION.md)
3. **Translation to Prolog**: Convert axioms for runtime reasoning (see SUMO_TO_PROLOG.md)
4. **Integration**: Bridge to ROS 2 via PySwip (see PROLOG_ROS2_BRIDGE.md)

## References

- SUMO Homepage: http://www.ontologyportal.org/
- SUO-KIF Specification: http://suo.ieee.org/SUO/KIF/
- SUMO GitHub: https://github.com/ontologyportal/sumo
- Sigma IDE Documentation: https://github.com/ontologyportal/sigmakee
