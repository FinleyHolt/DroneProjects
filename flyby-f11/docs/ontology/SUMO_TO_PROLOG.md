# SUMO to Prolog Translation Guide

## Overview

This guide explains how to translate SUMO ontology axioms (written in SUO-KIF) into Prolog rules for runtime reasoning. In the two-phase architecture:

- **Phase 1 (Planning)**: SUMO axioms are verified with Vampire
- **Phase 2 (Execution)**: SUMO axioms are compiled to Prolog for fast runtime queries

SWI-Prolog runs on the Jetson Orin NX onboard the UAV for real-time decision-making.

## Why Prolog for Runtime Reasoning?

1. **Fast Query Performance**: Prolog's unification is highly optimized
2. **Incremental Updates**: Assert/retract facts dynamically (sensor data)
3. **Embedded Deployment**: SWI-Prolog compiles to native ARM binaries
4. **ROS 2 Integration**: PySwip allows Python nodes to query Prolog knowledge base

## Translation Strategy

Three approaches:

1. **Manual Translation** (recommended for critical rules)
2. **Semi-Automatic** (scripts + human validation)
3. **Fully Automatic** (experimental, requires parsing SUO-KIF)

## Manual Translation: SUO-KIF to Prolog

### Example 1: Simple Subclass

**SUO-KIF:**
```lisp
(subclass UnmannedAerialVehicle Aircraft)
```

**Prolog:**
```prolog
subclass(unmanned_aerial_vehicle, aircraft).

% Transitive closure for inheritance
isa(X, Y) :- subclass(X, Y).
isa(X, Z) :- subclass(X, Y), isa(Y, Z).
```

### Example 2: Instance Declaration

**SUO-KIF:**
```lisp
(instance phantom4 UnmannedAerialVehicle)
```

**Prolog:**
```prolog
instance(phantom4, unmanned_aerial_vehicle).

% Type checking
has_type(X, T) :- instance(X, T).
has_type(X, T) :- instance(X, T1), isa(T1, T).
```

### Example 3: Binary Predicate (Altitude)

**SUO-KIF:**
```lisp
(instance altitude BinaryPredicate)
(domain altitude 1 UnmannedAerialVehicle)
(domain altitude 2 LengthMeasure)
```

**Prolog:**
```prolog
% No need to declare "instance altitude BinaryPredicate" in Prolog
% Just define the relation

% Type constraints (optional, for validation)
altitude_valid(UAV, ALT) :-
    has_type(UAV, unmanned_aerial_vehicle),
    number(ALT).

% Facts (asserted at runtime)
:- dynamic altitude/2.
% altitude(phantom4, 50).  % Asserted from sensor data
```

### Example 4: Conditional Rule (Safety Constraint)

**SUO-KIF:**
```lisp
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (altitude ?UAV ?ALT)
    (greaterThan ?ALT (MeasureFn 120 Meter)))
  (violatesSafetyConstraint ?UAV AltitudeLimit))
```

**Prolog:**
```prolog
violates_safety(UAV, altitude_limit) :-
    has_type(UAV, unmanned_aerial_vehicle),
    altitude(UAV, ALT),
    ALT > 120.
```

### Example 5: Existential Quantification

**SUO-KIF:**
```lisp
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (currentPhase ?UAV LandingPhase))
  (exists (?SPEED)
    (and
      (groundSpeed ?UAV ?SPEED)
      (lessThan ?SPEED (MeasureFn 2 MeterPerSecond)))))
```

**Prolog:**
```prolog
can_land(UAV) :-
    has_type(UAV, unmanned_aerial_vehicle),
    current_phase(UAV, landing_phase),
    ground_speed(UAV, SPEED),
    SPEED < 2.
```

Note: Prolog's variables are implicitly existentially quantified.

### Example 6: Universal Quantification (All Waypoints Must Be Safe)

**SUO-KIF:**
```lisp
(=>
  (and
    (instance ?MISSION UAVMission)
    (forall (?WP)
      (=>
        (waypointInMission ?WP ?MISSION)
        (not (isInNoFlyZone ?WP)))))
  (missionSafe ?MISSION))
```

**Prolog:**
```prolog
mission_safe(MISSION) :-
    instance(MISSION, uav_mission),
    \+ (waypoint_in_mission(WP, MISSION), is_in_no_fly_zone(WP)).

% Negation-as-failure: mission is safe if NO waypoint is in NFZ
```

## Translation Patterns

### Pattern 1: Subclass Hierarchy

**SUO-KIF:**
```lisp
(subclass Quadcopter Multirotor)
(subclass Multirotor UnmannedAerialVehicle)
(subclass UnmannedAerialVehicle Aircraft)
```

**Prolog:**
```prolog
subclass(quadcopter, multirotor).
subclass(multirotor, unmanned_aerial_vehicle).
subclass(unmanned_aerial_vehicle, aircraft).

% Transitive closure
isa(X, Y) :- subclass(X, Y).
isa(X, Z) :- subclass(X, Y), isa(Y, Z).

% Query: isa(quadcopter, aircraft) => true
```

### Pattern 2: Conjunctive Rules

**SUO-KIF:**
```lisp
(=>
  (and
    (instance ?UAV Quadcopter)
    (batteryLevel ?UAV ?LEVEL)
    (lessThan ?LEVEL (MeasureFn 20 Percent)))
  (mustExecute ?UAV ReturnToHome))
```

**Prolog:**
```prolog
must_execute(UAV, return_to_home) :-
    instance(UAV, quadcopter),
    battery_level(UAV, LEVEL),
    LEVEL < 20.
```

### Pattern 3: Disjunctive Rules

**SUO-KIF:**
```lisp
(=>
  (instance ?UAV UnmannedAerialVehicle)
  (or
    (currentPhase ?UAV TakeoffPhase)
    (currentPhase ?UAV CruisePhase)
    (currentPhase ?UAV LandingPhase)))
```

**Prolog:**
```prolog
valid_phase(UAV, PHASE) :-
    instance(UAV, unmanned_aerial_vehicle),
    (   PHASE = takeoff_phase
    ;   PHASE = cruise_phase
    ;   PHASE = landing_phase
    ).

% Or as separate clauses (more idiomatic):
valid_phase(UAV, takeoff_phase) :- instance(UAV, unmanned_aerial_vehicle).
valid_phase(UAV, cruise_phase) :- instance(UAV, unmanned_aerial_vehicle).
valid_phase(UAV, landing_phase) :- instance(UAV, unmanned_aerial_vehicle).
```

### Pattern 4: Numerical Comparisons

**SUO-KIF:**
```lisp
(greaterThan ?X ?Y)
(lessThan ?X ?Y)
(equal ?X ?Y)
```

**Prolog:**
```prolog
% Use built-in arithmetic comparisons
X > Y
X < Y
X =:= Y  % Arithmetic equality
X = Y    % Unification
```

### Pattern 5: Measurements and Units

**SUO-KIF:**
```lisp
(MeasureFn 120 Meter)
(MeasureFn 400 Foot)
```

**Prolog:**
```prolog
% Option 1: Use SI units everywhere (convert at assertion time)
% altitude(uav1, 120).  % Always in meters

% Option 2: Represent with unit
% altitude(uav1, measure(120, meter)).

% Recommended: Option 1 (simpler, faster queries)
```

## Complete Example: UAV Safety Rules

### SUMO Axioms (SUO-KIF)

```lisp
; File: uav_safety.kif

(subclass UnmannedAerialVehicle Aircraft)
(subclass Quadcopter UnmannedAerialVehicle)

(instance altitude BinaryPredicate)
(domain altitude 1 UnmannedAerialVehicle)
(domain altitude 2 LengthMeasure)

(instance batteryLevel BinaryPredicate)
(domain batteryLevel 1 UnmannedAerialVehicle)
(domain batteryLevel 2 RealNumber)

(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (altitude ?UAV ?ALT)
    (greaterThan ?ALT 120))
  (violatesSafety ?UAV altitude_limit))

(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (batteryLevel ?UAV ?LEVEL)
    (lessThan ?LEVEL 20))
  (mustExecute ?UAV return_to_home))

(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (violatesSafety ?UAV ?CONSTRAINT))
  (emergencyAction ?UAV land_immediately))
```

### Compiled Prolog Rules

```prolog
% File: uav_safety.pl

% Ontology structure
subclass(quadcopter, unmanned_aerial_vehicle).
subclass(unmanned_aerial_vehicle, aircraft).

isa(X, Y) :- subclass(X, Y).
isa(X, Z) :- subclass(X, Y), isa(Y, Z).

% Dynamic facts (updated at runtime from sensors)
:- dynamic altitude/2.
:- dynamic battery_level/2.
:- dynamic instance/2.

% Example instance
instance(uav1, quadcopter).

% Safety rules

% Altitude limit violation
violates_safety(UAV, altitude_limit) :-
    instance(UAV, unmanned_aerial_vehicle),
    altitude(UAV, ALT),
    ALT > 120.

% Low battery action
must_execute(UAV, return_to_home) :-
    instance(UAV, unmanned_aerial_vehicle),
    battery_level(UAV, LEVEL),
    LEVEL < 20.

% Emergency action on safety violation
emergency_action(UAV, land_immediately) :-
    instance(UAV, unmanned_aerial_vehicle),
    violates_safety(UAV, _).

% Query predicates
is_safe(UAV) :-
    instance(UAV, unmanned_aerial_vehicle),
    \+ violates_safety(UAV, _).

current_actions(UAV, ACTIONS) :-
    instance(UAV, unmanned_aerial_vehicle),
    findall(ACTION, must_execute(UAV, ACTION), ACTIONS).
```

### Runtime Usage (SWI-Prolog)

```prolog
?- [uav_safety].
true.

% Assert current state
?- assert(altitude(uav1, 50)).
true.

?- assert(battery_level(uav1, 80)).
true.

% Check safety
?- is_safe(uav1).
true.

% Simulate altitude violation
?- retract(altitude(uav1, 50)), assert(altitude(uav1, 150)).
true.

?- violates_safety(uav1, CONSTRAINT).
CONSTRAINT = altitude_limit.

?- emergency_action(uav1, ACTION).
ACTION = land_immediately.

% Simulate low battery
?- retract(battery_level(uav1, 80)), assert(battery_level(uav1, 15)).
true.

?- current_actions(uav1, ACTIONS).
ACTIONS = [return_to_home].
```

## Semi-Automatic Translation: Python Script

```python
# File: kif_to_prolog.py

import re

def translate_kif_to_prolog(kif_file, prolog_file):
    """
    Semi-automatic translation of SUO-KIF to Prolog.
    Handles basic patterns; manual review required.
    """
    with open(kif_file, 'r') as f:
        kif_content = f.read()

    prolog_rules = []

    # Pattern 1: (subclass A B)
    for match in re.finditer(r'\(subclass\s+(\w+)\s+(\w+)\)', kif_content):
        a, b = match.groups()
        prolog_rules.append(f"subclass({to_prolog_atom(a)}, {to_prolog_atom(b)}).")

    # Pattern 2: (instance X Y)
    for match in re.finditer(r'\(instance\s+(\w+)\s+(\w+)\)', kif_content):
        x, y = match.groups()
        prolog_rules.append(f"instance({to_prolog_atom(x)}, {to_prolog_atom(y)}).")

    # Pattern 3: Simple implication (=>  (and ...) ...)
    # This requires more sophisticated parsing - use manual translation

    with open(prolog_file, 'w') as f:
        f.write("% Auto-generated from SUO-KIF\n")
        f.write("% REQUIRES MANUAL REVIEW\n\n")
        f.write('\n'.join(prolog_rules))

    print(f"Translated {len(prolog_rules)} rules. Manual review required.")

def to_prolog_atom(kif_atom):
    """Convert KIF atom to Prolog atom (lowercase, underscores)."""
    # CamelCase -> snake_case
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', kif_atom)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

# Usage
translate_kif_to_prolog('uav_domain.kif', 'compiled_rules.pl')
```

## SWI-Prolog Installation on Jetson (ARM)

### Build from Source (Recommended for ARM)

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libreadline-dev \
    libgmp-dev \
    libssl-dev \
    libyaml-dev \
    libarchive-dev \
    libpcre3-dev

# Clone SWI-Prolog
cd ~/
git clone https://github.com/SWI-Prolog/swipl.git
cd swipl

# Build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install

# Verify
swipl --version
```

### Package Manager (if available)

```bash
sudo apt-add-repository ppa:swi-prolog/stable
sudo apt-get update
sudo apt-get install swi-prolog
```

## Integration with ROS 2 (PySwip)

See `PROLOG_ROS2_BRIDGE.md` for detailed integration guide.

Quick example:

```python
from pyswip import Prolog

# Load knowledge base
prolog = Prolog()
prolog.consult("uav_safety.pl")

# Assert current altitude from sensor
prolog.assertz(f"altitude(uav1, {current_altitude})")

# Query safety
safe = list(prolog.query("is_safe(uav1)"))
if not safe:
    print("UNSAFE - triggering emergency action")
```

## Validation Strategy

1. **Offline Verification**: Prove properties with Vampire using original SUMO axioms
2. **Translation Validation**: Compare Vampire proofs with Prolog query results
3. **Runtime Testing**: Validate Prolog rules against known scenarios

### Example Validation Test

```prolog
% test_validation.pl

:- [uav_safety].

% Test 1: Normal altitude is safe
test_altitude_safe :-
    assert(altitude(uav1, 50)),
    is_safe(uav1),
    retract(altitude(uav1, 50)),
    write('Test 1 PASSED'), nl.

% Test 2: High altitude triggers violation
test_altitude_violation :-
    assert(altitude(uav1, 150)),
    violates_safety(uav1, altitude_limit),
    retract(altitude(uav1, 150)),
    write('Test 2 PASSED'), nl.

% Test 3: Low battery triggers RTH
test_low_battery :-
    assert(battery_level(uav1, 15)),
    must_execute(uav1, return_to_home),
    retract(battery_level(uav1, 15)),
    write('Test 3 PASSED'), nl.

% Run all tests
run_tests :-
    test_altitude_safe,
    test_altitude_violation,
    test_low_battery,
    write('All tests PASSED'), nl.

% Execute: swipl -s test_validation.pl -g run_tests -t halt
```

## Performance Considerations

### Optimizing Prolog for Real-Time Queries

1. **Indexing**: SWI-Prolog automatically indexes first argument of predicates
2. **Tabling**: Use tabled predicates for expensive recursive queries
3. **Compilation**: Compile to native code with `qsave_program/2`
4. **Minimal Backtracking**: Structure rules to fail fast

### Example: Tabled Recursive Query

```prolog
:- table reachable/2.

reachable(X, Y) :- edge(X, Y).
reachable(X, Z) :- edge(X, Y), reachable(Y, Z).

% With tabling, reachable(A, B) is computed once and cached
```

## Limitations and Workarounds

| SUMO Feature | Prolog Translation | Workaround |
|--------------|-------------------|------------|
| Higher-order logic | Not supported | Use meta-predicates or external solver |
| Complex arithmetic | Limited | Pre-compute in Python, assert results |
| Probabilistic reasoning | Not native | Use external probabilistic logic library |
| Temporal reasoning | Limited | Use event calculus or interval algebra |

## Best Practices

1. **Keep It Simple**: Translate only the rules needed for runtime reasoning
2. **Validate Thoroughly**: Test Prolog rules against SUMO axioms
3. **Document Translation**: Comment each Prolog rule with original SUO-KIF
4. **Version Control**: Track both SUMO and Prolog versions together
5. **Modular Files**: Separate static ontology from dynamic facts

## Next Steps

1. **Translate UAV domain axioms** (see `examples/uav_domain.kif`)
2. **Create validation tests** for Prolog rules
3. **Integrate with ROS 2** (see `PROLOG_ROS2_BRIDGE.md`)
4. **Deploy to Jetson** (compile SWI-Prolog for ARM)

## References

- SWI-Prolog Manual: https://www.swi-prolog.org/pldoc/doc_for?object=manual
- SWI-Prolog Build Guide: https://www.swi-prolog.org/build/unix.html
- PySwip Documentation: https://github.com/yuce/pyswip
- SUMO to Prolog: http://www.ontologyportal.org/ (Sigma tools)
