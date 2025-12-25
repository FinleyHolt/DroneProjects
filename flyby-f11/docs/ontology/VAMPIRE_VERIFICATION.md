# Vampire Theorem Prover for Mission Verification

## Overview

Vampire is a first-order logic (FOL) theorem prover used to verify safety properties and mission plans. In the two-phase architecture, Vampire runs offline during mission planning to prove that a mission satisfies all safety constraints before deployment.

## Installation

Vampire is already cloned in `repos/vampire/`. To build:

```bash
cd repos/vampire
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install  # Optional: install system-wide
```

Or use pre-built binary:
```bash
cd repos/vampire/build
./bin/vampire --version
```

## Use Cases in UAV Domain

### 1. Safety Property Verification

**Question**: "Does the mission plan violate any no-fly zones?"

**Approach**: Model the mission as FOL axioms and query Vampire to find contradictions.

### 2. Precondition Checking

**Question**: "Can the UAV safely land given current battery and distance to home?"

**Approach**: Encode current state + landing constraints, ask Vampire if landing is safe.

### 3. Plan Validation

**Question**: "Is the waypoint sequence reachable without battery exhaustion?"

**Approach**: Model battery consumption and waypoint distances, prove reachability.

## TPTP Format Basics

Vampire uses TPTP (Thousands of Problems for Theorem Provers) syntax:

```tptp
% Comments start with %

% Type declarations
fof(uav_type, axiom, ![UAV]: (uav(UAV) => aircraft(UAV))).

% Facts
fof(phantom_is_uav, axiom, uav(phantom4)).
fof(current_altitude, axiom, altitude(phantom4, 50)).

% Rules
fof(altitude_limit, axiom,
  ![UAV, ALT]: (
    (uav(UAV) & altitude(UAV, ALT) & ALT > 120) =>
    violates_safety(UAV)
  )
).

% Conjecture (what we want to prove)
fof(safe_flight, conjecture, ~violates_safety(phantom4)).
```

## Example: No-Fly Zone Verification

### Problem Definition

Given:
- UAV current position: (lat: 37.7749, lon: -122.4194)
- Waypoint position: (lat: 37.7849, lon: -122.4294)
- No-fly zone: circle centered at (37.7799, -122.4244), radius 500m

Prove: The waypoint is NOT in the no-fly zone.

### TPTP Encoding

```tptp
% File: noflyzone_check.p

% Type declarations
fof(types, axiom, ![X]: (waypoint(X) => location(X))).
fof(types2, axiom, ![X]: (nofly_zone(X) => region(X))).

% Facts about locations
fof(waypoint1, axiom, waypoint(wp1)).
fof(wp1_coords, axiom, latitude(wp1, 37.7849)).
fof(wp1_coords2, axiom, longitude(wp1, -122.4294)).

fof(nfz1, axiom, nofly_zone(restricted_area_1)).
fof(nfz_center_lat, axiom, center_latitude(restricted_area_1, 37.7799)).
fof(nfz_center_lon, axiom, center_longitude(restricted_area_1, -122.4244)).
fof(nfz_radius, axiom, radius(restricted_area_1, 500)).

% Distance calculation (simplified Haversine)
fof(haversine, axiom,
  ![WP, NFZ, LAT1, LON1, LAT2, LON2, DIST]:
    (waypoint(WP) & nofly_zone(NFZ) &
     latitude(WP, LAT1) & longitude(WP, LON1) &
     center_latitude(NFZ, LAT2) & center_longitude(NFZ, LON2) &
     DIST = haversine_distance(LAT1, LON1, LAT2, LON2)) =>
    distance_to_zone(WP, NFZ, DIST)
).

% Actual distance (pre-computed or from external calculation)
fof(computed_distance, axiom, distance_to_zone(wp1, restricted_area_1, 750)).

% Safety rule
fof(nfz_safety, axiom,
  ![WP, NFZ, DIST, RAD]:
    (waypoint(WP) & nofly_zone(NFZ) &
     distance_to_zone(WP, NFZ, DIST) &
     radius(NFZ, RAD) &
     DIST < RAD) =>
    violates_nfz(WP)
).

% Conjecture: waypoint does NOT violate NFZ
fof(safe_waypoint, conjecture, ~violates_nfz(wp1)).
```

### Running Vampire

```bash
cd repos/vampire/build
./bin/vampire --mode casc noflyzone_check.p
```

**Expected Output:**
```
% SZS status Theorem for noflyzone_check.p
% Proof found
```

If the waypoint violated the NFZ, Vampire would return `CounterSatisfiable` or `Unsatisfiable`.

## Example: Battery Endurance Verification

### Problem Definition

Given:
- Current battery: 60%
- Distance to waypoint: 500m
- Distance from waypoint back to home: 600m
- Battery consumption: 1% per 50m traveled
- Safety margin: Must land with at least 20% battery

Prove: The mission is battery-safe.

### TPTP Encoding

```tptp
% File: battery_safety.p

% Facts
fof(current_battery, axiom, battery_level(uav1, 60)).
fof(distance_to_wp, axiom, distance(current_pos, waypoint1, 500)).
fof(distance_wp_to_home, axiom, distance(waypoint1, home, 600)).
fof(consumption_rate, axiom, battery_per_meter(0.02)).  % 1% per 50m
fof(safety_margin, axiom, min_battery(20)).

% Battery consumption rule
fof(battery_consumption, axiom,
  ![BAT_START, DIST, RATE, BAT_END]:
    (BAT_END = BAT_START - (DIST * RATE)) =>
    battery_after_travel(BAT_START, DIST, BAT_END)
).

% Mission segments
fof(segment1, axiom,
  battery_after_travel(60, 500, 50)  % 60 - (500 * 0.02) = 50
).

fof(segment2, axiom,
  battery_after_travel(50, 600, 38)  % 50 - (600 * 0.02) = 38
).

% Safety constraint
fof(battery_safe, axiom,
  ![BAT_FINAL, MIN]:
    (final_battery(BAT_FINAL) & min_battery(MIN) & BAT_FINAL >= MIN) =>
    mission_safe
).

fof(final_battery_def, axiom, final_battery(38)).

% Conjecture
fof(mission_battery_safe, conjecture, mission_safe).
```

### Running Vampire

```bash
./bin/vampire --mode casc battery_safety.p
```

**Expected Output:**
```
% SZS status Theorem for battery_safety.p
```

Mission is safe (38% > 20% minimum).

## Example: Altitude Constraint Verification

### Problem Definition

Verify that a mission profile never exceeds FAA Part 107 altitude limit (400 ft AGL).

### TPTP Encoding

```tptp
% File: altitude_check.p

% Mission waypoints with altitudes
fof(wp1_alt, axiom, altitude(waypoint1, 50)).
fof(wp2_alt, axiom, altitude(waypoint2, 100)).
fof(wp3_alt, axiom, altitude(waypoint3, 120)).
fof(wp4_alt, axiom, altitude(waypoint4, 80)).

% All waypoints in mission
fof(mission_wps, axiom,
  ![WP]: in_mission(WP) <=>
    (WP = waypoint1 | WP = waypoint2 | WP = waypoint3 | WP = waypoint4)
).

% FAA limit
fof(faa_limit, axiom, max_altitude(400)).

% Violation rule
fof(altitude_violation, axiom,
  ![WP, ALT, MAX]:
    (in_mission(WP) & altitude(WP, ALT) & max_altitude(MAX) & ALT > MAX) =>
    violates_faa(WP)
).

% Conjecture: No waypoint violates FAA limit
fof(faa_compliant, conjecture,
  ![WP]: (in_mission(WP) => ~violates_faa(WP))
).
```

### Running Vampire

```bash
./bin/vampire --mode casc altitude_check.p
```

## Converting SUMO to TPTP

SUMO axioms can be semi-automatically converted to TPTP:

### SUMO (SUO-KIF)
```lisp
(=>
  (and
    (instance ?UAV UnmannedAerialVehicle)
    (altitude ?UAV ?ALT)
    (greaterThan ?ALT (MeasureFn 120 Meter)))
  (violatesSafetyConstraint ?UAV AltitudeLimit))
```

### TPTP Equivalent
```tptp
fof(altitude_constraint, axiom,
  ![UAV, ALT]:
    (uav(UAV) & altitude(UAV, ALT) & ALT > 120) =>
    violates_safety(UAV)
).
```

### Translation Strategy

1. **Manual Translation**: For critical safety axioms (recommended for verification)
2. **Semi-Automatic**: Use SUMO's built-in TPTP export (if available in Sigma)
3. **Tool-Assisted**: Write Python script to parse `.kif` and generate `.p` files

## Vampire Command-Line Options

```bash
# Basic theorem proving
vampire --mode casc problem.p

# Time limit (in seconds)
vampire --mode casc --time_limit 60 problem.p

# Output proof
vampire --mode casc --proof tptp problem.p

# Verbose output
vampire --mode casc --show_preprocessed on problem.p

# Use specific strategy
vampire --mode casc --saturation_algorithm lrs problem.p
```

## Integration into Mission Planning Workflow

### Phase 1: Planning Mode (Offline)

```
Mission Planner (Python/C++)
    |
    v
Generate TPTP problem
    |
    v
Vampire Verification
    |
    +---> Theorem (SAFE) --> Deploy mission
    |
    +---> CounterSat (UNSAFE) --> Reject mission, alert operator
```

### Example Python Integration

```python
import subprocess

def verify_mission_safety(tptp_file):
    """
    Run Vampire to verify mission safety.
    Returns True if safe, False otherwise.
    """
    result = subprocess.run(
        ['vampire', '--mode', 'casc', '--time_limit', '30', tptp_file],
        capture_output=True,
        text=True
    )

    # Check for "Theorem" in output
    if "SZS status Theorem" in result.stdout:
        return True
    elif "SZS status CounterSatisfiable" in result.stdout:
        return False
    else:
        raise Exception(f"Vampire verification failed: {result.stdout}")

# Usage
if verify_mission_safety("mission_plan.p"):
    print("Mission is safe - deploying to UAV")
else:
    print("Mission violates safety constraints - aborting")
```

## Limitations

1. **Numeric Reasoning**: FOL provers struggle with complex arithmetic (use SMT solvers like Z3 for heavy numerics)
2. **Scalability**: Very large problem sets may timeout (use abstractions)
3. **Real Numbers**: FOL treats reals symbolically (approximations may be needed)

## Best Practices

1. **Simplify Math**: Pre-compute numeric values when possible (e.g., distance calculations)
2. **Modular Verification**: Break complex missions into smaller sub-problems
3. **Timeout Handling**: Set reasonable time limits (30-60 seconds for most UAV problems)
4. **Counterexample Analysis**: If Vampire finds a counterexample, extract it for debugging

## Next Steps

1. **Create TPTP problem files** for your mission scenarios
2. **Automate SUMO → TPTP translation** (see SUMO_TO_PROLOG.md for similar approach)
3. **Integrate with ROS 2 mission planner** (see examples/prolog_query_node.py)
4. **Test with real mission plans** from flyby-f11 scenarios

## References

- Vampire Homepage: https://vprover.github.io/
- TPTP Syntax Guide: http://www.tptp.org/
- CASC Competition: http://www.tptp.org/CASC/
- Vampire GitHub: https://github.com/vprover/vampire
