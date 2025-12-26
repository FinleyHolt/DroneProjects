# Ontology and Reasoning Infrastructure

This directory contains the formal reasoning system for mission planning and runtime constraint checking. The system uses a dual-mode architecture optimized for both offline planning (heavyweight) and online execution (lightweight).

## Directory Structure

```
ontology/
├── planning_mode/          # Heavyweight SUMO reasoning (offline)
│   ├── uav_domain.kif      # UAV ontology in KIF format
│   ├── test_scenarios/     # Test missions
│   └── logs/               # Planning logs
├── execution_mode/         # Lightweight Prolog runtime (online)
│   ├── uav_rules.pl        # Compiled mission rules
│   ├── templates/          # Prolog templates
│   └── logs/               # Execution logs
├── translation/            # SUMO → Prolog translator
│   ├── sumo_to_prolog.py   # Translation script
│   ├── TRANSLATION.md      # Translation documentation
│   └── validation/         # Translation validation
├── Containerfile.planning  # Planning mode container
├── Containerfile.execution # Execution mode container
└── README.md              # This file
```

## Two-Mode Architecture

### Planning Mode (Offline, Heavyweight)
- **Tool**: SUMO (Suggested Upper Merged Ontology) + Vampire ATP
- **Purpose**: Mission planning, safety verification, constraint analysis
- **When**: Pre-flight, on-demand
- **Performance**: Seconds to minutes
- **Container**: `flyby-f11-planning`

Planning mode performs:
- Mission feasibility analysis
- Safety property verification
- Constraint conflict detection
- Semantic reasoning about mission goals

### Execution Mode (Online, Lightweight)
- **Tool**: SWI-Prolog
- **Purpose**: Real-time constraint checking, runtime verification
- **When**: During flight, always running
- **Performance**: Microseconds
- **Container**: `flyby-f11-execution`

Execution mode provides:
- Real-time constraint queries (<10ms)
- Mission validity checks
- Safety monitoring
- Emergency decision support

## Development Phases

This directory is built across multiple development phases:

- **Phase 1**: Ontology toolchain setup (SUMO, Vampire)
- **Phase 2**: UAV domain ontology development
- **Phase 3**: SUMO to Prolog translation
- **Phase 4**: Execution mode runtime
- **Phase 5**: Mission planner integration

See [../.phases/](../.phases/) for detailed phase definitions.

## Quick Start

### Planning Mode

```bash
# Start planning container
podman run --rm -it \
  -v ./ontology:/workspace:z \
  flyby-f11-planning:latest

# Inside container - load ontology
cd /workspace/planning_mode
# Run SUMO reasoning...
```

### Execution Mode

```bash
# Start execution container
podman run --rm -it \
  -v ./ontology:/workspace:z \
  flyby-f11-execution:latest

# Inside container - query Prolog
swipl
?- consult('/workspace/execution_mode/uav_rules.pl').
?- mission_valid(waypoint_mission_1).
```

### Translation

```bash
# Translate SUMO ontology to Prolog
python3 translation/sumo_to_prolog.py \
  --input planning_mode/uav_domain.kif \
  --output execution_mode/uav_rules.pl
```

## File Formats

### KIF (Knowledge Interchange Format)
Used in planning mode for SUMO ontology:

```lisp
(subclass UAV Vehicle)
(documentation UAV "Unmanned Aerial Vehicle")
(=>
  (and
    (instance ?u UAV)
    (altitude ?u ?h))
  (lessThan ?h 400))  ; FAA Part 107 altitude limit
```

### Prolog
Used in execution mode for runtime queries:

```prolog
% UAV type hierarchy
uav(X) :- multirotor(X).
uav(X) :- fixed_wing(X).

% Altitude constraint (meters)
max_altitude(400).

% Mission validity check
mission_valid(Mission) :-
    mission_altitude(Mission, Alt),
    max_altitude(MaxAlt),
    Alt =< MaxAlt.
```

## Safety Properties

The ontology encodes critical safety properties:

1. **Altitude Limits**: FAA Part 107 (400ft / 122m AGL)
2. **Geofencing**: No-fly zones, restricted areas
3. **Battery Reserves**: Minimum 20% for return-to-home
4. **Collision Avoidance**: Minimum separation distances
5. **Weather Minimums**: Wind, visibility, precipitation limits

## Testing

```bash
# Test planning mode
podman run --rm \
  -v ./ontology:/workspace:z \
  flyby-f11-planning:latest \
  bash /workspace/planning_mode/test_scenarios/run_tests.sh

# Test execution mode
podman run --rm \
  -v ./ontology:/workspace:z \
  flyby-f11-execution:latest \
  swipl -g "consult('/workspace/execution_mode/uav_rules.pl'), run_tests, halt"
```

## References

- [ONTOLOGY_FOUNDATION.qmd](../ONTOLOGY_FOUNDATION.qmd) - Detailed design rationale
- [APPROACH.qmd](../APPROACH.qmd) - Overall system architecture
- [.phases/phase-01-ontology-toolchain/](../.phases/phase-01-ontology-toolchain/) - Setup instructions
- [SUMO Ontology](https://github.com/ontologyportal/sumo)
- [SWI-Prolog Documentation](https://www.swi-prolog.org/)
