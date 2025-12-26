# Phase 2: UAV Ontology Development

## Overview

Develop a comprehensive UAV domain ontology that formalizes drone concepts, mission types, constraints, and safety properties. This ontology extends SUMO with drone-specific knowledge and serves as the foundation for mission planning and verification.

## Human Description

Build out the UAV knowledge base in KIF (Knowledge Interchange Format) that captures:
- UAV physical properties (mass, dimensions, battery capacity)
- Flight constraints (altitude limits, no-fly zones, weather restrictions)
- Mission types (waypoint navigation, area coverage, inspection)
- Safety properties (collision avoidance, battery reserves, geofencing)
- Environmental factors (wind, visibility, GPS availability)

This ontology will be used by SUMO to reason about mission feasibility, detect conflicts, and verify safety properties before missions are executed.

## AI Agent Instructions

### Prerequisites
- Phase 1 completed (SUMO and Vampire installed)
- Understanding of KIF syntax
- Familiarity with SUMO base concepts

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Create UAV Class Hierarchy**
   - Location: `ontology/planning_mode/uav_domain.kif`
   - Define base classes: UAV, Multirotor, FixedWing
   - Specify physical attributes (mass, dimensions, propulsion)
   - Define capabilities (hover, vertical takeoff, payload)

2. **Define Mission Concepts**
   - Create Mission class hierarchy
   - Define mission types: WaypointMission, SurveyMission, InspectionMission
   - Specify mission parameters (waypoints, coverage area, target objects)
   - Define success criteria for each mission type

3. **Formalize Constraints**
   - Regulatory constraints (altitude limits, restricted zones)
   - Physical constraints (battery endurance, payload capacity)
   - Environmental constraints (wind speed, visibility, temperature)
   - Operational constraints (flight time, range, GPS requirements)

4. **Add Safety Axioms**
   - Collision avoidance rules
   - Battery reserve requirements
   - Geofencing boundaries
   - Emergency procedures (lost link, low battery)
   - Weather minimums (wind, rain, visibility)

5. **Create Test Scenarios**
   - Location: `ontology/planning_mode/test_scenarios/`
   - Define 3-5 test missions with varying complexity
   - Include both valid and invalid missions
   - Document expected reasoning outcomes

6. **Verify with SUMO**
   - Test ontology consistency
   - Run test scenarios through SUMO reasoner
   - Verify axioms are properly encoded
   - Check that safety violations are detected

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `uav_domain.kif` contains comprehensive UAV class hierarchy
- [ ] Mission types properly formalized with constraints
- [ ] Safety axioms correctly detect violations in test cases
- [ ] Ontology passes SUMO consistency checks
- [ ] Test scenarios execute and produce expected results
- [ ] Documentation explains key concepts and design decisions

### Verification

Run automated verification:
```bash
bash .phases/phase-02-uav-ontology/verification.sh
```

### Time Estimate
8-12 hours (includes ontology design, axiom formulation, testing)

### Common Pitfalls

- **Over-engineering**: Start simple, add complexity incrementally
- **KIF syntax errors**: Use SUMO's syntax checker frequently
- **Inconsistent axioms**: Test axioms in isolation before combining
- **Missing edge cases**: Consider unusual scenarios (GPS loss, strong wind, etc.)
- **Underspecified constraints**: Be precise about units, ranges, conditions

### References

- [SUMO KIF Syntax](https://github.com/ontologyportal/sumo/wiki/KIF-Syntax)
- [Ontology Engineering Best Practices](http://www.ontology-of-units-of-measure.org/)
- [FAA Part 107 Regulations](https://www.faa.gov/uas/commercial_operators/part_107) (for constraint formalization)
- [flyby-f11 Literature Review](../literature_review/SYNTHESIS.qmd)

### Dependencies
See `dependencies.json` - requires Phase 1 completion.

### Next Phase
After completion, proceed to Phase 3: SUMO to Prolog Translation
