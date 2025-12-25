# Ontology and Reasoning Tools Comparison

Detailed comparison of reasoning tools for autonomous drone decision-making in the flyby-f11 project.

## Overview

This document compares five critical reasoning tools across multiple dimensions relevant to embedded, real-time autonomous systems.

## Quick Comparison Matrix

| Tool | Type | Runtime | ROS 2 Integration | Real-time | Use Case |
|------|------|---------|-------------------|-----------|----------|
| **SUMO** | Ontology | N/A | Via reasoner | N/A | Domain modeling |
| **SWI-Prolog** | Logic Programming | Embeddable C++ | Excellent | Suitable | Runtime reasoning |
| **Vampire** | FOL Theorem Prover | Standalone | Via subprocess | No | Pre-flight verification |
| **Clingo** | ASP Solver | Python/C++ API | Good | Moderate | Mission planning |
| **E-Prover** | Equational Prover | Standalone | Via subprocess | No | Formal verification |

## Detailed Tool Analysis

### 1. SUMO (Suggested Upper Merged Ontology)

**Type**: Upper ontology / knowledge base

**Strengths**:
- Comprehensive top-level concepts (physical objects, processes, events)
- Spatial reasoning primitives (Region, SpatialRelation, Distance)
- Temporal logic (TimePoint, TimeInterval)
- Military domain coverage (relevant for defense applications)
- Well-documented with 25,000+ axioms

**Weaknesses**:
- Not executable on its own (requires reasoner)
- Heavy-weight for embedded systems
- Requires translation to executable logic format

**Integration Strategy**:
1. Extract relevant subset for drone domain
2. Convert to Prolog facts/rules for SWI-Prolog
3. Use as semantic foundation for mission-intent interpretation
4. Reference for developing custom domain ontology

**Recommended Subset**:
- `Merge.kif`: Core concepts (Object, Process, Agent)
- `Spatial.kif`: Spatial reasoning (Region, Trajectory)
- `Mid-level-ontology.kif`: Intermediate concepts
- Custom extracts for aviation/navigation

**ROS 2 Integration**: Indirect (via Prolog knowledge base)

---

### 2. SWI-Prolog

**Type**: Logic programming environment

**Strengths**:
- Embeddable in C/C++ via FFI
- Excellent ROS 2 integration potential
- Tabled resolution for memoization (performance)
- Constraint logic programming (CLP) for optimization
- Active development and community
- Low latency for simple queries (<1ms)

**Weaknesses**:
- Non-determinism can cause unbounded execution time
- Memory usage can grow with large fact bases
- Requires careful tuning for hard real-time constraints

**Integration Strategy**:
1. Create ROS 2 C++ node with embedded Prolog engine
2. Load SUMO-derived knowledge base at startup
3. Expose reasoning services via ROS 2 actions/services
4. Use for:
   - Mission intent parsing
   - Spatial query answering
   - Communications-denied decision-making
   - Behavior tree precondition evaluation

**Performance Considerations**:
- Use tabling for repeated queries
- Limit search depth with iterative deepening
- Pre-compile critical rules for speed
- Monitor query time; timeout long-running queries

**ROS 2 Integration**: Excellent (native C++ embedding)

**Example Use Case**:
```prolog
% Query: Is waypoint safe given current sensor data?
safe_waypoint(WP) :-
    not(obstacle_in_path(current_position, WP)),
    altitude(WP, Alt), Alt > minimum_safe_altitude,
    within_geofence(WP).
```

---

### 3. Vampire Theorem Prover

**Type**: First-order logic (FOL) automated theorem prover

**Strengths**:
- State-of-the-art FOL proving (CASC competition winner)
- Can prove complex logical statements
- Finds proofs or counterexamples
- Good for safety verification

**Weaknesses**:
- Not real-time (proving can take seconds to minutes)
- Requires expertise in FOL syntax (TPTP format)
- Standalone tool (subprocess invocation)
- Undecidable problems may not terminate

**Integration Strategy**:
1. Pre-flight verification only (not runtime)
2. Use to verify:
   - Mission plan consistency (no contradictory goals)
   - Safety properties (collision avoidance proofs)
   - Geofence compliance
3. Invoke via ROS 2 service during mission planning
4. Timeout if proof takes >5 seconds

**Example Verification**:
```tptp
% Verify: If all waypoints are safe, the mission is safe
fof(safe_mission, conjecture,
    (![WP]: (member(WP, mission_waypoints) => safe(WP)))
    => safe_mission).
```

**ROS 2 Integration**: Via subprocess (command-line tool)

**Recommended Use**: Offline verification, not runtime

---

### 4. Clingo (Potassco Answer Set Programming)

**Type**: ASP solver for declarative problem solving

**Strengths**:
- Declarative problem specification (logic rules)
- Generates optimal solutions (minimize/maximize objectives)
- Python and C++ APIs
- Good for planning under constraints
- Fast for moderate-sized problems (<1000 atoms)

**Weaknesses**:
- Solving time can be unpredictable
- Not suitable for hard real-time
- Requires learning ASP syntax

**Integration Strategy**:
1. Use for mission planning:
   - Route optimization (minimize fuel/time)
   - Task allocation (multi-drone coordination)
   - Contingency planning (alternative routes)
2. Offline planning preferred; runtime planning with timeouts
3. Python API integration via ROS 2 Python node
4. Generate behavior tree from ASP solution

**Example ASP Program**:
```asp
% Define waypoints
waypoint(wp1). waypoint(wp2). waypoint(wp3).

% Define edges with costs
edge(wp1, wp2, 10). edge(wp2, wp3, 15).

% Generate path visiting all waypoints
{ visit(WP, T) : waypoint(WP) } = 1 :- time(T).

% Minimize total cost
#minimize { C,WP1,WP2 : visit(WP1,T), visit(WP2,T+1), edge(WP1,WP2,C) }.
```

**ROS 2 Integration**: Good (Python API via rclpy, C++ via libclingo)

**Recommended Use**: Mission planning, offline or with timeouts

---

### 5. E-Prover

**Type**: Equational first-order logic theorem prover

**Strengths**:
- Optimized for equational reasoning
- Fast for certain problem classes
- Good documentation
- Complementary to Vampire (different strategies)

**Weaknesses**:
- Similar limitations to Vampire (not real-time)
- Standalone tool (subprocess)
- Requires FOL expertise

**Integration Strategy**:
1. Similar to Vampire: pre-flight verification
2. Use for:
   - Mathematical proofs (trajectory safety)
   - Geometric constraint solving
   - Algebraic properties of flight dynamics
3. May be faster than Vampire for equational problems

**ROS 2 Integration**: Via subprocess (command-line tool)

**Recommended Use**: Offline verification, complementary to Vampire

---

## Recommended Tool Stack for flyby-f11

### Phase 1 (Weeks 1-6): Foundation
**Focus**: SUMO + SWI-Prolog

1. **SUMO Ontology**:
   - Extract drone-relevant concepts
   - Build custom domain ontology
   - Convert to Prolog knowledge base

2. **SWI-Prolog**:
   - Embed in ROS 2 C++ node
   - Implement basic reasoning services
   - Prototype mission-intent queries

**Deliverable**: ROS 2 node providing Prolog reasoning services

---

### Phase 2 (Weeks 7-12): Planning
**Focus**: Clingo integration

1. **Clingo ASP Solver**:
   - Develop ASP mission planning models
   - Integrate via Python ROS 2 node
   - Generate behavior trees from ASP solutions

**Deliverable**: ASP-based mission planner

---

### Phase 3 (Weeks 13-18): Verification
**Focus**: Vampire + E-Prover

1. **Formal Verification**:
   - Formalize safety properties in FOL
   - Implement pre-flight verification service
   - Prove mission plan consistency

**Deliverable**: Automated safety verification system

---

## Integration Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    ROS 2 Autonomy Stack                 │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐      ┌───────────────┐              │
│  │  Mission     │─────▶│  Prolog       │              │
│  │  Manager     │      │  Reasoner     │              │
│  │  (C++)       │      │  (SWI-Prolog) │              │
│  └──────────────┘      └───────────────┘              │
│         │                      ▲                        │
│         │                      │                        │
│         │              ┌───────┴────────┐              │
│         │              │  SUMO-derived  │              │
│         │              │  Knowledge Base│              │
│         │              └────────────────┘              │
│         ▼                                               │
│  ┌──────────────┐                                      │
│  │  ASP Planner │                                      │
│  │  (Clingo)    │                                      │
│  └──────────────┘                                      │
│         │                                               │
│         ▼                                               │
│  ┌──────────────┐      ┌───────────────┐              │
│  │  Behavior    │      │  Verification │              │
│  │  Tree        │◀─────│  Service      │              │
│  │  Executor    │      │  (Vampire/E)  │              │
│  └──────────────┘      └───────────────┘              │
│         │                                               │
└─────────┼───────────────────────────────────────────────┘
          ▼
    ┌──────────────┐
    │  PX4 Flight  │
    │  Controller  │
    └──────────────┘
```

## Performance Targets

| Tool | Context | Latency Target | Memory Target |
|------|---------|----------------|---------------|
| SWI-Prolog | Runtime query | <10ms | <100MB |
| Clingo | Mission planning | <1s | <200MB |
| Vampire | Pre-flight verify | <5s | <500MB |
| E-Prover | Pre-flight verify | <5s | <500MB |

## Tool Selection Decision Tree

```
Is this a runtime decision?
├─ YES → Use SWI-Prolog
│        (embedded reasoning)
│
└─ NO → Is this planning/optimization?
        ├─ YES → Use Clingo
        │        (ASP-based planning)
        │
        └─ NO → Is this verification?
                └─ YES → Use Vampire/E-Prover
                         (formal verification)
```

## Development Priorities

### High Priority (Weeks 1-6)
1. SWI-Prolog embedding
2. SUMO subset extraction
3. Basic reasoning queries

### Medium Priority (Weeks 7-12)
1. Clingo integration
2. ASP mission models
3. Plan generation

### Lower Priority (Weeks 13-18)
1. Vampire verification
2. E-Prover integration
3. Formal proofs

## Further Reading

- **SUMO**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ontology/sumo/`
- **SWI-Prolog**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ontology/swi-prolog-docs/`
- **Clingo**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ontology/clingo/examples/`
- **Vampire**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ontology/vampire/README.md`
- **E-Prover**: `/home/finley/Github/DroneProjects/flyby-f11/docs/ontology/eprover/DOC/`

## Conclusion

**Recommended Stack**:
1. **SWI-Prolog** as primary runtime reasoner (embedded in ROS 2)
2. **SUMO** as ontological foundation (converted to Prolog)
3. **Clingo** for mission planning (Python ROS 2 node)
4. **Vampire** for pre-flight verification (subprocess)
5. **E-Prover** as backup verifier (subprocess)

This combination provides:
- ✅ Runtime reasoning (Prolog)
- ✅ Declarative planning (Clingo)
- ✅ Formal verification (Vampire/E)
- ✅ Semantic grounding (SUMO)
- ✅ ROS 2 integration (all tools)
