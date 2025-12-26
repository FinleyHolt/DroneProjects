# OWL 2 RL Profile Limitations

This document describes the OWL 2 RL (Rule Language) profile limitations and their impact on translating the UAV domain ontology from TPTP/FOL to OWL.

## Overview

OWL 2 RL is a syntactic subset of OWL 2 designed for:
- **Efficient rule-based reasoning** using forward chaining (materialization)
- **Polynomial time complexity** for standard reasoning tasks
- **Database integration** via translation to Datalog rules
- **Scalability** to large datasets (millions of triples)

Reasonable is a Rust-based OWL 2 RL reasoner that implements the RL ruleset.

## OWL 2 RL Supported Constructs

### Fully Supported

| Construct | Example | Status |
|-----------|---------|--------|
| Class assertions | `uav(flyby_f11)` | Supported |
| Subclass axioms | `Multirotor SubClassOf UAV` | Supported |
| Object properties | `hasLocation(uav, region)` | Supported |
| Data properties | `batteryLevel(uav, 75)` | Supported |
| Property domains/ranges | `hasSensor domain UAV` | Supported |
| Property characteristics | `transitive`, `symmetric` | Supported |
| Inverse properties | `inverseOf` | Supported |
| Property chains | `uncle = parent o brother` | Supported |
| HasValue restrictions | `hasColor value Red` | Supported |
| SomeValuesFrom (limited) | See limitations below | Partial |
| AllValuesFrom (limited) | See limitations below | Partial |
| Intersection | `UAV and NDAA_Compliant` | Supported |
| OneOf (nominals) | `{high, medium, low}` | Supported |

### Not Supported (OWL 2 RL Restrictions)

| Construct | Example | Workaround |
|-----------|---------|------------|
| Universal restrictions (head) | `∀hasComponent.Sensor` in superclass | Use rules or split |
| Existential restrictions (head) | `∃hasPilot.Human` in superclass | Requires materialization |
| Cardinality (unrestricted) | `>= 4 rotors` | Use closed-world assumption |
| Self restrictions | `loves Self` | Manual handling |
| Disjunction (head) | `A or B` | Split into multiple axioms |
| Negation (complex) | `not (A and B)` | Limited support |
| Reflexive properties (global) | `reflexive(related)` | Local assertions only |

## Translation from TPTP/FOL to OWL 2 RL

### Axioms That Translate Well

Our UAV domain ontology axioms that map cleanly to OWL 2 RL:

#### Type Hierarchy
```tptp
% Original TPTP
fof(multirotor_is_uav, axiom, ![X]: (multirotor(X) => uav(X))).
```
```owl
<!-- OWL 2 RL equivalent -->
<SubClassOf>
    <Class IRI="#Multirotor"/>
    <Class IRI="#UAV"/>
</SubClassOf>
```

#### Property Restrictions (Body Position)
```tptp
% Original: All multirotors can hover
fof(multirotor_hover, axiom, ![UAV]: (multirotor(UAV) => can_hover(UAV))).
```
```owl
<!-- OWL 2 RL: Subclass with property assertion -->
<SubClassOf>
    <Class IRI="#Multirotor"/>
    <ObjectHasValue>
        <ObjectProperty IRI="#canHover"/>
        <NamedIndividual IRI="#true"/>
    </ObjectHasValue>
</SubClassOf>
```

#### Safety Rules (Simple Implications)
```tptp
% Battery reserve requires RTL
fof(battery_reserve, axiom,
    ![UAV]: ((uav(UAV) & battery_below_reserve(UAV)) => must_return_to_launch(UAV))).
```
```owl
<!-- Translates to intersection in subclass -->
<SubClassOf>
    <ObjectIntersectionOf>
        <Class IRI="#UAV"/>
        <ObjectHasValue>
            <ObjectProperty IRI="#batteryBelowReserve"/>
            <NamedIndividual IRI="#true"/>
        </ObjectHasValue>
    </ObjectIntersectionOf>
    <ObjectHasValue>
        <ObjectProperty IRI="#mustAction"/>
        <NamedIndividual IRI="#ReturnToLaunch"/>
    </ObjectHasValue>
</SubClassOf>
```

### Axioms Requiring Modification

#### Complex Conjunctions
```tptp
% Original: Multi-condition geofence violation
fof(geofence_violation, axiom,
    ![UAV, Mission, Geofence]:
      ((uav(UAV) & mission(Mission) & assigned_uav(Mission, UAV) &
        has_mission_area(Mission, Geofence) & geofence(Geofence) &
        is_outside(UAV, Geofence))
       => geofence_violation(UAV))).
```

**OWL 2 RL Challenge**: Complex n-ary relations and existential chains.

**Workaround**: Flatten to property chains or use SWRL rules (if reasoner supports them).

#### Negation in Body
```tptp
% Safe state requires NO violations
fof(safe_state, axiom,
    ![UAV]: ((uav(UAV) & ~nfz_violation(UAV) & ~collision_imminent(UAV))
             => safe_state(UAV))).
```

**OWL 2 RL Challenge**: Negation as failure (not supported directly).

**Workaround**:
1. Use complementOf with closed-world assumption
2. Materialize violation states explicitly
3. Use SPARQL FILTER NOT EXISTS queries

#### Disjunction in Consequent
```tptp
% Emergency can trigger multiple responses
fof(emergency_response, axiom,
    ![UAV]: (emergency(UAV) => (land(UAV) | return_to_launch(UAV)))).
```

**OWL 2 RL Restriction**: Disjunction not allowed in superclass position.

**Workaround**: Define emergency response as abstract action, then specialize.

### Expected Translation Losses

| Axiom Category | TPTP Coverage | OWL 2 RL Coverage | Notes |
|----------------|---------------|-------------------|-------|
| Type hierarchy | 100% | 100% | Full support |
| Simple implications | 100% | 95% | Minor restructuring needed |
| Safety constraints | 100% | 80% | Negation requires workarounds |
| N-ary relations | 100% | 60% | Reification needed |
| Temporal reasoning | 100% | 30% | Limited temporal support |
| Quantified variables | 100% | 70% | Role restrictions are limited |

## Performance Characteristics

### Reasonable vs Vampire Comparison

| Metric | Vampire (TPTP) | Reasonable (OWL 2 RL) |
|--------|----------------|----------------------|
| Reasoning paradigm | Refutation | Forward chaining |
| Time complexity | Undecidable | Polynomial |
| Query style | Prove/disprove | Materialization + query |
| Memory | Grows with proof search | Proportional to data |
| Incremental updates | Full re-proof | Delta materialization |
| Latency | Query-time | Mostly pre-computed |

### Expected Benchmark Performance

For OWL 2 RL materialization followed by SPARQL queries:

| Query Category | Target p95 | Expected p95 | Notes |
|----------------|------------|--------------|-------|
| Safety | <10ms | 1-5ms | Simple class membership |
| Operational | <100ms | 10-50ms | Property chains |
| Planning | <1000ms | 50-200ms | Complex joins |

**Note**: Reasonable claims 7x speedup over Allegro GraphDB for certain workloads. Actual performance depends on:
- Ontology size (number of axioms)
- Data size (number of individuals)
- Query complexity
- Hardware (ARM vs x86)

## Recommendations for UAV Ontology

### Keep in OWL 2 RL
1. Type hierarchy (UAV, Multirotor, FlybyF11)
2. Simple capability implications (can_hover, can_vtol)
3. Property assertions (hasLocation, hasSensor)
4. Binary safety flags (geofence_violation, low_battery)

### Consider SWRL Extensions
1. Complex multi-variable rules
2. Temporal constraints (before, after, during)
3. Numeric comparisons (battery > threshold)

### Handle in Application Layer
1. Real-time sensor fusion
2. Path planning algorithms
3. Control loop decisions
4. Complex temporal reasoning

## Alternative Reasoners

If OWL 2 RL limitations are too restrictive:

| Reasoner | Profile | Language | Notes |
|----------|---------|----------|-------|
| Reasonable | OWL 2 RL | Rust | Fast, embedded-friendly |
| RDFox | OWL 2 RL + Datalog | C++ | Commercial, very fast |
| ELK | OWL 2 EL | Java | Good for large hierarchies |
| HermiT | OWL 2 DL | Java | Full expressivity, slower |
| Pellet | OWL 2 DL | Java | SWRL support |
| Vampire | FOL | C++ | Full first-order logic |

## References

1. OWL 2 Web Ontology Language Profiles: https://www.w3.org/TR/owl2-profiles/
2. Reasonable GitHub: https://github.com/gtfierro/reasonable
3. OWL 2 RL in RDF Graphs: https://www.w3.org/TR/owl2-profiles/#OWL_2_RL
4. W3C OWL 2 RL Rules: https://www.w3.org/TR/owl2-profiles/#Reasoning_in_OWL_2_RL_and_RDF_Graphs_using_Rules
