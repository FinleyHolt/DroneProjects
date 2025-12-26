# KIF to Prolog Translation Notes

## Overview

This document captures the effort required to translate axioms from the KIF (Knowledge Interchange Format) UAV domain ontology (`uav_domain.kif`) to SWI-Prolog (`uav_rules.pl`), along with observations about semantic differences and translation challenges.

**Source**: `ontology/planning_mode/uav_domain.kif` (1213 lines, ~100 axioms)
**Target**: `ontology/evaluation/prolog_benchmark/uav_rules.pl` (20 translated axioms)
**Date**: 2024-12-25

---

## Translation Effort Summary

### Time Spent

| Task | Time |
|------|------|
| Reading and understanding KIF ontology | ~15 minutes |
| Selecting 20 critical safety axioms | ~10 minutes |
| Writing Prolog translations | ~45 minutes |
| Creating test facts and benchmarks | ~20 minutes |
| **Total for 20 axioms** | **~90 minutes** |

### Per-Axiom Effort

- **Average time per axiom**: ~4.5 minutes
- **Simple axioms** (subclass declarations): ~1 minute each
- **Medium axioms** (capability rules with single condition): ~3 minutes each
- **Complex axioms** (safety rules with multiple conditions): ~8-10 minutes each

### Estimated Effort for Full Ontology (~100 axioms)

Using the observed per-axiom times:

| Axiom Complexity | Count (est.) | Time per Axiom | Subtotal |
|------------------|--------------|----------------|----------|
| Simple (subclass, instance) | ~40 | 1 min | 40 min |
| Medium (single implication) | ~30 | 3 min | 90 min |
| Complex (multi-condition) | ~30 | 10 min | 300 min |
| **Total** | **~100** | - | **~7-8 hours** |

**Additional overhead** for a production-quality translation:
- Testing and validation: +2-3 hours
- Documentation: +1 hour
- Edge case handling: +2 hours

**Realistic estimate for full ontology: 12-15 hours of focused work**

---

## What Was Difficult to Translate

### 1. Negation Semantics

**KIF** uses classical negation:
```lisp
(not (safeState ?UAV))
```

**Prolog** uses negation-as-failure:
```prolog
\+ safe_state(UAV)
```

**Impact**: In KIF, `(not X)` is a logical statement that X is false in the model. In Prolog, `\+ X` means "X cannot be proven." This difference matters when:
- Facts are incomplete (open-world vs closed-world assumption)
- Reasoning about unknown states

**Example issue**: If we don't have position quality data for a UAV, Prolog's `\+ has_valid_localization(UAV)` will succeed (triggering `must_land`), while in KIF the truth value might be unknown.

### 2. Universal Quantification

**KIF** has explicit universal quantification:
```lisp
(=> (instance ?UAV Multirotor) (canHover ?UAV))
```

**Prolog** uses implicit universal quantification via unification:
```prolog
can_hover(UAV) :- instance_of(UAV, multirotor).
```

**Impact**: Generally translates well, but KIF's ability to nest quantifiers and mix them with logical connectives doesn't map directly.

### 3. Measure Functions and Units

**KIF** uses SUMO-style measure functions:
```lisp
(MinimumObstacleDistance ?UAV (MeasureFn 3.0 Meter))
```

**Prolog** requires simplification to raw values:
```prolog
minimum_obstacle_distance(UAV, 3.0).
```

**Impact**: Unit information is lost. The Prolog version assumes consistent units (meters for distance, percent for battery, etc.). Unit conversion and dimensional analysis would require significant additional infrastructure.

### 4. Disjunction in Antecedents

**KIF** allows disjunction:
```lisp
(=> (or (instance ?UAV Multirotor) (instance ?UAV HybridVTOL))
    (canVerticalTakeoff ?UAV))
```

**Prolog** requires separate clauses:
```prolog
can_vertical_takeoff(UAV) :- instance_of(UAV, multirotor).
can_vertical_takeoff(UAV) :- instance_of(UAV, hybrid_vtol).
```

**Impact**: Minimal semantic loss, but code becomes more verbose.

### 5. Documentation and Metadata

**KIF** has rich documentation syntax:
```lisp
(documentation canHover EnglishLanguage
  "(canHover ?UAV) means ?UAV is capable of sustained stationary flight.")
```

**Prolog** comments are less structured:
```prolog
%% can_hover(UAV) - true if UAV is capable of sustained stationary flight
```

**Impact**: Formal documentation semantics are lost. Tools that query documentation cannot work the same way.

### 6. Domain and Range Constraints

**KIF** explicitly declares predicate signatures:
```lisp
(instance distanceTo TernaryPredicate)
(domain distanceTo 1 Object)
(domain distanceTo 2 Object)
(domain distanceTo 3 LengthMeasure)
```

**Prolog** has no native type system (in standard Prolog):
```prolog
%% distance_to(+Object, +Object, +Number)
distance_to(uav_alpha, building_1, 50.0).
```

**Impact**: Type safety is lost. Invalid calls like `distance_to(5, "hello", tree)` won't be caught.

---

## What Semantics Were Lost

### 1. Open-World Assumption

**Lost**: KIF operates under the open-world assumption where unknown facts are neither true nor false.

**Prolog behavior**: Closed-world assumption - anything not provable is false.

**Consequence**: A UAV with no battery data will be treated as having no battery (potentially triggering return-to-launch) rather than as having unknown battery status.

### 2. Monotonicity

**Lost**: Classical logic is monotonic (adding facts never invalidates previous conclusions).

**Prolog behavior**: Negation-as-failure is non-monotonic. Adding `has_position_quality(uav_delta, high_quality)` would change `must_land(uav_delta)` from true to false.

### 3. Consistency Checking

**Lost**: FOL reasoners like Vampire can detect logical inconsistencies in the knowledge base.

**Prolog behavior**: Prolog will happily contain contradictory facts without warning:
```prolog
is_within(uav_x, zone_a).
is_outside(uav_x, zone_a).  %% No error raised
```

### 4. Entailment vs Proof Search

**Lost**: FOL entailment (`KB |= Q`) is semantically different from Prolog proof search.

**Example**: In FOL, proving `safe_state(uav)` requires showing it follows from the KB. In Prolog, we attempt to construct a proof, which may fail even if the fact is true (e.g., due to infinite loops or ordering issues).

### 5. Higher-Order Patterns

**Lost**: Some KIF patterns are naturally higher-order:
```lisp
(actionPrecondition NavigateToWaypoint hasValidLocalization)
```

**Prolog workaround**: Use meta-predicates or flatten to first-order:
```prolog
action_precondition(navigate_to_waypoint, has_valid_localization).
has_precondition(Action, UAV) :-
    action_precondition(Action, Precondition),
    call(Precondition, UAV).
```

### 6. Subsumption Reasoning

**Lost**: OWL/Description Logic-style subsumption (is class A a subclass of class B?).

**Prolog workaround**: Manual transitive closure via `subclass_of/2`, but this doesn't support:
- Necessary and sufficient conditions
- Disjoint classes
- Property restrictions

---

## Prolog Advantages

Despite the semantic losses, Prolog offers some advantages:

1. **Performance**: Prolog's SLD resolution is highly optimized. The benchmark shows sub-millisecond query times for most safety checks.

2. **Procedural integration**: Easy to call external code, handle I/O, integrate with ROS systems.

3. **Debugging**: Interactive REPL makes testing and debugging intuitive.

4. **Deterministic execution**: Predictable execution model for real-time safety checks.

5. **Mature ecosystem**: Well-documented, stable, widely deployed.

---

## Recommendations

### When Prolog Translation is Worth It

1. **Runtime safety monitoring**: Where fast, predictable queries are needed during flight
2. **Simple rule-based reasoning**: Class hierarchy, capability inference, basic safety checks
3. **Integration with existing systems**: When Prolog already exists in the stack

### When to Use Vampire/FOL Directly

1. **Design-time verification**: Proving ontology consistency, checking for contradictions
2. **Complex reasoning**: Multi-step inference with universal/existential quantifiers
3. **Formal guarantees**: When you need mathematical proof that safety properties hold
4. **Rich semantics**: When units, documentation, domain constraints matter

### Hybrid Approach (Recommended)

Use **both**:
- **Vampire** for offline ontology validation, mission planning verification, and formal safety proofs
- **Prolog** for runtime safety monitoring with compiled rules derived from the verified ontology

This leverages FOL's semantic richness for verification while using Prolog's performance for real-time operation.

---

## Conclusion

Translating the full UAV ontology to Prolog would require approximately 12-15 hours of careful work. The translation is feasible but loses important semantic properties (open-world assumption, consistency checking, rich type system).

**Verdict**: Manual Prolog translation is **not** worth the effort compared to using Vampire directly for most reasoning tasks. However, a **small subset of performance-critical safety rules** (like those translated here) could reasonably be maintained in Prolog for runtime use, derived from the authoritative KIF/FOL ontology.

The ideal workflow:
1. Maintain the authoritative ontology in KIF
2. Use Vampire for verification and complex queries
3. Auto-generate or manually maintain a minimal Prolog subset for runtime safety checks
4. Validate that Prolog rules are consistent with FOL semantics through testing
