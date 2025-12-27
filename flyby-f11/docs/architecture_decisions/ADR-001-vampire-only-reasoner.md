# ADR-001: Single-Reasoner Architecture (Vampire Only)

**Status:** Accepted
**Date:** 2024-12-25
**Decision Makers:** Finley Holt

## Context

The Flyby F-11 UAV autonomy platform requires runtime reasoning for flight safety, mission planning, and regulatory compliance. The UAV Domain Ontology encodes critical safety axioms in SUMO/KIF format using first-order logic.

Phase 3 evaluated four candidate reasoners:
- **Vampire** - Full FOL theorem prover (C++)
- **ELK** - OWL 2 EL reasoner (Java)
- **Reasonable** - OWL 2 RL reasoner (Rust/Python)
- **SWI-Prolog** - Horn clause logic (C)

## Decision

**We select Vampire as the single reasoning engine for both offline planning and runtime tactical reasoning.**

## Rationale

### Why Not OWL Reasoners (ELK, Reasonable)?

OWL-based reasoners **cannot express our safety axioms** due to fundamental limitations of Description Logic:

| Safety Axiom | OWL Status |
|--------------|------------|
| Geofence violation | LOST - FOL quantification not expressible |
| No-fly zone violation | LOST - Conditional inference not supported |
| Battery reserve check | LOST - Numeric comparison not possible |
| Collision detection | LOST - Distance comparison not expressible |

**60% translation loss** represents complete loss of all conditional safety rules.

### Why Not Prolog (Despite 4,700x Performance Advantage)?

Prolog benchmarks demonstrated exceptional performance:
- **0.007ms** average for safety queries (vs Vampire's 41ms)
- All latency requirements met with significant margin

However, Prolog was rejected because:

| Factor | Impact |
|--------|--------|
| Translation work | 12-15 hours initial + ongoing maintenance |
| Semantic loss | ~30% (negation-as-failure differs from classical negation) |
| Dual maintenance | Two representations (KIF + Prolog) to keep in sync |
| Semantic drift risk | Prolog rules may diverge from KIF over time |

### Why Vampire ~50ms is Acceptable

The key insight is that **ontological reasoning belongs in the navigation layer, not the control layer**:

| Tier | Layer | Latency | Function |
|------|-------|---------|----------|
| 1 | Classical Control | <1ms | PID, motor control, attitude |
| 2 | Pre-computed Safety | <10ms | Obstacle buffers, geofence boundaries |
| 3 | Tactical Reasoning | ~50ms | "Am I violating NFZ?", "Battery critical?" |
| 4 | Mission Planning | ~100ms-1s | Route planning, regulatory compliance |

Real-time safety (<10ms) is handled by the classical control layer. Ontological reasoning queries the semantic state at 20Hz navigation rate, where 48ms latency fits within the 50ms period.

## Consequences

### Positive

1. **KIF/SUMO remains single source of truth** - No translation layer to maintain
2. **Zero semantic loss** - Full first-order logic expressivity preserved
3. **Simplified architecture** - One reasoner for both planning and runtime
4. **Reduced operational complexity** - No dual-representation synchronization

### Negative

1. **ARM performance unknown** - Vampire ARM64 build untested on Jetson
2. **Higher latency** - 48ms vs sub-millisecond (mitigated by tiered architecture)
3. **No fallback** - If Vampire fails on ARM, must implement Prolog translation

### Mitigation Strategies

**Query Caching:**
- Pre-flight verification of all safety axioms
- Cached geofence/NFZ boundaries as static facts
- Incremental updates only when state changes significantly
- Parallel execution of independent safety checks

**ARM Validation (Deferred):**
- Validate on actual Jetson Orin NX hardware before production
- If performance degrades >100ms, fallback to Prolog translation

## Alternatives Considered

1. **Prolog Runtime + Vampire Verification** - Rejected due to translation maintenance burden
2. **OWL 2 RL with SWRL Rules** - Rejected due to fundamental expressivity limitations
3. **Custom C++ Safety Monitor** - Deferred as fallback if ARM validation fails

## References

- [EVALUATION_REPORT.qmd](../../ontology/evaluation/EVALUATION_REPORT.qmd) - Complete benchmark results
- [decision_matrix.json](../../ontology/evaluation/decision_matrix.json) - Weighted scoring
- [APPROACH.qmd](../../APPROACH.qmd) - Updated architecture document
