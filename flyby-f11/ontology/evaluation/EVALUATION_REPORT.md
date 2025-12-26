# Phase 3: Runtime Reasoner Evaluation Report

**Author:** Finley Holt
**Date:** 2024-12-25
**Platform:** Flyby F-11 UAV (Jetson Orin NX 16GB)

## Executive Summary

This report presents empirical benchmarking results for candidate reasoning architectures for the Flyby F-11 UAV autonomous mission system. The evaluation tests the hypothesis that a "heavyweight" theorem prover (Vampire) may actually meet real-time requirements for our small domain ontology, potentially eliminating the need for lossy translation to lighter-weight reasoners.

### Key Finding

**[TO BE FILLED: Summary of which reasoner meets requirements]**

### Recommendation

**[TO BE FILLED: Clear recommendation with evidence]**

---

## 1. Introduction

### 1.1 Evaluation Objectives

1. Determine if SUMO + Vampire can meet real-time requirements (<10ms for safety queries)
2. Quantify translation losses when converting to OWL or Prolog
3. Compare performance across candidate reasoners
4. Identify the optimal architecture for flight-critical reasoning

### 1.2 Real-Time Requirements

| Query Category | Latency Requirement (p95) | Rationale |
|----------------|---------------------------|-----------|
| Safety-Critical | < 10ms | Must complete within flight control loop |
| Operational | < 100ms | Acceptable during active flight |
| Planning | < 1000ms | Acceptable during mission planning |

### 1.3 Hardware Targets

- **Development:** x86_64 Linux (this evaluation)
- **Deployment:** NVIDIA Jetson Orin NX 16GB (ARM aarch64)
  - 50 TOPS AI performance
  - 16GB unified memory
  - 8-core ARM CPU

---

## 2. Methodology

### 2.1 Benchmark Query Set

A standardized set of queries representing real flight operations:

**Safety-Critical Queries (6 total):**
- Geofence boundary check
- No-fly zone violation detection
- Battery reserve return check
- Collision imminent detection
- Critical sensor failure detection
- Safe state composite check

**Operational Queries (5 total):**
- Valid waypoint sequence
- Hover capability check
- Terrain traversable
- Weather constraint check
- NDAA compliance check

**Planning Queries (4 total):**
- Mission feasibility
- Regulatory compliance (FAA Part 107)
- Path safety analysis
- Capability matching

### 2.2 Benchmark Protocol

For each query and reasoner:
1. Cold start measurement (first query after process start)
2. 100 warm iterations for statistical validity
3. Metrics collected: min, max, mean, median, p95, p99
4. Memory usage tracked where possible

### 2.3 Ontology Size

- **KIF/SUMO format:** ~1200 lines, 15 sections
- **Classes defined:** ~50 (UAV types, sensors, regions, etc.)
- **Safety axioms:** 10+ critical rules
- **Properties/Relations:** ~40

---

## 3. Results

### 3.1 Vampire (SUMO + FOL) Results

| Query Category | Mean (ms) | p95 (ms) | p99 (ms) | Meets Requirement |
|----------------|-----------|----------|----------|-------------------|
| Safety-Critical | [TBD] | [TBD] | [TBD] | [TBD] |
| Operational | [TBD] | [TBD] | [TBD] | [TBD] |
| Planning | [TBD] | [TBD] | [TBD] | [TBD] |

**Cold Start:** [TBD] ms
**Peak Memory:** [TBD] MB

**Notes:**
- Full first-order logic expressivity
- No translation loss
- Process startup overhead

### 3.2 ELK (OWL 2 EL) Results

| Query Category | Mean (ms) | p95 (ms) | p99 (ms) | Meets Requirement |
|----------------|-----------|----------|----------|-------------------|
| Safety-Critical | [TBD] | [TBD] | [TBD] | [TBD] |
| Operational | [TBD] | [TBD] | [TBD] | [TBD] |
| Planning | [TBD] | [TBD] | [TBD] | [TBD] |

**Cold Start (JVM):** [TBD] ms
**Peak Memory:** [TBD] MB

**Translation Loss:**
- [TBD: What axioms couldn't be expressed]

### 3.3 Reasonable (OWL 2 RL) Results

| Query Category | Mean (ms) | p95 (ms) | p99 (ms) | Meets Requirement |
|----------------|-----------|----------|----------|-------------------|
| Safety-Critical | [TBD] | [TBD] | [TBD] | [TBD] |
| Operational | [TBD] | [TBD] | [TBD] | [TBD] |
| Planning | [TBD] | [TBD] | [TBD] | [TBD] |

**Cold Start:** [TBD] ms
**Peak Memory:** [TBD] MB

**Translation Loss:**
- [TBD: What axioms couldn't be expressed]

### 3.4 SWI-Prolog Results

| Query Category | Mean (ms) | p95 (ms) | p99 (ms) | Meets Requirement |
|----------------|-----------|----------|----------|-------------------|
| Safety-Critical | [TBD] | [TBD] | [TBD] | [TBD] |
| Operational | [TBD] | [TBD] | [TBD] | [TBD] |
| Planning | [TBD] | [TBD] | [TBD] | [TBD] |

**Cold Start:** [TBD] ms
**Peak Memory:** [TBD] MB

**Translation Effort:**
- Axioms translated: 15-20
- Time to translate: [TBD]
- Estimated full translation: [TBD]

---

## 4. Translation Loss Analysis

### 4.1 OWL Export Losses

| Safety Axiom | Status | Notes |
|--------------|--------|-------|
| Geofence violation | [PRESERVED/LOST/MODIFIED] | [Notes] |
| No-fly zone violation | [PRESERVED/LOST/MODIFIED] | [Notes] |
| Battery reserve | [PRESERVED/LOST/MODIFIED] | [Notes] |
| Collision detection | [PRESERVED/LOST/MODIFIED] | [Notes] |
| Lost localization | [PRESERVED/LOST/MODIFIED] | [Notes] |
| Weather constraints | [PRESERVED/LOST/MODIFIED] | [Notes] |

### 4.2 Prolog Translation Losses

| Aspect | Preservation | Notes |
|--------|--------------|-------|
| Class hierarchy | High | Direct mapping |
| Safety rules | [TBD] | [Notes] |
| Quantified axioms | [TBD] | [Notes] |
| Numerical constraints | [TBD] | [Notes] |

---

## 5. Decision Matrix

| Criterion | Weight | Vampire | ELK | Reasonable | Prolog |
|-----------|--------|---------|-----|------------|--------|
| Safety Query p95 | 30% | [TBD] | [TBD] | [TBD] | [TBD] |
| Memory Usage | 15% | [TBD] | [TBD] | [TBD] | [TBD] |
| Translation Loss | 25% | 100% | [TBD] | [TBD] | [TBD] |
| ARM Build Feasibility | 15% | [TBD] | [TBD] | [TBD] | [TBD] |
| Maintenance Burden | 15% | Low | [TBD] | [TBD] | [TBD] |
| **Weighted Score** | 100% | [TBD] | [TBD] | [TBD] | [TBD] |

---

## 6. ARM/Jetson Considerations

### 6.1 Build Status

| Reasoner | x86_64 Build | aarch64 Build | Notes |
|----------|--------------|---------------|-------|
| Vampire | ✓ | [TBD] | [Notes] |
| ELK | ✓ (JVM) | ✓ (OpenJDK) | Platform-independent |
| Reasonable | ✓ | [TBD] | Rust cross-compile |
| SWI-Prolog | ✓ | ✓ | Official ARM packages |

### 6.2 Expected Performance Variance

ARM performance may differ from x86 results due to:
- Different instruction sets
- Cache sizes and memory bandwidth
- Thermal throttling in embedded environment

**Recommendation:** Validate final choice on actual Jetson hardware before deployment.

---

## 7. Recommendation

### 7.1 Primary Recommendation

**[TO BE FILLED: Clear recommendation]**

**Rationale:**
1. [Reason 1]
2. [Reason 2]
3. [Reason 3]

### 7.2 Alternative Approaches

If the primary recommendation proves insufficient:
1. [Fallback option 1]
2. [Fallback option 2]

### 7.3 Impact on Downstream Phases

**If Vampire is recommended:**
- Phase 4 simplifies dramatically (no translation needed)
- Runtime uses Vampire directly in container
- Phases 5-6 may be eliminated or significantly reduced

**If OWL reasoner is recommended:**
- Phase 4 becomes OWL integration
- Use Sigma export or manual OWL conversion
- Update perception bridge for OWL

**If Prolog is recommended:**
- Proceed with original Phase 4 design
- Full manual translation required
- Estimate [TBD] hours of work

---

## 8. Conclusion

[TO BE FILLED: Final summary]

---

## Appendix A: Raw Benchmark Data

See `vampire_benchmark/results.json`, `elk_benchmark/results.json`, etc.

## Appendix B: Test Queries

All benchmark queries available in `benchmark_queries/`.

## Appendix C: Container Specifications

- `Containerfile.planning` - Vampire/SUMO environment
- `elk_benchmark/Containerfile` - ELK/OpenJDK
- `reasonable_benchmark/Containerfile` - Rust/Reasonable
- `prolog_benchmark/Containerfile` - SWI-Prolog
