# Phase 3: Runtime Reasoner Evaluation [COMPLETED]

## Status: COMPLETE

**Completion Date:** 2024-12-25
**Final Report:** [ontology/evaluation/EVALUATION_REPORT.qmd](../../ontology/evaluation/EVALUATION_REPORT.qmd)

---

## Architectural Decision: Single-Reasoner Architecture (Vampire Only)

After comprehensive empirical benchmarking of all candidate reasoning approaches, we determined that a **single-reasoner architecture using Vampire** is optimal for the Flyby F-11 platform.

### Key Benchmark Results

| Reasoner | Safety p95 | Operational p95 | Translation Loss | Suitable |
|----------|------------|-----------------|------------------|----------|
| Vampire  | 48ms       | 48ms            | 0%               | **YES**  |
| Prolog   | 0.01ms     | 0.02ms          | ~30%             | Rejected |
| ELK      | N/A        | ~3ms            | ~60%             | NO       |
| Reasonable | N/A      | ~5ms            | ~60%             | NO       |

### Critical Findings

1. **OWL Reasoners Unsuitable**: ELK and Reasonable cannot express our safety axioms at all due to Description Logic limitations. The ~60% translation loss represents complete loss of all conditional safety rules.

2. **Prolog Rejected Despite Performance**: While Prolog achieves 4,700x faster query times (0.007ms vs 48ms), the architectural decision rejects Prolog because:
   - 12-15 hours translation work required
   - ~30% semantic loss (negation-as-failure vs classical negation)
   - Ongoing maintenance of two representations
   - Risk of semantic drift between KIF and Prolog

3. **Vampire ~50ms is Acceptable**: The key insight is that **ontological reasoning belongs in the navigation layer, not the control layer**:

### Tiered Safety Architecture

| Tier | Layer | Latency | Function | Reasoner |
|------|-------|---------|----------|----------|
| 1 | Classical Control | <1ms | PID, motor control, attitude | None |
| 2 | Pre-computed Safety | <10ms | Obstacle buffers, geofence boundaries | Costmaps (no runtime reasoning) |
| 3 | Tactical Reasoning | ~50ms | "Am I violating NFZ?", "Battery critical?" | **Vampire** |
| 4 | Mission Planning | ~100ms-1s | Route planning, regulatory compliance | **Vampire** |

**Rationale**: Real-time safety (<10ms) is handled by the classical control layer (PID, obstacle buffers, velocity limits), not symbolic reasoning. Ontological reasoning queries run at 20Hz navigation rate, where 48ms latency is acceptable (20Hz = 50ms period).

### Final Recommendation

**Use Vampire as the single reasoning engine for both:**
- **Offline planning** - Mission verification, safety proofs (unlimited time)
- **Runtime tactical reasoning** - Safety queries at 20Hz navigation rate (~50ms acceptable)

**Benefits:**
- KIF/SUMO remains single source of truth
- No translation layer to maintain
- Full first-order logic expressivity
- Zero semantic loss

---

## Original Evaluation Objectives [COMPLETED]

1. ✅ Determine if Vampire meets real-time requirements
2. ✅ Quantify translation losses when converting to OWL or Prolog
3. ✅ Compare performance across candidate reasoners
4. ✅ Identify the optimal architecture for flight-critical reasoning

## Benchmark Infrastructure Created

- `ontology/evaluation/benchmark_queries/` - 15 standardized queries (safety, operational, planning)
- `ontology/evaluation/vampire_benchmark/` - Full benchmark results with 100 iterations per query
- `ontology/evaluation/prolog_benchmark/` - Prolog comparison benchmarks
- `ontology/evaluation/owl_export/` - OWL translation loss analysis
- `ontology/evaluation/decision_matrix.json` - Weighted scoring across all criteria
- `ontology/evaluation/visualizations/` - Charts and figures

## Impact on Downstream Phases

**Phase 4** (Vampire Runtime Integration):
- Replaces original "Execution Mode" (which planned Prolog runtime)
- Focus: Vampire ARM64 cross-compilation, ROS 2 integration, query caching

**Phase 5** (Perception Bridge):
- Use Vampire for symbolic reasoning instead of Prolog
- Assert facts via TPTP input files
- Query via Vampire subprocess

**Phase 6** (Phase Transition):
- Simplified: No longer need to swap between Planning Mode and Execution Mode
- Vampire runs in both modes (different query types, same engine)

---

## References

- [EVALUATION_REPORT.qmd](../../ontology/evaluation/EVALUATION_REPORT.qmd) - Complete evaluation report with all benchmarks
- [decision_matrix.json](../../ontology/evaluation/decision_matrix.json) - Weighted decision matrix
- [APPROACH.qmd](../../APPROACH.qmd) - Updated architecture document

---

## Verification

Phase 3 is considered complete. All benchmark artifacts exist:

```bash
bash .phases/phase-03-evaluation/verification.sh
```
