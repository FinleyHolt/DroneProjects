#!/bin/bash
# Phase 3: Runtime Reasoner Evaluation - Automated Verification
# This script verifies that the evaluation benchmarks were completed

# Don't exit on error - we track failures manually
set +e

PHASE_NAME="phase-03-evaluation"
PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
EVAL_DIR="$PROJECT_ROOT/ontology/evaluation"

echo "========================================"
echo "Verifying Phase 3: Runtime Reasoner Evaluation"
echo "========================================"
echo ""

# Track verification status
PASSED=0
FAILED=0
WARNINGS=0

# Helper function for checks
check() {
    local name="$1"
    local command="$2"

    echo -n "Checking $name... "
    if eval "$command" > /dev/null 2>&1; then
        echo "✓ PASS"
        ((PASSED++))
        return 0
    else
        echo "✗ FAIL"
        ((FAILED++))
        return 1
    fi
}

# Helper function for warnings (optional checks)
warn_check() {
    local name="$1"
    local command="$2"

    echo -n "Checking $name... "
    if eval "$command" > /dev/null 2>&1; then
        echo "✓ PASS"
        ((PASSED++))
        return 0
    else
        echo "⚠ WARNING (optional)"
        ((WARNINGS++))
        return 1
    fi
}

echo "--- Benchmark Query Set ---"

# 1. Check benchmark queries directory exists
check "benchmark_queries directory exists" \
    "test -d $EVAL_DIR/benchmark_queries"

# 2. Check for query files (individual TPTP files, not consolidated)
check "safety query files exist" \
    "ls $EVAL_DIR/benchmark_queries/safety_*.tptp 2>/dev/null | wc -l | grep -q '[1-9]'"

check "operational query files exist" \
    "ls $EVAL_DIR/benchmark_queries/operational_*.tptp 2>/dev/null | wc -l | grep -q '[1-9]'"

check "planning query files exist" \
    "ls $EVAL_DIR/benchmark_queries/planning_*.tptp 2>/dev/null | wc -l | grep -q '[1-9]'"

echo ""
echo "--- Vampire Benchmark ---"

# 3. Check Vampire benchmark results
check "vampire_benchmark directory exists" \
    "test -d $EVAL_DIR/vampire_benchmark"

check "vampire results.json exists" \
    "test -f $EVAL_DIR/vampire_benchmark/results.json"

# 4. Verify Vampire results contain required metrics
echo -n "Checking Vampire results contain latency metrics... "
if grep -q '"p95_ms"' "$EVAL_DIR/vampire_benchmark/results.json" 2>/dev/null || \
   grep -q '"p95_latency_ms"' "$EVAL_DIR/vampire_benchmark/results.json" 2>/dev/null; then
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL (missing p95 metrics)"
    ((FAILED++))
fi

echo ""
echo "--- OWL Export ---"

# 5. Check OWL export
check "owl_export directory exists" \
    "test -d $EVAL_DIR/owl_export"

check "uav_domain.owl exists" \
    "test -f $EVAL_DIR/owl_export/uav_domain.owl"

check "translation_report.md exists" \
    "test -f $EVAL_DIR/owl_export/translation_report.md"

check "axiom_preservation_matrix.json exists" \
    "test -f $EVAL_DIR/owl_export/axiom_preservation_matrix.json"

echo ""
echo "--- OWL Reasoner Benchmarks ---"

# 6. Check at least 2 OWL reasoners were benchmarked
OWL_REASONERS_TESTED=0

if test -d "$EVAL_DIR/elk_benchmark" && test -f "$EVAL_DIR/elk_benchmark/results.json"; then
    echo "Checking ELK benchmark... ✓ PASS"
    ((PASSED++))
    ((OWL_REASONERS_TESTED++))
else
    echo "Checking ELK benchmark... ✗ FAIL"
    ((FAILED++))
fi

if test -d "$EVAL_DIR/reasonable_benchmark" && test -f "$EVAL_DIR/reasonable_benchmark/results.json"; then
    echo "Checking Reasonable benchmark... ✓ PASS"
    ((PASSED++))
    ((OWL_REASONERS_TESTED++))
else
    echo "Checking Reasonable benchmark... ✗ FAIL"
    ((FAILED++))
fi

# LiRot is optional
if test -d "$EVAL_DIR/lirot_benchmark" && test -f "$EVAL_DIR/lirot_benchmark/results.json"; then
    echo "Checking LiRot benchmark... ✓ PASS (optional)"
    ((PASSED++))
    ((OWL_REASONERS_TESTED++))
else
    echo "Checking LiRot benchmark... ⊘ SKIP (optional)"
fi

echo -n "Checking at least 2 OWL reasoners tested... "
if [ $OWL_REASONERS_TESTED -ge 2 ]; then
    echo "✓ PASS ($OWL_REASONERS_TESTED tested)"
    ((PASSED++))
else
    echo "✗ FAIL (only $OWL_REASONERS_TESTED tested, need 2+)"
    ((FAILED++))
fi

echo ""
echo "--- Prolog Benchmark ---"

# 7. Check Prolog benchmark
check "prolog_benchmark directory exists" \
    "test -d $EVAL_DIR/prolog_benchmark"

check "prolog results.json exists" \
    "test -f $EVAL_DIR/prolog_benchmark/results.json"

check "uav_rules.pl exists" \
    "test -f $EVAL_DIR/prolog_benchmark/uav_rules.pl"

echo ""
echo "--- Cross-Platform Validation ---"

# 8. Check cross-platform results
check "cross_platform directory exists" \
    "test -d $EVAL_DIR/cross_platform"

check "x86_results.json exists" \
    "test -f $EVAL_DIR/cross_platform/x86_results.json"

# ARM results might be from QEMU emulation
warn_check "arm_results.json exists" \
    "test -f $EVAL_DIR/cross_platform/arm_results.json"

echo ""
echo "--- Final Deliverables ---"

# 9. Check evaluation report (either .qmd source or .pdf output)
check "EVALUATION_REPORT exists (.qmd or .pdf)" \
    "test -f $EVAL_DIR/EVALUATION_REPORT.qmd || test -f $EVAL_DIR/EVALUATION_REPORT.pdf"

# 10. Check report has substantial content
echo -n "Checking EVALUATION_REPORT has substantial content... "
if [ -f "$EVAL_DIR/EVALUATION_REPORT.qmd" ]; then
    FILE_SIZE=$(stat -c%s "$EVAL_DIR/EVALUATION_REPORT.qmd" 2>/dev/null || echo 0)
elif [ -f "$EVAL_DIR/EVALUATION_REPORT.pdf" ]; then
    FILE_SIZE=$(stat -c%s "$EVAL_DIR/EVALUATION_REPORT.pdf" 2>/dev/null || echo 0)
else
    FILE_SIZE=0
fi
if [ "$FILE_SIZE" -gt 5120 ]; then  # At least 5KB
    echo "✓ PASS ($((FILE_SIZE / 1024))KB)"
    ((PASSED++))
else
    echo "✗ FAIL (only $((FILE_SIZE / 1024))KB, expected 5KB+)"
    ((FAILED++))
fi

# 11. Check decision matrix
check "decision_matrix.json exists" \
    "test -f $EVAL_DIR/decision_matrix.json"

# 12. Check report contains recommendation
echo -n "Checking EVALUATION_REPORT contains architectural decision... "
if grep -qi "architectural" "$EVAL_DIR/EVALUATION_REPORT.qmd" 2>/dev/null || \
   grep -qi "recommend" "$EVAL_DIR/EVALUATION_REPORT.qmd" 2>/dev/null; then
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL (no recommendation found)"
    ((FAILED++))
fi

echo ""
echo "--- Performance Requirements ---"

# 13. Check safety-critical query performance
echo -n "Checking safety-critical query latency <10ms... "
if test -f "$EVAL_DIR/decision_matrix.json"; then
    # Extract best safety-critical latency from decision matrix
    BEST_LATENCY=$(python3 -c "
import json
try:
    with open('$EVAL_DIR/decision_matrix.json') as f:
        data = json.load(f)
    latencies = []
    for reasoner, metrics in data.get('reasoners', {}).items():
        if 'safety_critical_p95_ms' in metrics:
            latencies.append(metrics['safety_critical_p95_ms'])
    if latencies:
        print(min(latencies))
    else:
        print('N/A')
except:
    print('N/A')
" 2>/dev/null)

    if [ "$BEST_LATENCY" != "N/A" ]; then
        if [ $(echo "$BEST_LATENCY < 10" | bc -l 2>/dev/null || echo 0) -eq 1 ]; then
            echo "✓ PASS (best: ${BEST_LATENCY}ms)"
            ((PASSED++))
        else
            echo "⚠ WARNING (best: ${BEST_LATENCY}ms, target <10ms)"
            ((WARNINGS++))
        fi
    else
        echo "⊘ SKIP (could not parse latency)"
    fi
else
    echo "⊘ SKIP (decision_matrix.json not found)"
fi

# Print summary
echo ""
echo "========================================"
echo "Verification Summary"
echo "========================================"
echo "Passed:   $PASSED"
echo "Failed:   $FAILED"
echo "Warnings: $WARNINGS"
echo ""

if [ $FAILED -eq 0 ]; then
    echo "✓ Phase 3 verification SUCCESSFUL"
    echo ""
    echo "Key deliverables verified:"
    echo "  ✓ Benchmark query set defined"
    echo "  ✓ Vampire (SUMO + FOL) benchmarked"
    echo "  ✓ OWL export and translation analysis complete"
    echo "  ✓ At least 2 OWL reasoners benchmarked"
    echo "  ✓ SWI-Prolog benchmarked"
    echo "  ✓ Cross-platform results available"
    echo "  ✓ Evaluation report with recommendation"
    echo ""
    echo "Next steps:"
    echo "  1. Review EVALUATION_REPORT.md for architecture recommendation"
    echo "  2. Update downstream phases based on recommendation"
    echo "  3. If Vampire is fast enough: consider eliminating translation phase"
    echo "  4. If OWL reasoner wins: update Phase 4 for OWL integration"
    echo "  5. If Prolog wins: proceed with original translation plan"
    echo "  6. Run: bash scripts/next-phase.sh"
    exit 0
else
    echo "✗ Phase 3 verification FAILED"
    echo ""
    echo "Review failed checks above and ensure:"
    echo "  1. All reasoner candidates were benchmarked"
    echo "  2. Benchmark results contain required metrics"
    echo "  3. OWL export translation losses are documented"
    echo "  4. Cross-platform (ARM) testing was attempted"
    echo "  5. EVALUATION_REPORT.md contains clear recommendation"
    echo "  6. See TASK.md for detailed requirements"
    exit 1
fi
