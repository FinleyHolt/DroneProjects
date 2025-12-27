#!/bin/bash
# Phase 11: Simulation Benchmark Suite - Verification Script
set -e

echo "========================================"
echo "Phase 11: Simulation Benchmark Suite"
echo "Verification Script"
echo "========================================"
echo ""

PHASE_DIR="$(dirname "$0")"
PROJECT_ROOT="$(cd "$PHASE_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

PASS=0
FAIL=0

check() {
    local description="$1"
    local command="$2"

    echo -n "Checking: $description... "
    if eval "$command" > /dev/null 2>&1; then
        echo "PASS"
        ((PASS++))
    else
        echo "FAIL"
        ((FAIL++))
    fi
}

echo "=== Scenario Checks ==="
check "scenarios directory exists" "test -d benchmark/scenarios"
check "waypoint_easy.yaml exists" "test -f benchmark/scenarios/waypoint_easy.yaml"
check "waypoint_medium.yaml exists" "test -f benchmark/scenarios/waypoint_medium.yaml"
check "waypoint_hard.yaml exists" "test -f benchmark/scenarios/waypoint_hard.yaml"
check "obstacle_static.yaml exists" "test -f benchmark/scenarios/obstacle_static.yaml"
check "gps_denied.yaml exists" "test -f benchmark/scenarios/gps_denied.yaml"

echo ""
echo "=== Metrics Checks ==="
check "metrics directory exists" "test -d benchmark/metrics"

echo ""
echo "=== Baselines Checks ==="
check "baselines directory exists" "test -d benchmark/baselines"

echo ""
echo "=== Evaluation Framework Checks ==="
check "eval directory exists" "test -d benchmark/eval"

echo ""
echo "=== Results Checks ==="
check "results directory exists" "test -d benchmark/results"
check "summary.json exists" "test -f benchmark/results/summary.json"
check "figures directory exists" "test -d benchmark/results/figures"

echo ""
echo "=== Performance Target Checks ==="
if test -f benchmark/results/summary.json; then
    SR=$(python3 -c "import json; print(json.load(open('benchmark/results/summary.json'))['success_rate_percent'])" 2>/dev/null || echo "0")
    CR=$(python3 -c "import json; print(json.load(open('benchmark/results/summary.json'))['collision_rate_percent'])" 2>/dev/null || echo "100")
    SV=$(python3 -c "import json; print(json.load(open('benchmark/results/summary.json'))['constraint_violations'])" 2>/dev/null || echo "999")

    if [ "$(echo "$SR > 50" | bc -l)" -eq 1 ]; then
        echo "Checking: Success Rate > 50% (actual: ${SR}%)... PASS"
        ((PASS++))
    else
        echo "Checking: Success Rate > 50% (actual: ${SR}%)... FAIL"
        ((FAIL++))
    fi

    if [ "$(echo "$CR < 10" | bc -l)" -eq 1 ]; then
        echo "Checking: Collision Rate < 10% (actual: ${CR}%)... PASS"
        ((PASS++))
    else
        echo "Checking: Collision Rate < 10% (actual: ${CR}%)... FAIL"
        ((FAIL++))
    fi

    if [ "$SV" -eq 0 ]; then
        echo "Checking: Safety Violations = 0 (actual: ${SV})... PASS"
        ((PASS++))
    else
        echo "Checking: Safety Violations = 0 (actual: ${SV})... FAIL"
        ((FAIL++))
    fi
else
    echo "Checking: Performance targets... SKIP (summary.json not found)"
fi

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 11 verification PASSED"
exit 0
