#!/bin/bash
# Phase 12: Project-Drone Hardware Validation - Verification Script
set -e

echo "========================================"
echo "Phase 12: Project-Drone Hardware Validation"
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

echo "=== Deployment Configuration Checks ==="
check "deployment directory exists" "test -d deployment/project-drone"

echo ""
echo "=== Flight Log Checks ==="
check "flight logs directory exists" "test -d hardware_validation/project-drone/logs"

echo ""
echo "=== Report Checks ==="
check "performance.json exists" "test -f hardware_validation/project-drone/performance.json"
check "test_results.json exists" "test -f hardware_validation/project-drone/test_results.json"
check "SIM_TO_REAL.md exists" "test -f hardware_validation/project-drone/SIM_TO_REAL.md"

echo ""
echo "=== Test Results Checks ==="
if test -f hardware_validation/project-drone/test_results.json; then
    FLIGHTS=$(python3 -c "import json; print(json.load(open('hardware_validation/project-drone/test_results.json'))['successful_flights'])" 2>/dev/null || echo "0")
    COLLISIONS=$(python3 -c "import json; print(json.load(open('hardware_validation/project-drone/test_results.json'))['collision_count'])" 2>/dev/null || echo "999")

    if [ "$FLIGHTS" -ge 10 ]; then
        echo "Checking: 10+ successful flights (actual: ${FLIGHTS})... PASS"
        ((PASS++))
    else
        echo "Checking: 10+ successful flights (actual: ${FLIGHTS})... FAIL"
        ((FAIL++))
    fi

    if [ "$COLLISIONS" -eq 0 ]; then
        echo "Checking: Zero collisions (actual: ${COLLISIONS})... PASS"
        ((PASS++))
    else
        echo "Checking: Zero collisions (actual: ${COLLISIONS})... FAIL"
        ((FAIL++))
    fi
else
    echo "Checking: Test results... SKIP (test_results.json not found)"
fi

echo ""
echo "=== Performance Checks ==="
if test -f hardware_validation/project-drone/performance.json; then
    CPU=$(python3 -c "import json; print(json.load(open('hardware_validation/project-drone/performance.json'))['cpu_utilization_percent'])" 2>/dev/null || echo "100")

    if [ "$(echo "$CPU < 70" | bc -l)" -eq 1 ]; then
        echo "Checking: CPU utilization < 70% (actual: ${CPU}%)... PASS"
        ((PASS++))
    else
        echo "Checking: CPU utilization < 70% (actual: ${CPU}%)... FAIL"
        ((FAIL++))
    fi
else
    echo "Checking: Performance metrics... SKIP (performance.json not found)"
fi

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 12 verification PASSED"
exit 0
