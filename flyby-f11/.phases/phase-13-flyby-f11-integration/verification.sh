#!/bin/bash
# Phase 13: Flyby-F11 Integration - Verification Script
set -e

echo "========================================"
echo "Phase 13: Flyby-F11 Integration"
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
check "flyby-f11 deployment exists" "test -d deployment/flyby-f11"
check "mission configs exist" "test -d deployment/flyby-f11/missions"

echo ""
echo "=== MCTSSA Demo Checks ==="
check "MCTSSA demo package exists" "test -d deployment/flyby-f11/mctssa_demo"

echo ""
echo "=== Compliance Checks ==="
check "compliance documentation exists" "test -d deployment/flyby-f11/compliance"

echo ""
echo "=== Flight Log Checks ==="
check "flyby-f11 logs directory exists" "test -d hardware_validation/flyby-f11/logs"
check "test_results.json exists" "test -f hardware_validation/flyby-f11/test_results.json"

echo ""
echo "=== Test Results Checks ==="
if test -f hardware_validation/flyby-f11/test_results.json; then
    OUTDOOR=$(python3 -c "import json; print(json.load(open('hardware_validation/flyby-f11/test_results.json'))['outdoor_flight_count'])" 2>/dev/null || echo "0")
    COMMS_DENIED=$(python3 -c "import json; print(json.load(open('hardware_validation/flyby-f11/test_results.json'))['comms_denied_duration_minutes'])" 2>/dev/null || echo "0")

    if [ "$OUTDOOR" -ge 5 ]; then
        echo "Checking: 5+ outdoor flights (actual: ${OUTDOOR})... PASS"
        ((PASS++))
    else
        echo "Checking: 5+ outdoor flights (actual: ${OUTDOOR})... FAIL"
        ((FAIL++))
    fi

    if [ "$COMMS_DENIED" -ge 5 ]; then
        echo "Checking: 5+ min comms-denied (actual: ${COMMS_DENIED} min)... PASS"
        ((PASS++))
    else
        echo "Checking: 5+ min comms-denied (actual: ${COMMS_DENIED} min)... FAIL"
        ((FAIL++))
    fi
else
    echo "Checking: Test results... SKIP (test_results.json not found)"
fi

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 13 verification PASSED"
exit 0
