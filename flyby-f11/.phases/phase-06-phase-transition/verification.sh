#!/bin/bash
# Phase 6: Phase Transition Manager - Verification Script
set -e

echo "========================================"
echo "Phase 6: Phase Transition Manager"
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

echo "=== Package Structure Checks ==="
check "phase_transition package exists" "test -d ros2_ws/src/phase_transition"
check "package.xml exists" "test -f ros2_ws/src/phase_transition/package.xml"
check "transition_controller.py exists" "test -f ros2_ws/src/phase_transition/phase_transition/transition_controller.py"
check "memory_manager.py exists" "test -f ros2_ws/src/phase_transition/phase_transition/memory_manager.py"
check "mission_handoff.py exists" "test -f ros2_ws/src/phase_transition/phase_transition/mission_handoff.py"

echo ""
echo "=== Orchestration Checks ==="
check "transition-compose.yml exists" "test -f containers/compose/transition-compose.yml"

echo ""
echo "=== Message Checks ==="
check "mission messages directory exists" "test -d ros2_ws/src/phase_transition/msg"

echo ""
echo "=== Launch Checks ==="
check "launch directory exists" "test -d ros2_ws/src/phase_transition/launch"

echo ""
echo "=== Test Infrastructure Checks ==="
check "test directory exists" "test -d ros2_ws/src/phase_transition/test"

echo ""
echo "=== Container Checks ==="
check "Planning container exists" "podman images | grep -q flyby-f11-planning"
check "Execution container exists" "podman images | grep -q flyby-f11-execution"

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 6 verification PASSED"
exit 0
