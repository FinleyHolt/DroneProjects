#!/bin/bash
# Phase 10: AGGM Integration - Verification Script
set -e

echo "========================================"
echo "Phase 10: AGGM Integration"
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

echo "=== AGGM Module Checks ==="
check "aggm directory exists" "test -d ros2_ws/src/ontology_rl/ontology_rl/aggm"
check "aggm_core.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/aggm/aggm_core.py"
check "change_detector.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/aggm/change_detector.py"
check "goal_reasoner.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/aggm/goal_reasoner.py"
check "importance_weighting.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/aggm/importance_weighting.py"

echo ""
echo "=== Node Checks ==="
check "aggm_node.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/nodes/aggm_node.py"

echo ""
echo "=== Message Checks ==="
check "Goal.msg exists" "test -f ros2_ws/src/ontology_rl/msg/Goal.msg"

echo ""
echo "=== Test Checks ==="
check "AGGM test directory exists" "test -d ros2_ws/src/ontology_rl/test/aggm"

echo ""
echo "=== Documentation Checks ==="
check "AGGM integration docs exist" "test -f ros2_ws/src/ontology_rl/docs/aggm_integration.md"

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 10 verification PASSED"
exit 0
