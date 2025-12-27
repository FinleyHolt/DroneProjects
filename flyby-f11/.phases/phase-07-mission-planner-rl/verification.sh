#!/bin/bash
# Phase 7: Mission Planner RL Agent - Verification Script
set -e

echo "========================================"
echo "Phase 7: Mission Planner RL Agent"
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
check "ontology_rl package exists" "test -d ros2_ws/src/ontology_rl"
check "package.xml exists" "test -f ros2_ws/src/ontology_rl/package.xml"

echo ""
echo "=== Environment Checks ==="
check "mission_planner_env.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/envs/mission_planner_env.py"

echo ""
echo "=== Agent Checks ==="
check "mission_planner_sac.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/agents/mission_planner_sac.py"

echo ""
echo "=== Node Checks ==="
check "mission_planner_node.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/nodes/mission_planner_node.py"

echo ""
echo "=== Documentation Checks ==="
check "MDP documentation exists" "test -f ros2_ws/src/ontology_rl/docs/mission_planner_mdp.md"

echo ""
echo "=== Training Infrastructure Checks ==="
check "training directory exists" "test -d ros2_ws/src/ontology_rl/ontology_rl/training"
check "models directory exists" "test -d ros2_ws/src/ontology_rl/models/mission_planner"

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 7 verification PASSED"
exit 0
