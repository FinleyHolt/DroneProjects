#!/bin/bash
# Phase 8: Behavior Selector RL Agent - Verification Script
set -e

echo "========================================"
echo "Phase 8: Behavior Selector RL Agent"
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

echo "=== Environment Checks ==="
check "behavior_selector_env.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/envs/behavior_selector_env.py"

echo ""
echo "=== Agent Checks ==="
check "behavior_selector_ppo.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/agents/behavior_selector_ppo.py"

echo ""
echo "=== Node Checks ==="
check "behavior_selector_node.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/nodes/behavior_selector_node.py"

echo ""
echo "=== Behavior Definitions Checks ==="
check "behaviors directory exists" "test -d ros2_ws/src/ontology_rl/ontology_rl/behaviors"

echo ""
echo "=== Documentation Checks ==="
check "MDP documentation exists" "test -f ros2_ws/src/ontology_rl/docs/behavior_selector_mdp.md"

echo ""
echo "=== Model Checks ==="
check "models directory exists" "test -d ros2_ws/src/ontology_rl/models/behavior_selector"

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 8 verification PASSED"
exit 0
