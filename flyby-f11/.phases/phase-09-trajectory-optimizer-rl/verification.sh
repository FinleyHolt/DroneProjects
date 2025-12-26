#!/bin/bash
# Phase 9: Trajectory Optimizer RL Agent - Verification Script
set -e

echo "========================================"
echo "Phase 9: Trajectory Optimizer RL Agent"
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
check "trajectory_optimizer_env.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/envs/trajectory_optimizer_env.py"

echo ""
echo "=== Agent Checks ==="
check "trajectory_optimizer_td3.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/agents/trajectory_optimizer_td3.py"

echo ""
echo "=== Node Checks ==="
check "trajectory_optimizer_node.py exists" "test -f ros2_ws/src/ontology_rl/ontology_rl/nodes/trajectory_optimizer_node.py"

echo ""
echo "=== Dynamics Model Checks ==="
check "dynamics directory exists" "test -d ros2_ws/src/ontology_rl/ontology_rl/dynamics"

echo ""
echo "=== Documentation Checks ==="
check "MDP documentation exists" "test -f ros2_ws/src/ontology_rl/docs/trajectory_optimizer_mdp.md"

echo ""
echo "=== Model Checks ==="
check "models directory exists" "test -d ros2_ws/src/ontology_rl/models/trajectory_optimizer"

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 9 verification PASSED"
exit 0
