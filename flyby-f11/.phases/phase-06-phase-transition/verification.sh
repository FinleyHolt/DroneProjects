#!/bin/bash
# Phase 6: Mission Orchestration - Verification Script
# Updated for simplified single-reasoner architecture (no container swapping)
set -e

echo "========================================"
echo "Phase 6: Mission Orchestration"
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
        PASS=$((PASS + 1))
    else
        echo "FAIL"
        FAIL=$((FAIL + 1))
    fi
}

echo "=== Package Structure Checks ==="
check "mission_orchestrator package exists" "test -d ros2_ws/src/mission_orchestrator"
check "package.xml exists" "test -f ros2_ws/src/mission_orchestrator/package.xml"
check "CMakeLists.txt exists" "test -f ros2_ws/src/mission_orchestrator/CMakeLists.txt"

echo ""
echo "=== Core Module Checks ==="
check "mission_state_machine.py exists" "test -f ros2_ws/src/mission_orchestrator/mission_orchestrator/mission_state_machine.py"
check "mission_manager_node.py exists" "test -f ros2_ws/src/mission_orchestrator/mission_orchestrator/mission_manager_node.py"
check "safety_monitor.py exists" "test -f ros2_ws/src/mission_orchestrator/mission_orchestrator/safety_monitor.py"
check "__init__.py exists" "test -f ros2_ws/src/mission_orchestrator/mission_orchestrator/__init__.py"

echo ""
echo "=== Message/Service/Action Checks ==="
check "msg directory exists" "test -d ros2_ws/src/mission_orchestrator/msg"
check "srv directory exists" "test -d ros2_ws/src/mission_orchestrator/srv"
check "action directory exists" "test -d ros2_ws/src/mission_orchestrator/action"
check "Waypoint.msg exists" "test -f ros2_ws/src/mission_orchestrator/msg/Waypoint.msg"
check "MissionPlan.msg exists" "test -f ros2_ws/src/mission_orchestrator/msg/MissionPlan.msg"
check "MissionStatus.msg exists" "test -f ros2_ws/src/mission_orchestrator/msg/MissionStatus.msg"
check "SafetyState.msg exists" "test -f ros2_ws/src/mission_orchestrator/msg/SafetyState.msg"
check "VerifyMission.srv exists" "test -f ros2_ws/src/mission_orchestrator/srv/VerifyMission.srv"
check "StartMission.srv exists" "test -f ros2_ws/src/mission_orchestrator/srv/StartMission.srv"
check "ExecuteMission.action exists" "test -f ros2_ws/src/mission_orchestrator/action/ExecuteMission.action"

echo ""
echo "=== Launch and Config Checks ==="
check "launch directory exists" "test -d ros2_ws/src/mission_orchestrator/launch"
check "config directory exists" "test -d ros2_ws/src/mission_orchestrator/config"
check "launch file exists" "test -f ros2_ws/src/mission_orchestrator/launch/mission_orchestrator.launch.py"
check "params file exists" "test -f ros2_ws/src/mission_orchestrator/config/orchestrator_params.yaml"

echo ""
echo "=== Test Infrastructure Checks ==="
check "test directory exists" "test -d ros2_ws/src/mission_orchestrator/test"
check "state machine tests exist" "test -f ros2_ws/src/mission_orchestrator/test/test_mission_state_machine.py"
check "lifecycle tests exist" "test -f ros2_ws/src/mission_orchestrator/test/test_mission_lifecycle.py"

echo ""
echo "=== Quadlet Deployment Checks ==="
check "quadlet directory exists" "test -d quadlet"
check "mission-orchestrator.container exists" "test -f quadlet/mission-orchestrator.container"
check "vampire-reasoning.container exists" "test -f quadlet/vampire-reasoning.container"

echo ""
echo "=== Dependency Checks ==="
check "vampire_bridge package exists" "test -d ros2_ws/src/vampire_bridge"
check "VampireQuery service defined" "test -f ros2_ws/src/vampire_bridge/srv/VampireQuery.srv"

echo ""
echo "=== Code Quality Checks ==="
check "State machine has all states" "grep -q 'IDLE\|RECEIVING\|VERIFYING\|VERIFIED\|ARMING\|EXECUTING\|PAUSED\|RETURNING\|LANDING\|COMPLETED\|ABORTED' ros2_ws/src/mission_orchestrator/mission_orchestrator/mission_state_machine.py"
check "Safety monitor has 20Hz check" "grep -q 'safety_check_hz\|20' ros2_ws/src/mission_orchestrator/config/orchestrator_params.yaml"
check "Planning timeout is 5000ms" "grep -q 'planning_timeout_ms.*5000' ros2_ws/src/mission_orchestrator/config/orchestrator_params.yaml"
check "Tactical timeout is 100ms" "grep -q 'tactical_timeout_ms.*100' ros2_ws/src/mission_orchestrator/config/orchestrator_params.yaml"

echo ""
echo "=== Unit Test Execution ==="
MICROMAMBA="/home/finley/.local/bin/micromamba"
if [ -x "$MICROMAMBA" ]; then
    echo "Running state machine unit tests (via micromamba)..."
    if $MICROMAMBA run -n flyby-f11-eval python3 -m pytest ros2_ws/src/mission_orchestrator/test/test_mission_state_machine.py -v --tb=short 2>/dev/null; then
        echo "PASS: State machine tests passed"
        PASS=$((PASS + 1))
    else
        echo "FAIL: State machine tests failed"
        FAIL=$((FAIL + 1))
    fi
elif command -v python3 &> /dev/null && python3 -c "import pytest" 2>/dev/null; then
    echo "Running state machine unit tests..."
    if python3 -m pytest ros2_ws/src/mission_orchestrator/test/test_mission_state_machine.py -v --tb=short 2>/dev/null; then
        echo "PASS: State machine tests passed"
        PASS=$((PASS + 1))
    else
        echo "FAIL: State machine tests failed"
        FAIL=$((FAIL + 1))
    fi
else
    echo "SKIP: pytest not available (install via micromamba or pip)"
fi

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 6 verification PASSED"
echo ""
echo "Next steps:"
echo "  1. Build the package: colcon build --packages-select mission_orchestrator"
echo "  2. Run integration tests with vampire_bridge"
echo "  3. Deploy to Jetson using Quadlet containers"
exit 0
