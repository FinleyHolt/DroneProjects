#!/bin/bash
# Phase 6b: Simulation & Training Environment - Automated Verification

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "========================================"
echo "Verifying Phase 6b: Simulation & Training Env"
echo "========================================"
echo ""

PASSED=0
FAILED=0
WARNINGS=0

check() {
    local name="$1"
    local command="$2"

    echo -n "Checking $name... "
    if eval "$command" > /dev/null 2>&1; then
        echo "✓ PASS"
        PASSED=$((PASSED + 1))
        return 0
    else
        echo "✗ FAIL"
        FAILED=$((FAILED + 1))
        return 1
    fi
}

check_optional() {
    local name="$1"
    local command="$2"
    local hint="$3"

    echo -n "Checking $name... "
    if eval "$command" > /dev/null 2>&1; then
        echo "✓ PASS"
        PASSED=$((PASSED + 1))
        return 0
    else
        echo "⚠ WARN ($hint)"
        WARNINGS=$((WARNINGS + 1))
        return 0
    fi
}

# 1. Check ontology_rl package exists
check "ontology_rl package.xml exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/ontology_rl/package.xml"

# 2. Check environment files exist
check "base_env.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/ontology_rl/ontology_rl/envs/base_env.py"

check "mission_planner_env.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/ontology_rl/ontology_rl/envs/mission_planner_env.py"

# 3. Check training world exists
check "training_arena.world exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/training_arena.world"

# 4. Check launch file exists
check "training.launch.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/ontology_rl/launch/training.launch.py"

# 5. Check test files exist
check "test_env_creation.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/ontology_rl/test/test_env_creation.py"

# 6. Check Python syntax for environment files
echo -n "Checking base_env.py syntax... "
if python3 -c "import ast; ast.parse(open('$PROJECT_ROOT/ros2_ws/src/ontology_rl/ontology_rl/envs/base_env.py').read())" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
fi

echo -n "Checking mission_planner_env.py syntax... "
if python3 -c "import ast; ast.parse(open('$PROJECT_ROOT/ros2_ws/src/ontology_rl/ontology_rl/envs/mission_planner_env.py').read())" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
fi

# 7. Check Python dependencies available (optional - may be in container only)
check_optional "gymnasium installed" \
    "python3 -c 'import gymnasium'" \
    "install in container: pip install gymnasium"

check_optional "stable-baselines3 installed" \
    "python3 -c 'import stable_baselines3'" \
    "install in container: pip install stable-baselines3"

# 8. Check training world XML validity
echo -n "Checking training_arena.world XML syntax... "
if xmllint --noout "$PROJECT_ROOT/simulation/worlds/training_arena.world" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL (invalid XML)"
    FAILED=$((FAILED + 1))
fi

# Print summary
echo ""
echo "========================================"
echo "Verification Summary"
echo "========================================"
echo "Passed: $PASSED"
echo "Failed: $FAILED"
echo "Warnings: $WARNINGS"
echo ""

if [ $FAILED -eq 0 ]; then
    echo "✓ Phase 6b verification SUCCESSFUL"
    if [ $WARNINGS -gt 0 ]; then
        echo ""
        echo "Note: $WARNINGS optional dependencies missing on host."
        echo "These are expected to be available inside the container."
    fi
    echo ""
    echo "Next steps:"
    echo "  1. Build package in container:"
    echo "     podman run --rm -v \$(pwd)/ros2_ws:/ros2_ws:z localhost/flyby-f11-sim:latest \\"
    echo "       bash -c 'source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-select ontology_rl'"
    echo "  2. Run environment tests: pytest ros2_ws/src/ontology_rl/test/"
    echo "  3. Launch training environment: ros2 launch ontology_rl training.launch.py"
    echo "  4. Proceed to Phase 7: Mission Planner RL Agent"
    exit 0
else
    echo "✗ Phase 6b verification FAILED"
    echo ""
    echo "Review failed checks above and:"
    echo "  1. Create ontology_rl package with ros2 pkg create"
    echo "  2. Implement base_env.py and mission_planner_env.py"
    echo "  3. Create training_arena.world"
    echo "  4. Install Python dependencies (gymnasium, stable-baselines3)"
    exit 1
fi
