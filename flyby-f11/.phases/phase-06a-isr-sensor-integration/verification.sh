#!/bin/bash
# Phase 6a: ISR Sensor Integration - Automated Verification

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "========================================"
echo "Verifying Phase 6a: ISR Sensor Integration"
echo "========================================"
echo ""

PASSED=0
FAILED=0

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

# 1. Check flyby_f11_sensors package exists
check "flyby_f11_sensors package.xml exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_sensors/package.xml"

# 2. Check gimbal controller node exists
check "gimbal_controller_node.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_sensors/flyby_f11_sensors/gimbal_controller_node.py"

# 3. Check message definitions exist
check "GimbalCommand.msg exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_sensors/msg/GimbalCommand.msg"

check "GimbalState.msg exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_sensors/msg/GimbalState.msg"

# 4. Check flyby_f11_bringup package exists
check "flyby_f11_bringup package.xml exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_bringup/package.xml"

# 5. Check launch files exist
check "simulation.launch.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_bringup/launch/simulation.launch.py"

# 6. Check Python syntax for gimbal controller
echo -n "Checking gimbal_controller_node.py syntax... "
if python3 -c "import ast; ast.parse(open('$PROJECT_ROOT/ros2_ws/src/flyby_f11_sensors/flyby_f11_sensors/gimbal_controller_node.py').read())" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
fi

# 7. Check gimbal limits match ontology
echo -n "Checking gimbal limits match ontology... "
if grep -q "6.02" "$PROJECT_ROOT/ros2_ws/src/flyby_f11_sensors/flyby_f11_sensors/gimbal_controller_node.py" 2>/dev/null && \
   grep -q "2.09" "$PROJECT_ROOT/ros2_ws/src/flyby_f11_sensors/flyby_f11_sensors/gimbal_controller_node.py" 2>/dev/null && \
   grep -q "0.785" "$PROJECT_ROOT/ros2_ws/src/flyby_f11_sensors/flyby_f11_sensors/gimbal_controller_node.py" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL (gimbal limits don't match isr_extensions.kif)"
    FAILED=$((FAILED + 1))
fi

# 8. Check Gazebo models exist (input requirement)
check "f11_isr_camera model exists" \
    "test -f $PROJECT_ROOT/simulation/models/f11_isr_camera/model.sdf"

check "f11_lidar model exists" \
    "test -f $PROJECT_ROOT/simulation/models/f11_lidar/model.sdf"

# Print summary
echo ""
echo "========================================"
echo "Verification Summary"
echo "========================================"
echo "Passed: $PASSED"
echo "Failed: $FAILED"
echo ""

if [ $FAILED -eq 0 ]; then
    echo "✓ Phase 6a verification SUCCESSFUL"
    echo ""
    echo "Next steps:"
    echo "  1. Test gimbal controller in simulation"
    echo "  2. Verify sensor topics with ros2 topic list"
    echo "  3. Proceed to Phase 6b: Simulation & Training Environment"
    exit 0
else
    echo "✗ Phase 6a verification FAILED"
    echo ""
    echo "Review failed checks above and:"
    echo "  1. Create missing packages with ros2 pkg create"
    echo "  2. Implement gimbal controller node"
    echo "  3. Create message definitions"
    echo "  4. Create launch files"
    exit 1
fi
