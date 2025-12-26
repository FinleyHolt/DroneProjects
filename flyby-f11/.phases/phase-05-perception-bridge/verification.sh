#!/bin/bash
# Phase 5: Perception-to-Reasoning Bridge - Verification Script
set -e

echo "========================================"
echo "Phase 5: Perception-to-Reasoning Bridge"
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
check "perception_grounding package exists" "test -d ros2_ws/src/perception_grounding"
check "package.xml exists" "test -f ros2_ws/src/perception_grounding/package.xml"
check "setup.py exists" "test -f ros2_ws/src/perception_grounding/setup.py"

echo ""
echo "=== Node Files Checks ==="
check "object_grounding_node.py exists" "test -f ros2_ws/src/perception_grounding/perception_grounding/object_grounding_node.py"
check "terrain_grounding_node.py exists" "test -f ros2_ws/src/perception_grounding/perception_grounding/terrain_grounding_node.py"
check "spatial_relation_node.py exists" "test -f ros2_ws/src/perception_grounding/perception_grounding/spatial_relation_node.py"
check "event_detection_node.py exists" "test -f ros2_ws/src/perception_grounding/perception_grounding/event_detection_node.py"

echo ""
echo "=== Launch and Message Checks ==="
check "launch file exists" "test -f ros2_ws/src/perception_grounding/launch/grounding.launch.py"
check "custom messages directory exists" "test -d ros2_ws/src/perception_grounding/msg"

echo ""
echo "=== Test Infrastructure Checks ==="
check "test directory exists" "test -d ros2_ws/src/perception_grounding/test"
check "test data exists" "test -d ros2_ws/src/perception_grounding/test/data"

echo ""
echo "=== Build Check ==="
# Check if package builds (requires ROS 2 environment)
if command -v colcon &> /dev/null; then
    cd ros2_ws
    check "Package builds successfully" "colcon build --packages-select perception_grounding"
    cd "$PROJECT_ROOT"
else
    echo "Checking: Package builds... SKIP (colcon not found, likely not in ROS container)"
fi

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 5 verification PASSED"
exit 0
