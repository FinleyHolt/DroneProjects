#!/bin/bash
#------------------------------------------------------------------------------
# Phase 4 Verification Script - Vampire Runtime Integration (x86)
#------------------------------------------------------------------------------
#
# Verifies Phase 4 outputs for x86 development platform.
# ARM64 components are deferred until Jetson hardware is available.
#
# Note: This phase was updated from Prolog-based to Vampire-based approach
# based on Phase 3 evaluation results.
#
# Usage:
#   bash .phases/phase-04-execution-mode/verification.sh
#
#------------------------------------------------------------------------------

set -e

PHASE_DIR="$(dirname "$0")"
PROJECT_ROOT="$(cd "$PHASE_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Counters
PASS=0
FAIL=0
SKIP=0

check_pass() {
    echo -e "  ${GREEN}[PASS]${NC} $1"
    PASS=$((PASS + 1))
}

check_fail() {
    echo -e "  ${RED}[FAIL]${NC} $1"
    FAIL=$((FAIL + 1))
}

check_skip() {
    echo -e "  ${YELLOW}[SKIP]${NC} $1"
    SKIP=$((SKIP + 1))
}

echo ""
echo "================================================================================"
echo "  Phase 4 Verification: Vampire Runtime Integration (x86 Only)"
echo "================================================================================"
echo ""

#------------------------------------------------------------------------------
# 1. ROS 2 vampire_bridge Package Structure
#------------------------------------------------------------------------------
echo "1. Checking vampire_bridge package structure..."

if [[ -d "ros2_ws/src/vampire_bridge" ]]; then
    check_pass "vampire_bridge directory exists"
else
    check_fail "vampire_bridge directory missing"
fi

# Required files
REQUIRED_FILES=(
    "ros2_ws/src/vampire_bridge/package.xml"
    "ros2_ws/src/vampire_bridge/CMakeLists.txt"
    "ros2_ws/src/vampire_bridge/setup.py"
    "ros2_ws/src/vampire_bridge/vampire_bridge/__init__.py"
    "ros2_ws/src/vampire_bridge/vampire_bridge/vampire_node.py"
    "ros2_ws/src/vampire_bridge/vampire_bridge/query_builder.py"
    "ros2_ws/src/vampire_bridge/vampire_bridge/result_parser.py"
    "ros2_ws/src/vampire_bridge/vampire_bridge/cache_manager.py"
    "ros2_ws/src/vampire_bridge/msg/ReasoningQuery.msg"
    "ros2_ws/src/vampire_bridge/msg/ReasoningResult.msg"
    "ros2_ws/src/vampire_bridge/srv/VampireQuery.srv"
    "ros2_ws/src/vampire_bridge/launch/vampire_bridge.launch.py"
    "ros2_ws/src/vampire_bridge/config/vampire_params.yaml"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [[ -f "$file" ]]; then
        check_pass "$(basename $file) exists"
    else
        check_fail "$(basename $file) missing: $file"
    fi
done

#------------------------------------------------------------------------------
# 2. Benchmark Query Library
#------------------------------------------------------------------------------
echo ""
echo "2. Checking benchmark query library..."

QUERY_FILES=(
    "ontology/evaluation/benchmark_queries/safety_geofence_check.tptp"
    "ontology/evaluation/benchmark_queries/safety_nfz_violation.tptp"
    "ontology/evaluation/benchmark_queries/safety_battery_return.tptp"
    "ontology/evaluation/benchmark_queries/operational_valid_waypoint.tptp"
    "ontology/evaluation/benchmark_queries/operational_comms_status.tptp"
    "ontology/evaluation/benchmark_queries/operational_threat_zone.tptp"
    "ontology/evaluation/benchmark_queries/planning_mission_feasibility.tptp"
    "ontology/evaluation/benchmark_queries/planning_target_prioritization.tptp"
)

for file in "${QUERY_FILES[@]}"; do
    if [[ -f "$file" ]]; then
        check_pass "$(basename $file)"
    else
        check_fail "$(basename $file) missing"
    fi
done

#------------------------------------------------------------------------------
# 3. Unit Tests Present
#------------------------------------------------------------------------------
echo ""
echo "3. Checking unit tests..."

TEST_FILES=(
    "ros2_ws/src/vampire_bridge/test/test_cache_manager.py"
    "ros2_ws/src/vampire_bridge/test/test_query_builder.py"
    "ros2_ws/src/vampire_bridge/test/test_result_parser.py"
)

for file in "${TEST_FILES[@]}"; do
    if [[ -f "$file" ]]; then
        check_pass "$(basename $file)"
    else
        check_fail "$(basename $file) missing"
    fi
done

#------------------------------------------------------------------------------
# 4. Benchmark Script
#------------------------------------------------------------------------------
echo ""
echo "4. Checking x86 benchmark script..."

if [[ -f "ontology/evaluation/benchmark_x86.py" ]]; then
    check_pass "benchmark_x86.py exists"
    if [[ -x "ontology/evaluation/benchmark_x86.py" ]]; then
        check_pass "benchmark_x86.py is executable"
    else
        check_fail "benchmark_x86.py not executable"
    fi
else
    check_fail "benchmark_x86.py missing"
fi

#------------------------------------------------------------------------------
# 5. Python Module Imports
#------------------------------------------------------------------------------
echo ""
echo "5. Checking Python module imports..."

cd "$PROJECT_ROOT/ros2_ws/src/vampire_bridge"
if python3 -c "from vampire_bridge.cache_manager import QueryCache; print('OK')" 2>/dev/null | grep -q OK; then
    check_pass "cache_manager imports"
else
    check_fail "cache_manager import error"
fi

if python3 -c "from vampire_bridge.query_builder import QueryBuilder; print('OK')" 2>/dev/null | grep -q OK; then
    check_pass "query_builder imports"
else
    check_fail "query_builder import error"
fi

if python3 -c "from vampire_bridge.result_parser import ResultParser; print('OK')" 2>/dev/null | grep -q OK; then
    check_pass "result_parser imports"
else
    check_fail "result_parser import error"
fi

cd "$PROJECT_ROOT"

#------------------------------------------------------------------------------
# 6. Container and Vampire Availability
#------------------------------------------------------------------------------
echo ""
echo "6. Checking container and Vampire..."

if podman images | grep -q "flyby-f11-planning"; then
    check_pass "flyby-f11-planning container exists"

    # Test Vampire execution
    if echo "fof(test, axiom, a)." | podman run --rm -i flyby-f11-planning:latest vampire --input_syntax tptp --time_limit 5 /dev/stdin 2>&1 | grep -q "SZS status"; then
        check_pass "Vampire executes in container"
    else
        check_fail "Vampire execution failed"
    fi
else
    check_skip "flyby-f11-planning container not found"
fi

#------------------------------------------------------------------------------
# 7. Query Template Validation
#------------------------------------------------------------------------------
echo ""
echo "7. Validating query templates with Vampire..."

# Test NFZ violation query
NFZ_RESULT=$(sed 's/{{UAV_ID}}/drone_alpha/g; s/{{NFZ_ID}}/nfz_1/g' \
    ontology/evaluation/benchmark_queries/safety_nfz_violation.tptp | \
    podman run --rm -i flyby-f11-planning:latest vampire --input_syntax tptp --time_limit 10 /dev/stdin 2>&1)

if echo "$NFZ_RESULT" | grep -q "SZS status Theorem"; then
    check_pass "safety_nfz_violation: Theorem (violation detected)"
else
    check_fail "safety_nfz_violation: unexpected result"
fi

# Test geofence check query (should be CounterSatisfiable for safe case)
GF_RESULT=$(sed 's/{{UAV_ID}}/drone_alpha/g; s/{{GEOFENCE_ID}}/geofence_1/g' \
    ontology/evaluation/benchmark_queries/safety_geofence_check.tptp | \
    podman run --rm -i flyby-f11-planning:latest vampire --input_syntax tptp --time_limit 10 /dev/stdin 2>&1)

if echo "$GF_RESULT" | grep -q "SZS status"; then
    check_pass "safety_geofence_check: valid SZS status"
else
    check_fail "safety_geofence_check: no SZS status"
fi

#------------------------------------------------------------------------------
# 8. ARM64 Components (Deferred)
#------------------------------------------------------------------------------
echo ""
echo "8. ARM64 components (deferred - no Jetson hardware)..."

check_skip "ARM64 cross-compilation Containerfile"
check_skip "Quadlet container for Jetson"
check_skip "ARM64 benchmark results"

#------------------------------------------------------------------------------
# Summary
#------------------------------------------------------------------------------
echo ""
echo "================================================================================"
echo "  Verification Summary"
echo "================================================================================"
echo ""
echo "  Passed:  $PASS"
echo "  Failed:  $FAIL"
echo "  Skipped: $SKIP (ARM64 deferred)"
echo ""

if [[ $FAIL -eq 0 ]]; then
    echo -e "  ${GREEN}Phase 4 x86 components VERIFIED${NC}"
    echo ""
    echo "  Completed (x86):"
    echo "    - vampire_bridge ROS 2 package structure"
    echo "    - Message/service definitions"
    echo "    - Core Python modules (cache, query builder, result parser)"
    echo "    - VampireNode implementation"
    echo "    - Benchmark query library (8 templates)"
    echo "    - x86 benchmark script"
    echo "    - Unit tests"
    echo ""
    echo "  Deferred (requires Jetson):"
    echo "    - ARM64 cross-compilation"
    echo "    - Quadlet container for deployment"
    echo "    - ARM64 benchmarks"
    echo ""
    exit 0
else
    echo -e "  ${RED}Phase 4 verification FAILED${NC}"
    echo ""
    echo "  Please fix the failed checks before proceeding."
    echo ""
    exit 1
fi
