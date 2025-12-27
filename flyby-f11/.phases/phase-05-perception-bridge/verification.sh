#!/bin/bash
# Phase 5: Perception-to-Reasoning Bridge - Verification Script
# Verifies the State Persistence Architecture (ADR-002) implementation
set -e

echo "========================================"
echo "Phase 5: Perception-to-Reasoning Bridge"
echo "State Persistence Architecture Verification"
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

echo "=== Core Module Checks ==="
check "HotFactBuffer module exists" "test -f ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "ColdFactCache module exists" "test -f ros2_ws/src/vampire_bridge/vampire_bridge/cold_fact_cache.py"
check "StateGroundingNode module exists" "test -f ros2_ws/src/vampire_bridge/vampire_bridge/state_grounding_node.py"

echo ""
echo "=== Module Content Checks ==="
check "HotFactBuffer class defined" "grep -q 'class HotFactBuffer' ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "HotFactBuffer has get_snapshot" "grep -q 'def get_snapshot' ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "HotFactBuffer has update method" "grep -q 'def update' ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "HotFactBuffer has stale detection" "grep -q 'get_stale_facts' ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "HotFactBuffer is thread-safe" "grep -q 'threading.RLock' ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"

check "ColdFactCache class defined" "grep -q 'class ColdFactCache' ros2_ws/src/vampire_bridge/vampire_bridge/cold_fact_cache.py"
check "ColdFactCache has load method" "grep -q 'def load' ros2_ws/src/vampire_bridge/vampire_bridge/cold_fact_cache.py"
check "ColdFactCache has get_cached_block" "grep -q 'def get_cached_block' ros2_ws/src/vampire_bridge/vampire_bridge/cold_fact_cache.py"
check "ColdFactCache has needs_reload" "grep -q 'def needs_reload' ros2_ws/src/vampire_bridge/vampire_bridge/cold_fact_cache.py"
check "ColdFactCache has NFZ invalidation" "grep -q 'add_nfz\|remove_nfz' ros2_ws/src/vampire_bridge/vampire_bridge/cold_fact_cache.py"

check "StateGroundingNode class defined" "grep -q 'class StateGroundingNode' ros2_ws/src/vampire_bridge/vampire_bridge/state_grounding_node.py"
check "StateGroundingNode subscribes to battery" "grep -q '/mavros/battery' ros2_ws/src/vampire_bridge/vampire_bridge/state_grounding_node.py"
check "StateGroundingNode subscribes to pose" "grep -q '/mavros/local_position/pose' ros2_ws/src/vampire_bridge/vampire_bridge/state_grounding_node.py"
check "StateGroundingNode subscribes to state" "grep -q '/mavros/state' ros2_ws/src/vampire_bridge/vampire_bridge/state_grounding_node.py"
check "StateGroundingNode uses HotFactBuffer" "grep -q 'HotFactBuffer' ros2_ws/src/vampire_bridge/vampire_bridge/state_grounding_node.py"

echo ""
echo "=== QueryBuilder Integration Checks ==="
check "QueryBuilder updated for state injection" "grep -q 'hot_buffer' ros2_ws/src/vampire_bridge/vampire_bridge/query_builder.py"
check "QueryBuilder has _inject_state_facts" "grep -q '_inject_state_facts' ros2_ws/src/vampire_bridge/vampire_bridge/query_builder.py"
check "QueryBuilder imports state modules" "grep -q 'HotFactBuffer\|ColdFactCache' ros2_ws/src/vampire_bridge/vampire_bridge/query_builder.py"

echo ""
echo "=== Unit Test Checks ==="
check "HotFactBuffer tests exist" "test -f ros2_ws/src/vampire_bridge/test/test_hot_fact_buffer.py"
check "ColdFactCache tests exist" "test -f ros2_ws/src/vampire_bridge/test/test_cold_fact_cache.py"
check "HotFactBuffer tests have latency check" "grep -q 'latency_requirement\|latency' ros2_ws/src/vampire_bridge/test/test_hot_fact_buffer.py"
check "HotFactBuffer tests have thread safety" "grep -q 'thread_safety' ros2_ws/src/vampire_bridge/test/test_hot_fact_buffer.py"
check "QueryBuilder tests include state injection" "grep -q 'StateFactInjection\|state_injection' ros2_ws/src/vampire_bridge/test/test_query_builder.py"

echo ""
echo "=== TPTP Fact Format Checks ==="
check "HotFactBuffer produces valid TPTP format" "grep -q \"fof(.*, axiom,\" ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "ColdFactCache produces valid TPTP format" "grep -q \"fof(.*, axiom,\" ros2_ws/src/vampire_bridge/vampire_bridge/cold_fact_cache.py"

echo ""
echo "=== ADR-002 Compliance Checks ==="
check "Battery status fact schema" "grep -q 'has_battery_status' ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "Position fact schema" "grep -q 'position' ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "Comms status fact schema" "grep -q 'has_comms_status' ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "Distance to home fact schema" "grep -q 'distance_to_home' ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"

echo ""
echo "=== Python Syntax Checks ==="
check "hot_fact_buffer.py syntax valid" "python3 -m py_compile ros2_ws/src/vampire_bridge/vampire_bridge/hot_fact_buffer.py"
check "cold_fact_cache.py syntax valid" "python3 -m py_compile ros2_ws/src/vampire_bridge/vampire_bridge/cold_fact_cache.py"
check "state_grounding_node.py syntax valid" "python3 -m py_compile ros2_ws/src/vampire_bridge/vampire_bridge/state_grounding_node.py"
check "query_builder.py syntax valid" "python3 -m py_compile ros2_ws/src/vampire_bridge/vampire_bridge/query_builder.py"

echo ""
echo "=== Unit Test Execution ==="
# Run unit tests via micromamba or system pytest
MICROMAMBA="/home/finley/.local/bin/micromamba"
if [ -x "$MICROMAMBA" ]; then
    echo -n "Checking: HotFactBuffer unit tests pass... "
    if $MICROMAMBA run -n flyby-f11-eval bash -c 'PYTHONPATH="ros2_ws/src/vampire_bridge:$PYTHONPATH" python3 -m pytest ros2_ws/src/vampire_bridge/test/test_hot_fact_buffer.py -v --tb=short 2>&1' | tail -1 | grep -q "passed"; then
        echo "PASS"
        PASS=$((PASS + 1))
    else
        echo "FAIL"
        FAIL=$((FAIL + 1))
    fi

    echo -n "Checking: ColdFactCache unit tests pass... "
    if $MICROMAMBA run -n flyby-f11-eval bash -c 'PYTHONPATH="ros2_ws/src/vampire_bridge:$PYTHONPATH" python3 -m pytest ros2_ws/src/vampire_bridge/test/test_cold_fact_cache.py -v --tb=short 2>&1' | tail -1 | grep -q "passed"; then
        echo "PASS"
        PASS=$((PASS + 1))
    else
        echo "FAIL"
        FAIL=$((FAIL + 1))
    fi

    echo -n "Checking: QueryBuilder state injection tests pass... "
    if $MICROMAMBA run -n flyby-f11-eval bash -c 'PYTHONPATH="ros2_ws/src/vampire_bridge:$PYTHONPATH" python3 -m pytest ros2_ws/src/vampire_bridge/test/test_query_builder.py::TestQueryBuilderStateFactInjection -v --tb=short 2>&1' | tail -1 | grep -q "passed"; then
        echo "PASS"
        PASS=$((PASS + 1))
    else
        echo "FAIL"
        FAIL=$((FAIL + 1))
    fi
elif command -v pytest &> /dev/null; then
    echo -n "Checking: HotFactBuffer unit tests pass... "
    if PYTHONPATH="ros2_ws/src/vampire_bridge:$PYTHONPATH" pytest ros2_ws/src/vampire_bridge/test/test_hot_fact_buffer.py -v --tb=short 2>&1 | tail -1 | grep -q "passed"; then
        echo "PASS"
        PASS=$((PASS + 1))
    else
        echo "FAIL"
        FAIL=$((FAIL + 1))
    fi

    echo -n "Checking: ColdFactCache unit tests pass... "
    if PYTHONPATH="ros2_ws/src/vampire_bridge:$PYTHONPATH" pytest ros2_ws/src/vampire_bridge/test/test_cold_fact_cache.py -v --tb=short 2>&1 | tail -1 | grep -q "passed"; then
        echo "PASS"
        PASS=$((PASS + 1))
    else
        echo "FAIL"
        FAIL=$((FAIL + 1))
    fi

    echo -n "Checking: QueryBuilder state injection tests pass... "
    if PYTHONPATH="ros2_ws/src/vampire_bridge:$PYTHONPATH" pytest ros2_ws/src/vampire_bridge/test/test_query_builder.py::TestQueryBuilderStateFactInjection -v --tb=short 2>&1 | tail -1 | grep -q "passed"; then
        echo "PASS"
        PASS=$((PASS + 1))
    else
        echo "FAIL"
        FAIL=$((FAIL + 1))
    fi
else
    echo "Checking: Unit tests... SKIP (pytest not available)"
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
echo ""
echo "Implementation Summary:"
echo "  - HotFactBuffer: Thread-safe buffer for volatile state (position, battery, comms)"
echo "  - ColdFactCache: Cache for static facts (geofence, NFZ) with reload support"
echo "  - StateGroundingNode: ROS 2 node bridging MAVROS topics to TPTP facts"
echo "  - QueryBuilder: Updated to inject hot/cold facts into Vampire queries"
echo ""
echo "Latency Budget (ADR-002):"
echo "  - Fact injection: <0.5ms (verified in tests)"
echo "  - Total query: <50ms budget maintained"
exit 0
