#!/bin/bash
# Phase 4: Execution Mode Runtime - Verification Script
set -e

echo "========================================"
echo "Phase 4: Execution Mode Runtime"
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

echo "=== File Existence Checks ==="
check "Containerfile.execution exists" "test -f ontology/Containerfile.execution"
check "query_interface.py exists" "test -f ontology/execution_mode/query_interface.py"
check "prolog_bridge package exists" "test -d ros2_ws/src/prolog_bridge"
check "prolog_bridge package.xml exists" "test -f ros2_ws/src/prolog_bridge/package.xml"
check "benchmark suite exists" "test -d ontology/execution_mode/benchmarks"
check "benchmark results exist" "test -f ontology/execution_mode/benchmarks/results.json"

echo ""
echo "=== Container Build Checks ==="
check "Execution container image exists" "podman images | grep -q flyby-f11-execution"

echo ""
echo "=== Functional Checks ==="
check "Prolog rules load successfully" "podman run --rm -v ./ontology:/workspace:z flyby-f11-execution:latest swipl -g 'consult(\"/workspace/execution_mode/uav_rules.pl\"), halt(0)' -t 'halt(1)'"

# Performance check - query latency
echo ""
echo "=== Performance Checks ==="
if test -f ontology/execution_mode/benchmarks/results.json; then
    # Extract avg_query_latency_ms from results
    AVG_LATENCY=$(python3 -c "import json; print(json.load(open('ontology/execution_mode/benchmarks/results.json'))['avg_query_latency_ms'])" 2>/dev/null || echo "999")
    if [ "$(echo "$AVG_LATENCY < 10" | bc -l)" -eq 1 ]; then
        echo "Checking: Query latency < 10ms (actual: ${AVG_LATENCY}ms)... PASS"
        ((PASS++))
    else
        echo "Checking: Query latency < 10ms (actual: ${AVG_LATENCY}ms)... FAIL"
        ((FAIL++))
    fi

    # Extract memory_footprint_mb
    MEMORY=$(python3 -c "import json; print(json.load(open('ontology/execution_mode/benchmarks/results.json'))['memory_footprint_mb'])" 2>/dev/null || echo "999")
    if [ "$(echo "$MEMORY < 100" | bc -l)" -eq 1 ]; then
        echo "Checking: Memory < 100MB (actual: ${MEMORY}MB)... PASS"
        ((PASS++))
    else
        echo "Checking: Memory < 100MB (actual: ${MEMORY}MB)... FAIL"
        ((FAIL++))
    fi
else
    echo "Checking: Performance benchmarks... SKIP (results.json not found)"
fi

echo ""
echo "========================================"
echo "Results: $PASS passed, $FAIL failed"
echo "========================================"

if [ $FAIL -gt 0 ]; then
    exit 1
fi

echo ""
echo "Phase 4 verification PASSED"
exit 0
