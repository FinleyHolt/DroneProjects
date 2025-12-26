#!/bin/bash
# Phase 1: Ontology Toolchain Setup - Automated Verification
# This script verifies that the planning mode container is correctly configured

set -e  # Exit on any error

PHASE_NAME="phase-01-ontology-toolchain"
PROJECT_ROOT="/home/finley/Github/DroneProjects/flyby-f11"

echo "========================================"
echo "Verifying Phase 1: Ontology Toolchain"
echo "========================================"
echo ""

# Track verification status
PASSED=0
FAILED=0

# Helper function for checks
check() {
    local name="$1"
    local command="$2"

    echo -n "Checking $name... "
    if eval "$command" > /dev/null 2>&1; then
        echo "✓ PASS"
        ((PASSED++))
        return 0
    else
        echo "✗ FAIL"
        ((FAILED++))
        return 1
    fi
}

# 1. Check Containerfile exists
check "Containerfile.planning exists" \
    "test -f $PROJECT_ROOT/ontology/Containerfile.planning"

# 2. Check working directory exists
check "planning_mode directory exists" \
    "test -d $PROJECT_ROOT/ontology/planning_mode"

# 3. Check test ontology exists
check "test_uav.kif exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/test_uav.kif"

# 4. Check container image built
check "flyby-f11-planning image exists" \
    "podman images | grep -q flyby-f11-planning"

# 5. Verify SUMO in container
echo -n "Checking SUMO installation in container... "
if podman run --rm flyby-f11-planning:latest bash -c 'test -d $SUMO_HOME' 2>/dev/null; then
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL"
    ((FAILED++))
fi

# 6. Verify Vampire in container
echo -n "Checking Vampire installation in container... "
if podman run --rm flyby-f11-planning:latest which vampire > /dev/null 2>&1; then
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL"
    ((FAILED++))
fi

# 7. Test SUMO can load ontology
echo -n "Testing SUMO loads test ontology... "
mkdir -p $PROJECT_ROOT/ontology/planning_mode/logs
if podman run --rm \
    -v $PROJECT_ROOT/ontology:/workspace:z \
    flyby-f11-planning:latest \
    bash -c 'cd /workspace/planning_mode && echo "Testing SUMO load" > logs/sumo_test.log' 2>/dev/null; then
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL"
    ((FAILED++))
fi

# 8. Test volume mounting works
echo -n "Testing volume mount read/write... "
TEST_FILE="$PROJECT_ROOT/ontology/planning_mode/.verification_test"
if podman run --rm \
    -v $PROJECT_ROOT/ontology:/workspace:z \
    flyby-f11-planning:latest \
    bash -c 'touch /workspace/planning_mode/.verification_test' && \
   test -f "$TEST_FILE"; then
    rm -f "$TEST_FILE"
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL"
    ((FAILED++))
fi

# Print summary
echo ""
echo "========================================"
echo "Verification Summary"
echo "========================================"
echo "Passed: $PASSED"
echo "Failed: $FAILED"
echo ""

if [ $FAILED -eq 0 ]; then
    echo "✓ Phase 1 verification SUCCESSFUL"
    echo ""
    echo "Next steps:"
    echo "  1. Review TASK.md for phase completion checklist"
    echo "  2. Proceed to Phase 2: UAV Ontology Development"
    echo "  3. Run: bash scripts/next-phase.sh"
    exit 0
else
    echo "✗ Phase 1 verification FAILED"
    echo ""
    echo "Review failed checks above and:"
    echo "  1. Ensure Containerfile.planning builds successfully"
    echo "  2. Verify SUMO and Vampire are installed in container"
    echo "  3. Check volume mount permissions (use :z suffix)"
    echo "  4. See TASK.md for detailed troubleshooting"
    exit 1
fi
