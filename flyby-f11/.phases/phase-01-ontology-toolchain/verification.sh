#!/bin/bash
# Phase 1: Ontology Toolchain Setup - Automated Verification

PROJECT_ROOT="/home/finley/Github/DroneProjects/flyby-f11"
cd "$PROJECT_ROOT"

echo "========================================"
echo "Phase 1: Ontology Toolchain Verification"
echo "========================================"
echo ""

PASSED=0
FAILED=0

pass() { echo "✓ $1"; PASSED=$((PASSED + 1)); }
fail() { echo "✗ $1"; FAILED=$((FAILED + 1)); }

# File checks
echo "--- File Structure ---"
[ -f ontology/Containerfile.planning ] && pass "Containerfile.planning exists" || fail "Containerfile.planning missing"
[ -f ontology/planning_mode/test_uav.kif ] && pass "test_uav.kif exists" || fail "test_uav.kif missing"
[ -f ontology/planning_mode/test_theorem.tptp ] && pass "test_theorem.tptp exists" || fail "test_theorem.tptp missing"
[ -f ontology/planning_mode/validate_kif.py ] && pass "validate_kif.py exists" || fail "validate_kif.py missing"

# Container checks
echo ""
echo "--- Container ---"
if podman images | grep -q flyby-f11-planning; then
    pass "Container image exists"
    SIZE=$(podman images flyby-f11-planning --format "{{.Size}}")
    echo "  Image size: $SIZE"
else
    fail "Container image missing"
fi

# SUMO check
echo ""
echo "--- SUMO Ontology ---"
KIF_COUNT=$(podman run --rm flyby-f11-planning:latest sh -c 'ls $SUMO_HOME/*.kif | wc -l' 2>/dev/null)
if [ "$KIF_COUNT" -gt 50 ]; then
    pass "SUMO KIF files present ($KIF_COUNT files)"
else
    fail "SUMO KIF files missing or incomplete"
fi

# Vampire check
echo ""
echo "--- Vampire ATP ---"
if podman run --rm flyby-f11-planning:latest vampire --version 2>/dev/null | grep -q "Vampire"; then
    pass "Vampire installed"
else
    fail "Vampire not found"
fi

# KIF syntax validation
echo ""
echo "--- KIF Syntax Test ---"
if python3 ontology/planning_mode/validate_kif.py ontology/planning_mode/test_uav.kif > /dev/null 2>&1; then
    pass "test_uav.kif syntax valid"
else
    fail "test_uav.kif syntax invalid"
fi

# Vampire proof test
echo ""
echo "--- Theorem Proving ---"
PROOF_RESULT=$(podman run --rm -v "$PROJECT_ROOT/ontology:/workspace:z" \
    flyby-f11-planning:latest \
    vampire --input_syntax tptp /workspace/planning_mode/test_theorem.tptp 2>&1)
if echo "$PROOF_RESULT" | grep -q "SZS status Theorem"; then
    pass "Vampire proves test theorem"
else
    fail "Vampire proof failed"
fi

# Volume mount test
echo ""
echo "--- Volume Mount ---"
TEST_FILE="ontology/planning_mode/.verify_test"
if podman run --rm -v "$PROJECT_ROOT/ontology:/workspace:z" \
    flyby-f11-planning:latest \
    touch /workspace/planning_mode/.verify_test 2>/dev/null && [ -f "$TEST_FILE" ]; then
    rm -f "$TEST_FILE"
    pass "Volume mount read/write works"
else
    fail "Volume mount failed"
fi

# Summary
echo ""
echo "========================================"
echo "Results: $PASSED passed, $FAILED failed"
echo "========================================"

if [ $FAILED -eq 0 ]; then
    echo "✓ Phase 1 COMPLETE"
    echo ""
    echo "Next: bash scripts/next-phase.sh"
    exit 0
else
    echo "✗ Phase 1 FAILED - fix issues above"
    exit 1
fi
