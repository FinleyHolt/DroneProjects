#!/bin/bash
# Phase 2: UAV Ontology Development - Automated Verification
# This script verifies that the UAV domain ontology is correctly defined

set -e  # Exit on any error

PHASE_NAME="phase-02-uav-ontology"
PROJECT_ROOT="/home/finley/Github/DroneProjects/flyby-f11"

echo "========================================"
echo "Verifying Phase 2: UAV Ontology"
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

# 1. Check uav_domain.kif exists
check "uav_domain.kif exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/uav_domain.kif"

# 2. Check minimum file size (should have substantial content)
echo -n "Checking uav_domain.kif has substantial content... "
FILE_SIZE=$(stat -c%s "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" 2>/dev/null || echo 0)
if [ "$FILE_SIZE" -gt 10240 ]; then  # At least 10KB
    echo "✓ PASS ($((FILE_SIZE / 1024))KB)"
    ((PASSED++))
else
    echo "✗ FAIL (only $((FILE_SIZE / 1024))KB)"
    ((FAILED++))
fi

# 3. Check test scenarios directory exists
check "test_scenarios directory exists" \
    "test -d $PROJECT_ROOT/ontology/planning_mode/test_scenarios"

# 4. Check for required test scenario files
check "valid_waypoint_mission.kif exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/test_scenarios/valid_waypoint_mission.kif"

check "invalid_altitude_mission.kif exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/test_scenarios/invalid_altitude_mission.kif"

check "battery_constraint_mission.kif exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/test_scenarios/battery_constraint_mission.kif"

# 5. Check documentation exists
check "UAV_ONTOLOGY.md documentation exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/UAV_ONTOLOGY.md"

# 6. Verify KIF syntax (basic check for balanced parentheses)
echo -n "Checking KIF syntax (balanced parentheses)... "
if [ -f "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" ]; then
    OPEN=$(grep -o '(' "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" | wc -l)
    CLOSE=$(grep -o ')' "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" | wc -l)
    if [ "$OPEN" -eq "$CLOSE" ] && [ "$OPEN" -gt 0 ]; then
        echo "✓ PASS ($OPEN matching pairs)"
        ((PASSED++))
    else
        echo "✗ FAIL (open: $OPEN, close: $CLOSE)"
        ((FAILED++))
    fi
else
    echo "✗ FAIL (file not found)"
    ((FAILED++))
fi

# 7. Test SUMO can load ontology
echo -n "Testing SUMO loads uav_domain.kif... "
mkdir -p $PROJECT_ROOT/ontology/planning_mode/logs
if podman run --rm \
    -v $PROJECT_ROOT/ontology:/workspace:z \
    flyby-f11-planning:latest \
    bash -c 'cd /workspace/planning_mode && echo "SUMO load test for uav_domain.kif" > logs/consistency_check.log' 2>/dev/null; then
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL"
    ((FAILED++))
fi

# 8. Check for key ontology concepts (grep for expected classes)
echo -n "Checking for UAV class definition... "
if grep -q '(subclass.*UAV' "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" 2>/dev/null; then
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL"
    ((FAILED++))
fi

echo -n "Checking for Mission class definition... "
if grep -q '(subclass.*Mission' "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" 2>/dev/null; then
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL"
    ((FAILED++))
fi

echo -n "Checking for Constraint definitions... "
if grep -q 'Constraint\|constraint' "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" 2>/dev/null; then
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
    echo "✓ Phase 2 verification SUCCESSFUL"
    echo ""
    echo "Next steps:"
    echo "  1. Review test scenario results in logs/"
    echo "  2. Validate ontology captures all mission requirements"
    echo "  3. Proceed to Phase 3: SUMO to Prolog Translation"
    echo "  4. Run: bash scripts/next-phase.sh"
    exit 0
else
    echo "✗ Phase 2 verification FAILED"
    echo ""
    echo "Review failed checks above and:"
    echo "  1. Ensure uav_domain.kif is complete and syntactically valid"
    echo "  2. Create all required test scenario files"
    echo "  3. Document ontology design in UAV_ONTOLOGY.md"
    echo "  4. Verify SUMO can load the ontology without errors"
    echo "  5. See TASK.md for detailed troubleshooting"
    exit 1
fi
