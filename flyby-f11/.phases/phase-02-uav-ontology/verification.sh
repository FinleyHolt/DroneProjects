#!/bin/bash
# Phase 2: UAV Ontology Development - Automated Verification
# This script verifies that the UAV domain ontology is correctly defined
# Updated: 2024-12-26 - Added ISR extensions verification

PHASE_NAME="phase-02-uav-ontology"
PROJECT_ROOT="/home/finley/Github/DroneProjects/flyby-f11"

echo "========================================"
echo "Verifying Phase 2: UAV Ontology"
echo "(Including ISR Extensions)"
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
        PASSED=$((PASSED + 1))
        return 0
    else
        echo "✗ FAIL"
        FAILED=$((FAILED + 1))
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
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL (only $((FILE_SIZE / 1024))KB)"
    FAILED=$((FAILED + 1))
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
        PASSED=$((PASSED + 1))
    else
        echo "✗ FAIL (open: $OPEN, close: $CLOSE)"
        FAILED=$((FAILED + 1))
    fi
else
    echo "✗ FAIL (file not found)"
    FAILED=$((FAILED + 1))
fi

# 7. Run ontology consistency check with Vampire
echo -n "Testing ontology consistency with Vampire... "
mkdir -p $PROJECT_ROOT/ontology/planning_mode/logs
if [ -f "$PROJECT_ROOT/ontology/planning_mode/test_scenarios/consistency_check.tptp" ]; then
    VAMPIRE_OUTPUT=$(podman run --rm \
        -v $PROJECT_ROOT/ontology:/workspace:z \
        flyby-f11-planning:latest \
        vampire --input_syntax tptp --time_limit 30 \
        /workspace/planning_mode/test_scenarios/consistency_check.tptp 2>&1)
    echo "$VAMPIRE_OUTPUT" > $PROJECT_ROOT/ontology/planning_mode/logs/consistency_check.log
    if echo "$VAMPIRE_OUTPUT" | grep -q "CounterSatisfiable\|Satisfiable"; then
        echo "✓ PASS (no contradictions)"
        PASSED=$((PASSED + 1))
    else
        echo "✗ FAIL (inconsistency detected)"
        FAILED=$((FAILED + 1))
    fi
else
    echo "✗ FAIL (consistency_check.tptp not found)"
    FAILED=$((FAILED + 1))
fi

# ============================================================================
# ISR EXTENSIONS VERIFICATION (Phase 2 Extension)
# ============================================================================

echo ""
echo "----------------------------------------"
echo "Verifying ISR Extensions..."
echo "----------------------------------------"
echo ""

# 7a. Check isr_extensions.kif exists
check "isr_extensions.kif exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/isr_extensions.kif"

# 7b. Check isr_extensions.kif has substantial content
echo -n "Checking isr_extensions.kif has substantial content... "
ISR_FILE_SIZE=$(stat -c%s "$PROJECT_ROOT/ontology/planning_mode/isr_extensions.kif" 2>/dev/null || echo 0)
if [ "$ISR_FILE_SIZE" -gt 5120 ]; then  # At least 5KB
    echo "✓ PASS ($((ISR_FILE_SIZE / 1024))KB)"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL (only $((ISR_FILE_SIZE / 1024))KB)"
    FAILED=$((FAILED + 1))
fi

# 7c. Check ISR KIF syntax (balanced parentheses)
echo -n "Checking isr_extensions.kif syntax (balanced parentheses)... "
if [ -f "$PROJECT_ROOT/ontology/planning_mode/isr_extensions.kif" ]; then
    ISR_OPEN=$(grep -o '(' "$PROJECT_ROOT/ontology/planning_mode/isr_extensions.kif" | wc -l)
    ISR_CLOSE=$(grep -o ')' "$PROJECT_ROOT/ontology/planning_mode/isr_extensions.kif" | wc -l)
    if [ "$ISR_OPEN" -eq "$ISR_CLOSE" ] && [ "$ISR_OPEN" -gt 0 ]; then
        echo "✓ PASS ($ISR_OPEN matching pairs)"
        PASSED=$((PASSED + 1))
    else
        echo "✗ FAIL (open: $ISR_OPEN, close: $ISR_CLOSE)"
        FAILED=$((FAILED + 1))
    fi
else
    echo "✗ FAIL (file not found)"
    FAILED=$((FAILED + 1))
fi

# 7d. Check canonical test scenarios directory exists
check "canonical test scenarios directory exists" \
    "test -d $PROJECT_ROOT/ontology/planning_mode/test_scenarios/canonical"

# 7e. Check for canonical test scenario files
check "canonical_1_comms_denied.tptp exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/test_scenarios/canonical/canonical_1_comms_denied.tptp"

check "canonical_2_dynamic_nfz.tptp exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/test_scenarios/canonical/canonical_2_dynamic_nfz.tptp"

check "canonical_3_multi_objective.tptp exists" \
    "test -f $PROJECT_ROOT/ontology/planning_mode/test_scenarios/canonical/canonical_3_multi_objective.tptp"

check "run_canonical_tests.sh exists and is executable" \
    "test -x $PROJECT_ROOT/ontology/planning_mode/test_scenarios/canonical/run_canonical_tests.sh"

# 7f. Check for key ISR extension concepts
echo -n "Checking for CommsStatus definition... "
if grep -q 'CommsStatus' "$PROJECT_ROOT/ontology/planning_mode/isr_extensions.kif" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
fi

echo -n "Checking for ThreatZone definition... "
if grep -q 'ThreatZone' "$PROJECT_ROOT/ontology/planning_mode/isr_extensions.kif" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
fi

echo -n "Checking for DynamicNoFlyZone definition... "
if grep -q 'DynamicNoFlyZone' "$PROJECT_ROOT/ontology/planning_mode/isr_extensions.kif" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
fi

echo -n "Checking for TargetOfInterest definition... "
if grep -q 'TargetOfInterest' "$PROJECT_ROOT/ontology/planning_mode/isr_extensions.kif" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
fi

# 7g. Run canonical tests with Vampire
echo ""
echo -n "Running canonical test 1 (Comms-Denied)... "
if [ -f "$PROJECT_ROOT/ontology/planning_mode/test_scenarios/canonical/canonical_1_comms_denied.tptp" ]; then
    CANON1_OUTPUT=$(podman run --rm \
        -v $PROJECT_ROOT/ontology:/workspace:z \
        flyby-f11-planning:latest \
        vampire --input_syntax tptp --time_limit 30 \
        /workspace/planning_mode/test_scenarios/canonical/canonical_1_comms_denied.tptp 2>&1)
    echo "$CANON1_OUTPUT" > $PROJECT_ROOT/ontology/planning_mode/logs/canonical_1_comms_denied.log
    if echo "$CANON1_OUTPUT" | grep -q "SZS status Theorem"; then
        echo "✓ PASS (theorem proved)"
        PASSED=$((PASSED + 1))
    else
        echo "✗ FAIL (theorem not proved)"
        FAILED=$((FAILED + 1))
    fi
else
    echo "✗ FAIL (test file not found)"
    FAILED=$((FAILED + 1))
fi

echo -n "Running canonical test 2 (Dynamic NFZ)... "
if [ -f "$PROJECT_ROOT/ontology/planning_mode/test_scenarios/canonical/canonical_2_dynamic_nfz.tptp" ]; then
    CANON2_OUTPUT=$(podman run --rm \
        -v $PROJECT_ROOT/ontology:/workspace:z \
        flyby-f11-planning:latest \
        vampire --input_syntax tptp --time_limit 30 \
        /workspace/planning_mode/test_scenarios/canonical/canonical_2_dynamic_nfz.tptp 2>&1)
    echo "$CANON2_OUTPUT" > $PROJECT_ROOT/ontology/planning_mode/logs/canonical_2_dynamic_nfz.log
    if echo "$CANON2_OUTPUT" | grep -q "SZS status Theorem"; then
        echo "✓ PASS (theorem proved)"
        PASSED=$((PASSED + 1))
    else
        echo "✗ FAIL (theorem not proved)"
        FAILED=$((FAILED + 1))
    fi
else
    echo "✗ FAIL (test file not found)"
    FAILED=$((FAILED + 1))
fi

echo -n "Running canonical test 3 (Multi-Objective)... "
if [ -f "$PROJECT_ROOT/ontology/planning_mode/test_scenarios/canonical/canonical_3_multi_objective.tptp" ]; then
    CANON3_OUTPUT=$(podman run --rm \
        -v $PROJECT_ROOT/ontology:/workspace:z \
        flyby-f11-planning:latest \
        vampire --input_syntax tptp --time_limit 30 \
        /workspace/planning_mode/test_scenarios/canonical/canonical_3_multi_objective.tptp 2>&1)
    echo "$CANON3_OUTPUT" > $PROJECT_ROOT/ontology/planning_mode/logs/canonical_3_multi_objective.log
    if echo "$CANON3_OUTPUT" | grep -q "SZS status Theorem"; then
        echo "✓ PASS (theorem proved)"
        PASSED=$((PASSED + 1))
    else
        echo "✗ FAIL (theorem not proved)"
        FAILED=$((FAILED + 1))
    fi
else
    echo "✗ FAIL (test file not found)"
    FAILED=$((FAILED + 1))
fi

echo ""
echo "----------------------------------------"

# 8. Check for key ontology concepts (grep for expected classes)
echo -n "Checking for UAV class definition... "
if grep -q '(subclass.*UAV' "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
fi

echo -n "Checking for Mission class definition... "
if grep -q '(subclass.*Mission' "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
fi

echo -n "Checking for Constraint definitions... "
if grep -q 'Constraint\|constraint' "$PROJECT_ROOT/ontology/planning_mode/uav_domain.kif" 2>/dev/null; then
    echo "✓ PASS"
    PASSED=$((PASSED + 1))
else
    echo "✗ FAIL"
    FAILED=$((FAILED + 1))
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
