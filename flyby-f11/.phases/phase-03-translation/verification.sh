#!/bin/bash
# Phase 3: SUMO to Prolog Translation - Automated Verification
# This script verifies that the translation pipeline works correctly

set -e  # Exit on any error

PHASE_NAME="phase-03-translation"
PROJECT_ROOT="/home/finley/Github/DroneProjects/flyby-f11"

echo "========================================"
echo "Verifying Phase 3: SUMO to Prolog Translation"
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

# 1. Check translator implementation exists
check "sumo_to_prolog.py exists" \
    "test -f $PROJECT_ROOT/ontology/translation/sumo_to_prolog.py"

# 2. Check translator is executable
check "sumo_to_prolog.py is executable" \
    "test -x $PROJECT_ROOT/ontology/translation/sumo_to_prolog.py"

# 3. Check translation documentation exists
check "TRANSLATION.md exists" \
    "test -f $PROJECT_ROOT/ontology/translation/TRANSLATION.md"

# 4. Check Prolog templates directory exists
check "templates directory exists" \
    "test -d $PROJECT_ROOT/ontology/execution_mode/templates"

# 5. Check for required template files
check "constraint_checking.pl exists" \
    "test -f $PROJECT_ROOT/ontology/execution_mode/templates/constraint_checking.pl"

check "mission_validation.pl exists" \
    "test -f $PROJECT_ROOT/ontology/execution_mode/templates/mission_validation.pl"

check "safety_monitoring.pl exists" \
    "test -f $PROJECT_ROOT/ontology/execution_mode/templates/safety_monitoring.pl"

# 6. Check generated Prolog rules exist
check "uav_rules.pl exists" \
    "test -f $PROJECT_ROOT/ontology/execution_mode/uav_rules.pl"

# 7. Check minimum file size for generated rules
echo -n "Checking uav_rules.pl has substantial content... "
FILE_SIZE=$(stat -c%s "$PROJECT_ROOT/ontology/execution_mode/uav_rules.pl" 2>/dev/null || echo 0)
if [ "$FILE_SIZE" -gt 5120 ]; then  # At least 5KB
    echo "✓ PASS ($((FILE_SIZE / 1024))KB)"
    ((PASSED++))
else
    echo "✗ FAIL (only $((FILE_SIZE / 1024))KB)"
    ((FAILED++))
fi

# 8. Check validation directory exists
check "validation directory exists" \
    "test -d $PROJECT_ROOT/ontology/translation/validation"

# 9. Test translator runs without errors
echo -n "Testing translator executes... "
mkdir -p $PROJECT_ROOT/ontology/translation/logs
if cd $PROJECT_ROOT && \
   python3 ontology/translation/sumo_to_prolog.py \
   --input ontology/planning_mode/uav_domain.kif \
   --output ontology/execution_mode/uav_rules_test.pl \
   > ontology/translation/logs/translation.log 2>&1; then
    echo "✓ PASS"
    ((PASSED++))
    rm -f ontology/execution_mode/uav_rules_test.pl
else
    echo "✗ FAIL (check logs/translation.log)"
    ((FAILED++))
fi

# 10. Test Prolog rules load in SWI-Prolog
echo -n "Testing Prolog rules load in SWI-Prolog... "
if command -v swipl > /dev/null 2>&1; then
    if swipl -g "consult('$PROJECT_ROOT/ontology/execution_mode/uav_rules.pl'),halt" -t 'halt(1)' 2>/dev/null; then
        echo "✓ PASS"
        ((PASSED++))
    else
        echo "✗ FAIL (syntax errors in generated Prolog)"
        ((FAILED++))
    fi
else
    echo "⊘ SKIP (SWI-Prolog not installed)"
    # Don't count as pass or fail - it's optional on host
fi

# 11. Check for basic Prolog predicates in generated file
echo -n "Checking for UAV predicate definitions... "
if grep -q ':-' "$PROJECT_ROOT/ontology/execution_mode/uav_rules.pl" 2>/dev/null; then
    echo "✓ PASS"
    ((PASSED++))
else
    echo "✗ FAIL (no Prolog rules found)"
    ((FAILED++))
fi

# 12. Performance test (if available)
if [ -f "$PROJECT_ROOT/ontology/translation/logs/performance_test.log" ]; then
    echo -n "Checking query performance... "
    AVG_TIME=$(grep -oP 'avg_time: \K[0-9.]+' "$PROJECT_ROOT/ontology/translation/logs/performance_test.log" | head -1)
    if [ -n "$AVG_TIME" ] && [ $(echo "$AVG_TIME < 10" | bc) -eq 1 ]; then
        echo "✓ PASS (${AVG_TIME}ms avg)"
        ((PASSED++))
    else
        echo "⚠ WARNING (${AVG_TIME}ms avg - target <10ms)"
    fi
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
    echo "✓ Phase 3 verification SUCCESSFUL"
    echo ""
    echo "Next steps:"
    echo "  1. Review translation logs for any warnings"
    echo "  2. Compare SUMO vs Prolog results for test scenarios"
    echo "  3. Optimize Prolog rules if query time > 10ms"
    echo "  4. Proceed to Phase 4: Execution Mode Runtime"
    echo "  5. Run: bash scripts/next-phase.sh"
    exit 0
else
    echo "✗ Phase 3 verification FAILED"
    echo ""
    echo "Review failed checks above and:"
    echo "  1. Implement sumo_to_prolog.py translator"
    echo "  2. Ensure translator generates valid Prolog syntax"
    echo "  3. Create all required template files"
    echo "  4. Verify generated rules load in SWI-Prolog"
    echo "  5. Document translation process in TRANSLATION.md"
    echo "  6. See TASK.md for detailed troubleshooting"
    exit 1
fi
