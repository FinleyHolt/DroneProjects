#!/bin/bash
#------------------------------------------------------------------------------
# Canonical Test Runner for Flyby F-11 UAV Ontology
#------------------------------------------------------------------------------
#
# Runs all 3 canonical test scenarios through Vampire theorem prover.
# Based on ONTOLOGY_FOUNDATION.qmd canonical problems.
#
# Usage:
#   ./run_canonical_tests.sh           # Run all tests
#   ./run_canonical_tests.sh --verbose # Run with detailed output
#
# Exit codes:
#   0 - All tests passed
#   1 - One or more tests failed
#
# Timing requirement: Each test should complete in <100ms
#
#------------------------------------------------------------------------------

set -e

# Configuration
PROJECT_ROOT="/home/finley/Github/DroneProjects/flyby-f11"
TEST_DIR="$PROJECT_ROOT/ontology/planning_mode/test_scenarios/canonical"
LOG_DIR="$PROJECT_ROOT/ontology/planning_mode/logs"
CONTAINER_IMAGE="flyby-f11-planning:latest"
TIME_LIMIT=30  # seconds (generous limit, tests should be <100ms)
VERBOSE=false

# Parse arguments
if [[ "$1" == "--verbose" || "$1" == "-v" ]]; then
    VERBOSE=true
fi

# Create log directory
mkdir -p "$LOG_DIR"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test counters
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0
TOTAL_TIME_MS=0

echo ""
echo "================================================================================"
echo "  Flyby F-11 UAV Ontology - Canonical Test Suite"
echo "  Running Vampire Theorem Prover Tests"
echo "================================================================================"
echo ""

#------------------------------------------------------------------------------
# Function: run_vampire_test
# Arguments:
#   $1 - Test name (for display)
#   $2 - Test file path (relative to workspace)
#   $3 - Expected result ("Theorem" for provable conjecture)
#------------------------------------------------------------------------------
run_vampire_test() {
    local test_name="$1"
    local test_file="$2"
    local expected="$3"
    local log_file="$LOG_DIR/$(basename "$test_file" .tptp).log"

    TESTS_RUN=$((TESTS_RUN + 1))

    echo -n "  [$TESTS_RUN] $test_name... "

    # Capture start time
    local start_time=$(date +%s%3N)

    # Run Vampire in container
    local output
    if ! output=$(podman run --rm \
        -v "$PROJECT_ROOT/ontology:/workspace:z" \
        "$CONTAINER_IMAGE" \
        vampire --input_syntax tptp --time_limit "$TIME_LIMIT" \
        "/workspace/planning_mode/test_scenarios/canonical/$(basename "$test_file")" 2>&1); then
        # Vampire returns non-zero for various reasons, check output
        :
    fi

    # Capture end time
    local end_time=$(date +%s%3N)
    local duration_ms=$((end_time - start_time))
    TOTAL_TIME_MS=$((TOTAL_TIME_MS + duration_ms))

    # Save full output to log
    echo "$output" > "$log_file"

    # Check result
    local result=""
    if echo "$output" | grep -q "SZS status Theorem"; then
        result="Theorem"
    elif echo "$output" | grep -q "SZS status CounterSatisfiable"; then
        result="CounterSatisfiable"
    elif echo "$output" | grep -q "SZS status Satisfiable"; then
        result="Satisfiable"
    elif echo "$output" | grep -q "SZS status Timeout"; then
        result="Timeout"
    elif echo "$output" | grep -q "SZS status"; then
        result=$(echo "$output" | grep "SZS status" | head -1 | sed 's/.*SZS status //' | awk '{print $1}')
    else
        result="Unknown"
    fi

    # Evaluate pass/fail
    local status_msg=""
    local status_color=""

    if [[ "$result" == "$expected" ]]; then
        TESTS_PASSED=$((TESTS_PASSED + 1))
        status_color="$GREEN"
        status_msg="PASS"
    else
        TESTS_FAILED=$((TESTS_FAILED + 1))
        status_color="$RED"
        status_msg="FAIL"
    fi

    # Print result
    echo -e "${status_color}${status_msg}${NC} (${duration_ms}ms) [Result: $result]"

    # Print timing warning if >100ms
    if [[ $duration_ms -gt 100 ]]; then
        echo -e "    ${YELLOW}WARNING: Test exceeded 100ms target${NC}"
    fi

    # Verbose output
    if [[ "$VERBOSE" == "true" && "$result" != "$expected" ]]; then
        echo "    Expected: $expected"
        echo "    Got: $result"
        echo "    Log: $log_file"
        echo ""
        echo "    --- Output Preview ---"
        echo "$output" | head -20 | sed 's/^/    /'
        echo "    --- End Preview ---"
        echo ""
    fi

    return 0
}

#------------------------------------------------------------------------------
# Run all canonical tests
#------------------------------------------------------------------------------

echo "Running Canonical Problem Tests:"
echo "--------------------------------"
echo ""

# Test 1: Communications-Denied Surveillance
run_vampire_test \
    "Comms-Denied Autonomous Mode" \
    "canonical_1_comms_denied.tptp" \
    "Theorem"

# Test 2: Dynamic NFZ Adaptation
run_vampire_test \
    "Dynamic NFZ Path Invalidation" \
    "canonical_2_dynamic_nfz.tptp" \
    "Theorem"

# Test 3: Multi-Objective ISR Prioritization
run_vampire_test \
    "Critical Target Prioritization" \
    "canonical_3_multi_objective.tptp" \
    "Theorem"

#------------------------------------------------------------------------------
# Print summary
#------------------------------------------------------------------------------

echo ""
echo "================================================================================"
echo "  Test Summary"
echo "================================================================================"
echo ""
echo "  Tests Run:    $TESTS_RUN"
echo -e "  Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "  Tests Failed: ${RED}$TESTS_FAILED${NC}"
echo ""
echo "  Total Time:   ${TOTAL_TIME_MS}ms"
echo "  Avg Time:     $((TOTAL_TIME_MS / TESTS_RUN))ms per test"
echo ""

# Timing evaluation
if [[ $((TOTAL_TIME_MS / TESTS_RUN)) -le 100 ]]; then
    echo -e "  ${GREEN}Timing: All tests within 100ms target${NC}"
else
    echo -e "  ${YELLOW}Timing: Some tests exceeded 100ms target${NC}"
fi

echo ""
echo "  Logs saved to: $LOG_DIR"
echo ""

# Final status
if [[ $TESTS_FAILED -eq 0 ]]; then
    echo -e "================================================================================"
    echo -e "  ${GREEN}ALL CANONICAL TESTS PASSED${NC}"
    echo -e "================================================================================"
    echo ""
    echo "  The ontology correctly reasons about:"
    echo "    - Communications-denied autonomous authority"
    echo "    - Dynamic NFZ path intersection detection"
    echo "    - Multi-objective target prioritization"
    echo ""
    exit 0
else
    echo -e "================================================================================"
    echo -e "  ${RED}CANONICAL TESTS FAILED${NC}"
    echo -e "================================================================================"
    echo ""
    echo "  Review failed tests and check:"
    echo "    - Axiom definitions in isr_extensions.kif"
    echo "    - Test scenario setup in canonical/*.tptp"
    echo "    - Log files for detailed Vampire output"
    echo ""
    exit 1
fi
