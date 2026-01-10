#!/bin/bash
# ==============================================================================
# Flyby F-11 RL Pipeline - Complete Test Suite Runner
# ==============================================================================
#
# This script runs all tests in sequence:
# 1. Static analysis (no dependencies required)
# 2. Unit tests (with mocked Isaac Sim)
# 3. Integration tests (requires Isaac Sim container)
# 4. E2E smoke test (requires Isaac Sim container)
#
# Usage:
#   ./scripts/run_all_tests.sh           # Run all tests
#   ./scripts/run_all_tests.sh --static  # Only static tests
#   ./scripts/run_all_tests.sh --unit    # Only unit tests
#   ./scripts/run_all_tests.sh --quick   # Quick mode (static + unit only)
#   ./scripts/run_all_tests.sh --all     # All tests including container tests
#
# ==============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
TESTS_DIR="${PROJECT_ROOT}/tests"

# Default settings
RUN_STATIC=true
RUN_UNIT=true
RUN_INTEGRATION=false
RUN_E2E=false
VERBOSE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --static)
            RUN_STATIC=true
            RUN_UNIT=false
            RUN_INTEGRATION=false
            RUN_E2E=false
            shift
            ;;
        --unit)
            RUN_STATIC=false
            RUN_UNIT=true
            RUN_INTEGRATION=false
            RUN_E2E=false
            shift
            ;;
        --integration)
            RUN_STATIC=false
            RUN_UNIT=false
            RUN_INTEGRATION=true
            RUN_E2E=false
            shift
            ;;
        --e2e)
            RUN_STATIC=false
            RUN_UNIT=false
            RUN_INTEGRATION=false
            RUN_E2E=true
            shift
            ;;
        --quick)
            RUN_STATIC=true
            RUN_UNIT=true
            RUN_INTEGRATION=false
            RUN_E2E=false
            shift
            ;;
        --all)
            RUN_STATIC=true
            RUN_UNIT=true
            RUN_INTEGRATION=true
            RUN_E2E=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --static       Run only static analysis tests"
            echo "  --unit         Run only unit tests"
            echo "  --integration  Run only integration tests (requires Isaac Sim)"
            echo "  --e2e          Run only end-to-end smoke tests (requires Isaac Sim)"
            echo "  --quick        Run static + unit tests only (default)"
            echo "  --all          Run all tests"
            echo "  -v, --verbose  Verbose output"
            echo "  -h, --help     Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Function to print section header
print_header() {
    echo -e "\n${BLUE}================================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================================================${NC}\n"
}

# Function to print status
print_status() {
    if [ "$2" == "pass" ]; then
        echo -e "${GREEN}[PASS]${NC} $1"
    elif [ "$2" == "fail" ]; then
        echo -e "${RED}[FAIL]${NC} $1"
    elif [ "$2" == "skip" ]; then
        echo -e "${YELLOW}[SKIP]${NC} $1"
    else
        echo -e "[INFO] $1"
    fi
}

# Track results
STATIC_RESULT=0
UNIT_RESULT=0
INTEGRATION_RESULT=0
E2E_RESULT=0

# ==============================================================================
# Static Analysis Tests
# ==============================================================================
if [ "$RUN_STATIC" = true ]; then
    print_header "STATIC ANALYSIS TESTS"

    cd "$PROJECT_ROOT"

    # Check if pytest is available
    if ! command -v python3 &> /dev/null; then
        print_status "Python3 not found" "fail"
        STATIC_RESULT=1
    else
        # Run static tests
        echo "Running static analysis tests..."

        if [ "$VERBOSE" = true ]; then
            python3 -m pytest tests/test_static.py -v --tb=short
        else
            python3 -m pytest tests/test_static.py -v --tb=line
        fi
        STATIC_RESULT=$?

        if [ $STATIC_RESULT -eq 0 ]; then
            print_status "Static analysis tests" "pass"
        else
            print_status "Static analysis tests" "fail"
        fi
    fi
fi

# ==============================================================================
# Unit Tests
# ==============================================================================
if [ "$RUN_UNIT" = true ]; then
    print_header "UNIT TESTS (Mocked Isaac Sim)"

    cd "$PROJECT_ROOT"

    echo "Running unit tests with mocked dependencies..."

    if [ "$VERBOSE" = true ]; then
        python3 -m pytest tests/test_unit.py -v --tb=short
    else
        python3 -m pytest tests/test_unit.py -v --tb=line
    fi
    UNIT_RESULT=$?

    if [ $UNIT_RESULT -eq 0 ]; then
        print_status "Unit tests" "pass"
    else
        print_status "Unit tests" "fail"
    fi
fi

# ==============================================================================
# Integration Tests (requires Isaac Sim)
# ==============================================================================
if [ "$RUN_INTEGRATION" = true ]; then
    print_header "INTEGRATION TESTS (Requires Isaac Sim)"

    # Check if we're inside Isaac Sim environment
    if [ -d "/isaac-sim" ]; then
        echo "Running integration tests inside Isaac Sim container..."

        if [ "$VERBOSE" = true ]; then
            /isaac-sim/python.sh -m pytest /workspace/tests/test_integration.py -v --tb=short
        else
            /isaac-sim/python.sh -m pytest /workspace/tests/test_integration.py -v --tb=line
        fi
        INTEGRATION_RESULT=$?

        if [ $INTEGRATION_RESULT -eq 0 ]; then
            print_status "Integration tests" "pass"
        else
            print_status "Integration tests" "fail"
        fi
    else
        print_status "Integration tests - Not in Isaac Sim container, skipping" "skip"
        echo "  To run integration tests, use:"
        echo "    ./scripts/run_smoke_test_in_container.sh --integration"
    fi
fi

# ==============================================================================
# E2E Smoke Tests (requires Isaac Sim)
# ==============================================================================
if [ "$RUN_E2E" = true ]; then
    print_header "END-TO-END SMOKE TESTS (Requires Isaac Sim)"

    # Check if we're inside Isaac Sim environment
    if [ -d "/isaac-sim" ]; then
        echo "Running E2E smoke tests inside Isaac Sim container..."

        if [ "$VERBOSE" = true ]; then
            /isaac-sim/python.sh -m pytest /workspace/tests/test_e2e_smoke.py -v -s --tb=short
        else
            /isaac-sim/python.sh /workspace/tests/test_e2e_smoke.py --quick
        fi
        E2E_RESULT=$?

        if [ $E2E_RESULT -eq 0 ]; then
            print_status "E2E smoke tests" "pass"
        else
            print_status "E2E smoke tests" "fail"
        fi
    else
        print_status "E2E smoke tests - Not in Isaac Sim container, skipping" "skip"
        echo "  To run E2E tests, use:"
        echo "    ./scripts/run_smoke_test_in_container.sh --e2e"
    fi
fi

# ==============================================================================
# Summary
# ==============================================================================
print_header "TEST SUMMARY"

TOTAL_PASS=0
TOTAL_FAIL=0

if [ "$RUN_STATIC" = true ]; then
    if [ $STATIC_RESULT -eq 0 ]; then
        print_status "Static Analysis" "pass"
        ((TOTAL_PASS++))
    else
        print_status "Static Analysis" "fail"
        ((TOTAL_FAIL++))
    fi
fi

if [ "$RUN_UNIT" = true ]; then
    if [ $UNIT_RESULT -eq 0 ]; then
        print_status "Unit Tests" "pass"
        ((TOTAL_PASS++))
    else
        print_status "Unit Tests" "fail"
        ((TOTAL_FAIL++))
    fi
fi

if [ "$RUN_INTEGRATION" = true ]; then
    if [ -d "/isaac-sim" ]; then
        if [ $INTEGRATION_RESULT -eq 0 ]; then
            print_status "Integration Tests" "pass"
            ((TOTAL_PASS++))
        else
            print_status "Integration Tests" "fail"
            ((TOTAL_FAIL++))
        fi
    else
        print_status "Integration Tests" "skip"
    fi
fi

if [ "$RUN_E2E" = true ]; then
    if [ -d "/isaac-sim" ]; then
        if [ $E2E_RESULT -eq 0 ]; then
            print_status "E2E Smoke Tests" "pass"
            ((TOTAL_PASS++))
        else
            print_status "E2E Smoke Tests" "fail"
            ((TOTAL_FAIL++))
        fi
    else
        print_status "E2E Smoke Tests" "skip"
    fi
fi

echo ""
echo -e "Total: ${GREEN}${TOTAL_PASS} passed${NC}, ${RED}${TOTAL_FAIL} failed${NC}"

# Exit with failure if any test failed
if [ $TOTAL_FAIL -gt 0 ]; then
    exit 1
fi

exit 0
