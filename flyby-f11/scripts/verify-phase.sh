#!/bin/bash
# Flyby F-11 Phase Verification Script
# Verify completion status of one or all phases

set -e

PROJECT_ROOT="/home/finley/Github/DroneProjects/flyby-f11"
PHASES_DIR="$PROJECT_ROOT/.phases"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Usage information
usage() {
    echo "Usage: $0 [phase-name] [--verbose]"
    echo ""
    echo "Verify completion status of phases"
    echo ""
    echo "Options:"
    echo "  [phase-name]  Specific phase to verify (default: all phases)"
    echo "  --verbose     Show detailed verification output"
    echo ""
    echo "Examples:"
    echo "  $0                              # Verify all phases"
    echo "  $0 phase-01-ontology-toolchain  # Verify specific phase"
    echo "  $0 --verbose                    # Verify all with details"
    exit 1
}

# Parse arguments
PHASE_NAME=""
VERBOSE=false

while [ $# -gt 0 ]; do
    case "$1" in
        --verbose)
            VERBOSE=true
            ;;
        --help|-h)
            usage
            ;;
        *)
            PHASE_NAME="$1"
            ;;
    esac
    shift
done

echo -e "${BLUE}========================================"
echo "Flyby F-11 Phase Verification"
echo -e "========================================${NC}"
echo ""

# Verify single phase
verify_phase() {
    local phase="$1"
    local phase_dir="$PHASES_DIR/$phase"
    local verify_script="$phase_dir/verification.sh"

    if [ ! -d "$phase_dir" ]; then
        echo -e "${RED}✗ $phase - NOT FOUND${NC}"
        return 1
    fi

    if [ ! -f "$verify_script" ]; then
        echo -e "${YELLOW}⊘ $phase - NO VERIFICATION SCRIPT${NC}"
        return 1
    fi

    # Make executable
    chmod +x "$verify_script"

    # Run verification
    if [ "$VERBOSE" = true ]; then
        echo -e "${BLUE}Verifying $phase:${NC}"
        if bash "$verify_script"; then
            echo -e "${GREEN}✓ $phase - PASSED${NC}"
            return 0
        else
            echo -e "${RED}✗ $phase - FAILED${NC}"
            return 1
        fi
    else
        if bash "$verify_script" > /dev/null 2>&1; then
            echo -e "${GREEN}✓ $phase - PASSED${NC}"
            return 0
        else
            echo -e "${RED}✗ $phase - FAILED${NC}"
            return 1
        fi
    fi
}

# Verify all phases
if [ -z "$PHASE_NAME" ]; then
    # Get all phases
    PHASES=$(ls -1 "$PHASES_DIR" 2>/dev/null | grep "^phase-" | sort || true)

    if [ -z "$PHASES" ]; then
        echo -e "${RED}No phases found in $PHASES_DIR${NC}"
        exit 1
    fi

    TOTAL=0
    PASSED=0
    FAILED=0
    NO_SCRIPT=0

    echo "Verifying all phases:"
    echo ""

    for phase in $PHASES; do
        ((TOTAL++))
        if verify_phase "$phase"; then
            ((PASSED++))
        else
            if [ ! -f "$PHASES_DIR/$phase/verification.sh" ]; then
                ((NO_SCRIPT++))
            else
                ((FAILED++))
            fi
        fi
    done

    echo ""
    echo -e "${BLUE}========================================"
    echo "Summary"
    echo -e "========================================${NC}"
    echo "Total phases:      $TOTAL"
    echo -e "${GREEN}Passed:            $PASSED${NC}"
    echo -e "${RED}Failed:            $FAILED${NC}"
    if [ $NO_SCRIPT -gt 0 ]; then
        echo -e "${YELLOW}No verification:   $NO_SCRIPT${NC}"
    fi
    echo ""

    if [ $FAILED -eq 0 ]; then
        echo -e "${GREEN}✓ All phases with verification passed${NC}"
        exit 0
    else
        echo -e "${RED}✗ Some phases failed verification${NC}"
        echo ""
        echo "To see details for a failed phase:"
        echo "  bash scripts/verify-phase.sh <phase-name> --verbose"
        exit 1
    fi

# Verify specific phase
else
    if verify_phase "$PHASE_NAME"; then
        exit 0
    else
        echo ""
        echo "To see detailed output:"
        echo "  bash scripts/verify-phase.sh $PHASE_NAME --verbose"
        exit 1
    fi
fi
