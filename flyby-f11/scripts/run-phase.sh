#!/bin/bash
# Flyby F-11 Phase Execution Script
# Execute a specific development phase with automated verification

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
    echo "Usage: $0 <phase-name> [--verify-only] [--skip-verification]"
    echo ""
    echo "Execute a development phase with automated verification"
    echo ""
    echo "Options:"
    echo "  <phase-name>           Phase to execute (e.g., phase-01-ontology-toolchain)"
    echo "  --verify-only          Only run verification, skip execution"
    echo "  --skip-verification    Execute without running verification"
    echo ""
    echo "Available phases:"
    ls -1 "$PHASES_DIR" 2>/dev/null | grep "^phase-" || echo "  No phases found"
    echo ""
    echo "Examples:"
    echo "  $0 phase-01-ontology-toolchain"
    echo "  $0 phase-02-uav-ontology --verify-only"
    exit 1
}

# Parse arguments
if [ $# -lt 1 ]; then
    usage
fi

PHASE_NAME="$1"
VERIFY_ONLY=false
SKIP_VERIFICATION=false

shift
while [ $# -gt 0 ]; do
    case "$1" in
        --verify-only)
            VERIFY_ONLY=true
            ;;
        --skip-verification)
            SKIP_VERIFICATION=true
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
    shift
done

# Validate phase exists
PHASE_DIR="$PHASES_DIR/$PHASE_NAME"
if [ ! -d "$PHASE_DIR" ]; then
    echo -e "${RED}Error: Phase '$PHASE_NAME' not found${NC}"
    echo "Available phases:"
    ls -1 "$PHASES_DIR" | grep "^phase-"
    exit 1
fi

# Load phase metadata
TASK_FILE="$PHASE_DIR/TASK.md"
VERIFY_SCRIPT="$PHASE_DIR/verification.sh"
INPUTS_FILE="$PHASE_DIR/inputs.json"
OUTPUTS_FILE="$PHASE_DIR/outputs.json"
DEPS_FILE="$PHASE_DIR/dependencies.json"

echo -e "${BLUE}========================================"
echo "Flyby F-11 Phase Execution"
echo -e "========================================${NC}"
echo ""
echo -e "Phase: ${GREEN}$PHASE_NAME${NC}"
echo "Location: $PHASE_DIR"
echo ""

# Check dependencies
if [ -f "$DEPS_FILE" ]; then
    echo -e "${YELLOW}Checking dependencies...${NC}"

    # Extract dependency phases using grep and basic parsing
    DEPS=$(grep -oP '"depends_on":\s*\[\s*"\K[^"]+' "$DEPS_FILE" || true)

    if [ -n "$DEPS" ]; then
        echo "Required phases:"
        for dep in $DEPS; do
            echo "  - $dep"

            # Check if dependency phase verification passes
            DEP_VERIFY="$PHASES_DIR/$dep/verification.sh"
            if [ -f "$DEP_VERIFY" ]; then
                if bash "$DEP_VERIFY" > /dev/null 2>&1; then
                    echo -e "    ${GREEN}✓ Completed${NC}"
                else
                    echo -e "    ${RED}✗ Not completed${NC}"
                    echo ""
                    echo -e "${RED}Error: Dependency $dep has not been completed${NC}"
                    echo "Complete $dep before running $PHASE_NAME"
                    exit 1
                fi
            fi
        done
        echo ""
    else
        echo "  No dependencies"
        echo ""
    fi
fi

# Display task information
if [ -f "$TASK_FILE" ]; then
    echo -e "${BLUE}Task Description:${NC}"
    # Extract overview section
    sed -n '/## Overview/,/## Human Description/p' "$TASK_FILE" | head -n -1 | tail -n +2
    echo ""
fi

# Verify-only mode
if [ "$VERIFY_ONLY" = true ]; then
    echo -e "${YELLOW}Running verification only...${NC}"
    echo ""

    if [ -f "$VERIFY_SCRIPT" ]; then
        chmod +x "$VERIFY_SCRIPT"
        bash "$VERIFY_SCRIPT"
        exit $?
    else
        echo -e "${RED}Error: No verification script found${NC}"
        exit 1
    fi
fi

# Interactive execution
echo -e "${YELLOW}========================================"
echo "Phase Execution"
echo -e "========================================${NC}"
echo ""
echo "This phase requires manual implementation following TASK.md"
echo ""
echo "Steps to complete:"
echo "  1. Read: $TASK_FILE"
echo "  2. Review required inputs: $INPUTS_FILE"
echo "  3. Implement tasks as described"
echo "  4. Verify outputs match: $OUTPUTS_FILE"
echo "  5. Run verification: bash $VERIFY_SCRIPT"
echo ""

# Ask if user wants to view task details
read -p "View detailed task instructions? (y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    less "$TASK_FILE"
fi

echo ""
echo -e "${YELLOW}When you've completed the implementation, run:${NC}"
echo "  bash scripts/run-phase.sh $PHASE_NAME --verify-only"
echo ""

# Optionally run verification at the end
if [ "$SKIP_VERIFICATION" = false ]; then
    read -p "Run verification now? (y/n): " -n 1 -r
    echo ""

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo ""
        if [ -f "$VERIFY_SCRIPT" ]; then
            chmod +x "$VERIFY_SCRIPT"
            bash "$VERIFY_SCRIPT"
            exit $?
        else
            echo -e "${RED}Error: No verification script found${NC}"
            exit 1
        fi
    fi
fi

echo -e "${GREEN}Phase execution script completed${NC}"
