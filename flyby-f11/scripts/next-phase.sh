#!/bin/bash
# Flyby F-11 Next Phase Advisor
# Suggests the next phase to work on based on completion status and dependencies

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PHASES_DIR="$PROJECT_ROOT/.phases"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================"
echo "Flyby F-11 Next Phase Advisor"
echo -e "========================================${NC}"
echo ""

# Get all phases in order
PHASES=$(ls -1 "$PHASES_DIR" 2>/dev/null | grep "^phase-" | sort || true)

if [ -z "$PHASES" ]; then
    echo -e "${RED}No phases found in $PHASES_DIR${NC}"
    exit 1
fi

# Check completion status of each phase
COMPLETED_PHASES=()
INCOMPLETE_PHASES=()
AVAILABLE_PHASES=()

for phase in $PHASES; do
    phase_dir="$PHASES_DIR/$phase"
    verify_script="$phase_dir/verification.sh"

    if [ -f "$verify_script" ]; then
        chmod +x "$verify_script"
        if bash "$verify_script" > /dev/null 2>&1; then
            COMPLETED_PHASES+=("$phase")
        else
            INCOMPLETE_PHASES+=("$phase")
        fi
    else
        INCOMPLETE_PHASES+=("$phase")
    fi
done

# Display current progress
echo -e "${GREEN}Completed Phases (${#COMPLETED_PHASES[@]}):${NC}"
if [ ${#COMPLETED_PHASES[@]} -eq 0 ]; then
    echo "  None yet - start with Phase 1!"
else
    for phase in "${COMPLETED_PHASES[@]}"; do
        echo -e "  ${GREEN}✓${NC} $phase"
    done
fi
echo ""

# Find next available phase
# A phase is available if all its dependencies are completed
for phase in "${INCOMPLETE_PHASES[@]}"; do
    phase_dir="$PHASES_DIR/$phase"
    deps_file="$phase_dir/dependencies.json"

    # Check if all dependencies are met
    ALL_DEPS_MET=true

    if [ -f "$deps_file" ]; then
        # Extract dependencies
        DEPS=$(grep -oP '"depends_on":\s*\[\s*"\K[^"]+' "$deps_file" || true)

        if [ -n "$DEPS" ]; then
            for dep in $DEPS; do
                # Check if dependency is in completed list
                if [[ ! " ${COMPLETED_PHASES[@]} " =~ " ${dep} " ]]; then
                    ALL_DEPS_MET=false
                    break
                fi
            done
        fi
    fi

    if [ "$ALL_DEPS_MET" = true ]; then
        AVAILABLE_PHASES+=("$phase")
    fi
done

# Recommend next phase
echo -e "${YELLOW}Available Phases (dependencies met):${NC}"
if [ ${#AVAILABLE_PHASES[@]} -eq 0 ]; then
    echo "  None - check incomplete phases for missing dependencies"
else
    for phase in "${AVAILABLE_PHASES[@]}"; do
        echo -e "  ${YELLOW}○${NC} $phase"
    done
fi
echo ""

# Suggest next phase to work on
if [ ${#AVAILABLE_PHASES[@]} -gt 0 ]; then
    NEXT_PHASE="${AVAILABLE_PHASES[0]}"

    echo -e "${BLUE}========================================"
    echo "Recommended Next Phase"
    echo -e "========================================${NC}"
    echo ""
    echo -e "Phase: ${GREEN}$NEXT_PHASE${NC}"
    echo ""

    # Display task overview
    TASK_FILE="$PHASES_DIR/$NEXT_PHASE/TASK.md"
    if [ -f "$TASK_FILE" ]; then
        echo -e "${BLUE}Overview:${NC}"
        sed -n '/## Overview/,/## Human Description/p' "$TASK_FILE" | head -n -1 | tail -n +2
        echo ""

        echo -e "${BLUE}Time Estimate:${NC}"
        grep "### Time Estimate" -A 1 "$TASK_FILE" | tail -n 1
        echo ""
    fi

    # Display dependencies
    DEPS_FILE="$PHASES_DIR/$NEXT_PHASE/dependencies.json"
    if [ -f "$DEPS_FILE" ]; then
        DEPS=$(grep -oP '"depends_on":\s*\[\s*"\K[^"]+' "$DEPS_FILE" || true)

        if [ -n "$DEPS" ]; then
            echo -e "${BLUE}Dependencies (all completed):${NC}"
            for dep in $DEPS; do
                echo -e "  ${GREEN}✓${NC} $dep"
            done
            echo ""
        fi
    fi

    # Provide command to start
    echo -e "${YELLOW}To start this phase:${NC}"
    echo "  bash scripts/run-phase.sh $NEXT_PHASE"
    echo ""
    echo -e "${YELLOW}To view task details:${NC}"
    echo "  less $PHASES_DIR/$NEXT_PHASE/TASK.md"
    echo ""

elif [ ${#COMPLETED_PHASES[@]} -eq ${#PHASES[@]//phase-/} ]; then
    echo -e "${GREEN}========================================"
    echo "🎉 All Phases Complete!"
    echo -e "========================================${NC}"
    echo ""
    echo "Congratulations! You've completed all development phases."
    echo ""
    echo "Next steps:"
    echo "  1. Deploy to Jetson using Quadlet (see quadlet/README.md)"
    echo "  2. Run integration tests on hardware"
    echo "  3. Begin mission testing"
    echo ""

else
    echo -e "${YELLOW}========================================"
    echo "Blocked Phases"
    echo -e "========================================${NC}"
    echo ""
    echo "All available phases completed. The following phases are blocked:"
    echo ""

    for phase in "${INCOMPLETE_PHASES[@]}"; do
        # Check if it's in available list
        if [[ ! " ${AVAILABLE_PHASES[@]} " =~ " ${phase} " ]]; then
            echo -e "${RED}✗ $phase${NC}"

            # Show missing dependencies
            deps_file="$PHASES_DIR/$phase/dependencies.json"
            if [ -f "$deps_file" ]; then
                DEPS=$(grep -oP '"depends_on":\s*"\K[^"]+' "$deps_file" || true)

                if [ -n "$DEPS" ]; then
                    echo "  Missing dependencies:"
                    for dep in $DEPS; do
                        if [[ ! " ${COMPLETED_PHASES[@]} " =~ " ${dep} " ]]; then
                            echo -e "    ${RED}✗${NC} $dep"
                        fi
                    done
                fi
            fi
            echo ""
        fi
    done
fi

# Progress summary
echo -e "${BLUE}========================================"
echo "Progress Summary"
echo -e "========================================${NC}"
TOTAL_PHASES=$(echo "$PHASES" | wc -l)
COMPLETED_COUNT=${#COMPLETED_PHASES[@]}
PERCENT=$((COMPLETED_COUNT * 100 / TOTAL_PHASES))

echo "Completed: $COMPLETED_COUNT / $TOTAL_PHASES ($PERCENT%)"
echo ""

# Progress bar
BAR_LENGTH=40
FILLED=$((COMPLETED_COUNT * BAR_LENGTH / TOTAL_PHASES))
EMPTY=$((BAR_LENGTH - FILLED))

echo -n "["
for ((i=0; i<FILLED; i++)); do echo -n "="; done
for ((i=0; i<EMPTY; i++)); do echo -n " "; done
echo "] $PERCENT%"
echo ""
