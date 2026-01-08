#!/bin/bash
# Run ISR scenario training with Isaac Sim
#
# This script launches training for the fast ISR training environment
# inside the Isaac Sim container with proper GPU passthrough.
#
# Usage:
#   ./scripts/run_phase2_isr_training.sh comms_denied
#   ./scripts/run_phase2_isr_training.sh nfz_avoidance --num_envs 64
#   ./scripts/run_phase2_isr_training.sh multi_objective --trainer dummy

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Default values
SCENARIO="${1:-comms_denied}"
shift || true  # Shift args if scenario was provided

# Isaac Sim settings
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-/isaac-sim}"
PYTHON_PATH="$ISAAC_SIM_PATH/python.sh"

# Check if running inside container
if [ -f "$PYTHON_PATH" ]; then
    echo "============================================================"
    echo "ISR Training - Scenario: $SCENARIO"
    echo "============================================================"
    echo ""
    echo "Project directory: $PROJECT_DIR"
    echo "Isaac Sim path: $ISAAC_SIM_PATH"
    echo "Additional args: $@"
    echo ""

    # Set PYTHONPATH to include training module
    export PYTHONPATH="$PROJECT_DIR:$PYTHONPATH"

    # Run training script
    "$PYTHON_PATH" "$PROJECT_DIR/training/scripts/train_scenario.py" \
        --scenario "$SCENARIO" \
        --headless \
        "$@"
else
    echo "============================================================"
    echo "Isaac Sim not found at $ISAAC_SIM_PATH"
    echo "============================================================"
    echo ""
    echo "This script should be run inside the Isaac Sim container."
    echo ""
    echo "To start the container, run:"
    echo "  podman-compose up"
    echo ""
    echo "Then exec into it:"
    echo "  podman exec -it flyby-f11-dev bash"
    echo ""
    echo "And run this script:"
    echo "  ./scripts/run_phase2_isr_training.sh $SCENARIO $@"
    echo ""
    echo "Alternatively, run the phase 2 test first:"
    echo "  ./scripts/run_phase2_test.sh"
    exit 1
fi
