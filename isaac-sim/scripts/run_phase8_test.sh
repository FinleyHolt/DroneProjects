#!/bin/bash
# Run Phase 8: YOLO Perception with Temporal Tracking test
# Usage: ./run_phase8_test.sh [--headless]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "Phase 8: YOLO Perception with Temporal Tracking"
echo "=============================================="

# Check if running headless
HEADLESS_FLAG=""
if [[ "$1" == "--headless" ]]; then
    HEADLESS_FLAG="--headless"
    echo "Running in HEADLESS mode"
else
    echo "Running with GUI"
fi

# Create output directory
mkdir -p "$WORKSPACE_DIR/output/phase8_perception"

# Run the test
echo ""
echo "Starting Phase 8 test..."
echo ""

/isaac-sim/python.sh "$SCRIPT_DIR/phase8_yolo_perception_test.py" $HEADLESS_FLAG

echo ""
echo "=============================================="
echo "Phase 8 test complete!"
echo "Check $WORKSPACE_DIR/output/phase8_perception/ for debug images"
echo "=============================================="
