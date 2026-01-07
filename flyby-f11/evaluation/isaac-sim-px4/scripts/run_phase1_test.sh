#!/bin/bash
# Phase 1 Test Runner: Render Frame Skipping
#
# Tests the render frame skipping optimization for faster RL training.
# Produces video evidence and performance metrics.
#
# Usage:
#   ./scripts/run_phase1_test.sh                    # Default: render_interval=4
#   ./scripts/run_phase1_test.sh --render-interval 2  # Custom interval
#   ./scripts/run_phase1_test.sh --test-steps 2000    # More steps
#
# Output:
#   output/videos/phase1_render_skip_test.mp4  - Annotated video
#   output/videos/phase1_render_skip_report.json - Performance metrics
#
# Success Criteria:
#   - FPS >= 90 (1.5x baseline of 60)
#   - Video shows frame skip annotations
#   - Detections visible on rendered frames

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Default parameters
RENDER_INTERVAL=4
TEST_STEPS=1000
EXTRA_ARGS=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --render-interval)
            RENDER_INTERVAL="$2"
            shift 2
            ;;
        --test-steps)
            TEST_STEPS="$2"
            shift 2
            ;;
        --help)
            echo "Phase 1 Test: Render Frame Skipping"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --render-interval N  Render every Nth frame (default: 4)"
            echo "  --test-steps N       Number of test steps (default: 1000)"
            echo "  --help               Show this help message"
            echo ""
            echo "Output:"
            echo "  output/videos/phase1_render_skip_test.mp4"
            echo "  output/videos/phase1_render_skip_report.json"
            exit 0
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift
            ;;
    esac
done

echo "========================================"
echo "Phase 1: Render Frame Skipping Test"
echo "========================================"
echo "Workspace: $WORKSPACE_DIR"
echo "Render Interval: $RENDER_INTERVAL"
echo "Test Steps: $TEST_STEPS"
echo ""

# Build the command
CMD="/workspace/training/tests/test_render_skipping.py --headless --render-interval $RENDER_INTERVAL --test-steps $TEST_STEPS $EXTRA_ARGS"

echo "Running: $CMD"
echo ""

podman run --rm --replace --name flyby-f11-phase1-test \
  --device nvidia.com/gpu=all \
  -v "$WORKSPACE_DIR:/workspace:Z" \
  -e ACCEPT_EULA=Y \
  -e PYTHONUNBUFFERED=1 \
  --network=host \
  --entrypoint /isaac-sim/python.sh \
  localhost/isaac-sim-px4:5.1.0-px4-1.14.3 \
  $CMD

# Check output
if [ -f "$WORKSPACE_DIR/output/videos/phase1_render_skip_test.mp4" ]; then
    echo ""
    echo "========================================"
    echo "Test Complete!"
    echo "========================================"
    echo "Video: output/videos/phase1_render_skip_test.mp4"
    echo "Report: output/videos/phase1_render_skip_report.json"
    echo ""

    # Show report summary if jq is available
    if command -v jq &> /dev/null && [ -f "$WORKSPACE_DIR/output/videos/phase1_render_skip_report.json" ]; then
        echo "Results Summary:"
        jq '.success_criteria' "$WORKSPACE_DIR/output/videos/phase1_render_skip_report.json"
    fi
else
    echo ""
    echo "WARNING: Output video not found"
    exit 1
fi
