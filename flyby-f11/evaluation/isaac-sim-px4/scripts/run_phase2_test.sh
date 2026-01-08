#!/bin/bash
# Phase 2 Test Runner: Direct Dynamics Environment
#
# Tests the direct dynamics environment for fast RL training without PX4 SITL.
# Produces video evidence and FPS benchmarks.
#
# Usage:
#   ./scripts/run_phase2_test.sh                    # Default: render_interval=4
#   ./scripts/run_phase2_test.sh --render-interval 2  # Custom interval
#   ./scripts/run_phase2_test.sh --test-steps 3000    # More steps
#   ./scripts/run_phase2_test.sh --benchmark-only     # FPS benchmark only (no video)
#
# Output:
#   output/videos/phase2_direct_dynamics_test.mp4   - Annotated video
#   output/videos/phase2_direct_dynamics_report.json - Performance metrics
#
# Success Criteria:
#   - FPS >= 500 without rendering (render_interval=1000)
#   - FPS >= 200 with render_interval=4
#   - Drone maintains stable hover and responds to commands

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Default parameters
RENDER_INTERVAL=4
TEST_STEPS=2000
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
        --benchmark-only)
            EXTRA_ARGS="$EXTRA_ARGS --benchmark-only"
            shift
            ;;
        --help)
            echo "Phase 2 Test: Direct Dynamics Environment"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --render-interval N  Render every Nth frame (default: 4)"
            echo "  --test-steps N       Number of test steps (default: 2000)"
            echo "  --benchmark-only     Only run FPS benchmarks, skip video recording"
            echo "  --help               Show this help message"
            echo ""
            echo "Output:"
            echo "  output/videos/phase2_direct_dynamics_test.mp4"
            echo "  output/videos/phase2_direct_dynamics_report.json"
            echo ""
            echo "This test validates the direct dynamics environment which bypasses"
            echo "PX4 SITL for faster RL training. Expected speedups:"
            echo "  - 500+ FPS without rendering"
            echo "  - 200+ FPS with render_interval=4"
            echo "  - Compare to ~60-128 FPS with PX4 SITL (Phase 1)"
            exit 0
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift
            ;;
    esac
done

echo "========================================"
echo "Phase 2: Direct Dynamics Environment Test"
echo "========================================"
echo "Workspace: $WORKSPACE_DIR"
echo "Render Interval: $RENDER_INTERVAL"
echo "Test Steps: $TEST_STEPS"
echo ""

# Build the command
CMD="/workspace/training/tests/test_direct_dynamics.py --headless --render-interval $RENDER_INTERVAL --test-steps $TEST_STEPS $EXTRA_ARGS"

echo "Running: $CMD"
echo ""

podman run --rm --replace --name flyby-f11-phase2-test \
  --device nvidia.com/gpu=all \
  -v "$WORKSPACE_DIR:/workspace:Z" \
  -e ACCEPT_EULA=Y \
  -e PYTHONUNBUFFERED=1 \
  --network=host \
  --entrypoint /isaac-sim/python.sh \
  localhost/isaac-sim-px4:5.1.0-px4-1.14.3 \
  $CMD

# Check output
REPORT_FILE="$WORKSPACE_DIR/output/videos/phase2_direct_dynamics_report.json"

if [ -f "$REPORT_FILE" ]; then
    echo ""
    echo "========================================"
    echo "Test Complete!"
    echo "========================================"

    if [ -f "$WORKSPACE_DIR/output/videos/phase2_direct_dynamics_test.mp4" ]; then
        echo "Video: output/videos/phase2_direct_dynamics_test.mp4"
    fi
    echo "Report: output/videos/phase2_direct_dynamics_report.json"
    echo ""

    # Show report summary if jq is available
    if command -v jq &> /dev/null; then
        echo "Benchmark Results:"
        jq '.benchmarks | to_entries | .[] | "\(.key): \(.value.fps | floor) FPS"' "$REPORT_FILE" -r
        echo ""
        echo "Success Criteria:"
        jq '.success_criteria' "$REPORT_FILE"
    fi
else
    echo ""
    echo "WARNING: Report file not found"
    exit 1
fi
