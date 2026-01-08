#!/bin/bash
# Phase 2 Test Runner: ISR Training Environment
#
# Tests the fast training environment with:
# - DirectRLEnv pattern (set_external_force_and_torque)
# - Frustum-filtered ground truth observations
# - Gimbal control
#
# Usage:
#   ./scripts/run_phase2_isr_test.sh                    # Default: 64 envs
#   ./scripts/run_phase2_isr_test.sh --num-envs 1024    # Benchmark scale
#   ./scripts/run_phase2_isr_test.sh --benchmark-only   # FPS only
#
# Output:
#   output/videos/phase2_isr_training_report.json
#
# Success Criteria:
#   - Drone maintains stable flight
#   - Gimbal responds to commands
#   - Frustum filtering works correctly
#   - FPS scales with num_envs (target: ~5k per 64 envs)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Default parameters
NUM_ENVS=64
TEST_STEPS=1000
EXTRA_ARGS=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --num-envs)
            NUM_ENVS="$2"
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
            echo "Phase 2 Test: ISR Training Environment"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --num-envs N       Number of parallel environments (default: 64)"
            echo "  --test-steps N     Number of test steps (default: 1000)"
            echo "  --benchmark-only   Only run FPS benchmark"
            echo "  --help             Show this help message"
            echo ""
            echo "Output:"
            echo "  output/videos/phase2_isr_training_report.json"
            echo ""
            echo "This test validates the fast ISR training environment which uses:"
            echo "  - set_external_force_and_torque() for drone dynamics"
            echo "  - Frustum-filtered ground truth observations"
            echo "  - Gimbal control via state integration"
            echo ""
            echo "Expected FPS (scales with num_envs):"
            echo "  - 64 envs:   ~5,000 total steps/sec"
            echo "  - 256 envs:  ~20,000 total steps/sec"
            echo "  - 1024 envs: ~50,000+ total steps/sec"
            exit 0
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift
            ;;
    esac
done

echo "========================================"
echo "Phase 2: ISR Training Environment Test"
echo "========================================"
echo "Workspace: $WORKSPACE_DIR"
echo "Parallel Environments: $NUM_ENVS"
echo "Test Steps: $TEST_STEPS"
echo ""

# Build the command
CMD="/workspace/training/tests/test_isr_training_env.py --headless --num-envs $NUM_ENVS --test-steps $TEST_STEPS $EXTRA_ARGS"

echo "Running: $CMD"
echo ""

podman run --rm --replace --name flyby-f11-phase2-isr-test \
  --device nvidia.com/gpu=all \
  -v "$WORKSPACE_DIR:/workspace:Z" \
  -e ACCEPT_EULA=Y \
  -e PYTHONUNBUFFERED=1 \
  --network=host \
  --entrypoint /isaac-sim/python.sh \
  localhost/isaac-sim-px4:5.1.0-px4-1.14.3 \
  $CMD

# Check output
REPORT_FILE="$WORKSPACE_DIR/output/videos/phase2_isr_training_report.json"

if [ -f "$REPORT_FILE" ]; then
    echo ""
    echo "========================================"
    echo "Test Complete!"
    echo "========================================"
    echo "Report: output/videos/phase2_isr_training_report.json"
    echo ""

    # Show report summary if jq is available
    if command -v jq &> /dev/null; then
        echo "Benchmark Results:"
        jq '.benchmark | "  Envs: \(.num_envs), FPS: \(.fps_total | floor)"' "$REPORT_FILE" -r

        echo ""
        echo "Success Criteria:"
        jq '.success_criteria' "$REPORT_FILE"
    fi
else
    echo ""
    echo "WARNING: Report file not found"
    exit 1
fi
