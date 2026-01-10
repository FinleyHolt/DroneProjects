#!/bin/bash
# ==============================================================================
# ISR RL Policy Training Script
# ==============================================================================
# Trains SearchPolicy (PPO) and DwellPolicy (SAC) for ISR drone autonomy.
#
# Usage:
#   ./scripts/run_training.sh                    # Full training (~24 hours)
#   ./scripts/run_training.sh --quick            # Quick test (~5 min)
#   ./scripts/run_training.sh --search-only      # Train SearchPolicy only
#   ./scripts/run_training.sh --dwell-only       # Train DwellPolicy only
#
# Container: localhost/isaac-sim-px4:5.1.0-px4-1.14.3
# ==============================================================================

set -e

# Get script directory (for volume mounts)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Container settings
IMAGE="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

# Default values
QUICK_TEST=false
SEARCH_ONLY=false
DWELL_ONLY=false
NUM_ENVS=1024
SEARCH_STEPS=2000000
DWELL_STEPS=1500000

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --quick)
            QUICK_TEST=true
            NUM_ENVS=64
            SEARCH_STEPS=10000
            DWELL_STEPS=10000
            shift
            ;;
        --search-only)
            SEARCH_ONLY=true
            shift
            ;;
        --dwell-only)
            DWELL_ONLY=true
            shift
            ;;
        --num-envs)
            NUM_ENVS="$2"
            shift 2
            ;;
        --search-steps)
            SEARCH_STEPS="$2"
            shift 2
            ;;
        --dwell-steps)
            DWELL_STEPS="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo ""
            echo "Usage: $0 [options]"
            echo "  --quick          Quick test (~5 min, 10k steps)"
            echo "  --search-only    Train SearchPolicy only"
            echo "  --dwell-only     Train DwellPolicy only"
            echo "  --num-envs N     Number of parallel environments"
            echo "  --search-steps N Total steps for SearchPolicy"
            echo "  --dwell-steps N  Total steps for DwellPolicy"
            exit 1
            ;;
    esac
done

# Output directory
if [ "$QUICK_TEST" = true ]; then
    OUTPUT_BASE="outputs/quick_test"
else
    OUTPUT_BASE="outputs/full"
fi

echo "=============================================="
echo "ISR RL Policy Training"
echo "=============================================="
echo "Quick test:        $QUICK_TEST"
echo "Num envs:          $NUM_ENVS"
echo "Search steps:      $SEARCH_STEPS"
echo "Dwell steps:       $DWELL_STEPS"
echo "Image:             $IMAGE"
echo "Project dir:       $PROJECT_DIR"
echo "Output dir:        $OUTPUT_BASE"
echo "=============================================="

# Create output directories with proper permissions
# Use podman unshare to handle container user-mapped files without sudo
clean_with_podman() {
    local dir="$1"
    if [ -d "$dir" ]; then
        # Try normal rm first, then use podman unshare for container-owned files
        rm -rf "$dir" 2>/dev/null || podman unshare rm -rf "$dir" 2>/dev/null || true
    fi
}

if [ -d "$PROJECT_DIR/$OUTPUT_BASE" ]; then
    echo "Cleaning old output directory..."
    clean_with_podman "$PROJECT_DIR/$OUTPUT_BASE"
fi

# Create fresh directories (don't clean logs - just create subdirs)
mkdir -p "$PROJECT_DIR/$OUTPUT_BASE/search_policy" "$PROJECT_DIR/$OUTPUT_BASE/dwell_policy"
mkdir -p "$PROJECT_DIR/logs/training"
chmod 777 "$PROJECT_DIR/$OUTPUT_BASE" "$PROJECT_DIR/$OUTPUT_BASE/search_policy" "$PROJECT_DIR/$OUTPUT_BASE/dwell_policy" 2>/dev/null || true
chmod 777 "$PROJECT_DIR/logs" "$PROJECT_DIR/logs/training" 2>/dev/null || true

# Common podman run arguments (matching working run_test.sh pattern)
PODMAN_ARGS=(
    --rm
    --device nvidia.com/gpu=all
    -v "$PROJECT_DIR/training:/workspace/training:Z"
    -v "$PROJECT_DIR/$OUTPUT_BASE:/workspace/$OUTPUT_BASE:Z"
    -v "$PROJECT_DIR/logs:/workspace/logs:Z"
    -e ACCEPT_EULA=Y
    -e PYTHONUNBUFFERED=1
    --network host
    --entrypoint /isaac-sim/python.sh
)

# Log file
LOG_FILE="$PROJECT_DIR/logs/rl_training_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to: $LOG_FILE"
echo ""

# Run training scripts directly (not via detached container + exec)
# This matches the working minimal test execution pattern
{
    # Train SearchPolicy
    if [ "$DWELL_ONLY" = false ]; then
        echo "=============================================="
        echo "Training SearchPolicy"
        echo "=============================================="
        podman run "${PODMAN_ARGS[@]}" "$IMAGE" \
            /workspace/training/scripts/train_search_policy.py \
            --num-envs "$NUM_ENVS" \
            --total-steps "$SEARCH_STEPS" \
            --output-dir "/workspace/$OUTPUT_BASE/search_policy" \
            --device cuda:0
    fi

    # Train DwellPolicy
    if [ "$SEARCH_ONLY" = false ]; then
        echo "=============================================="
        echo "Training DwellPolicy"
        echo "=============================================="
        podman run "${PODMAN_ARGS[@]}" "$IMAGE" \
            /workspace/training/scripts/train_dwell_policy.py \
            --num-envs "$NUM_ENVS" \
            --total-steps "$DWELL_STEPS" \
            --output-dir "/workspace/$OUTPUT_BASE/dwell_policy" \
            --device cuda:0
    fi
} 2>&1 | tee "$LOG_FILE"

# Capture exit code
EXIT_CODE=${PIPESTATUS[0]}

echo ""
echo "=============================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo "Training completed successfully!"
else
    echo "Training exited with code: $EXIT_CODE"
fi
echo "Log file: $LOG_FILE"
echo "Outputs:  $PROJECT_DIR/$OUTPUT_BASE"
echo "=============================================="

exit $EXIT_CODE
