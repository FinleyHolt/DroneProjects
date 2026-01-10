#!/bin/bash
# Train with GUI + Ground Truth perception (fast, for debugging)
# Use this to debug environment/reward issues without YOLO overhead

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo " Flyby F-11 Training: GUI + Ground Truth"
echo "=============================================="
echo ""
echo "Config: Ground truth perception (no YOLO), with GUI"
echo "Use this for debugging environment and reward shaping"
echo ""

cd "$WORKSPACE_DIR"

# Override perception mode to ground_truth via environment variable
export FLYBY_PERCEPTION_MODE=ground_truth

/isaac-sim/python.sh scripts/training/train_canonical.py \
    --problem comms_denied \
    --agent-type trajectory_optimizer \
    --config config/training_config.yaml \
    --checkpoint-dir ./checkpoints \
    --log-dir ./logs \
    --gui \
    "$@"
