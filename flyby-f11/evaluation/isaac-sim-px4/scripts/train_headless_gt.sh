#!/bin/bash
# Train headless + Ground Truth perception (fastest, for rapid iteration)
# Use this for fast training iteration when testing RL algorithms

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo " Flyby F-11 Training: Headless + Ground Truth"
echo "=============================================="
echo ""
echo "Config: Ground truth perception, no GUI (fastest)"
echo "Use this for rapid RL algorithm iteration"
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
    --headless \
    "$@"
