#!/bin/bash
# Train headless + YOLO inference (full perception pipeline, no GUI)
# Use this for actual training runs - faster without rendering overhead

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo " Flyby F-11 Training: Headless + YOLO Inference"
echo "=============================================="
echo ""
echo "Config: Full perception pipeline, no GUI"
echo "Use this for production training runs"
echo ""

cd "$WORKSPACE_DIR"

/isaac-sim/python.sh scripts/training/train_canonical.py \
    --problem comms_denied \
    --agent-type trajectory_optimizer \
    --config config/training_config.yaml \
    --checkpoint-dir ./checkpoints \
    --log-dir ./logs \
    --headless \
    "$@"
