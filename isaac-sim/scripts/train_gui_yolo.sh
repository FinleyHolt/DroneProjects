#!/bin/bash
# Train with GUI + YOLO inference (full perception pipeline)
# Use this to visually validate training is working correctly

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo " Flyby F-11 Training: GUI + YOLO Inference"
echo "=============================================="
echo ""
echo "Config: Full perception pipeline with live YOLO detection"
echo "Use this to watch training and verify perception is working"
echo ""

cd "$WORKSPACE_DIR"

/isaac-sim/python.sh scripts/training/train_canonical.py \
    --problem comms_denied \
    --agent-type trajectory_optimizer \
    --config config/training_config.yaml \
    --checkpoint-dir ./checkpoints \
    --log-dir ./logs \
    --gui \
    "$@"
