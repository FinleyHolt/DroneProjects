#!/bin/bash
# =============================================================================
# Flyby F-11 Training Runner
# =============================================================================
#
# Starts the Isaac Sim container and runs the specified training script.
#
# Usage:
#   ./run_training.sh gui-gt      # GUI + Ground Truth (debugging)
#   ./run_training.sh gui-yolo    # GUI + YOLO perception
#   ./run_training.sh headless-gt # Headless + Ground Truth (fast)
#   ./run_training.sh headless-yolo # Headless + YOLO (production)
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

IMAGE_NAME="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"
CONTAINER_NAME="flyby-f11-training"

# Parse training mode
MODE="${1:-gui-gt}"

case "$MODE" in
    gui-gt|gui-ground-truth)
        TRAIN_SCRIPT="scripts/train_gui_gt.sh"
        NEEDS_GUI=true
        echo "Mode: GUI + Ground Truth (for debugging)"
        ;;
    gui-yolo)
        TRAIN_SCRIPT="scripts/train_gui_yolo.sh"
        NEEDS_GUI=true
        echo "Mode: GUI + YOLO perception"
        ;;
    headless-gt|headless-ground-truth)
        TRAIN_SCRIPT="scripts/train_headless_gt.sh"
        NEEDS_GUI=false
        echo "Mode: Headless + Ground Truth (fast training)"
        ;;
    headless-yolo)
        TRAIN_SCRIPT="scripts/train_headless_yolo.sh"
        NEEDS_GUI=false
        echo "Mode: Headless + YOLO (production training)"
        ;;
    *)
        echo "Unknown mode: $MODE"
        echo ""
        echo "Usage: $0 <mode>"
        echo ""
        echo "Modes:"
        echo "  gui-gt        GUI + Ground Truth (for debugging)"
        echo "  gui-yolo      GUI + YOLO perception"
        echo "  headless-gt   Headless + Ground Truth (fast)"
        echo "  headless-yolo Headless + YOLO (production)"
        exit 1
        ;;
esac

echo "============================================================================="
echo "FLYBY F-11 RL TRAINING"
echo "============================================================================="
echo "Image: $IMAGE_NAME"
echo "Script: $TRAIN_SCRIPT"
echo "============================================================================="

# Clean up any existing container
podman rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Build run arguments
RUN_ARGS="--name $CONTAINER_NAME"
RUN_ARGS="$RUN_ARGS --device nvidia.com/gpu=all"
RUN_ARGS="$RUN_ARGS --network host"
RUN_ARGS="$RUN_ARGS --ipc host"
RUN_ARGS="$RUN_ARGS --security-opt label=disable"
RUN_ARGS="$RUN_ARGS -e ACCEPT_EULA=Y"
RUN_ARGS="$RUN_ARGS -e PRIVACY_CONSENT=Y"
RUN_ARGS="$RUN_ARGS -e OMNI_KIT_ACCEPT_EULA=YES"

# Mount workspace
RUN_ARGS="$RUN_ARGS -v $SCRIPT_DIR:/workspace:Z"

if [ "$NEEDS_GUI" = true ]; then
    echo ""
    echo "[Setup] Configuring X11 for GUI..."

    # Allow local X11 connections
    xhost +local: 2>/dev/null || echo "[WARN] xhost command failed"

    RUN_ARGS="$RUN_ARGS -e DISPLAY=:0"
    RUN_ARGS="$RUN_ARGS -v /tmp/.X11-unix:/tmp/.X11-unix:rw"

    # Find and mount Xauthority (Wayland/Mutter)
    XAUTH_FILE=$(ls /run/user/1000/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
    if [ -n "$XAUTH_FILE" ]; then
        RUN_ARGS="$RUN_ARGS -v $XAUTH_FILE:/root/.Xauthority:ro"
        RUN_ARGS="$RUN_ARGS -e XAUTHORITY=/root/.Xauthority"
        echo "[Setup] Using Xauthority: $XAUTH_FILE"
    elif [ -n "$XAUTHORITY" ]; then
        RUN_ARGS="$RUN_ARGS -v $XAUTHORITY:/root/.Xauthority:ro"
        RUN_ARGS="$RUN_ARGS -e XAUTHORITY=/root/.Xauthority"
        echo "[Setup] Using XAUTHORITY: $XAUTHORITY"
    else
        echo "[WARN] No Xauthority found - GUI may not work"
    fi
fi

echo ""
echo "[Run] Starting training..."
echo "============================================================================="
echo ""

# Run the training script inside the container
# Use --user root to avoid permission issues with X11
# Use --entrypoint to override default container entrypoint
podman run $RUN_ARGS --user root --entrypoint /bin/bash "$IMAGE_NAME" "/workspace/$TRAIN_SCRIPT" "${@:2}"
