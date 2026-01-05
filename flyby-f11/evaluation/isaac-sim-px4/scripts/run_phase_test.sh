#!/bin/bash
# =============================================================================
# Flyby F-11 Phase Test Runner
# =============================================================================
#
# Runs incremental development phase tests in the Isaac Sim container with
# proper X11/GUI setup for Wayland environments.
#
# Usage:
#   ./run_phase_test.sh <phase>
#
# Phases:
#   2  - Procedural world test (phase2_procgen_test.py)
#   3  - Camera + ground truth perception (phase3_camera_gt_test.py)
#   4  - Gymnasium wrapper test (when created)
#   5  - Basic reward signal test (when created)
#   6  - SAC training loop test (when created)
#   7  - Safety filter test (when created)
#   8  - Full YOLO perception test (when created)
#   9  - Comms-denied scenario test (when created)
#
# Examples:
#   ./run_phase_test.sh 2      # Run Phase 2 test
#   ./run_phase_test.sh 3      # Run Phase 3 test
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

# Use the main Isaac Sim + PX4 image (includes ultralytics for YOLO)
IMAGE_NAME="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

# Parse phase number and optional --headless flag
PHASE="${1:-}"
HEADLESS_FLAG=""

# Check for --headless in arguments
for arg in "$@"; do
    if [ "$arg" = "--headless" ]; then
        HEADLESS_FLAG="--headless"
    fi
done

if [ -z "$PHASE" ]; then
    echo "Usage: $0 <phase_number> [--headless]"
    echo ""
    echo "Available phases:"
    echo "  2  - Procedural world + PX4 flight"
    echo "  3  - Ground truth perception (frustum-based)"
    echo "  3b - Camera YOLO test"
    echo "  3c - Live YOLO inference"
    echo "  4  - Gymnasium wrapper"
    echo "  5  - Reward function test"
    echo "  6  - SAC training loop"
    echo "  7  - Safety filter (Vampire ATP)"
    echo "  8  - Full YOLO perception + ByteTrack"
    echo "  9  - Comms-denied scenario (TODO)"
    exit 1
fi

# Map phase to script
case "$PHASE" in
    2)
        TEST_SCRIPT="scripts/phase2_procgen_test.py"
        PHASE_NAME="Procedural World + PX4 Flight"
        ;;
    3)
        TEST_SCRIPT="scripts/phase3_camera_gt_test.py"
        PHASE_NAME="Ground Truth Perception (no camera)"
        ;;
    3b)
        TEST_SCRIPT="scripts/phase3b_camera_yolo_test.py"
        PHASE_NAME="Camera YOLO Test"
        ;;
    3c)
        TEST_SCRIPT="scripts/phase3c_live_yolo_test.py"
        PHASE_NAME="Live YOLO Inference"
        ;;
    4)
        TEST_SCRIPT="scripts/phase4_gymnasium_test.py"
        PHASE_NAME="Gymnasium Wrapper"
        ;;
    5)
        TEST_SCRIPT="scripts/phase5_reward_test.py"
        PHASE_NAME="Basic Reward Signal"
        ;;
    6)
        TEST_SCRIPT="scripts/phase6_sac_test.py"
        PHASE_NAME="SAC Training Loop"
        ;;
    7)
        TEST_SCRIPT="scripts/phase7_safety_test.py"
        PHASE_NAME="Safety Filter (Vampire)"
        ;;
    8)
        TEST_SCRIPT="scripts/phase8_yolo_test.py"
        PHASE_NAME="Full YOLO Perception"
        ;;
    9)
        TEST_SCRIPT="scripts/phase9_comms_denied_test.py"
        PHASE_NAME="Comms-Denied Scenario"
        ;;
    *)
        echo "Unknown phase: $PHASE"
        exit 1
        ;;
esac

# Check if test script exists
if [ ! -f "$PROJECT_DIR/$TEST_SCRIPT" ]; then
    echo "Error: Test script not found: $TEST_SCRIPT"
    echo "This phase test hasn't been created yet."
    exit 1
fi

echo "============================================================================="
echo "FLYBY F-11 PHASE $PHASE TEST: $PHASE_NAME"
echo "============================================================================="
echo "Image: $IMAGE_NAME"
echo "Script: $TEST_SCRIPT"
echo "============================================================================="

# Clean up any existing container
CONTAINER_NAME="flyby-phase${PHASE}-test"
podman rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Setup X11 for GUI
echo ""
echo "[Setup] Configuring X11 for GUI..."

# Allow local X11 connections
xhost +local: 2>/dev/null || echo "[WARN] xhost command failed"

# Find Xauthority file (Wayland/Mutter creates these dynamically)
XAUTH_FILE=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
if [ -z "$XAUTH_FILE" ]; then
    # Fallback to standard X11
    XAUTH_FILE="$HOME/.Xauthority"
fi

if [ ! -f "$XAUTH_FILE" ]; then
    echo "[WARN] No Xauthority file found - GUI may not work"
    echo "       Tried: /run/user/$(id -u)/.mutter-Xwaylandauth.* and ~/.Xauthority"
fi

echo "[Setup] Using Xauthority: $XAUTH_FILE"
echo "[Setup] Display: ${DISPLAY:-:0}"
echo ""
echo "[Run] Starting Phase $PHASE test..."
echo "============================================================================="
echo ""

# Run the test in the container
# Key configuration:
#   --device nvidia.com/gpu=all   : GPU passthrough for rendering
#   --network host                : Access to PX4 MAVLink ports
#   --ipc host                    : Shared memory for Isaac Sim
#   -e DISPLAY=:0                 : X11 display
#   -v /tmp/.X11-unix             : X11 socket
#   -v $XAUTH_FILE                : X11 authentication
#   -v $SHADER_CACHE_DIR          : Persistent shader cache (speeds up startup)
#   --user root                   : Avoid permission issues with X11
#   GPU env vars                  : Ensure NVIDIA GPU is used for Vulkan/OpenGL
#
podman run --rm \
    --name "$CONTAINER_NAME" \
    --device nvidia.com/gpu=all \
    --network host \
    --ipc host \
    --security-opt label=disable \
    --ulimit memlock=-1:-1 \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    -e OMNI_KIT_ACCEPT_EULA=YES \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    -e MESA_VK_DEVICE_SELECT=10de:* \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$XAUTH_FILE:/root/.Xauthority:ro" \
    -e XAUTHORITY=/root/.Xauthority \
    -v "$PROJECT_DIR:/workspace:Z" \
    --user root \
    --entrypoint /bin/bash \
    "$IMAGE_NAME" \
    -c "rm -f /tmp/px4_lock-* /tmp/hub-isaac-sim.lock 2>/dev/null; /isaac-sim/python.sh /workspace/$TEST_SCRIPT $HEADLESS_FLAG"

echo ""
echo "============================================================================="
echo "Phase $PHASE test complete"
echo "============================================================================="
