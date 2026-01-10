#!/bin/bash
# =============================================================================
# Phase 9: Comms-Denied Integration Test with Video Output
# =============================================================================
#
# Validates all Phase 2-8 components work together in a realistic comms-denied
# scenario. Runs headless at accelerated speed and outputs a video with the
# drone's camera view, YOLO detections, and telemetry HUD.
#
# Usage:
#   ./run_phase9_test.sh           # Run with GUI (camera/video works)
#   ./run_phase9_test.sh --headless # Run headless (ontology test only)
#
# Output:
#   output/phase9_comms_denied/phase9_comms_denied.mp4
#
# Test Sequence:
#   1. Ontology triggers takeoff to mission altitude
#   2. Random RL actions (untrained agent) move drone, gimbal pans
#   3. YOLO+ByteTrack detects vehicles/people, tracks across frames
#   4. Comms denied at T+30s, logged as status change
#   5. Ontology triggers RTL when battery < distance-based reserve
#   6. Ontology triggers landing, episode ends
#
# Success Criteria:
#   - Takeoff executed by ontology (not RL)
#   - YOLO detections appear in video when targets visible
#   - Comms denial logged at T+30s
#   - RTL triggered when battery < required_reserve
#   - Landing completed without crash
#   - Video file generated with detection overlays and telemetry HUD
#
# KNOWN LIMITATION: Camera/video capture requires GUI mode due to Isaac Sim 5.x
# headless rendering bug. In headless mode, the test validates ontology/safety
# components but camera frames will not be captured.
# See: https://github.com/isaac-sim/IsaacLab/issues/3250
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

# Image with Isaac Sim + PX4 + YOLO
IMAGE_NAME="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

# Parse --headless flag
HEADLESS_FLAG=""
for arg in "$@"; do
    if [ "$arg" = "--headless" ]; then
        HEADLESS_FLAG="--headless"
    fi
done

echo "============================================================================="
echo "FLYBY F-11 PHASE 9: Comms-Denied Integration Test"
echo "============================================================================="
echo "Image: $IMAGE_NAME"
echo "Script: scripts/phase9_comms_denied_test.py"
if [ -n "$HEADLESS_FLAG" ]; then
    echo "Mode: HEADLESS (no GUI, video recorded)"
else
    echo "Mode: GUI (for debugging)"
fi
echo "============================================================================="

# Clean up any existing container
CONTAINER_NAME="flyby-phase9-test"
podman rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Create output directory
OUTPUT_DIR="$PROJECT_DIR/output/phase9_comms_denied"
mkdir -p "$OUTPUT_DIR"
echo ""
echo "[Setup] Output directory: $OUTPUT_DIR"

# Setup X11 for GUI (if not headless)
if [ -z "$HEADLESS_FLAG" ]; then
    echo "[Setup] Configuring X11 for GUI..."
    xhost +local: 2>/dev/null || echo "[WARN] xhost command failed"
fi

# Find Xauthority file (for GUI mode)
XAUTH_FILE=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
if [ -z "$XAUTH_FILE" ]; then
    XAUTH_FILE="$HOME/.Xauthority"
fi
echo "[Setup] Using Xauthority: $XAUTH_FILE"
echo ""
echo "[Run] Starting Phase 9 test..."
echo "============================================================================="
echo ""

# Build podman arguments
PODMAN_ARGS=(
    run --rm
    --name "$CONTAINER_NAME"
    --device nvidia.com/gpu=all
    --network host
    --ipc host
    --security-opt label=disable
    --ulimit memlock=-1:-1
    -e ACCEPT_EULA=Y
    -e PRIVACY_CONSENT=Y
    -e OMNI_KIT_ACCEPT_EULA=YES
    -e __NV_PRIME_RENDER_OFFLOAD=1
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia
    -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
    -e MESA_VK_DEVICE_SELECT=10de:*
    -v "$PROJECT_DIR:/workspace:Z"
    --user root
    --entrypoint /bin/bash
)

# Add GUI support if not headless
if [ -z "$HEADLESS_FLAG" ]; then
    PODMAN_ARGS+=(
        -e "DISPLAY=${DISPLAY:-:0}"
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        -v "$XAUTH_FILE:/root/.Xauthority:ro"
        -e XAUTHORITY=/root/.Xauthority
    )
fi

# Run the container
podman "${PODMAN_ARGS[@]}" \
    "$IMAGE_NAME" \
    -c "rm -f /tmp/px4_lock-* /tmp/hub-isaac-sim.lock 2>/dev/null; \
        /isaac-sim/python.sh /workspace/scripts/phase9_comms_denied_test.py $HEADLESS_FLAG 2>&1 | tee /workspace/output/phase9_comms_denied/test.log"

echo ""
echo "============================================================================="
echo "Phase 9 test complete!"
echo "============================================================================="
echo ""
echo "Output files:"
ls -la "$OUTPUT_DIR/" 2>/dev/null || echo "  (no output files found)"
echo ""

# Check if video was generated
if [ -f "$OUTPUT_DIR/phase9_comms_denied.mp4" ]; then
    VIDEO_SIZE=$(du -h "$OUTPUT_DIR/phase9_comms_denied.mp4" | cut -f1)
    echo "Video: $OUTPUT_DIR/phase9_comms_denied.mp4 ($VIDEO_SIZE)"
    echo ""
    echo "To view the video:"
    echo "  mpv $OUTPUT_DIR/phase9_comms_denied.mp4"
    echo "  # or"
    echo "  ffplay $OUTPUT_DIR/phase9_comms_denied.mp4"
else
    echo "[WARN] Video file not found - check logs for errors"
fi
echo "============================================================================="
