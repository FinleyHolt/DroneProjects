#!/bin/bash
# =============================================================================
# Depth + Avoidance Navigation Test
# =============================================================================
#
# Two modes:
#   1. Default: Uses ground-truth depth from Isaac Sim for isolated
#      navigation/avoidance testing without perception noise
#   2. --with-ros: Full integration test using actual ROS 2 nodes
#      (flyby_depth + local_avoidance) for end-to-end validation
#
# Usage:
#   ./run_depth_avoidance_test.sh                      # pillar_field, headless
#   ./run_depth_avoidance_test.sh pillar_field         # specific scenario
#   ./run_depth_avoidance_test.sh --gui                # with GUI
#   ./run_depth_avoidance_test.sh --with-ros           # full ROS 2 pipeline
#
# Options:
#   --headless    Run without GUI (default)
#   --gui         Run with GUI for visualization
#   --with-ros    Use ROS 2 depth estimation + avoidance nodes (full pipeline)
#   --scenario    Scenario name (default: pillar_field)
#   --max-steps   Maximum simulation steps (default: 3000)
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

# Image with Isaac Sim + PX4
IMAGE_NAME="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

# Defaults
SCENARIO="pillar_field"
HEADLESS_FLAG="--headless"
WITH_ROS=false
MAX_STEPS=3000
GUI_MODE=false
SAVE_BENCHMARK=false
BENCHMARK_INTERVAL=30

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --scenario)
            SCENARIO="$2"
            shift 2
            ;;
        --headless)
            HEADLESS_FLAG="--headless"
            GUI_MODE=false
            shift
            ;;
        --gui)
            HEADLESS_FLAG="--gui"
            GUI_MODE=true
            shift
            ;;
        --with-ros)
            WITH_ROS=true
            shift
            ;;
        --max-steps)
            MAX_STEPS="$2"
            shift 2
            ;;
        --save-benchmark)
            SAVE_BENCHMARK=true
            shift
            ;;
        --benchmark-interval)
            BENCHMARK_INTERVAL="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [scenario] [options]"
            echo ""
            echo "Scenarios:"
            echo "  pillar_field     - 8 massive pillars requiring weaving (default)"
            echo "  simple_transit   - Single obstacle between A and B"
            echo "  obstacle_field   - 10 random tall obstacles"
            echo "  narrow_passage   - Two obstacles forming narrow gap"
            echo "  stress_test      - 20 dense obstacles"
            echo "  building_canyon  - Urban canyon environment"
            echo ""
            echo "Options:"
            echo "  --headless           Run without GUI (default)"
            echo "  --gui                Run with GUI for visualization"
            echo "  --with-ros           Use ROS 2 depth estimation + avoidance nodes"
            echo "  --max-steps N        Maximum simulation steps (default: 3000)"
            echo "  --save-benchmark     Save benchmark dataset for offline testing"
            echo "  --benchmark-interval Frames between benchmark captures (default: 30)"
            echo ""
            exit 0
            ;;
        *)
            # Positional argument - treat as scenario name
            SCENARIO="$1"
            shift
            ;;
    esac
done

echo "============================================================================="
echo "FLYBY F-11: Depth + Avoidance Navigation Test"
echo "============================================================================="
echo "Image: $IMAGE_NAME"
echo "Scenario: $SCENARIO"
if $WITH_ROS; then
    echo "Mode: FULL PIPELINE (ROS 2 depth + avoidance nodes)"
else
    echo "Mode: GROUND TRUTH (Isaac Sim depth, isolated nav test)"
fi
if $GUI_MODE; then
    echo "Display: GUI"
else
    echo "Display: HEADLESS"
fi
echo "Max Steps: $MAX_STEPS"
echo "============================================================================="

# Clean up any existing container
CONTAINER_NAME="flyby-depth-avoidance-test"
podman rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Create output directory
OUTPUT_DIR="$PROJECT_DIR/output/depth_avoidance_tests"
mkdir -p "$OUTPUT_DIR"
echo ""
echo "[Setup] Output directory: $OUTPUT_DIR"

# Setup X11 for GUI (if not headless)
if $GUI_MODE; then
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
echo "[Run] Starting depth + avoidance test..."
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
if $GUI_MODE; then
    PODMAN_ARGS+=(
        -e "DISPLAY=${DISPLAY:-:0}"
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        -v "$XAUTH_FILE:/root/.Xauthority:ro"
        -e XAUTHORITY=/root/.Xauthority
    )
fi

# Build Python script arguments
PYTHON_ARGS="--scenario $SCENARIO $HEADLESS_FLAG --max-steps $MAX_STEPS"
WITH_ROS_SHELL=false
if $WITH_ROS; then
    PYTHON_ARGS="$PYTHON_ARGS --with-ros"
    WITH_ROS_SHELL=true
fi
if $SAVE_BENCHMARK; then
    PYTHON_ARGS="$PYTHON_ARGS --save-benchmark --benchmark-interval $BENCHMARK_INTERVAL"
fi

# Build container command
CONTAINER_CMD="rm -f /tmp/px4_lock-* /tmp/hub-isaac-sim.lock 2>/dev/null"
if $WITH_ROS; then
    CONTAINER_CMD="$CONTAINER_CMD; echo '[Setup] Installing Depth Anything V2 dependencies...'; /isaac-sim/python.sh -m pip install -q transformers 2>/dev/null || true"
fi
CONTAINER_CMD="$CONTAINER_CMD; /isaac-sim/python.sh /workspace/scripts/depth_avoidance_test.py $PYTHON_ARGS 2>&1 | tee /workspace/output/depth_avoidance_tests/test_${SCENARIO}.log"

# Run the container
podman "${PODMAN_ARGS[@]}" \
    "$IMAGE_NAME" \
    -c "$CONTAINER_CMD"

echo ""
echo "============================================================================="
echo "Depth + Avoidance test complete!"
echo "============================================================================="
echo ""
echo "Output files:"
ls -la "$OUTPUT_DIR/" 2>/dev/null || echo "  (no output files found)"
echo ""

# Find most recent test output
LATEST_DIR=$(ls -td "$OUTPUT_DIR/${SCENARIO}_"* 2>/dev/null | head -1)
if [ -n "$LATEST_DIR" ] && [ -d "$LATEST_DIR" ]; then
    echo "Latest test output: $LATEST_DIR"
    ls -la "$LATEST_DIR/"

    if [ -f "$LATEST_DIR/video.mp4" ]; then
        VIDEO_SIZE=$(du -h "$LATEST_DIR/video.mp4" | cut -f1)
        echo ""
        echo "Video: $LATEST_DIR/video.mp4 ($VIDEO_SIZE)"
        echo ""
        echo "To view the video:"
        echo "  mpv $LATEST_DIR/video.mp4"
    fi

    if [ -f "$LATEST_DIR/metrics.json" ]; then
        echo ""
        echo "Metrics:"
        cat "$LATEST_DIR/metrics.json"
    fi
else
    echo "[WARN] No test output directory found - check logs for errors"
fi
echo "============================================================================="
