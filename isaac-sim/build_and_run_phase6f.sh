#!/bin/bash
# =============================================================================
# Build and Run Phase 6f Perception Smoke Test
# =============================================================================
#
# This script builds the perception smoke test container and runs it.
#
# Usage:
#   ./build_and_run_phase6f.sh          # Build and run with GUI
#   ./build_and_run_phase6f.sh --headless  # Build and run headless
#   ./build_and_run_phase6f.sh --build-only  # Just build, don't run
#   ./build_and_run_phase6f.sh --run-only    # Just run (assumes built)
#
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

IMAGE_NAME="localhost/isaac-sim-px4:phase6f-smoke"
CONTAINER_NAME="flyby-perception-test"

# Parse arguments
BUILD=true
RUN=true
HEADLESS=false
DURATION=60

for arg in "$@"; do
    case $arg in
        --build-only)
            RUN=false
            ;;
        --run-only)
            BUILD=false
            ;;
        --headless)
            HEADLESS=true
            ;;
        --duration=*)
            DURATION="${arg#*=}"
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--build-only|--run-only] [--headless] [--duration=N]"
            exit 1
            ;;
    esac
done

echo "============================================================================="
echo "FLYBY F-11 PHASE 6F - PERCEPTION SMOKE TEST"
echo "============================================================================="
echo "Image: $IMAGE_NAME"
echo "Build: $BUILD | Run: $RUN | Headless: $HEADLESS | Duration: ${DURATION}s"
echo "============================================================================="

# =============================================================================
# BUILD
# =============================================================================
if [ "$BUILD" = true ]; then
    echo ""
    echo "[BUILD] Building container image..."
    echo "This may take 10-15 minutes on first build (PX4 compilation)."
    echo ""

    # Check if GPU is available for shader warmup during build
    if command -v nvidia-smi &> /dev/null; then
        echo "[BUILD] GPU detected - shaders will be pre-compiled"
        BUILD_ARGS="--device nvidia.com/gpu=all"
    else
        echo "[BUILD] No GPU detected - shaders will compile at runtime"
        BUILD_ARGS=""
    fi

    podman build $BUILD_ARGS \
        -f Containerfile.phase6f \
        -t "$IMAGE_NAME" \
        .

    echo ""
    echo "[BUILD] Image built successfully: $IMAGE_NAME"
fi

# =============================================================================
# RUN
# =============================================================================
if [ "$RUN" = true ]; then
    echo ""
    echo "[RUN] Starting smoke test container..."

    # Clean up any existing container
    podman rm -f "$CONTAINER_NAME" 2>/dev/null || true

    # Build run command
    RUN_ARGS="--rm --name $CONTAINER_NAME"
    RUN_ARGS="$RUN_ARGS --device nvidia.com/gpu=all"
    RUN_ARGS="$RUN_ARGS --network host"
    RUN_ARGS="$RUN_ARGS --ipc host"

    if [ "$HEADLESS" = false ]; then
        # GUI mode - need X11 access
        echo "[RUN] Running with GUI (X11 forwarding)"
        xhost +local: 2>/dev/null || echo "[WARN] xhost not available"

        RUN_ARGS="$RUN_ARGS -e DISPLAY=$DISPLAY"
        RUN_ARGS="$RUN_ARGS -v /tmp/.X11-unix:/tmp/.X11-unix:rw"

        # Try to find Xauthority
        if [ -n "$XAUTHORITY" ]; then
            RUN_ARGS="$RUN_ARGS -v $XAUTHORITY:/root/.Xauthority:ro"
        fi

        SCRIPT_ARGS="--duration $DURATION"
    else
        echo "[RUN] Running headless (no GUI)"
        SCRIPT_ARGS="--headless --duration $DURATION"
    fi

    # Create output directory on host
    OUTPUT_DIR="/tmp/perception_output_host"
    mkdir -p "$OUTPUT_DIR"
    RUN_ARGS="$RUN_ARGS -v $OUTPUT_DIR:/tmp/perception_output:Z"

    echo ""
    echo "[RUN] Command:"
    echo "  podman run $RUN_ARGS $IMAGE_NAME $SCRIPT_ARGS"
    echo ""
    echo "[RUN] Output will be saved to: $OUTPUT_DIR"
    echo "============================================================================="
    echo ""

    # Run the container
    podman run $RUN_ARGS "$IMAGE_NAME" $SCRIPT_ARGS

    echo ""
    echo "============================================================================="
    echo "[DONE] Smoke test complete!"
    echo ""
    echo "Output saved to: $OUTPUT_DIR"
    echo "Check annotated images:"
    echo "  ls -la $OUTPUT_DIR"
    echo ""
    if command -v feh &> /dev/null; then
        echo "View images with: feh $OUTPUT_DIR/*.png"
    elif command -v eog &> /dev/null; then
        echo "View images with: eog $OUTPUT_DIR/"
    fi
    echo "============================================================================="
fi
