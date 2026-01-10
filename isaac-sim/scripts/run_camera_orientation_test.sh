#!/bin/bash
# Camera Orientation Grid Test
# Creates a grid of cameras with different orientations to find the correct settings

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

IMAGE_NAME="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"
CONTAINER_NAME="flyby-camera-orientation-test"

echo "============================================================================="
echo "Camera Orientation Grid Test"
echo "============================================================================="

# Clean up
podman rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Create output directory
OUTPUT_DIR="$PROJECT_DIR/output"
mkdir -p "$OUTPUT_DIR"

# Run the test
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
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v "$PROJECT_DIR:/workspace:Z" \
    --user root \
    --entrypoint /bin/bash \
    "$IMAGE_NAME" \
    -c "rm -f /tmp/px4_lock-* /tmp/hub-isaac-sim.lock 2>/dev/null; \
        /isaac-sim/python.sh /workspace/scripts/camera_orientation_test.py 2>&1"

echo ""
echo "============================================================================="
echo "Test complete!"
echo "============================================================================="

# Find latest output
LATEST_DIR=$(ls -td "$OUTPUT_DIR/camera_orientation_test_"* 2>/dev/null | head -1)
if [ -n "$LATEST_DIR" ] && [ -d "$LATEST_DIR" ]; then
    echo "Output: $LATEST_DIR"
    ls -la "$LATEST_DIR/"

    if [ -f "$LATEST_DIR/orientation_grid.jpg" ]; then
        echo ""
        echo "Grid image: $LATEST_DIR/orientation_grid.jpg"
        echo ""
        echo "To view:"
        echo "  feh $LATEST_DIR/orientation_grid.jpg"
    fi
fi
echo "============================================================================="
