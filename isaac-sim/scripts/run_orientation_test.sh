#!/bin/bash
# Run vehicle orientation test in Isaac Sim
# Based on run_isaac_gui.sh pattern

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
CONTAINER_NAME="orientation-test"
IMAGE="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

echo "=== Vehicle Orientation Test ==="
echo "This will spawn all vehicle/tank models in a grid."
echo "Check that each vehicle is wheels-down with correct front orientation."
echo ""

# Clean up any existing container with same name
podman rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Find the current Xauthority file (changes on each login with Wayland)
XAUTH=$(find /run/user/$(id -u) -name '.mutter-Xwayland*' -type f 2>/dev/null | head -1)
if [ -z "$XAUTH" ]; then
    XAUTH="$HOME/.Xauthority"
fi

if [ ! -f "$XAUTH" ]; then
    echo "Error: No Xauthority file found"
    exit 1
fi

echo "Using Xauthority: $XAUTH"
echo "Display: $DISPLAY"

# Allow local connections to X server
xhost +local: 2>/dev/null || true

# Shader cache directory for persistent shader compilation
SHADER_CACHE_DIR="$HOME/.cache/isaac-sim-shaders"
mkdir -p "$SHADER_CACHE_DIR"

echo "Starting Isaac Sim container: $CONTAINER_NAME"
echo "Project dir: $PROJECT_DIR"
echo "Shader cache: $SHADER_CACHE_DIR"
echo ""

# Start container in background with sleep infinity
podman run -d --name "$CONTAINER_NAME" \
    --device nvidia.com/gpu=all \
    --entrypoint /bin/bash \
    -e DISPLAY="$DISPLAY" \
    -e XAUTHORITY=/tmp/.Xauthority \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    -e OMNI_KIT_ACCEPT_EULA=YES \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    -e MESA_VK_DEVICE_SELECT=10de:* \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$XAUTH":/tmp/.Xauthority:ro \
    -v "$PROJECT_DIR/extensions":/workspace/extensions:Z \
    -v "$PROJECT_DIR/scripts":/workspace/scripts:Z \
    -v "$SHADER_CACHE_DIR":/root/.cache/ov:z \
    --network host \
    --ipc host \
    --security-opt label=disable \
    --ulimit memlock=-1:-1 \
    "$IMAGE" \
    -c "sleep infinity"

echo "Container started. Running orientation test..."
echo ""

# Run the test script
echo "Running test script (this will launch the Isaac Sim window)..."
podman exec "$CONTAINER_NAME" /isaac-sim/python.sh /workspace/scripts/test_vehicle_orientations.py "$@"

RESULT=$?
if [ $RESULT -ne 0 ]; then
    echo ""
    echo "Script exited with code $RESULT"
    echo "Container is still running for debugging:"
    echo "  podman exec -it $CONTAINER_NAME bash"
    echo ""
    echo "To clean up: podman rm -f $CONTAINER_NAME"
else
    echo ""
    echo "Test completed successfully."
    echo "Cleaning up container..."
    podman rm -f "$CONTAINER_NAME" 2>/dev/null || true
fi
