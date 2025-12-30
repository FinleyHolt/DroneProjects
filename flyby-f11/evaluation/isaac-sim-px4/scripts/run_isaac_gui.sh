#!/bin/bash
# run_isaac_gui.sh - Launch Isaac Sim container with stable GUI support
#
# Handles Wayland/XWayland quirks for reliable GUI display

set -e

CONTAINER_NAME="${1:-isaac-gui}"
IMAGE="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

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

# Get script directory for volume mounts
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "Starting Isaac Sim container: $CONTAINER_NAME"
echo "Project dir: $PROJECT_DIR"

# Override entrypoint to use GUI mode instead of headless
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
    -v "$PROJECT_DIR/extensions":/workspace/extensions:z \
    -v "$PROJECT_DIR/scripts":/workspace/scripts:z \
    --network host \
    --ipc host \
    --security-opt label=disable \
    --ulimit memlock=-1:-1 \
    "$IMAGE" \
    -c "sleep infinity"

echo ""
echo "Container started. To run the world generator test:"
echo "  podman exec $CONTAINER_NAME /isaac-sim/python.sh /workspace/scripts/test_world_generator.py --size 100 --duration 300"
echo ""
echo "To open an interactive shell:"
echo "  podman exec -it $CONTAINER_NAME bash"
echo ""
echo "To stop:"
echo "  podman rm -f $CONTAINER_NAME"
