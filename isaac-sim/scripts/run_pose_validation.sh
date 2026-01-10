#!/bin/bash
# run_pose_validation.sh - Launch pose validation grid with GUI for visual inspection
#
# Creates a visual grid showing all figure types at all 24 discrete yaw angles
# Run this to verify that vehicles and people spawn upright with correct orientations

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
CONTAINER_NAME="pose-validation"
IMAGE="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

echo "========================================"
echo "POSE VALIDATION - Visual Grid Test"
echo "========================================"
echo "Project dir: $PROJECT_DIR"
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
    echo "Make sure X11 is running (or XWayland on Wayland)"
    exit 1
fi

echo "Using Xauthority: $XAUTH"
echo "Display: $DISPLAY"

# Allow local connections to X server
xhost +local: 2>/dev/null || true

# Shader cache directory for persistent shader compilation
SHADER_CACHE_DIR="$HOME/.cache/isaac-sim-shaders"
mkdir -p "$SHADER_CACHE_DIR"

echo "Shader cache: $SHADER_CACHE_DIR"
echo ""
echo "Starting Isaac Sim with pose validation grid..."
echo "This will take a moment to initialize..."
echo ""

# Run with GUI support
podman run --rm --name "$CONTAINER_NAME" \
    --device nvidia.com/gpu=all \
    -e DISPLAY="$DISPLAY" \
    -e XAUTHORITY=/tmp/.Xauthority \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    -e OMNI_KIT_ACCEPT_EULA=YES \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    -e MESA_VK_DEVICE_SELECT=10de:* \
    -e PYTHONUNBUFFERED=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$XAUTH":/tmp/.Xauthority:ro \
    -v "$PROJECT_DIR":/workspace:Z \
    -v "$SHADER_CACHE_DIR":/root/.cache/ov:z \
    --network host \
    --ipc host \
    --security-opt label=disable \
    --ulimit memlock=-1:-1 \
    --entrypoint /isaac-sim/python.sh \
    "$IMAGE" \
    /workspace/scripts/pose_validation.py
