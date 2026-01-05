#!/bin/bash
# ==============================================================================
# Build script for Isaac Sim + PX4 + ROS 2 container
# ==============================================================================
#
# This script:
# 1. Copies ROS 2 packages from flyby-f11/ros2_ws to the build context
# 2. Builds the container with GPU support for shader warmup
#
# Usage:
#   ./scripts/build_container.sh [--no-cache]
#
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$(dirname "$SCRIPT_DIR")"
# ROS 2 workspace is at flyby-f11/ros2_ws/src (two levels up from isaac-sim-px4)
FLYBY_ROOT="$(dirname "$(dirname "$BUILD_DIR")")"
ROS2_WS_DIR="$FLYBY_ROOT/ros2_ws/src"

cd "$BUILD_DIR"

echo "=============================================="
echo "Flyby F-11 Isaac Sim Container Build"
echo "=============================================="
echo "Build directory: $BUILD_DIR"
echo "ROS 2 workspace: $ROS2_WS_DIR"
echo ""

# Create ros2_packages directory
mkdir -p ros2_packages

# Copy ROS 2 packages
echo "Copying ROS 2 packages..."
for pkg in flyby_msgs flyby_perception flyby_autonomy; do
    if [ -d "$ROS2_WS_DIR/$pkg" ]; then
        echo "  - $pkg"
        rm -rf "ros2_packages/$pkg"
        cp -r "$ROS2_WS_DIR/$pkg" "ros2_packages/"
    else
        echo "  WARNING: $pkg not found at $ROS2_WS_DIR/$pkg"
    fi
done
echo ""

# Check for YOLO model weights
if [ ! -f "yolo11x.pt" ]; then
    echo "WARNING: yolo11x.pt not found in build directory"
    echo "  Download from: https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x.pt"
    echo "  Or mount at runtime: -v ./yolo11x.pt:/workspace/models/yolo11x.pt"
    echo ""
fi

# Build arguments
BUILD_ARGS=""
if [ "$1" = "--no-cache" ]; then
    BUILD_ARGS="--no-cache"
    echo "Building with --no-cache"
fi

# Build container
echo "Building container..."
echo "  Tag: localhost/isaac-sim-px4:5.1.0-px4-1.14.3"
echo ""

podman build \
    --device nvidia.com/gpu=all \
    $BUILD_ARGS \
    -t localhost/isaac-sim-px4:5.1.0-px4-1.14.3 \
    .

echo ""
echo "=============================================="
echo "Build complete!"
echo "=============================================="
echo ""
echo "Run with GUI:"
echo "  xhost +local:"
echo "  podman run -d --name flyby-f11 --device nvidia.com/gpu=all \\"
echo "    -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\"
echo "    --network host localhost/isaac-sim-px4:5.1.0-px4-1.14.3"
echo ""
echo "Run headless:"
echo "  podman run -d --name flyby-f11 --device nvidia.com/gpu=all \\"
echo "    --network host localhost/isaac-sim-px4:5.1.0-px4-1.14.3"
