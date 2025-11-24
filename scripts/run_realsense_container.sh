#!/bin/bash
# Build and run minimal RealSense D455 container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Building RealSense D455 container..."
docker build -f "$REPO_ROOT/docker/Dockerfile.d455" -t realsense-d455 "$REPO_ROOT"

echo "Running RealSense D455 container with USB device access..."
docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device-cgroup-rule='c 81:* rmw' \
    --device-cgroup-rule='c 189:* rmw' \
    realsense-d455 bash -c "rs-enumerate-devices && bash"
