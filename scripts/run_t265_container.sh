#!/bin/bash
# Build and run minimal RealSense T265 container (legacy SDK v2.50.0)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Building RealSense T265 container (this may take several minutes)..."
docker build -f "$REPO_ROOT/docker/Dockerfile.t265" -t realsense-t265 "$REPO_ROOT"

echo "Running RealSense T265 container with USB device access..."
docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device-cgroup-rule='c 81:* rmw' \
    --device-cgroup-rule='c 189:* rmw' \
    realsense-t265 bash -c "rs-enumerate-devices && bash"
