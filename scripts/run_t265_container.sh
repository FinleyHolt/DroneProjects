#!/bin/bash
# Build and run minimal RealSense T265 container (legacy SDK v2.50.0)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "[T265] Building RealSense T265 container..."
docker build -f "$REPO_ROOT/docker/Dockerfile.t265" -t realsense-t265 "$REPO_ROOT"

echo "[T265] Allowing local X access for GUI examples (if needed)..."
xhost +local:root >/dev/null 2>&1 || true

echo "[T265] Running RealSense T265 container with USB + X11 access..."
docker run -it --rm \
    --privileged \
    --net=host \
    -v /dev:/dev \
    -v /run/udev:/run/udev:ro \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    realsense-t265 \
    bash -lc "echo '[T265] Inside container:'; \
              lsusb | grep -Ei '8087:0b37|03e7:2150' || echo 'T265 not visible via lsusb'; \
              echo; \
              echo '[T265] rs-enumerate-devices output:'; \
              rs-enumerate-devices || echo 'rs-enumerate-devices failed'; \
              echo; \
              bash"
