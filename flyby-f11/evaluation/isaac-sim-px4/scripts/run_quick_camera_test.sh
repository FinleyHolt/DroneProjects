#!/bin/bash
# Quick camera test - 30 second max

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

IMAGE_NAME="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

podman rm -f quick-camera-test 2>/dev/null || true

podman run --rm \
    --name quick-camera-test \
    --device nvidia.com/gpu=all \
    --network host \
    --ipc host \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    -v "$PROJECT_DIR:/workspace:Z" \
    --user root \
    --entrypoint /bin/bash \
    "$IMAGE_NAME" \
    -c "/isaac-sim/python.sh /workspace/scripts/quick_camera_test.py 2>&1"
