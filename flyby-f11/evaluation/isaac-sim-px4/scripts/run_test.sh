#!/bin/bash
# Isaac Sim + PX4 Drone Functions Test Runner
# Run this script to execute the full drone functions check

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "========================================"
echo "Isaac Sim + PX4 Drone Functions Test"
echo "========================================"
echo "Workspace: $WORKSPACE_DIR"
echo "Script: drone_functions_check.py"
echo ""

podman run --rm --name flyby-f11-test \
  --device nvidia.com/gpu=all \
  -v "$WORKSPACE_DIR:/workspace:Z" \
  -e ACCEPT_EULA=Y \
  -e PYTHONUNBUFFERED=1 \
  --network=host \
  --entrypoint /isaac-sim/python.sh \
  localhost/isaac-sim-px4:5.1.0-px4-1.14.3 \
  /workspace/scripts/drone_functions_check.py --headless
