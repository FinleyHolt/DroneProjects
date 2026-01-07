#!/bin/bash
# Isaac Sim + PX4 Drone Functions Test Runner
# Run this script to execute drone flight tests with optional profile selection
#
# Usage:
#   ./scripts/run_test.sh                      # Full test (default, ~12 min)
#   ./scripts/run_test.sh --profile detection  # Detection test (~2-3 min)
#   ./scripts/run_test.sh --profile tracking   # Tracking test (~2-3 min)
#   ./scripts/run_test.sh --profile gimbal     # Gimbal test (~1-2 min)
#   ./scripts/run_test.sh --profile nav        # Navigation test (~3-4 min)
#   ./scripts/run_test.sh --profile stress     # Stress test (~2 min)
#   ./scripts/run_test.sh --list-profiles      # List available profiles

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Parse arguments
PROFILE=""
EXTRA_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --list-profiles)
            EXTRA_ARGS="--list-profiles"
            shift
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift
            ;;
    esac
done

echo "========================================"
echo "Isaac Sim + PX4 Drone Functions Test"
echo "========================================"
echo "Workspace: $WORKSPACE_DIR"
echo "Script: drone_functions_check.py"

if [ -n "$PROFILE" ]; then
    echo "Profile: $PROFILE"
fi
echo ""

# Build the command
CMD="/workspace/scripts/drone_functions_check.py --headless"

if [ -n "$PROFILE" ]; then
    CMD="$CMD --profile $PROFILE"
fi

if [ -n "$EXTRA_ARGS" ]; then
    CMD="$CMD $EXTRA_ARGS"
fi

podman run --rm --name flyby-f11-test \
  --device nvidia.com/gpu=all \
  -v "$WORKSPACE_DIR:/workspace:Z" \
  -e ACCEPT_EULA=Y \
  -e PYTHONUNBUFFERED=1 \
  --network=host \
  --entrypoint /isaac-sim/python.sh \
  localhost/isaac-sim-px4:5.1.0-px4-1.14.3 \
  $CMD
