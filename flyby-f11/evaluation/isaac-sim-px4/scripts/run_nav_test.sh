#!/bin/bash
# Navigation Obstacle Avoidance Test Runner
# Runs headless Isaac Sim tests with obstacle scenarios
#
# Usage:
#   ./scripts/run_nav_test.sh                              # Run simple_transit (default)
#   ./scripts/run_nav_test.sh --scenario obstacle_field    # Run specific scenario
#   ./scripts/run_nav_test.sh --scenario all               # Run all scenarios
#   ./scripts/run_nav_test.sh --gui                        # Run with GUI
#   ./scripts/run_nav_test.sh --record-video               # Record video
#   ./scripts/run_nav_test.sh --list-scenarios             # List available scenarios

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Parse arguments
SCENARIO="simple_transit"
HEADLESS="--headless"
EXTRA_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --scenario)
            SCENARIO="$2"
            shift 2
            ;;
        --gui)
            HEADLESS=""
            shift
            ;;
        --record-video)
            EXTRA_ARGS="$EXTRA_ARGS --record-video"
            shift
            ;;
        --list-scenarios)
            EXTRA_ARGS="--list-scenarios"
            shift
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift
            ;;
    esac
done

echo "========================================"
echo "Navigation Obstacle Avoidance Test"
echo "========================================"
echo "Workspace: $WORKSPACE_DIR"
echo "Scenario: $SCENARIO"
if [ -n "$HEADLESS" ]; then
    echo "Mode: Headless"
else
    echo "Mode: GUI"
fi
echo ""

# Build the command
CMD="/workspace/scripts/nav_obstacle_test.py --scenario $SCENARIO $HEADLESS $EXTRA_ARGS"

# Allow X11 access for GUI mode
if [ -z "$HEADLESS" ]; then
    xhost +local: 2>/dev/null || true
fi

podman run --rm --name flyby-nav-test \
  --device nvidia.com/gpu=all \
  -v "$WORKSPACE_DIR:/workspace:Z" \
  -e ACCEPT_EULA=Y \
  -e PYTHONUNBUFFERED=1 \
  ${HEADLESS:+-e "DISPLAY="} \
  ${HEADLESS:-"-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"} \
  --network=host \
  --entrypoint /isaac-sim/python.sh \
  localhost/isaac-sim-px4:5.1.0-px4-1.14.3 \
  $CMD
