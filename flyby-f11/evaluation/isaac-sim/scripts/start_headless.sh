#!/bin/bash
# Start Isaac Sim in headless mode with Pegasus Simulator
#
# This script:
# 1. Starts ArduPilot SITL in background
# 2. Launches Isaac Sim headless with Pegasus extension
# 3. Loads the outdoor training world
#
# Usage: ./start_headless.sh [--no-sitl]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/.."

# Parse arguments
START_SITL=true
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-sitl)
            START_SITL=false
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "============================================"
echo "  Flyby F-11 Isaac Sim Evaluation"
echo "============================================"
echo ""

# Start ArduPilot SITL if requested
if [ "$START_SITL" = true ]; then
    echo "[1/3] Starting ArduPilot SITL..."
    cd "${ARDUPILOT_HOME}/Tools/autotest"

    # Start SITL in background
    # Using sim_vehicle.py with JSON backend for Pegasus
    python3 sim_vehicle.py \
        -v ArduCopter \
        --model JSON \
        --out=udp:127.0.0.1:14550 \
        --out=udp:127.0.0.1:14551 \
        -I 0 \
        --no-rebuild \
        --no-mavproxy \
        &

    SITL_PID=$!
    echo "ArduPilot SITL started (PID: ${SITL_PID})"

    # Wait for SITL to initialize
    sleep 5
else
    echo "[1/3] Skipping ArduPilot SITL (--no-sitl)"
fi

echo ""
echo "[2/3] Configuring Isaac Sim..."

# Ensure Pegasus extension is enabled
PEGASUS_EXTENSION_PATH="${PEGASUS_HOME}/extensions"

# Create extension config if needed
mkdir -p ~/.local/share/ov/data/Kit/Isaac-Sim/4.2/user.config.d/
cat > ~/.local/share/ov/data/Kit/Isaac-Sim/4.2/user.config.d/pegasus.toml << EOF
[app.extensions]
folders = ["${PEGASUS_EXTENSION_PATH}"]

[settings.pegasus]
ardupilot_dir = "${ARDUPILOT_HOME}"
EOF

echo ""
echo "[3/3] Starting Isaac Sim headless..."

cd /isaac-sim

# Start Isaac Sim with Pegasus extension loaded
# --headless flag runs without GUI
# --enable flag loads extensions
./isaac-sim.sh \
    --headless \
    --enable omni.isaac.ros2_bridge \
    --enable pegasus.simulator \
    --ext-folder "${PEGASUS_EXTENSION_PATH}" \
    "$@"

# Cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    if [ -n "$SITL_PID" ]; then
        kill $SITL_PID 2>/dev/null || true
    fi
}
trap cleanup EXIT
