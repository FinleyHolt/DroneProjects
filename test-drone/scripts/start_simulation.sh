#!/usr/bin/env bash
set -euo pipefail

# Comprehensive simulation launcher: PX4 SITL + Gazebo + QGroundControl
# This script handles the complete workflow inside the container

echo "=== Test Drone Simulation Launcher ==="
echo

# Check if we're inside the container
if [ ! -d "/opt/PX4-Autopilot" ]; then
    echo "ERROR: This script must be run inside the test-drone-simulation container"
    echo "Please run: docker exec -it test-drone-simulation bash"
    echo "Then run this script again"
    exit 1
fi

# Step 1: Check/build PX4 if needed (BEFORE launching any GUI)
if [ ! -f /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4 ]; then
    echo "=========================================="
    echo " FIRST RUN: Building PX4 SITL"
    echo "=========================================="
    echo "This will take 5-10 minutes on first run."
    echo "Subsequent runs will be instant."
    echo ""
    echo "Building PX4..."
    /opt/build_px4_sitl.sh
    echo ""
    echo "=========================================="
    echo " Build complete! Starting simulation..."
    echo "=========================================="
    echo ""
    sleep 2
else
    echo "PX4 SITL already built, skipping build step"
    echo ""

    # Ensure custom worlds are symlinked (in case they were added after build)
    CUSTOM_WORLDS_DIR="/workspace/simulation/worlds"
    PX4_WORLDS_DIR="/opt/PX4-Autopilot/Tools/simulation/gz/worlds"

    if [ -d "$CUSTOM_WORLDS_DIR" ] && [ -d "$PX4_WORLDS_DIR" ]; then
        for world_file in "$CUSTOM_WORLDS_DIR"/*.sdf; do
            if [ -f "$world_file" ]; then
                world_name=$(basename "$world_file")
                target_link="$PX4_WORLDS_DIR/$world_name"

                # Create symlink if it doesn't exist
                if [ ! -e "$target_link" ]; then
                    ln -s "$world_file" "$target_link"
                    echo "Linked custom world: $world_name"
                fi
            fi
        done
    fi
fi

# Step 2: Initialize QGroundControl config directory if first run
QGC_CONFIG_DIR="/home/qgcuser/.config/QGroundControl.org"
if [ ! -d "$QGC_CONFIG_DIR" ]; then
    echo "First run detected - initializing QGroundControl config..."
    mkdir -p "$QGC_CONFIG_DIR"
    chown -R qgcuser:qgcuser "$QGC_CONFIG_DIR"
    # Let QGC create initial config by running and closing
    su - qgcuser -c "DISPLAY=$DISPLAY timeout 3 /opt/squashfs-root/AppRun || true"
    sleep 2
fi

# Step 3: Verify X11 display access
echo "Verifying X11 display access..."
echo "DISPLAY=$DISPLAY"

# Note: xhost may not be available in container, but X11 forwarding can still work
# via mounted /tmp/.X11-unix socket. So we don't exit on failure here.
if ! xhost > /dev/null 2>&1; then
    echo "WARNING: xhost command not available in container (this is normal)"
    echo "X11 forwarding via /tmp/.X11-unix socket should still work"
else
    echo "X11 access verified via xhost"
fi
echo

# Step 4: Start QGroundControl as non-root user
echo "Starting QGroundControl..."
cd /opt
su - qgcuser -c "DISPLAY=$DISPLAY /opt/squashfs-root/AppRun" &
QGC_PID=$!
echo "QGroundControl started (PID: $QGC_PID)"
sleep 3

# Verify QGC is still running
if ! kill -0 $QGC_PID 2>/dev/null; then
    echo "WARNING: QGroundControl exited unexpectedly"
    echo "Continuing anyway (QGC is optional for simulation)..."
fi

# Step 5: Start PX4 SITL with Gazebo
echo
echo "Starting PX4 SITL + Gazebo Harmonic..."

# Check if running in headless mode
if [ -n "${HEADLESS:-}" ]; then
    echo "Running in HEADLESS mode (no GUI)"
else
    echo "Gazebo GUI will open automatically"
fi

echo
echo "Controls:"
echo "  - QGroundControl: Already running (separate window)"
echo "  - Gazebo: Will open in a moment"
echo "  - To stop: Press Ctrl+C"
echo "  - To run headless: Set HEADLESS=1 environment variable"
echo

cd /opt/PX4-Autopilot

# Trap to cleanup on exit
cleanup() {
    echo
    echo "Shutting down..."
    kill $QGC_PID 2>/dev/null || true
    pkill -f QGroundControl || true
    pkill -f gz || true
    pkill -f px4 || true
}
trap cleanup EXIT

# Launch PX4 SITL with custom forest world
# Use test_drone world (outdoor forest environment with trees, boulders, and bushes)
echo "Launching: PX4_GZ_WORLD=test_drone make px4_sitl gz_x500"
echo

PX4_GZ_WORLD=test_drone make px4_sitl gz_x500
