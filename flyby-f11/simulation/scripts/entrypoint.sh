#!/bin/bash
# Flyby F-11 Simulation Container Entrypoint
#
# Environment variables:
#   HEADLESS=true       - Run without GUI (for training)
#   SITL_SPEEDUP=N      - Simulation speedup factor
#   SITL_INSTANCE=N     - Instance number for parallel runs
#   GAZEBO_ONLY=true    - Start only Gazebo (no SITL)
#   SITL_ONLY=true      - Start only SITL (no Gazebo)
#   LAUNCH_MAVROS=true  - Also start MAVROS bridge

set -e

echo "==================================="
echo "Flyby F-11 Simulation Environment"
echo "==================================="

# Source ROS 2
source /opt/ros/humble/setup.bash

# Default values
HEADLESS=${HEADLESS:-false}
SITL_SPEEDUP=${SITL_SPEEDUP:-1}
SITL_INSTANCE=${SITL_INSTANCE:-0}
GAZEBO_ONLY=${GAZEBO_ONLY:-false}
SITL_ONLY=${SITL_ONLY:-false}
LAUNCH_MAVROS=${LAUNCH_MAVROS:-false}

# Calculate ports for this instance
SITL_PORT=$((5760 + SITL_INSTANCE * 10))
MAVLINK_PORT=$((14550 + SITL_INSTANCE))

echo "Configuration:"
echo "  HEADLESS:      $HEADLESS"
echo "  SITL_SPEEDUP:  $SITL_SPEEDUP"
echo "  SITL_INSTANCE: $SITL_INSTANCE"
echo "  SITL_PORT:     $SITL_PORT"
echo "  MAVLINK_PORT:  $MAVLINK_PORT"
echo ""

# Handle headless mode
if [ "$HEADLESS" = "true" ]; then
    echo "Starting Xvfb for headless rendering..."
    Xvfb :99 -screen 0 1920x1080x24 &
    export DISPLAY=:99
    sleep 2
fi

# Function to start Gazebo
start_gazebo() {
    echo "Starting Gazebo Harmonic..."

    # Always start the server first (required for both headless and GUI modes)
    gz sim -s /simulation/worlds/training_world.sdf &
    GAZEBO_SERVER_PID=$!
    echo "Gazebo server started with PID: $GAZEBO_SERVER_PID"

    # Wait for server to initialize before starting GUI
    sleep 3

    if [ "$HEADLESS" = "true" ]; then
        # Headless mode - server only (already started above)
        GAZEBO_PID=$GAZEBO_SERVER_PID
        echo "Running in headless mode (server only)"
    else
        # GUI mode - start a single GUI that connects to the running server
        gz sim -g &
        GAZEBO_GUI_PID=$!
        echo "Gazebo GUI started with PID: $GAZEBO_GUI_PID"
        GAZEBO_PID=$GAZEBO_SERVER_PID
    fi

    # Wait for Gazebo to be ready
    sleep 2
}

# Function to start ArduPilot SITL
start_sitl() {
    echo "Starting ArduPilot SITL..."

    # Set home location from environment or default (MCTSSA at Camp Pendleton)
    HOME_LAT=${SITL_HOME_LAT:-33.3853}
    HOME_LON=${SITL_HOME_LON:--117.5653}
    HOME_ALT=${SITL_HOME_ALT:-100}
    HOME_YAW=${SITL_HOME_YAW:-0}

    cd /home/ardupilot/ardupilot

    # Run arducopter with Gazebo plugin integration
    # The ArduPilot Gazebo plugin communicates via UDP ports 9002/9003
    # --model gazebo-iris tells SITL to use Gazebo physics
    build/sitl/bin/arducopter \
        --instance $SITL_INSTANCE \
        --home $HOME_LAT,$HOME_LON,$HOME_ALT,$HOME_YAW \
        --model gazebo-iris \
        --speedup $SITL_SPEEDUP \
        --defaults /simulation/configs/arducopter.parm \
        --serial0 tcp:$SITL_PORT &

    SITL_PID=$!
    echo "SITL started with PID: $SITL_PID"
    echo "  Home: $HOME_LAT,$HOME_LON,$HOME_ALT"
    echo "  Model: gazebo-iris (connected to Gazebo plugin)"
    echo "  TCP Console:  tcp://localhost:$SITL_PORT"
    echo "  Gazebo FDM:   udp://127.0.0.1:9002-9003"

    # Wait for SITL to be ready
    echo "Waiting for SITL to initialize..."
    sleep 10
}

# Function to start MAVROS
start_mavros() {
    echo "Starting MAVROS..."

    ros2 launch mavros apm.launch \
        fcu_url:=udp://127.0.0.1:$MAVLINK_PORT@127.0.0.1:$((MAVLINK_PORT + 1)) \
        gcs_url:=udp://@localhost:$((MAVLINK_PORT + 2)) \
        tgt_system:=1 \
        tgt_component:=1 &

    MAVROS_PID=$!
    echo "MAVROS started with PID: $MAVROS_PID"
}

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down..."

    # Kill processes
    [ -n "$MAVROS_PID" ] && kill $MAVROS_PID 2>/dev/null
    [ -n "$SITL_PID" ] && kill $SITL_PID 2>/dev/null
    [ -n "$GAZEBO_GUI_PID" ] && kill $GAZEBO_GUI_PID 2>/dev/null
    [ -n "$GAZEBO_SERVER_PID" ] && kill $GAZEBO_SERVER_PID 2>/dev/null
    [ -n "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null

    # Kill any remaining sim_vehicle processes
    pkill -f sim_vehicle.py 2>/dev/null || true
    pkill -f arducopter 2>/dev/null || true
    pkill -f gz 2>/dev/null || true

    echo "Shutdown complete"
    exit 0
}

# Set up signal handling
trap cleanup SIGINT SIGTERM

# Start services based on mode
if [ "$GAZEBO_ONLY" = "true" ]; then
    start_gazebo
elif [ "$SITL_ONLY" = "true" ]; then
    start_sitl
else
    # Start both
    start_gazebo
    start_sitl
fi

# Optionally start MAVROS
if [ "$LAUNCH_MAVROS" = "true" ]; then
    start_mavros
fi

echo ""
echo "==================================="
echo "Simulation environment ready!"
echo "==================================="
echo ""
echo "SITL MAVLink: udp://127.0.0.1:$MAVLINK_PORT"
echo "SITL TCP:     tcp://localhost:$SITL_PORT"
echo ""

# If command provided, run it
if [ $# -gt 0 ]; then
    exec "$@"
else
    # Keep container running - wait on server process specifically
    echo "Press Ctrl+C to shutdown"
    echo "Monitoring simulation processes..."

    # Wait on the server process (the critical one)
    # If GUI crashes, server keeps running
    while kill -0 $GAZEBO_SERVER_PID 2>/dev/null; do
        sleep 5
    done

    echo "Gazebo server exited"
fi
