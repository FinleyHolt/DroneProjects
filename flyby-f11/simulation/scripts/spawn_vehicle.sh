#!/bin/bash
# Spawn ArduPilot SITL vehicle for Flyby F-11 simulation
#
# Usage:
#   ./spawn_vehicle.sh                    # Default spawn at center
#   ./spawn_vehicle.sh --location corner_ne
#   ./spawn_vehicle.sh --instance 1       # Second instance
#   ./spawn_vehicle.sh --speedup 2        # 2x speed
#   ./spawn_vehicle.sh --headless         # No console

set -e

# Default values
LOCATION="training_center"
INSTANCE=0
SPEEDUP=1
HEADLESS=false
CONSOLE=true

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -l|--location)
            LOCATION="$2"
            shift 2
            ;;
        -i|--instance)
            INSTANCE="$2"
            shift 2
            ;;
        -s|--speedup)
            SPEEDUP="$2"
            shift 2
            ;;
        --headless)
            HEADLESS=true
            CONSOLE=false
            shift
            ;;
        --no-console)
            CONSOLE=false
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -l, --location NAME   Spawn location (default: training_center)"
            echo "  -i, --instance N      Instance number (default: 0)"
            echo "  -s, --speedup N       Simulation speedup (default: 1)"
            echo "  --headless            Run without console/GUI"
            echo "  --no-console          Disable MAVProxy console"
            echo "  -h, --help            Show this help"
            echo ""
            echo "Available locations:"
            echo "  training_center      - Center of training world"
            echo "  training_corner_ne   - Northeast corner"
            echo "  training_corner_sw   - Southwest corner"
            echo "  training_near_nfz1   - Near cylindrical NFZ"
            echo "  training_near_nfz2   - Near rectangular NFZ"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Calculate ports for this instance
SITL_PORT=$((5760 + INSTANCE * 10))
MAVLINK_PORT=$((14550 + INSTANCE))

echo "========================================"
echo "Flyby F-11 SITL Vehicle Spawn"
echo "========================================"
echo "Location:     $LOCATION"
echo "Instance:     $INSTANCE"
echo "Speedup:      ${SPEEDUP}x"
echo "SITL Port:    $SITL_PORT"
echo "MAVLink Port: $MAVLINK_PORT"
echo "========================================"

# Check if ArduPilot directory exists
ARDUPILOT_DIR="${ARDUPILOT_HOME:-/home/ardupilot/ardupilot}"
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "Error: ArduPilot not found at $ARDUPILOT_DIR"
    echo "Set ARDUPILOT_HOME environment variable"
    exit 1
fi

# Check if locations file exists
LOCATIONS_FILE="/simulation/configs/locations.txt"
if [ ! -f "$LOCATIONS_FILE" ]; then
    LOCATIONS_FILE="$(dirname "$0")/../configs/locations.txt"
fi

# Build SITL command
CMD="cd $ARDUPILOT_DIR && sim_vehicle.py -v ArduCopter"
CMD="$CMD --instance $INSTANCE"
CMD="$CMD --out=udp:127.0.0.1:$MAVLINK_PORT"
CMD="$CMD -L $LOCATION"
CMD="$CMD -S $SPEEDUP"

if [ -f "$LOCATIONS_FILE" ]; then
    CMD="$CMD --locations=$LOCATIONS_FILE"
fi

PARAM_FILE="/simulation/configs/arducopter.parm"
if [ ! -f "$PARAM_FILE" ]; then
    PARAM_FILE="$(dirname "$0")/../configs/arducopter.parm"
fi
if [ -f "$PARAM_FILE" ]; then
    CMD="$CMD --add-param-file=$PARAM_FILE"
fi

if [ "$CONSOLE" = "false" ]; then
    CMD="$CMD --no-mavproxy"
fi

echo ""
echo "Starting SITL..."
echo "Command: $CMD"
echo ""

# Run SITL
eval $CMD
