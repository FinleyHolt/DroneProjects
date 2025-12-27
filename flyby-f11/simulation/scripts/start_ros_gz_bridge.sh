#!/bin/bash
#
# Start ROS GZ Bridge for F-11 ISR Drone
#
# This script starts the ros_gz_bridge to connect Gazebo simulation
# topics to ROS 2 topics for the F-11 ISR drone.
#
# Usage:
#   ./start_ros_gz_bridge.sh                    # Use default config
#   ./start_ros_gz_bridge.sh /path/to/config    # Use custom config
#
# Prerequisites:
#   - ROS 2 Humble sourced: source /opt/ros/humble/setup.bash
#   - ros_gz_bridge package installed
#   - Gazebo simulation running with F-11 ISR model
#
# Bridged Topics (GZ -> ROS):
#   /f11/camera/image_raw   - Camera image from ISR payload (1920x1080)
#   /f11/camera/camera_info - Camera intrinsic parameters
#   /f11/imu                - IMU data at 250 Hz
#   /f11/gps                - GPS NavSat fix at 10 Hz
#
# Bridged Topics (ROS -> GZ):
#   /f11/gimbal/pitch_cmd   - Gimbal pitch command (radians)
#   /f11/gimbal/yaw_cmd     - Gimbal yaw command (radians)

set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIMULATION_DIR="$(dirname "$SCRIPT_DIR")"
DEFAULT_CONFIG="${SIMULATION_DIR}/config/ros_gz_bridge.yaml"

# Use provided config or default
CONFIG_FILE="${1:-$DEFAULT_CONFIG}"

# Verify config exists
if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Error: Configuration file not found: $CONFIG_FILE"
    echo "Usage: $0 [config_file]"
    exit 1
fi

# Check if ROS 2 is sourced
if [[ -z "$ROS_DISTRO" ]]; then
    echo "Warning: ROS 2 does not appear to be sourced."
    echo "Attempting to source /opt/ros/humble/setup.bash..."
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
    else
        echo "Error: Could not find ROS 2 Humble installation."
        exit 1
    fi
fi

echo "=============================================="
echo "  F-11 ISR ROS GZ Bridge"
echo "=============================================="
echo "Config file: $CONFIG_FILE"
echo "ROS distro:  $ROS_DISTRO"
echo ""
echo "Bridged topics (GZ -> ROS):"
echo "  /f11/camera/image_raw"
echo "  /f11/camera/camera_info"
echo "  /f11/imu"
echo "  /f11/gps"
echo ""
echo "Bridged topics (ROS -> GZ):"
echo "  /f11/gimbal/pitch_cmd"
echo "  /f11/gimbal/yaw_cmd"
echo "=============================================="
echo ""

# Start the bridge
exec ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:="$CONFIG_FILE"
