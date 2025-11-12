#!/bin/bash
# Environment setup for LLMDrone development
# Source this file before running simulations or building code
# Usage: source setup/env.sh

# Determine repository root (where this script's parent directory is)
if [ -n "${BASH_SOURCE[0]}" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    export LLMDRONE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
elif [ -n "${ZSH_VERSION}" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${(%):-%x}")" && pwd)"
    export LLMDRONE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
else
    echo "Error: Unable to determine script directory. Please set LLMDRONE_ROOT manually."
    return 1
fi

# PX4-Autopilot configuration
export PX4_ROOT="${LLMDRONE_ROOT}/.deps/PX4-Autopilot"
export PX4_VERSION="v1.14.0"

# Check if PX4 is installed
if [ ! -d "${PX4_ROOT}" ]; then
    echo "Warning: PX4-Autopilot not found at ${PX4_ROOT}"
    echo "Run: make clone-px4 or setup/clone_px4.sh"
    return 1
fi

# Gazebo configuration
export GAZEBO_MODEL_PATH="${LLMDRONE_ROOT}/sim/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${LLMDRONE_ROOT}/sim/worlds:${GAZEBO_RESOURCE_PATH}"

# PX4 SITL home position (configurable for different test locations)
# Default: Marine Corps Base Camp Pendleton, CA
export PX4_HOME_LAT=33.2316
export PX4_HOME_LON=-117.3736
export PX4_HOME_ALT=86.0

# ROS 2 domain ID (isolate ROS 2 DDS traffic from other systems)
# Default: 42 (arbitrary, change if conflicts occur)
export ROS_DOMAIN_ID=42

# ROS 2 workspace (once created)
if [ -d "${LLMDRONE_ROOT}/ros2_ws/install" ]; then
    source "${LLMDRONE_ROOT}/ros2_ws/install/setup.bash"
else
    # Check if ROS 2 base installation exists
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source "/opt/ros/humble/setup.bash"
    fi
fi

# Python virtual environment (once created for LLM work)
if [ -d "${LLMDRONE_ROOT}/llm/venv" ]; then
    source "${LLMDRONE_ROOT}/llm/venv/bin/activate"
fi

# Add scripts to PATH for convenience
export PATH="${LLMDRONE_ROOT}/scripts:${PATH}"

# Print confirmation
echo "LLMDrone environment loaded:"
echo "  LLMDRONE_ROOT: ${LLMDRONE_ROOT}"
echo "  PX4_ROOT: ${PX4_ROOT}"
echo "  PX4_VERSION: ${PX4_VERSION}"
echo "  PX4_HOME: ${PX4_HOME_LAT}, ${PX4_HOME_LON}, ${PX4_HOME_ALT}m"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

if [ -d "${LLMDRONE_ROOT}/ros2_ws/install" ]; then
    echo "  ROS 2 workspace: sourced"
else
    echo "  ROS 2 workspace: not yet created"
fi
