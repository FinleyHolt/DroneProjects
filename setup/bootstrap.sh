#!/bin/bash
# Bootstrap script for LLMDrone development environment
# Installs system dependencies required for PX4 SITL and ROS 2

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STATE_DIR="${REPO_ROOT}/.setup_state"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "LLMDrone Bootstrap Setup"
echo "=========================================="
echo

# Create state directory for tracking installation progress
mkdir -p "${STATE_DIR}"

# Run system check first
echo -e "${BLUE}Running system requirements check...${NC}"
if ! "${SCRIPT_DIR}/check_system.sh"; then
    echo -e "${RED}System check failed. Please address the issues above.${NC}"
    exit 1
fi
echo

# Check if already bootstrapped
if [ -f "${STATE_DIR}/bootstrap.done" ]; then
    echo -e "${GREEN}Bootstrap already completed.${NC}"
    echo "To re-run bootstrap, delete: ${STATE_DIR}/bootstrap.done"
    echo
    exit 0
fi

echo "Installing system dependencies..."
echo

# Update package lists
echo -e "${BLUE}Updating package lists...${NC}"
sudo apt update

# Install build essentials
echo -e "${BLUE}Installing build tools...${NC}"
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    ca-certificates \
    gnupg \
    lsb-release

# Install Python dependencies
echo -e "${BLUE}Installing Python tools...${NC}"
sudo apt install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv

# Install PX4 dependencies (minimal set for SITL)
echo -e "${BLUE}Installing PX4 SITL dependencies...${NC}"
sudo apt install -y \
    openjdk-11-jre \
    libxml2-utils \
    python3-jinja2 \
    python3-toml \
    python3-numpy \
    python3-yaml \
    python3-packaging \
    python3-jsonschema \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libeigen3-dev \
    libgstreamer-plugins-base1.0-dev

# Install Gazebo dependencies
echo -e "${BLUE}Installing Gazebo dependencies...${NC}"
sudo apt install -y \
    libgazebo11 \
    gazebo11 \
    libgazebo11-dev

# Install ROS 2 preparation tools (actual ROS 2 installation comes later)
echo -e "${BLUE}Installing ROS 2 preparation tools...${NC}"
sudo apt install -y \
    software-properties-common \
    python3-colcon-common-extensions

# Install git LFS for large binary files (Gazebo models, LLM weights)
echo -e "${BLUE}Installing Git LFS...${NC}"
if ! command -v git-lfs &> /dev/null; then
    sudo apt install -y git-lfs
    git lfs install
else
    echo "Git LFS already installed"
fi

# Mark bootstrap as complete
touch "${STATE_DIR}/bootstrap.done"

echo
echo "=========================================="
echo -e "${GREEN}Bootstrap completed successfully${NC}"
echo "=========================================="
echo
echo "Next steps:"
echo "  1. Run: make clone-px4  (or setup/clone_px4.sh)"
echo "  2. Source environment: source setup/env.sh"
echo "  3. Launch simulation: make sim"
echo
echo "To start over, run: rm -rf ${STATE_DIR}"
echo
