#!/bin/bash
# System requirements checker for LLMDrone project
# Validates Ubuntu version, RAM, disk space, and GPU drivers

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "LLMDrone System Requirements Check"
echo "=========================================="
echo

# Check Ubuntu version
echo -n "Checking Ubuntu version... "
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [[ "$ID" == "ubuntu" ]]; then
        VERSION_NUM="${VERSION_ID}"
        if [[ "$VERSION_NUM" == "22.04" ]] || [[ "$VERSION_NUM" == "24.04" ]]; then
            echo -e "${GREEN}OK${NC} (Ubuntu ${VERSION_NUM})"
        else
            echo -e "${YELLOW}WARNING${NC} (Ubuntu ${VERSION_NUM})"
            echo "  Recommended: Ubuntu 22.04 or 24.04 LTS"
            echo "  Your version may work but is untested"
        fi
    else
        echo -e "${RED}FAIL${NC}"
        echo "  This system is not Ubuntu. Detected: ${ID}"
        echo "  LLMDrone requires Ubuntu 22.04 or 24.04 LTS"
        exit 1
    fi
else
    echo -e "${RED}FAIL${NC}"
    echo "  Cannot determine OS version (/etc/os-release not found)"
    exit 1
fi

# Check RAM
echo -n "Checking available RAM... "
TOTAL_RAM_KB=$(grep MemTotal /proc/meminfo | awk '{print $2}')
TOTAL_RAM_GB=$((TOTAL_RAM_KB / 1024 / 1024))
if [ "$TOTAL_RAM_GB" -ge 16 ]; then
    echo -e "${GREEN}OK${NC} (${TOTAL_RAM_GB} GB)"
else
    echo -e "${RED}FAIL${NC} (${TOTAL_RAM_GB} GB)"
    echo "  Minimum requirement: 16 GB RAM"
    echo "  Your system may experience memory issues during compilation"
    exit 1
fi

# Check disk space
echo -n "Checking available disk space... "
AVAILABLE_GB=$(df "${REPO_ROOT}" | awk 'NR==2 {printf "%.0f", $4/1024/1024}')
if [ "$AVAILABLE_GB" -ge 50 ]; then
    echo -e "${GREEN}OK${NC} (${AVAILABLE_GB} GB available)"
else
    echo -e "${RED}FAIL${NC} (${AVAILABLE_GB} GB available)"
    echo "  Minimum requirement: 50 GB free disk space"
    echo "  PX4 build + dependencies + Gazebo models require significant storage"
    exit 1
fi

# Check for NVIDIA GPU and drivers
echo -n "Checking NVIDIA GPU... "
if command -v nvidia-smi &> /dev/null; then
    DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -n1)
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n1)

    if [ -n "$DRIVER_VERSION" ]; then
        # Extract major version number
        DRIVER_MAJOR=$(echo "$DRIVER_VERSION" | cut -d. -f1)

        if [ "$DRIVER_MAJOR" -ge 525 ]; then
            echo -e "${GREEN}OK${NC}"
            echo "  GPU: ${GPU_NAME}"
            echo "  Driver: ${DRIVER_VERSION} (CUDA 12+ compatible)"
        else
            echo -e "${YELLOW}WARNING${NC}"
            echo "  GPU: ${GPU_NAME}"
            echo "  Driver: ${DRIVER_VERSION}"
            echo "  Recommended: Driver >=525.x for CUDA 12 support"
            echo "  You may experience issues with vision models"
        fi
    else
        echo -e "${YELLOW}WARNING${NC}"
        echo "  nvidia-smi found but cannot query driver version"
        echo "  GPU acceleration may not work properly"
    fi
else
    echo -e "${YELLOW}WARNING${NC}"
    echo "  No NVIDIA GPU detected or drivers not installed"
    echo "  Simulation will run but vision models will use CPU (slower)"
    echo "  For GPU support, install NVIDIA drivers: sudo ubuntu-drivers autoinstall"
fi

# Check for proxy configuration
echo -n "Checking proxy configuration... "
if [ -n "$http_proxy" ] || [ -n "$https_proxy" ] || [ -n "$HTTP_PROXY" ] || [ -n "$HTTPS_PROXY" ]; then
    echo -e "${YELLOW}DETECTED${NC}"
    echo "  http_proxy: ${http_proxy:-${HTTP_PROXY:-not set}}"
    echo "  https_proxy: ${https_proxy:-${HTTPS_PROXY:-not set}}"
    echo "  Git and apt should respect these settings automatically"
    echo "  If you experience download issues, ensure git is configured:"
    echo "    git config --global http.proxy \$http_proxy"
else
    echo -e "${GREEN}None${NC} (direct internet connection)"
fi

# Check for git
echo -n "Checking for git... "
if command -v git &> /dev/null; then
    GIT_VERSION=$(git --version | awk '{print $3}')
    echo -e "${GREEN}OK${NC} (version ${GIT_VERSION})"
else
    echo -e "${RED}FAIL${NC}"
    echo "  Git is required but not installed"
    echo "  Install with: sudo apt install git"
    exit 1
fi

echo
echo "=========================================="
echo -e "${GREEN}System check completed successfully${NC}"
echo "=========================================="
echo
echo "You can proceed with running: make setup"
echo
