#!/bin/bash
# Clone PX4-Autopilot into .deps/ directory and run its setup script
# Pinned to v1.14.0 for stability

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STATE_DIR="${REPO_ROOT}/.setup_state"
DEPS_DIR="${REPO_ROOT}/.deps"
PX4_DIR="${DEPS_DIR}/PX4-Autopilot"
PX4_VERSION="v1.14.0"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "PX4-Autopilot Setup"
echo "=========================================="
echo

# Create state and deps directories
mkdir -p "${STATE_DIR}"
mkdir -p "${DEPS_DIR}"

# Check if already cloned
if [ -f "${STATE_DIR}/px4_cloned.done" ] && [ -d "${PX4_DIR}" ]; then
    echo -e "${GREEN}PX4-Autopilot already cloned and configured.${NC}"
    echo "Location: ${PX4_DIR}"
    echo "Version: ${PX4_VERSION}"
    echo
    echo "To re-clone PX4, run:"
    echo "  rm -rf ${PX4_DIR} ${STATE_DIR}/px4_cloned.done"
    echo
    exit 0
fi

# Check if directory exists but state file doesn't (incomplete installation)
if [ -d "${PX4_DIR}" ] && [ ! -f "${STATE_DIR}/px4_cloned.done" ]; then
    echo -e "${YELLOW}Warning: PX4 directory exists but installation was not completed.${NC}"
    echo "Removing incomplete installation..."
    rm -rf "${PX4_DIR}"
fi

# Clone PX4-Autopilot
echo -e "${BLUE}Cloning PX4-Autopilot ${PX4_VERSION}...${NC}"
echo "This may take several minutes depending on your connection speed."
echo

git clone https://github.com/PX4/PX4-Autopilot.git --recursive --branch "${PX4_VERSION}" "${PX4_DIR}"

echo
echo -e "${GREEN}Clone completed.${NC}"
echo

# Run PX4 setup script for Ubuntu (SITL only, no NuttX firmware)
echo -e "${BLUE}Running PX4 dependency setup...${NC}"
echo "This will install additional dependencies required by PX4 SITL."
echo "You may be prompted for sudo password."
echo

cd "${PX4_DIR}"

# Run PX4's Ubuntu setup script without NuttX (embedded firmware) toolchain
# --no-nuttx flag skips ARM cross-compilation tools (saves ~2GB and setup time)
if [ -f "Tools/setup/ubuntu.sh" ]; then
    bash Tools/setup/ubuntu.sh --no-nuttx
else
    echo -e "${RED}Error: PX4 setup script not found at Tools/setup/ubuntu.sh${NC}"
    echo "The PX4 repository structure may have changed."
    exit 1
fi

cd "${REPO_ROOT}"

# Mark as complete
touch "${STATE_DIR}/px4_cloned.done"

echo
echo "=========================================="
echo -e "${GREEN}PX4 setup completed successfully${NC}"
echo "=========================================="
echo
echo "PX4-Autopilot location: ${PX4_DIR}"
echo "Version: ${PX4_VERSION}"
echo
echo "Next steps:"
echo "  1. Source environment: source setup/env.sh"
echo "  2. Build and run SITL: make sim"
echo
echo "To clean up and start over:"
echo "  rm -rf ${DEPS_DIR} ${STATE_DIR}"
echo
