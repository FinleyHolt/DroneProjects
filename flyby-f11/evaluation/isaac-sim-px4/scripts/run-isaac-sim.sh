#!/bin/bash
# Isaac Sim 5.1.0 Launcher Helper
#
# Provides convenient wrappers for common Isaac Sim operations.
# Designed to be used inside the container.
#
# Usage:
#   ./run-isaac-sim.sh gui       # Launch GUI
#   ./run-isaac-sim.sh headless  # Launch headless
#   ./run-isaac-sim.sh test      # Run MVE test
#   ./run-isaac-sim.sh shell     # Interactive Python shell

set -e

ISAACSIM_PATH="${ISAACSIM_PATH:-/isaac-sim}"
PEGASUS_PATH="${PEGASUS_PATH:-/pegasus}"
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${GREEN}============================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}============================================${NC}"
}

print_warning() {
    echo -e "${YELLOW}WARNING: $1${NC}"
}

print_error() {
    echo -e "${RED}ERROR: $1${NC}"
}

# Check if Isaac Sim is available
check_isaac_sim() {
    if [[ ! -d "$ISAACSIM_PATH" ]]; then
        print_error "Isaac Sim not found at $ISAACSIM_PATH"
        exit 1
    fi
}

# Setup environment for Isaac Sim 5.1.0
setup_environment() {
    # Add Pegasus to extension search paths
    export ISAAC_SIM_EXTENSIONS_PATH="${PEGASUS_PATH}/extensions:${ISAAC_SIM_EXTENSIONS_PATH}"

    # Ensure PX4 is in PATH
    export PATH="/px4/build/px4_sitl_default/bin:${PATH}"

    # Accept EULA
    export ACCEPT_EULA=Y
    export PRIVACY_CONSENT=Y
    export OMNI_KIT_ACCEPT_EULA=YES
}

# Launch Isaac Sim GUI
launch_gui() {
    print_header "Launching Isaac Sim 5.1.0 GUI"
    check_isaac_sim
    setup_environment

    cd "$ISAACSIM_PATH"
    ./isaac-sim.sh
}

# Launch Isaac Sim headless
launch_headless() {
    print_header "Launching Isaac Sim 5.1.0 Headless"
    check_isaac_sim
    setup_environment

    cd "$ISAACSIM_PATH"
    ./isaac-sim.sh --headless
}

# Run MVE test script
run_test() {
    print_header "Running PX4 + Pegasus MVE Test"
    check_isaac_sim
    setup_environment

    local headless_flag=""
    if [[ "$1" == "--headless" ]]; then
        headless_flag="--headless"
    fi

    # Use isaac_run for 5.1.0 (new launcher)
    # Falls back to isaac-sim.sh python if isaac_run not available
    if command -v isaac_run &> /dev/null; then
        isaac_run python3 "${SCRIPTS_DIR}/test_px4_pegasus.py" $headless_flag
    else
        "${ISAACSIM_PATH}/isaac-sim.sh" --python "${SCRIPTS_DIR}/test_px4_pegasus.py" $headless_flag
    fi
}

# Interactive Python shell with Isaac Sim
launch_shell() {
    print_header "Isaac Sim Python Shell"
    check_isaac_sim
    setup_environment

    cd "$ISAACSIM_PATH"
    ./isaac-sim.sh --python
}

# Check GPU availability
check_gpu() {
    print_header "GPU Check"

    echo "NVIDIA Driver:"
    nvidia-smi --query-gpu=driver_version,name,memory.total --format=csv
    echo ""

    echo "Vulkan ICD:"
    ls -la /usr/share/vulkan/icd.d/ 2>/dev/null || echo "No Vulkan ICD found"
    echo ""

    echo "Display:"
    echo "  DISPLAY=$DISPLAY"
    echo "  XAUTHORITY=$XAUTHORITY"
}

# Print help
print_help() {
    echo "Isaac Sim 5.1.0 Launcher Helper"
    echo ""
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  gui              Launch Isaac Sim with GUI"
    echo "  headless         Launch Isaac Sim in headless mode"
    echo "  test             Run MVE test (GUI mode)"
    echo "  test --headless  Run MVE test (headless mode)"
    echo "  shell            Interactive Python shell"
    echo "  check-gpu        Check GPU availability"
    echo "  help             Show this help message"
    echo ""
    echo "Environment Variables:"
    echo "  ISAACSIM_PATH    Path to Isaac Sim (default: /isaac-sim)"
    echo "  PEGASUS_PATH     Path to Pegasus (default: /pegasus)"
}

# Main
case "${1:-help}" in
    gui)
        launch_gui
        ;;
    headless)
        launch_headless
        ;;
    test)
        run_test "${2:-}"
        ;;
    shell)
        launch_shell
        ;;
    check-gpu)
        check_gpu
        ;;
    help|--help|-h)
        print_help
        ;;
    *)
        print_error "Unknown command: $1"
        print_help
        exit 1
        ;;
esac
