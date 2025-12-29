#!/bin/bash
# Launch Isaac Sim in GUI or headless mode
#
# Usage:
#   ./run-isaac-sim.sh           # GUI mode with Pegasus (default)
#   ./run-isaac-sim.sh --headless  # Headless mode with Pegasus
#   ./run-isaac-sim.sh --shell     # Interactive shell only
#   ./run-isaac-sim.sh --test      # Run ArduPilot integration test
#
# Prerequisites:
#   - NVIDIA GPU with driver 535+
#   - Podman with nvidia-container-toolkit
#   - Isaac Sim container pulled: podman pull nvcr.io/nvidia/isaac-sim:4.2.0

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVAL_DIR="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Parse arguments
MODE="gui"
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            MODE="headless"
            shift
            ;;
        --shell)
            MODE="shell"
            shift
            ;;
        --test)
            MODE="test"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--headless|--shell|--test]"
            echo ""
            echo "Options:"
            echo "  --headless  Run without GUI (for CI/training)"
            echo "  --shell     Start interactive shell only"
            echo "  --test      Run ArduPilot integration test"
            echo "  -h, --help  Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}  Isaac Sim Launcher - ${MODE} mode${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""

# Check for NVIDIA GPU
echo -e "${YELLOW}[1/4] Checking GPU...${NC}"
if ! nvidia-smi --query-gpu=name --format=csv,noheader &>/dev/null; then
    echo -e "${RED}Error: NVIDIA GPU not detected${NC}"
    exit 1
fi
GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader)
echo "  Found: $GPU_NAME"

# Check container image
echo -e "${YELLOW}[2/4] Checking Isaac Sim container...${NC}"
if ! podman image exists nvcr.io/nvidia/isaac-sim:4.2.0; then
    echo -e "${RED}Error: Isaac Sim container not found${NC}"
    echo "  Run: podman pull nvcr.io/nvidia/isaac-sim:4.2.0"
    exit 1
fi
echo "  Container: nvcr.io/nvidia/isaac-sim:4.2.0"

# Setup X11 for GUI mode
if [ "$MODE" = "gui" ]; then
    echo -e "${YELLOW}[3/4] Setting up X11 access...${NC}"

    # Allow local X11 connections
    if command -v xhost &>/dev/null; then
        xhost +local: &>/dev/null
        echo "  X11 access: enabled for local connections"
    else
        echo -e "${YELLOW}  Warning: xhost not found, GUI may not work${NC}"
    fi

    # Check display
    if [ -z "$DISPLAY" ]; then
        echo -e "${RED}Error: DISPLAY not set${NC}"
        exit 1
    fi
    echo "  Display: $DISPLAY"
else
    echo -e "${YELLOW}[3/4] Headless mode - skipping X11 setup${NC}"
fi

# Launch container
echo -e "${YELLOW}[4/4] Launching container...${NC}"
echo ""

cd "$EVAL_DIR"

case $MODE in
    gui)
        echo "Starting Isaac Sim GUI..."
        echo "First launch will take 10-20 minutes for shader compilation."
        echo ""
        podman-compose up isaac-sim-gui
        ;;
    headless)
        echo "Starting Isaac Sim headless..."
        echo ""
        podman-compose up isaac-sim-headless
        ;;
    shell)
        echo "Starting interactive shell..."
        echo "Run './isaac-sim.sh' to start Isaac Sim manually."
        echo ""
        podman run -it --rm \
            --device nvidia.com/gpu=all \
            -e ACCEPT_EULA=Y \
            -e PRIVACY_CONSENT=Y \
            -e DISPLAY=${DISPLAY:-:0} \
            -e OMNI_KIT_ACCEPT_EULA=YES \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v ${XAUTHORITY:-~/.Xauthority}:/root/.Xauthority:ro \
            -v "$EVAL_DIR:/workspace:z" \
            --network host \
            --ipc host \
            --security-opt label=disable \
            -w /isaac-sim \
            nvcr.io/nvidia/isaac-sim:4.2.0 \
            /bin/bash
        ;;
    test)
        echo "Running ArduPilot + Pegasus integration test..."
        echo "Make sure ArduPilot SITL is running:"
        echo "  podman-compose --profile sitl up ardupilot-sitl"
        echo ""
        podman exec -it isaac-sim-dev \
            /isaac-sim/python.sh /workspace/scripts/test_ardupilot_pegasus.py
        ;;
esac
