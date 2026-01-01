#!/bin/bash
# ==============================================================================
# Flyby F-11 RL Pipeline - Run Smoke Tests in Isaac Sim Container
# ==============================================================================
#
# This script launches the Isaac Sim container and runs tests inside it.
#
# Prerequisites:
#   - Podman installed
#   - Isaac Sim container built: localhost/isaac-sim-px4:6e-test
#   - NVIDIA GPU available
#
# Usage:
#   ./scripts/run_smoke_test_in_container.sh            # Quick smoke test
#   ./scripts/run_smoke_test_in_container.sh --full     # Full test suite
#   ./scripts/run_smoke_test_in_container.sh --e2e      # E2E smoke test only
#   ./scripts/run_smoke_test_in_container.sh --integration  # Integration tests
#
# ==============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CONTAINER_IMAGE="${ISAAC_SIM_IMAGE:-localhost/isaac-sim-px4:6e-test}"
CONTAINER_NAME="flyby-f11-test-$$"
WORKSPACE_MOUNT="${PROJECT_ROOT}:/workspace:Z"

# Test mode
TEST_MODE="quick"
PYTEST_ARGS=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --quick)
            TEST_MODE="quick"
            shift
            ;;
        --full)
            TEST_MODE="full"
            shift
            ;;
        --e2e)
            TEST_MODE="e2e"
            shift
            ;;
        --integration)
            TEST_MODE="integration"
            shift
            ;;
        --all)
            TEST_MODE="all"
            shift
            ;;
        -v|--verbose)
            PYTEST_ARGS="-v -s"
            shift
            ;;
        --image)
            CONTAINER_IMAGE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --quick        Quick smoke test (default) - 50 steps, 1 episode"
            echo "  --full         Full smoke test - 100 steps, 3 episodes"
            echo "  --e2e          Run E2E smoke tests"
            echo "  --integration  Run integration tests"
            echo "  --all          Run all container tests"
            echo "  -v, --verbose  Verbose output"
            echo "  --image IMAGE  Use specified container image"
            echo "  -h, --help     Show this help"
            echo ""
            echo "Container image: ${CONTAINER_IMAGE}"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# ==============================================================================
# Pre-flight Checks
# ==============================================================================

echo -e "${BLUE}================================================================${NC}"
echo -e "${BLUE}Flyby F-11 RL Pipeline - Container Tests${NC}"
echo -e "${BLUE}================================================================${NC}"
echo ""

# Check for podman
if ! command -v podman &> /dev/null; then
    echo -e "${RED}Error: podman not found${NC}"
    exit 1
fi

# Check for container image
if ! podman image exists "$CONTAINER_IMAGE"; then
    echo -e "${RED}Error: Container image not found: ${CONTAINER_IMAGE}${NC}"
    echo ""
    echo "Build it with:"
    echo "  cd evaluation/isaac-sim-px4"
    echo "  podman build -f Containerfile -t isaac-sim-px4:6e-test ."
    exit 1
fi

# Check for GPU
if ! command -v nvidia-smi &> /dev/null; then
    echo -e "${YELLOW}Warning: nvidia-smi not found - GPU may not be available${NC}"
fi

echo "Configuration:"
echo "  Image: ${CONTAINER_IMAGE}"
echo "  Test mode: ${TEST_MODE}"
echo "  Workspace: ${PROJECT_ROOT}"
echo ""

# ==============================================================================
# Build Test Command
# ==============================================================================

case $TEST_MODE in
    quick)
        TEST_CMD="/isaac-sim/python.sh /workspace/tests/test_e2e_smoke.py --quick"
        ;;
    full)
        TEST_CMD="/isaac-sim/python.sh /workspace/tests/test_e2e_smoke.py --steps 100 --episodes 3"
        ;;
    e2e)
        TEST_CMD="/isaac-sim/python.sh -m pytest /workspace/tests/test_e2e_smoke.py ${PYTEST_ARGS} -v"
        ;;
    integration)
        TEST_CMD="/isaac-sim/python.sh -m pytest /workspace/tests/test_integration.py ${PYTEST_ARGS} -v"
        ;;
    all)
        TEST_CMD="bash -c '/isaac-sim/python.sh -m pytest /workspace/tests/test_integration.py -v && /isaac-sim/python.sh -m pytest /workspace/tests/test_e2e_smoke.py -v'"
        ;;
esac

# ==============================================================================
# Run Container
# ==============================================================================

echo -e "${BLUE}Starting container...${NC}"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up container...${NC}"
    podman rm -f "$CONTAINER_NAME" 2>/dev/null || true
}
trap cleanup EXIT

# Run the container
echo "Command: ${TEST_CMD}"
echo ""

podman run --rm \
    --name "$CONTAINER_NAME" \
    --device nvidia.com/gpu=all \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    -v "${WORKSPACE_MOUNT}" \
    --security-opt label=disable \
    --ipc host \
    "$CONTAINER_IMAGE" \
    $TEST_CMD

TEST_EXIT_CODE=$?

# ==============================================================================
# Results
# ==============================================================================

echo ""
echo -e "${BLUE}================================================================${NC}"
echo -e "${BLUE}TEST RESULTS${NC}"
echo -e "${BLUE}================================================================${NC}"

if [ $TEST_EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}All tests PASSED${NC}"
else
    echo -e "${RED}Tests FAILED (exit code: ${TEST_EXIT_CODE})${NC}"
fi

exit $TEST_EXIT_CODE
