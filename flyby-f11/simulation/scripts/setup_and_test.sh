#!/bin/bash
#
# Flyby F-11 Simulation Setup and Test Script
#
# This script:
# 1. Checks for nvidia-container-toolkit installation
# 2. Configures CDI for GPU passthrough
# 3. Tests GPU access in containers
# 4. Runs SITL integration tests
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if nvidia-container-toolkit is installed
check_nvidia_toolkit() {
    log_info "Checking for nvidia-container-toolkit..."

    if ! command -v nvidia-ctk &> /dev/null; then
        log_error "nvidia-container-toolkit is NOT installed"
        echo ""
        echo "Please install it with:"
        echo "  yay -S nvidia-container-toolkit"
        echo ""
        return 1
    fi

    log_info "✓ nvidia-container-toolkit is installed"
    return 0
}

# Function to check and generate CDI configuration
setup_cdi() {
    log_info "Checking CDI configuration..."

    if [ ! -f /etc/cdi/nvidia.yaml ]; then
        log_warn "CDI configuration not found, generating..."

        sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

        if [ $? -eq 0 ]; then
            log_info "✓ CDI configuration generated at /etc/cdi/nvidia.yaml"
        else
            log_error "Failed to generate CDI configuration"
            return 1
        fi
    else
        log_info "✓ CDI configuration exists"
    fi

    return 0
}

# Function to test GPU access in container
test_gpu_access() {
    log_info "Testing GPU access in container..."

    if podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi &> /dev/null; then
        log_info "✓ GPU access works in containers"
        echo ""
        echo "GPU Information:"
        podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
        return 0
    else
        log_error "GPU access test failed"
        return 1
    fi
}

# Function to test Gazebo with GPU
test_gazebo_gpu() {
    log_info "Testing Gazebo with GPU rendering..."

    # Check if DISPLAY is set
    if [ -z "$DISPLAY" ]; then
        log_warn "DISPLAY not set, skipping GUI test"
        return 0
    fi

    log_info "Starting Gazebo with GPU for 15 seconds..."

    timeout 15 podman run --rm \
        --device nvidia.com/gpu=all \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
        -e GAZEBO_ONLY=true \
        localhost/flyby-f11-sim:latest &

    GAZEBO_PID=$!
    sleep 10

    if ps -p $GAZEBO_PID > /dev/null; then
        log_info "✓ Gazebo started successfully with GPU"
        kill $GAZEBO_PID 2>/dev/null || true
        wait $GAZEBO_PID 2>/dev/null || true
        return 0
    else
        log_warn "Gazebo process exited early"
        return 1
    fi
}

# Function to run basic SITL tests
run_sitl_tests() {
    log_info "Running SITL integration tests..."

    # Start simulation in background
    log_info "Starting simulation stack (headless mode)..."

    podman run --rm --network host \
        -e HEADLESS=true \
        --name flyby-test-sitl \
        localhost/flyby-f11-sim:latest &

    SITL_PID=$!

    # Wait for SITL to initialize
    log_info "Waiting 15 seconds for SITL to initialize..."
    sleep 15

    # Check if SITL is still running
    if ! ps -p $SITL_PID > /dev/null; then
        log_error "SITL process died during startup"
        return 1
    fi

    log_info "✓ SITL is running, executing tests..."

    # Run pymavlink test
    log_info "Running MAVLink connectivity test..."

    python3 /home/finley/Github/DroneProjects/flyby-f11/simulation/scripts/test_sitl_basic.py \
        --connection tcp:127.0.0.1:5762

    TEST_RESULT=$?

    # Cleanup
    log_info "Stopping simulation..."
    kill $SITL_PID 2>/dev/null || true
    wait $SITL_PID 2>/dev/null || true

    # Cleanup any remaining processes
    podman stop flyby-test-sitl 2>/dev/null || true

    if [ $TEST_RESULT -eq 0 ]; then
        log_info "✓ SITL tests PASSED"
        return 0
    else
        log_error "SITL tests FAILED"
        return 1
    fi
}

# Main execution
main() {
    echo "=========================================="
    echo "Flyby F-11 Simulation Setup and Test"
    echo "=========================================="
    echo ""

    FAILED_STEPS=0

    # Step 1: Check nvidia-container-toolkit
    if ! check_nvidia_toolkit; then
        log_error "Please install nvidia-container-toolkit first"
        exit 1
    fi
    echo ""

    # Step 2: Setup CDI
    if ! setup_cdi; then
        log_error "CDI setup failed"
        ((FAILED_STEPS++))
    fi
    echo ""

    # Step 3: Test GPU access
    if ! test_gpu_access; then
        log_error "GPU access test failed"
        ((FAILED_STEPS++))
    fi
    echo ""

    # Step 4: Test Gazebo GPU (optional)
    if ! test_gazebo_gpu; then
        log_warn "Gazebo GPU test skipped or failed (non-critical)"
    fi
    echo ""

    # Step 5: Run SITL tests
    if ! run_sitl_tests; then
        log_error "SITL tests failed"
        ((FAILED_STEPS++))
    fi
    echo ""

    # Summary
    echo "=========================================="
    echo "TEST SUMMARY"
    echo "=========================================="

    if [ $FAILED_STEPS -eq 0 ]; then
        log_info "✓ ALL TESTS PASSED"
        echo ""
        echo "Your simulation environment is ready!"
        echo ""
        echo "Usage:"
        echo "  # Headless training mode:"
        echo "  podman run --rm --network host -e HEADLESS=true localhost/flyby-f11-sim:latest"
        echo ""
        echo "  # With GPU rendering:"
        echo "  podman run --rm --device nvidia.com/gpu=all --network host \\"
        echo "    -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro \\"
        echo "    localhost/flyby-f11-sim:latest"
        echo ""
        exit 0
    else
        log_error "$FAILED_STEPS test step(s) failed"
        exit 1
    fi
}

# Run main
main "$@"
