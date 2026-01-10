#!/bin/bash
# GitHub Actions Self-Hosted Runner Setup for DroneProjects
# This script sets up a runner that executes tests inside the Isaac Sim container

set -e

RUNNER_VERSION="2.321.0"
RUNNER_DIR="${HOME}/actions-runner"
REPO_URL="https://github.com/FinleyHolt/DroneProjects"

echo "=== DroneProjects GitHub Actions Runner Setup ==="
echo ""

# Check prerequisites
command -v podman >/dev/null 2>&1 || { echo "Error: podman is required"; exit 1; }
command -v gh >/dev/null 2>&1 || { echo "Error: gh CLI is required"; exit 1; }

# Check if Isaac Sim container exists
if ! podman image exists localhost/isaac-sim-px4:5.1.0; then
    echo "Warning: Isaac Sim container not found. Build it first:"
    echo "  cd isaac-sim && podman-compose build"
    echo ""
fi

# Create runner directory
mkdir -p "${RUNNER_DIR}"
cd "${RUNNER_DIR}"

# Download runner if not present
if [ ! -f "./config.sh" ]; then
    echo "Downloading GitHub Actions runner v${RUNNER_VERSION}..."
    curl -sL -o actions-runner.tar.gz \
        "https://github.com/actions/runner/releases/download/v${RUNNER_VERSION}/actions-runner-linux-x64-${RUNNER_VERSION}.tar.gz"
    tar xzf actions-runner.tar.gz
    rm actions-runner.tar.gz
    echo "Runner downloaded."
else
    echo "Runner already downloaded."
fi

# Get registration token via gh CLI
echo ""
echo "Getting registration token from GitHub..."
RUNNER_TOKEN=$(gh api repos/FinleyHolt/DroneProjects/actions/runners/registration-token \
    --method POST --jq '.token' 2>/dev/null) || {
    echo "Error: Could not get runner token. Make sure you're authenticated:"
    echo "  gh auth login"
    echo "  gh auth refresh -s admin:repo_hook"
    exit 1
}

# Configure runner
echo "Configuring runner..."
./config.sh \
    --url "${REPO_URL}" \
    --token "${RUNNER_TOKEN}" \
    --name "droneprojects-$(cat /etc/hostname 2>/dev/null || echo 'local')" \
    --labels "self-hosted,linux,x64,gpu,isaac-sim" \
    --work "_work" \
    --replace

echo ""
echo "=== Runner configured! ==="
echo ""
echo "To start the runner:"
echo "  cd ${RUNNER_DIR}"
echo "  ./run.sh"
echo ""
echo "To install as a service (runs on boot):"
echo "  cd ${RUNNER_DIR}"
echo "  sudo ./svc.sh install"
echo "  sudo ./svc.sh start"
echo ""
echo "To check status:"
echo "  sudo ./svc.sh status"
echo ""
