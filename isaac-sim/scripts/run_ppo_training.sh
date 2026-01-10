#!/bin/bash
# ==============================================================================
# ISR PPO Training Script
# ==============================================================================
# Runs PPO training inside the Isaac Sim container with proper GPU access.
#
# Usage:
#   ./scripts/run_ppo_training.sh [scenario] [num_envs] [total_timesteps]
#
# Examples:
#   ./scripts/run_ppo_training.sh                           # Default: area_coverage, 64 envs, 1M steps
#   ./scripts/run_ppo_training.sh area_coverage 128 5000000 # 5M steps with 128 envs
#   ./scripts/run_ppo_training.sh multi_objective 64        # multi_objective scenario
#
# Container: localhost/isaac-sim-px4:5.1.0-px4-1.14.3
# ==============================================================================

set -e

# Configuration
SCENARIO="${1:-area_coverage}"
NUM_ENVS="${2:-64}"
TOTAL_TIMESTEPS="${3:-1000000}"
CONTAINER_NAME="flyby-ppo-training"
IMAGE="localhost/isaac-sim-px4:5.1.0-px4-1.14.3"

# Get script directory (for volume mounts)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "ISR PPO Training"
echo "=============================================="
echo "Scenario:        $SCENARIO"
echo "Num envs:        $NUM_ENVS"
echo "Total timesteps: $TOTAL_TIMESTEPS"
echo "Container:       $CONTAINER_NAME"
echo "Project dir:     $PROJECT_DIR"
echo "=============================================="

# Create output directories
mkdir -p "$PROJECT_DIR/logs" "$PROJECT_DIR/checkpoints"

# Stop and remove any existing container with same name
if podman container exists "$CONTAINER_NAME" 2>/dev/null; then
    echo "Stopping existing container..."
    podman stop "$CONTAINER_NAME" 2>/dev/null || true
    podman rm "$CONTAINER_NAME" 2>/dev/null || true
fi

# Start container with GPU access and volume mounts
echo ""
echo "Starting container..."
podman run -d \
    --name "$CONTAINER_NAME" \
    --device nvidia.com/gpu=all \
    -v "$PROJECT_DIR/training:/workspace/training:Z" \
    -v "$PROJECT_DIR/logs:/workspace/logs:Z" \
    -v "$PROJECT_DIR/checkpoints:/workspace/checkpoints:Z" \
    --network host \
    "$IMAGE"

# Wait for container to be ready
echo "Waiting for container to start..."
sleep 3

# Run training inside container using Isaac Sim's Python
echo ""
echo "Starting PPO training..."
echo "=============================================="

# Run training and capture output to log file
LOG_FILE="$PROJECT_DIR/logs/training_${SCENARIO}_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to: $LOG_FILE"
echo ""

podman exec -it "$CONTAINER_NAME" /isaac-sim/python.sh \
    /workspace/training/scripts/train_isr_ppo.py \
    --scenario "$SCENARIO" \
    --num_envs "$NUM_ENVS" \
    --total_timesteps "$TOTAL_TIMESTEPS" \
    --save_path /workspace/checkpoints \
    --log_path /workspace/logs \
    2>&1 | tee "$LOG_FILE"

# Capture exit code
EXIT_CODE=${PIPESTATUS[0]}

echo ""
echo "=============================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo "Training completed successfully!"
else
    echo "Training exited with code: $EXIT_CODE"
fi
echo "Log file: $LOG_FILE"
echo "=============================================="

# Stop container
echo ""
echo "Stopping container..."
podman stop "$CONTAINER_NAME"
podman rm "$CONTAINER_NAME"

exit $EXIT_CODE
