#!/bin/bash
# Scenario Training Runner
#
# Runs scenario-based training inside the Isaac Sim container.
#
# Usage:
#   ./scripts/run_scenario_training.sh area_coverage       # Area coverage scenario
#   ./scripts/run_scenario_training.sh nfz_avoidance       # NFZ avoidance scenario
#   ./scripts/run_scenario_training.sh multi_objective     # Multi-objective scenario
#   ./scripts/run_scenario_training.sh area_coverage --num_envs 256
#
# Output:
#   Training logs to stdout
#   Checkpoints to checkpoints/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Default parameters
SCENARIO="${1:-area_coverage}"
shift || true

NUM_ENVS=""
EXTRA_ARGS=""

# Parse additional arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --num_envs|--num-envs)
            NUM_ENVS="--num_envs $2"
            shift 2
            ;;
        --help)
            echo "Scenario Training Runner"
            echo ""
            echo "Usage: $0 SCENARIO [OPTIONS]"
            echo ""
            echo "Scenarios:"
            echo "  area_coverage    - Survey assigned area to achieve coverage target"
            echo "  nfz_avoidance    - Transit to destination avoiding pop-up NFZs"
            echo "  multi_objective  - Maximize target value while managing threat exposure"
            echo ""
            echo "Options:"
            echo "  --num_envs N     Number of parallel environments (overrides config)"
            echo "  --device DEVICE  Device for training (default: cuda:0)"
            echo "  --trainer TYPE   Training library: dummy, rl_games, sb3 (default: dummy)"
            echo "  --checkpoint P   Path to checkpoint to resume from"
            echo "  --help           Show this help message"
            echo ""
            exit 0
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift
            ;;
    esac
done

echo "========================================"
echo "Scenario Training: $SCENARIO"
echo "========================================"
echo "Workspace: $WORKSPACE_DIR"
echo "Extra args: $NUM_ENVS $EXTRA_ARGS"
echo ""

# Build the command
CMD="/workspace/training/scripts/train_scenario.py --scenario $SCENARIO $NUM_ENVS $EXTRA_ARGS"

echo "Running: $CMD"
echo ""

podman run --rm --replace --name flyby-f11-scenario-training \
  --device nvidia.com/gpu=all \
  -v "$WORKSPACE_DIR:/workspace:Z" \
  -e ACCEPT_EULA=Y \
  -e PYTHONUNBUFFERED=1 \
  --network=host \
  --entrypoint /isaac-sim/python.sh \
  localhost/isaac-sim-px4:5.1.0-px4-1.14.3 \
  $CMD
