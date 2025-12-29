#!/bin/bash
# Isaac Sim with Pegasus Simulator Extension
# This script launches Isaac Sim with the Pegasus extension folder pre-configured

set -e

SCRIPT_DIR=$(dirname ${BASH_SOURCE})

# Add Pegasus extension folder and launch Isaac Sim
exec /isaac-sim/kit/kit \
    /isaac-sim/apps/omni.isaac.sim.kit \
    --ext-folder /isaac-sim/apps \
    --ext-folder /workspace/PegasusSimulator/extensions \
    --enable pegasus.simulator \
    "$@"
