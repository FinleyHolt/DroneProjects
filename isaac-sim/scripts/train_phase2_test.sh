#!/bin/bash
# Phase 2 test wrapper - runs the procgen test with GUI
set -e

echo "============================================"
echo "Phase 2: Procedural World Test"
echo "============================================"

# Run the phase 2 test script
/isaac-sim/python.sh /workspace/scripts/phase2_procgen_test.py
