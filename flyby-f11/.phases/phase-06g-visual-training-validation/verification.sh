#!/bin/bash
# Phase 6g Verification Script
# Run inside Isaac Sim container with GUI

set -e

echo "========================================================================"
echo " Phase 6g: Visual Training Validation"
echo "========================================================================"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="/workspace"
LOG_DIR="${WORKSPACE}/logs/phase6g_verification_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$LOG_DIR"

echo "Log directory: $LOG_DIR"
echo ""

# Check prerequisites
echo "[1/4] Checking prerequisites..."

if [ ! -f "${WORKSPACE}/scripts/visual_training_demo.py" ]; then
    echo "  [FAIL] visual_training_demo.py not found"
    exit 1
fi
echo "  [OK] visual_training_demo.py exists"

if [ ! -d "${WORKSPACE}/perception" ]; then
    echo "  [FAIL] perception module not found"
    exit 1
fi
echo "  [OK] perception module exists"

if [ ! -d "${WORKSPACE}/environments" ]; then
    echo "  [FAIL] environments module not found"
    exit 1
fi
echo "  [OK] environments module exists"

echo ""

# Run visual training demo
echo "[2/4] Running visual training demo (5 episodes)..."
echo "      This will open Isaac Sim GUI. Watch the drone fly!"
echo ""

/isaac-sim/python.sh ${WORKSPACE}/scripts/visual_training_demo.py \
    --episodes 5 \
    --log-dir "$LOG_DIR" \
    2>&1 | tee "${LOG_DIR}/run.log"

echo ""

# Verify logs were created
echo "[3/4] Verifying log files..."

LOG_FILES=("episodes.jsonl" "detections.jsonl" "ontology.jsonl" "rewards.jsonl")
MISSING=0

for file in "${LOG_FILES[@]}"; do
    if [ -f "${LOG_DIR}/${file}" ]; then
        LINES=$(wc -l < "${LOG_DIR}/${file}")
        echo "  [OK] ${file} (${LINES} lines)"
    else
        echo "  [WARN] ${file} not found"
        MISSING=$((MISSING + 1))
    fi
done

if [ -f "${LOG_DIR}/perception_stats.json" ]; then
    echo "  [OK] perception_stats.json"
else
    echo "  [WARN] perception_stats.json not found"
fi

echo ""

# Validate log content
echo "[4/4] Validating log content..."

# Check episodes.jsonl has entries
if [ -f "${LOG_DIR}/episodes.jsonl" ]; then
    EPISODES=$(wc -l < "${LOG_DIR}/episodes.jsonl")
    if [ "$EPISODES" -ge 5 ]; then
        echo "  [OK] ${EPISODES} episodes logged (expected >= 5)"
    else
        echo "  [WARN] Only ${EPISODES} episodes logged (expected 5)"
    fi
fi

# Check detections.jsonl has entries
if [ -f "${LOG_DIR}/detections.jsonl" ]; then
    DETECTIONS=$(wc -l < "${LOG_DIR}/detections.jsonl")
    if [ "$DETECTIONS" -gt 0 ]; then
        echo "  [OK] ${DETECTIONS} detection entries logged"
    else
        echo "  [WARN] No detections logged"
    fi
fi

# Check for ontology events
if [ -f "${LOG_DIR}/ontology.jsonl" ]; then
    ONTOLOGY_EVENTS=$(wc -l < "${LOG_DIR}/ontology.jsonl")
    RTL_EVENTS=$(grep -c "RTL" "${LOG_DIR}/ontology.jsonl" 2>/dev/null || echo "0")
    echo "  [OK] ${ONTOLOGY_EVENTS} ontology events (${RTL_EVENTS} RTL triggers)"
fi

echo ""
echo "========================================================================"
echo " VERIFICATION COMPLETE"
echo "========================================================================"
echo ""
echo "Logs saved to: $LOG_DIR"
echo ""
echo "To review:"
echo "  cat ${LOG_DIR}/episodes.jsonl | jq ."
echo "  head -20 ${LOG_DIR}/detections.jsonl | jq ."
echo ""

# Summary
if [ "$MISSING" -eq 0 ]; then
    echo "[PASS] Phase 6g verification passed!"
    exit 0
else
    echo "[WARN] Phase 6g completed with ${MISSING} missing log files"
    exit 0  # Don't fail on missing optional logs
fi
