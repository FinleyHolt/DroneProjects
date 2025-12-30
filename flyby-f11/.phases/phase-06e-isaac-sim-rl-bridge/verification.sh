#!/bin/bash
# Phase 6e Verification Script
# Isaac Sim RL Training Bridge

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=============================================="
echo "Phase 6e: Isaac Sim RL Bridge Verification"
echo "=============================================="
echo ""

PASS=0
FAIL=0

check_file() {
    local file="$1"
    local desc="$2"
    if [ -f "$PROJECT_ROOT/$file" ]; then
        echo "[PASS] $desc"
        ((PASS++))
    else
        echo "[FAIL] $desc - File not found: $file"
        ((FAIL++))
    fi
}

check_pattern() {
    local file="$1"
    local pattern="$2"
    local desc="$3"
    if [ -f "$PROJECT_ROOT/$file" ] && grep -q "$pattern" "$PROJECT_ROOT/$file"; then
        echo "[PASS] $desc"
        ((PASS++))
    else
        echo "[FAIL] $desc - Pattern '$pattern' not found in $file"
        ((FAIL++))
    fi
}

echo "1. Checking required files..."
echo "--------------------------------------------"

check_file "evaluation/isaac-sim-px4/environments/gymnasium_wrapper.py" \
    "Gymnasium wrapper exists"

check_file "evaluation/isaac-sim-px4/environments/action_bridge.py" \
    "Action bridge exists"

check_file "evaluation/isaac-sim-px4/environments/safety_filter.py" \
    "Safety filter exists"

check_file "evaluation/isaac-sim-px4/scripts/training/agents/sac_agent.py" \
    "SAC agent module exists"

check_file "evaluation/isaac-sim-px4/scripts/training/agents/__init__.py" \
    "Agents package init exists"

check_file "evaluation/isaac-sim-px4/scripts/training/test_training_loop.py" \
    "Training integration test exists"

echo ""
echo "2. Checking code patterns..."
echo "--------------------------------------------"

check_pattern "evaluation/isaac-sim-px4/environments/gymnasium_wrapper.py" \
    "gym.Env" "Gymnasium wrapper inherits gym.Env"

check_pattern "evaluation/isaac-sim-px4/environments/gymnasium_wrapper.py" \
    "observation_space" "Gymnasium wrapper defines observation_space"

check_pattern "evaluation/isaac-sim-px4/environments/gymnasium_wrapper.py" \
    "action_space" "Gymnasium wrapper defines action_space"

check_pattern "evaluation/isaac-sim-px4/environments/action_bridge.py" \
    "mavlink" "Action bridge uses MAVLink"

check_pattern "evaluation/isaac-sim-px4/environments/safety_filter.py" \
    "vampire" "Safety filter integrates Vampire"

check_pattern "evaluation/isaac-sim-px4/scripts/training/agents/sac_agent.py" \
    "stable_baselines3" "SAC agent uses SB3"

check_pattern "evaluation/isaac-sim-px4/scripts/training/train_canonical.py" \
    "stable_baselines3\|create_sac_agent\|create_ppo_agent" \
    "Training script uses real agents"

check_pattern "evaluation/isaac-sim-px4/Containerfile.sitl" \
    "stable-baselines3" "Container includes SB3"

echo ""
echo "3. Checking Python syntax..."
echo "--------------------------------------------"

for pyfile in \
    "evaluation/isaac-sim-px4/environments/gymnasium_wrapper.py" \
    "evaluation/isaac-sim-px4/environments/action_bridge.py" \
    "evaluation/isaac-sim-px4/environments/safety_filter.py" \
    "evaluation/isaac-sim-px4/scripts/training/agents/sac_agent.py" \
    "evaluation/isaac-sim-px4/scripts/training/test_training_loop.py"
do
    if [ -f "$PROJECT_ROOT/$pyfile" ]; then
        if python3 -m py_compile "$PROJECT_ROOT/$pyfile" 2>/dev/null; then
            echo "[PASS] Syntax OK: $(basename $pyfile)"
            ((PASS++))
        else
            echo "[FAIL] Syntax error: $pyfile"
            ((FAIL++))
        fi
    fi
done

echo ""
echo "=============================================="
echo "Verification Summary"
echo "=============================================="
echo "Passed: $PASS"
echo "Failed: $FAIL"
echo ""

if [ $FAIL -eq 0 ]; then
    echo "Phase 6e verification PASSED"
    echo ""
    echo "Next steps:"
    echo "  1. Build container: podman build -f evaluation/isaac-sim-px4/Containerfile.sitl -t isaac-sim-px4:latest ."
    echo "  2. Run integration test: podman exec <container> /isaac-sim/python.sh /workspace/scripts/training/test_training_loop.py"
    echo "  3. Start training: podman exec <container> /isaac-sim/python.sh /workspace/scripts/training/train_canonical.py --problem comms_denied"
    exit 0
else
    echo "Phase 6e verification FAILED"
    echo "Please implement missing components before proceeding to Phase 7"
    exit 1
fi
