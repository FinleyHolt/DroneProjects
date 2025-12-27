#!/bin/bash
# Phase 6c: Diverse Training Worlds - Automated Verification

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "========================================"
echo "Verifying Phase 6c: Diverse Training Worlds"
echo "========================================"
echo ""

PASSED=0
FAILED=0

check() {
    local name="$1"
    local command="$2"

    echo -n "Checking $name... "
    if eval "$command" > /dev/null 2>&1; then
        echo "PASS"
        PASSED=$((PASSED + 1))
        return 0
    else
        echo "FAIL"
        FAILED=$((FAILED + 1))
        return 1
    fi
}

# 1. Check world SDF files exist
echo "--- World SDF Files ---"
check "urban_training.sdf exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/urban_training.sdf"

check "rural_training.sdf exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/rural_training.sdf"

check "industrial_training.sdf exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/industrial_training.sdf"

check "coastal_training.sdf exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/coastal_training.sdf"

check "randomized_training.sdf exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/randomized_training.sdf"

# 2. Check world config YAML files exist
echo ""
echo "--- World Configuration Files ---"
check "world_configs directory exists" \
    "test -d $PROJECT_ROOT/simulation/worlds/world_configs"

check "urban_config.yaml exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/world_configs/urban_config.yaml"

check "rural_config.yaml exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/world_configs/rural_config.yaml"

check "industrial_config.yaml exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/world_configs/industrial_config.yaml"

check "coastal_config.yaml exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/world_configs/coastal_config.yaml"

check "randomized_config.yaml exists" \
    "test -f $PROJECT_ROOT/simulation/worlds/world_configs/randomized_config.yaml"

# 3. Validate SDF files are well-formed XML
echo ""
echo "--- SDF XML Validation ---"
for world in urban rural industrial coastal randomized; do
    sdf_file="$PROJECT_ROOT/simulation/worlds/${world}_training.sdf"
    if [ -f "$sdf_file" ]; then
        echo -n "Checking ${world}_training.sdf is valid XML... "
        if xmllint --noout "$sdf_file" 2>/dev/null; then
            echo "PASS"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL"
            FAILED=$((FAILED + 1))
        fi
    fi
done

# 4. Validate YAML configs parse correctly
echo ""
echo "--- YAML Config Validation ---"
for world in urban rural industrial coastal randomized; do
    config_file="$PROJECT_ROOT/simulation/worlds/world_configs/${world}_config.yaml"
    if [ -f "$config_file" ]; then
        echo -n "Checking ${world}_config.yaml is valid YAML... "
        if python3 -c "import yaml; yaml.safe_load(open('$config_file'))" 2>/dev/null; then
            echo "PASS"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL"
            FAILED=$((FAILED + 1))
        fi
    fi
done

# 5. Check each config has required sections
echo ""
echo "--- Config Content Validation ---"
for world in urban rural industrial coastal randomized; do
    config_file="$PROJECT_ROOT/simulation/worlds/world_configs/${world}_config.yaml"
    if [ -f "$config_file" ]; then
        # Check for NFZ definitions (at least 2)
        echo -n "Checking ${world}_config.yaml has 2+ NFZ regions... "
        nfz_count=$(python3 -c "
import yaml
with open('$config_file') as f:
    cfg = yaml.safe_load(f)
    nfz = cfg.get('no_fly_zones', [])
    print(len(nfz))
" 2>/dev/null)
        if [ "$nfz_count" -ge 2 ] 2>/dev/null; then
            echo "PASS ($nfz_count found)"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL (found $nfz_count, need 2+)"
            FAILED=$((FAILED + 1))
        fi

        # Check for landing zones (at least 3)
        echo -n "Checking ${world}_config.yaml has 3+ landing zones... "
        lz_count=$(python3 -c "
import yaml
with open('$config_file') as f:
    cfg = yaml.safe_load(f)
    lz = cfg.get('landing_zones', [])
    print(len(lz))
" 2>/dev/null)
        if [ "$lz_count" -ge 3 ] 2>/dev/null; then
            echo "PASS ($lz_count found)"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL (found $lz_count, need 3+)"
            FAILED=$((FAILED + 1))
        fi
    fi
done

# 6. Check world_loader.py exists and is valid Python
echo ""
echo "--- WorldLoader Utility ---"
check "world_loader.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/ontology_rl/ontology_rl/utils/world_loader.py"

echo -n "Checking world_loader.py syntax... "
if python3 -c "import ast; ast.parse(open('$PROJECT_ROOT/ros2_ws/src/ontology_rl/ontology_rl/utils/world_loader.py').read())" 2>/dev/null; then
    echo "PASS"
    PASSED=$((PASSED + 1))
else
    echo "FAIL"
    FAILED=$((FAILED + 1))
fi

# 7. Check randomize_world.py script exists
echo ""
echo "--- Domain Randomization Script ---"
check "randomize_world.py exists" \
    "test -f $PROJECT_ROOT/scripts/randomize_world.py"

echo -n "Checking randomize_world.py syntax... "
if python3 -c "import ast; ast.parse(open('$PROJECT_ROOT/scripts/randomize_world.py').read())" 2>/dev/null; then
    echo "PASS"
    PASSED=$((PASSED + 1))
else
    echo "FAIL"
    FAILED=$((FAILED + 1))
fi

# 8. Check SDF files contain required elements
echo ""
echo "--- SDF Content Validation ---"
for world in urban rural industrial coastal randomized; do
    sdf_file="$PROJECT_ROOT/simulation/worlds/${world}_training.sdf"
    if [ -f "$sdf_file" ]; then
        # Check for physics element
        echo -n "Checking ${world}_training.sdf has physics config... "
        if grep -q "<physics" "$sdf_file" 2>/dev/null; then
            echo "PASS"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL"
            FAILED=$((FAILED + 1))
        fi

        # Check for ground plane
        echo -n "Checking ${world}_training.sdf has ground plane... "
        if grep -q "ground_plane\|ground\|terrain" "$sdf_file" 2>/dev/null; then
            echo "PASS"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL"
            FAILED=$((FAILED + 1))
        fi

        # Check for NFZ visual marker
        echo -n "Checking ${world}_training.sdf has NFZ markers... "
        if grep -q "nfz\|no_fly\|restricted" "$sdf_file" 2>/dev/null; then
            echo "PASS"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL"
            FAILED=$((FAILED + 1))
        fi

        # Check for F-11 include
        echo -n "Checking ${world}_training.sdf includes F-11 model... "
        if grep -q "f11_isr\|f11_base" "$sdf_file" 2>/dev/null; then
            echo "PASS"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL"
            FAILED=$((FAILED + 1))
        fi
    fi
done

# Print summary
echo ""
echo "========================================"
echo "Verification Summary"
echo "========================================"
echo "Passed: $PASSED"
echo "Failed: $FAILED"
echo ""

if [ $FAILED -eq 0 ]; then
    echo "Phase 6c verification SUCCESSFUL"
    echo ""
    echo "Next steps:"
    echo "  1. Test world loading in Gazebo simulation"
    echo "  2. Verify NFZ-to-TPTP conversion with Vampire"
    echo "  3. Proceed to Phase 6d: Perception Pipeline"
    exit 0
else
    echo "Phase 6c verification FAILED"
    echo ""
    echo "Review failed checks above and:"
    echo "  1. Create missing world SDF files"
    echo "  2. Create world configuration YAML files"
    echo "  3. Implement WorldLoader utility"
    echo "  4. Create domain randomization script"
    exit 1
fi
