#!/bin/bash
# Phase 6d: Perception Pipeline - Automated Verification

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "========================================"
echo "Verifying Phase 6d: Perception Pipeline"
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

# 1. Check perception package exists
echo "--- Package Structure ---"
check "flyby_f11_perception package.xml exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/package.xml"

check "setup.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/setup.py"

# 2. Check core Python modules exist
echo ""
echo "--- Core Nodes ---"
check "object_detector_node.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/flyby_f11_perception/object_detector_node.py"

check "tptp_fact_generator.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/flyby_f11_perception/tptp_fact_generator.py"

check "perception_bridge_node.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/flyby_f11_perception/perception_bridge_node.py"

# 3. Check Python syntax
echo ""
echo "--- Python Syntax Validation ---"
for node in object_detector_node tptp_fact_generator perception_bridge_node; do
    py_file="$PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/flyby_f11_perception/${node}.py"
    if [ -f "$py_file" ]; then
        echo -n "Checking ${node}.py syntax... "
        if python3 -c "import ast; ast.parse(open('$py_file').read())" 2>/dev/null; then
            echo "PASS"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL"
            FAILED=$((FAILED + 1))
        fi
    fi
done

# 4. Check configuration files
echo ""
echo "--- Configuration Files ---"
check "config directory exists" \
    "test -d $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/config"

check "detection_classes.yaml exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/config/detection_classes.yaml"

check "tptp_templates.yaml exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/config/tptp_templates.yaml"

# 5. Validate YAML configs
echo ""
echo "--- YAML Config Validation ---"
for config in detection_classes tptp_templates; do
    config_file="$PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/config/${config}.yaml"
    if [ -f "$config_file" ]; then
        echo -n "Checking ${config}.yaml is valid YAML... "
        if python3 -c "import yaml; yaml.safe_load(open('$config_file'))" 2>/dev/null; then
            echo "PASS"
            PASSED=$((PASSED + 1))
        else
            echo "FAIL"
            FAILED=$((FAILED + 1))
        fi
    fi
done

# 6. Check message definitions
echo ""
echo "--- Message Definitions ---"
check "msg directory exists" \
    "test -d $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/msg"

check "Detection.msg exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/msg/Detection.msg"

check "DetectionArray.msg exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/msg/DetectionArray.msg"

# 7. Check launch file
echo ""
echo "--- Launch Files ---"
check "launch directory exists" \
    "test -d $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/launch"

check "perception.launch.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/launch/perception.launch.py"

echo -n "Checking perception.launch.py syntax... "
if python3 -c "import ast; ast.parse(open('$PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/launch/perception.launch.py').read())" 2>/dev/null; then
    echo "PASS"
    PASSED=$((PASSED + 1))
else
    echo "FAIL"
    FAILED=$((FAILED + 1))
fi

# 8. Check detection config has ontology mappings
echo ""
echo "--- Ontology Mapping Validation ---"
echo -n "Checking detection classes have ontology mappings... "
config_file="$PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/config/detection_classes.yaml"
if [ -f "$config_file" ]; then
    has_mappings=$(python3 -c "
import yaml
with open('$config_file') as f:
    cfg = yaml.safe_load(f)
    classes = cfg.get('detection_classes', {})
    for cid, cinfo in classes.items():
        if 'ontology_class' not in cinfo:
            exit(1)
    exit(0)
" 2>/dev/null && echo "yes" || echo "no")
    if [ "$has_mappings" = "yes" ]; then
        echo "PASS"
        PASSED=$((PASSED + 1))
    else
        echo "FAIL (missing ontology_class mappings)"
        FAILED=$((FAILED + 1))
    fi
else
    echo "FAIL (config file not found)"
    FAILED=$((FAILED + 1))
fi

# 9. Check TPTP templates have required fact types
echo -n "Checking TPTP templates include required facts... "
template_file="$PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/config/tptp_templates.yaml"
if [ -f "$template_file" ]; then
    has_templates=$(python3 -c "
import yaml
with open('$template_file') as f:
    cfg = yaml.safe_load(f)
    templates = cfg.get('fact_templates', {})
    required = ['observed_object', 'distance_to']
    for req in required:
        if req not in templates:
            exit(1)
    exit(0)
" 2>/dev/null && echo "yes" || echo "no")
    if [ "$has_templates" = "yes" ]; then
        echo "PASS"
        PASSED=$((PASSED + 1))
    else
        echo "FAIL (missing required templates)"
        FAILED=$((FAILED + 1))
    fi
else
    echo "FAIL (template file not found)"
    FAILED=$((FAILED + 1))
fi

# 10. Check TPTP fact format is valid
echo ""
echo "--- TPTP Syntax Validation ---"
echo -n "Checking TPTP fact format... "
# Test that template produces valid-looking TPTP
valid_format=$(python3 -c "
import re
# Sample fact that should be generated
sample = 'fof(obs_1, axiom, observedObject(building_1, StaticObstacle, position(10.5, 20.3, 0.0), confidence(0.85), time(12345.67))).'
# Basic TPTP pattern: fof(name, role, formula).
pattern = r'^fof\([a-z_0-9]+, [a-z]+, .+\)\.$'
if re.match(pattern, sample, re.IGNORECASE):
    print('valid')
else:
    print('invalid')
" 2>/dev/null)
if [ "$valid_format" = "valid" ]; then
    echo "PASS"
    PASSED=$((PASSED + 1))
else
    echo "FAIL"
    FAILED=$((FAILED + 1))
fi

# 11. Check for test files
echo ""
echo "--- Test Files ---"
check "test directory exists" \
    "test -d $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/test"

check "test_tptp_generator.py exists" \
    "test -f $PROJECT_ROOT/ros2_ws/src/flyby_f11_perception/test/test_tptp_generator.py"

# Print summary
echo ""
echo "========================================"
echo "Verification Summary"
echo "========================================"
echo "Passed: $PASSED"
echo "Failed: $FAILED"
echo ""

if [ $FAILED -eq 0 ]; then
    echo "Phase 6d verification SUCCESSFUL"
    echo ""
    echo "Next steps:"
    echo "  1. Build package: colcon build --packages-select flyby_f11_perception"
    echo "  2. Test with simulation: ros2 launch flyby_f11_perception perception.launch.py"
    echo "  3. Verify TPTP facts: ros2 topic echo /perception/tptp_facts"
    echo "  4. Proceed to Phase 6b: Simulation & Training Environment"
    exit 0
else
    echo "Phase 6d verification FAILED"
    echo ""
    echo "Review failed checks above and:"
    echo "  1. Create flyby_f11_perception package"
    echo "  2. Implement detector and TPTP generator nodes"
    echo "  3. Create configuration files with ontology mappings"
    echo "  4. Define message types"
    exit 1
fi
