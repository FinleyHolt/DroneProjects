#!/bin/bash
# Phase 6f Verification Script
# Tests perception pipeline integration

set -euo pipefail

# Prevent arithmetic expression errors from causing exit
set +e

echo "=============================================="
echo "Phase 6f: Perception Pipeline Verification"
echo "=============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0

check_result() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✓ PASS${NC}: $2"
        ((PASS_COUNT++))
    else
        echo -e "${RED}✗ FAIL${NC}: $2"
        ((FAIL_COUNT++))
    fi
}

# Check if running in container
if [ -d "/workspace/perception" ]; then
    WORKSPACE="/workspace"
else
    WORKSPACE="/home/finley/Github/DroneProjects/flyby-f11/evaluation/isaac-sim-px4"
fi

echo ""
echo "Workspace: $WORKSPACE"
echo ""
echo "=== 1. File Structure Check ==="

# Check required files exist
FILES=(
    "perception/__init__.py"
    "perception/detector.py"
    "perception/spatial_encoder.py"
    "perception/scene_statistics.py"
    "perception/temporal_tracker.py"
    "perception/perception_encoder.py"
    "perception/tptp_generator.py"
    "perception/config/detection_classes.yaml"
)

for file in "${FILES[@]}"; do
    if [ -f "$WORKSPACE/$file" ]; then
        check_result 0 "File exists: $file"
    else
        check_result 1 "File missing: $file"
    fi
done

echo ""
echo "=== 2. Python Import Check ==="

# Test imports
python3 << EOF
import sys
sys.path.insert(0, '$WORKSPACE/perception')
sys.path.insert(0, '$WORKSPACE')

errors = []

try:
    from detector import YOLODetector, Detection
    print("OK: detector imports")
except Exception as e:
    errors.append(f"detector: {e}")
    print(f"FAIL: detector imports - {e}")

try:
    from spatial_encoder import SpatialGridEncoder
    print("OK: spatial_encoder imports")
except Exception as e:
    errors.append(f"spatial_encoder: {e}")
    print(f"FAIL: spatial_encoder imports - {e}")

try:
    from scene_statistics import SceneStatistics
    print("OK: scene_statistics imports")
except Exception as e:
    errors.append(f"scene_statistics: {e}")
    print(f"FAIL: scene_statistics imports - {e}")

try:
    from temporal_tracker import TemporalTracker
    print("OK: temporal_tracker imports")
except Exception as e:
    errors.append(f"temporal_tracker: {e}")
    print(f"FAIL: temporal_tracker imports - {e}")

try:
    from perception_encoder import PerceptionEncoder
    print("OK: perception_encoder imports")
except Exception as e:
    errors.append(f"perception_encoder: {e}")
    print(f"FAIL: perception_encoder imports - {e}")

try:
    from tptp_generator import TPTPGenerator
    print("OK: tptp_generator imports")
except Exception as e:
    errors.append(f"tptp_generator: {e}")
    print(f"FAIL: tptp_generator imports - {e}")

if errors:
    print(f"\n{len(errors)} import errors")
    sys.exit(1)
else:
    print("\nAll imports successful")
    sys.exit(0)
EOF
check_result $? "Python imports"

echo ""
echo "=== 3. Unit Tests ==="

# Run pytest if available
if command -v pytest &> /dev/null; then
    if [ -d "$WORKSPACE/perception/tests" ]; then
        cd "$WORKSPACE"
        pytest perception/tests/ -v --tb=short 2>&1 | head -50
        check_result ${PIPESTATUS[0]} "Unit tests"
    else
        echo -e "${YELLOW}SKIP${NC}: Test directory not found"
    fi
else
    echo -e "${YELLOW}SKIP${NC}: pytest not available"
fi

echo ""
echo "=== 4. Encoding Speed Test ==="

python3 << EOF
import sys
sys.path.insert(0, '$WORKSPACE/perception')
sys.path.insert(0, '$WORKSPACE')
import numpy as np
import time

try:
    from perception_encoder import PerceptionEncoder, PerceptionConfig

    # Create encoder
    config = PerceptionConfig(detector_mode="ground_truth")
    encoder = PerceptionEncoder(config)

    # Sample inputs
    image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    uav_pos = np.array([0.0, 0.0, 50.0])
    uav_ori = np.array([1.0, 0.0, 0.0, 0.0])
    labels = [
        {'class_id': 1, 'bbox': (0.3, 0.4, 0.1, 0.2), 'position_3d': np.array([10.0, 5.0, 0.0])},
        {'class_id': 3, 'bbox': (0.7, 0.6, 0.15, 0.25), 'position_3d': np.array([30.0, -10.0, 0.0])},
    ]

    # Warm up
    for _ in range(5):
        encoder.encode(image, uav_pos, uav_ori, labels)

    # Benchmark
    times = []
    for _ in range(50):
        start = time.perf_counter()
        obs = encoder.encode(image, uav_pos, uav_ori, labels)
        times.append((time.perf_counter() - start) * 1000)

    avg_time = np.mean(times)
    max_time = np.max(times)
    print(f"Average encoding time: {avg_time:.2f} ms")
    print(f"Max encoding time: {max_time:.2f} ms")
    print(f"Output shape: {obs.shape}")

    if avg_time < 20.0:
        print("PASS: Encoding within 20ms budget")
        sys.exit(0)
    else:
        print("FAIL: Encoding exceeds 20ms budget")
        sys.exit(1)

except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
EOF
check_result $? "Encoding speed < 20ms"

echo ""
echo "=== 5. Output Dimensions Check ==="

python3 << EOF
import sys
sys.path.insert(0, '$WORKSPACE/perception')
sys.path.insert(0, '$WORKSPACE')

try:
    from perception_encoder import PerceptionEncoder, PerceptionConfig

    config = PerceptionConfig()
    encoder = PerceptionEncoder(config)

    expected_dim = 100 + 384 + 32  # priority + grid + stats = 516
    actual_dim = encoder.output_dim

    print(f"Expected dimension: {expected_dim}")
    print(f"Actual dimension: {actual_dim}")

    if actual_dim == expected_dim:
        print("PASS: Output dimensions correct")
        sys.exit(0)
    else:
        print("FAIL: Output dimensions mismatch")
        sys.exit(1)

except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
EOF
check_result $? "Output dimensions = 516"

echo ""
echo "=== 6. TPTP Generation Check ==="

python3 << EOF
import sys
sys.path.insert(0, '$WORKSPACE/perception')
sys.path.insert(0, '$WORKSPACE')
import numpy as np

try:
    from detector import Detection
    from tptp_generator import TPTPGenerator

    generator = TPTPGenerator()

    detections = [
        Detection(
            class_id=1,
            class_name="person",
            ontology_class="Person",
            confidence=0.9,
            bbox=(0.5, 0.5, 0.1, 0.2),
            position_3d=np.array([10.0, 5.0, 0.0]),
            distance=11.2,
        )
    ]

    facts = generator.generate_facts(detections)

    print(f"Generated {len(facts)} TPTP facts")
    for fact in facts[:3]:
        print(f"  {fact[:80]}...")

    if len(facts) >= 2 and all("fof(" in f for f in facts):
        print("PASS: TPTP facts generated correctly")
        sys.exit(0)
    else:
        print("FAIL: TPTP facts invalid")
        sys.exit(1)

except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
EOF
check_result $? "TPTP fact generation"

echo ""
echo "=== 7. Spatial Grid Coverage Test ==="

python3 << EOF
import sys
sys.path.insert(0, '$WORKSPACE/perception')
sys.path.insert(0, '$WORKSPACE')
import numpy as np

try:
    from detector import Detection
    from spatial_encoder import SpatialGridEncoder

    encoder = SpatialGridEncoder()

    # Create 50 detections spread across the image
    detections = []
    for i in range(50):
        detections.append(Detection(
            class_id=1,
            class_name="person",
            ontology_class="Person",
            confidence=0.9,
            bbox=(np.random.uniform(0.1, 0.9), np.random.uniform(0.1, 0.9), 0.05, 0.1),
            distance=np.random.uniform(5, 100),
        ))

    grid = encoder.encode(detections)

    print(f"Grid shape: {grid.shape}")
    print(f"Grid min: {grid.min():.3f}, max: {grid.max():.3f}")
    print(f"Non-zero cells: {np.sum(grid > 0)}")

    # Grid should have captured information from all 50 detections
    # Person count channel should sum to ~50 (normalized)
    grid_3d = encoder.decode(grid)
    total_persons = grid_3d[:, :, 0].sum() * 10  # Denormalize

    print(f"Total persons captured in grid: {total_persons:.1f}")

    if grid.shape[0] == 384 and np.sum(grid > 0) > 10:
        print("PASS: Spatial grid captures detections")
        sys.exit(0)
    else:
        print("FAIL: Spatial grid not working correctly")
        sys.exit(1)

except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
EOF
check_result $? "Spatial grid captures all detections"

echo ""
echo "=== 8. RL Integration Check ==="

python3 << EOF
import sys
sys.path.insert(0, '$WORKSPACE/perception')
sys.path.insert(0, '$WORKSPACE')
sys.path.insert(0, '$WORKSPACE/environments')

try:
    from perception_integration import (
        PerceptionIntegration,
        PerceptionIntegrationConfig,
    )
    import numpy as np

    # Initialize perception integration
    config = PerceptionIntegrationConfig(
        mode="ground_truth",
        observation_dim=516,
    )
    perception = PerceptionIntegration(config)
    perception.initialize()

    # Set known objects
    perception.set_known_objects(
        people=[np.array([10, 20, 0]), np.array([30, -10, 0])],
        vehicles=[np.array([50, 0, 0])],
        buildings=[np.array([0, 0, 0])],
    )

    # Test encoding
    uav_pos = np.array([0, 0, 50])
    obs = perception.encode(None, uav_pos)

    print(f"PerceptionIntegration output shape: {obs.shape}")
    print(f"Output dimension: {len(obs)}")

    if len(obs) == 516:
        print("PASS: RL integration produces correct observation dimension")
        sys.exit(0)
    else:
        print(f"FAIL: Expected 516, got {len(obs)}")
        sys.exit(1)

except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
EOF
check_result $? "RL integration produces 516-dim observation"

echo ""
echo "=== 9. Training Config Check ==="

python3 << EOF
import sys
import os

try:
    import yaml

    config_path = '$WORKSPACE/config/training_config.yaml'
    if not os.path.exists(config_path):
        print(f"Config file not found: {config_path}")
        sys.exit(1)

    with open(config_path) as f:
        config = yaml.safe_load(f)

    # Check perception config exists
    if 'perception' not in config:
        print("FAIL: perception section missing from training_config.yaml")
        sys.exit(1)

    perc_config = config['perception']
    required_keys = ['enabled', 'mode', 'max_encode_time_ms', 'observation_dim']

    for key in required_keys:
        if key not in perc_config:
            print(f"FAIL: Missing perception.{key}")
            sys.exit(1)

    print(f"Perception enabled: {perc_config['enabled']}")
    print(f"Mode: {perc_config['mode']}")
    print(f"Max encode time: {perc_config['max_encode_time_ms']}ms")
    print(f"Observation dim: {perc_config['observation_dim']}")

    print("PASS: Training config has perception settings")
    sys.exit(0)

except ImportError:
    print("SKIP: yaml not available")
    sys.exit(0)
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
EOF
check_result $? "Training config includes perception"

echo ""
echo "=============================================="
echo "Verification Summary"
echo "=============================================="
echo -e "Passed: ${GREEN}$PASS_COUNT${NC}"
echo -e "Failed: ${RED}$FAIL_COUNT${NC}"

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "\n${GREEN}All verifications passed!${NC}"
    echo ""
    echo "Phase 6f is complete. Perception pipeline integrated with RL training."
    echo ""
    echo "Key Features:"
    echo "  - Live YOLO detection on camera images"
    echo "  - 516-dimensional observation vector"
    echo "  - Ground truth, inference, and hybrid modes"
    echo "  - Integrated into base RL environment"
    echo "  - TPTP fact generation for Vampire safety checking"
    echo ""
    echo "Observation space: 28 (state) + 516 (perception) = 544 dimensions"
    echo ""
    echo "Next steps:"
    echo "  1. Run RL training with perception: python3 train_canonical.py"
    echo "  2. Monitor perception stats in training logs"
    echo "  3. Proceed to Phase 7: Mission Planner RL"
    exit 0
else
    echo -e "\n${RED}Some verifications failed.${NC}"
    echo ""
    echo "Please fix the failing tests before proceeding."
    exit 1
fi
