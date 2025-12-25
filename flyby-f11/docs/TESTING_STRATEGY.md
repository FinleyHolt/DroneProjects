---
title: "Flyby-F11 Testing Strategy"
author: "Finley Holt"
date: 2025-12-25
format:
  pdf:
    documentclass: article
    geometry:
      - margin=1in
    fontsize: 11pt
    number-sections: true
    toc: true
    include-in-header:
      text: |
        \usepackage{fancyhdr}
        \pagestyle{fancy}
        \fancyhf{}
        \fancyhead[L]{\textit{Testing Strategy}}
        \fancyhead[R]{\thepage}
        \renewcommand{\headrulewidth}{0.4pt}
---

# Testing Strategy

This document defines the comprehensive testing strategy for the flyby-f11 ontology-constrained RL autonomy system, covering unit testing, integration testing, simulation testing, hardware-in-the-loop (HIL) testing, and verification/validation approaches.

## Testing Pyramid

The flyby-f11 testing strategy follows a pyramid structure prioritizing fast, isolated tests at the base and slower, integrated tests at the top:

```
           /\
          /  \    Hardware Flight Tests (10s of tests)
         /    \
        /      \  Hardware-in-the-Loop (100s of tests)
       /        \
      /          \ Simulation Tests (1000s of scenarios)
     /            \
    /              \ Integration Tests (100s of tests)
   /                \
  /                  \ Unit Tests (1000s of tests)
 /____________________\
```

**Testing Philosophy**:
- Write tests before/during implementation (TDD encouraged)
- Every bug gets a regression test
- CI/CD runs all tests on every commit
- 80%+ code coverage for critical components
- 100% coverage for safety-critical ontology rules

## Unit Testing

### Python Unit Tests (pytest)

**Test Structure**:
```
ros2_ws/src/<package>/
├── <package>/
│   ├── __init__.py
│   └── module.py
└── test/
    ├── test_module.py
    └── test_integration.py
```

**Example: Test Prolog Grounding Node**:

```python
# test/test_object_grounding.py
import pytest
from perception_grounding.object_grounding import ObjectGroundingNode
from geometry_msgs.msg import Point

class TestObjectGrounding:
    @pytest.fixture
    def node(self):
        """Create node instance for testing."""
        import rclpy
        rclpy.init()
        node = ObjectGroundingNode()
        yield node
        node.destroy_node()
        rclpy.shutdown()

    def test_bbox_to_prolog_fact(self, node):
        """Test bounding box conversion to Prolog fact."""
        bbox = {
            'class': 'person',
            'confidence': 0.92,
            'xmin': 100, 'ymin': 150,
            'xmax': 200, 'ymax': 300
        }
        fact = node.bbox_to_prolog_fact(bbox, obj_id=123)

        assert "objectType(obj_123, 'person')" in fact
        assert "confidence(obj_123, 0.92)" in fact

    def test_spatial_relation_computation(self, node):
        """Test spatial relation derivation from positions."""
        drone_pos = Point(x=0.0, y=0.0, z=-5.0)
        obj_pos = Point(x=10.0, y=0.0, z=-5.0)

        relation = node.compute_spatial_relation(drone_pos, obj_pos)

        assert relation['direction'] == 'north'
        assert pytest.approx(relation['distance'], 0.1) == 10.0

    def test_safety_critical_object_detection(self, node):
        """Test that high-importance objects trigger safety checks."""
        # Simulate person detection at close range
        detection = {
            'class': 'person',
            'confidence': 0.95,
            'position': Point(x=3.0, y=0.0, z=-5.0)  # 3m ahead
        }

        safety_flag = node.check_safety_importance(detection)
        assert safety_flag is True  # Should trigger safety check

    def test_prolog_assertion(self, node, mocker):
        """Test that Prolog facts are asserted correctly."""
        mock_prolog = mocker.patch.object(node, 'prolog')

        fact = "objectType(obj_123, 'person')"
        node.assert_fact(fact)

        mock_prolog.assertz.assert_called_once_with(fact)
```

**Run pytest**:
```bash
cd ~/Github/DroneProjects/flyby-f11/ros2_ws

# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select perception_grounding

# Run with coverage
colcon test --packages-select perception_grounding --pytest-args --cov

# View results
colcon test-result --verbose
```

**Test Coverage**:
```bash
# Install coverage tools
pip install pytest-cov

# Run with coverage report
pytest test/ --cov=perception_grounding --cov-report=html

# View HTML report
firefox htmlcov/index.html
```

### C++ Unit Tests (gtest)

**Test Structure**:
```
ros2_ws/src/<package>/
├── src/
│   └── reasoner_node.cpp
└── test/
    └── test_reasoner.cpp
```

**Example: Test Ontology Reasoner**:

```cpp
// test/test_reasoner.cpp
#include <gtest/gtest.h>
#include "ontology_interface/prolog_reasoner.hpp"

class PrologReasonerTest : public ::testing::Test {
protected:
  void SetUp() override {
    reasoner_ = std::make_shared<PrologReasoner>();
    reasoner_->load_rules("test_rules.pl");
  }

  std::shared_ptr<PrologReasoner> reasoner_;
};

TEST_F(PrologReasonerTest, CanExecuteSafeAction) {
  // Test that safe actions are allowed
  bool result = reasoner_->query("canExecute(moveForward)");
  EXPECT_TRUE(result);
}

TEST_F(PrologReasonerTest, BlocksUnsafeAction) {
  // Simulate obstacle ahead
  reasoner_->assert_fact("distance(drone, obstacle_1, 2.0)");

  bool result = reasoner_->query("canExecute(moveForward)");
  EXPECT_FALSE(result);  // Should block due to close obstacle
}

TEST_F(PrologReasonerTest, MustAvoidHighImportanceObject) {
  // Simulate person detected
  reasoner_->assert_fact("objectType(obj_123, person)");
  reasoner_->assert_fact("distance(drone, obj_123, 8.0)");

  std::vector<std::string> results;
  reasoner_->query_all("mustAvoid(X)", results);

  EXPECT_EQ(results.size(), 1);
  EXPECT_EQ(results[0], "obj_123");
}

TEST_F(PrologReasonerTest, SpatialReasoningBetween) {
  // Test ternary spatial relation
  reasoner_->assert_fact("position(drone, [0, 0, -5])");
  reasoner_->assert_fact("position(waypoint_a, [-10, 0, -5])");
  reasoner_->assert_fact("position(waypoint_b, [10, 0, -5])");

  bool result = reasoner_->query("between(drone, waypoint_a, waypoint_b)");
  EXPECT_TRUE(result);
}

TEST_F(PrologReasonerTest, QueryLatencyBenchmark) {
  // Ensure query latency < 10ms for real-time performance
  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < 100; ++i) {
    reasoner_->query("canExecute(moveForward)");
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  double avg_latency_ms = duration.count() / 100.0 / 1000.0;

  EXPECT_LT(avg_latency_ms, 10.0);  // <10ms average
}
```

**CMakeLists.txt configuration**:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_reasoner test/test_reasoner.cpp)
  target_link_libraries(test_reasoner ${PROJECT_NAME})

  ament_add_gtest(test_safety_constraints test/test_safety_constraints.cpp)
  target_link_libraries(test_safety_constraints ${PROJECT_NAME})
endif()
```

**Run gtest**:
```bash
# Build with tests
colcon build --packages-select ontology_interface --cmake-args -DBUILD_TESTING=ON

# Run tests
colcon test --packages-select ontology_interface

# View detailed results
colcon test-result --verbose
cat build/ontology_interface/test_results/ontology_interface/test_reasoner.gtest.xml
```

## Integration Testing

### ROS 2 Launch Tests

**Test launch files with multiple nodes**:

```python
# test/test_perception_pipeline_integration.py
import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import pytest
import rclpy
from std_msgs.msg import String

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch nodes for integration test."""
    return LaunchDescription([
        # Camera simulator
        Node(
            package='perception_grounding',
            executable='camera_simulator',
            name='camera_sim'
        ),
        # YOLO detection node
        Node(
            package='perception_pipeline',
            executable='yolo_detector',
            name='yolo'
        ),
        # Object grounding node
        Node(
            package='perception_grounding',
            executable='object_grounding_node',
            name='object_grounding'
        ),
        ReadyToTest()
    ])

class TestPerceptionPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_end_to_end_detection_to_prolog(self, proc_output):
        """Test camera → YOLO → grounding → Prolog pipeline."""
        node = rclpy.create_node('test_node')

        # Subscribe to Prolog facts topic
        received_facts = []
        def callback(msg):
            received_facts.append(msg.data)

        subscription = node.create_subscription(
            String,
            '/ontology/grounded_objects',
            callback,
            10
        )

        # Wait for messages
        rclpy.spin_once(node, timeout_sec=5.0)

        # Verify Prolog facts received
        assert len(received_facts) > 0
        assert any("objectType" in fact for fact in received_facts)

        node.destroy_node()
```

**Run launch tests**:
```bash
colcon test --packages-select perception_grounding --pytest-args -v
```

### Multi-Node Integration Tests

**Test ontology-RL interaction**:

```python
# test/test_ontology_rl_integration.py
import rclpy
from rclpy.node import Node
from ontology_interface.srv import QuerySafety
from ontology_rl.srv import GetAction

class TestOntologyRLIntegration:
    def test_rl_action_filtered_by_ontology(self):
        """Test that RL actions are constrained by ontology."""
        node = rclpy.create_node('test_node')

        # Call RL agent for action
        rl_client = node.create_client(GetAction, '/rl/get_action')
        rl_request = GetAction.Request()
        rl_request.state = [0.0, 0.0, -5.0, 0.0, 0.0]  # State vector

        rl_future = rl_client.call_async(rl_request)
        rclpy.spin_until_future_complete(node, rl_future)
        rl_action = rl_future.result().action

        # Query ontology for safety
        ont_client = node.create_client(QuerySafety, '/ontology/query_safety')
        ont_request = QuerySafety.Request()
        ont_request.action_name = rl_action.name
        ont_request.target_pose = rl_action.target_pose

        ont_future = ont_client.call_async(ont_request)
        rclpy.spin_until_future_complete(node, ont_future)
        is_safe = ont_future.result().is_safe

        # Verify RL action respects ontology constraints
        assert is_safe is True  # RL should only output safe actions

        node.destroy_node()
```

## Simulation Testing

### ArduPilot SITL Test Scenarios

**Automated SITL test suite**:

```python
# test/sitl/test_waypoint_navigation.py
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

async def test_waypoint_navigation():
    """Test autonomous waypoint navigation in SITL."""
    # Connect to SITL
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    # Arm and takeoff
    await drone.action.arm()
    await drone.action.takeoff()

    # Wait for altitude
    async for position in drone.telemetry.position():
        if position.relative_altitude_m > 4.5:
            break

    # Start offboard mode
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -5, 0))
    await drone.offboard.start()

    # Navigate to waypoints
    waypoints = [
        (10, 0, -5, 0),
        (10, 10, -5, 90),
        (0, 10, -5, 180),
        (0, 0, -5, 270)
    ]

    for north, east, down, yaw in waypoints:
        await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
        await asyncio.sleep(10)  # Wait for arrival

        # Verify position
        async for pos in drone.telemetry.position():
            assert abs(pos.latitude_deg - expected_lat) < 0.0001
            assert abs(pos.longitude_deg - expected_lon) < 0.0001
            break

    # Land
    await drone.offboard.stop()
    await drone.action.land()

    print("✓ Waypoint navigation test passed")

# Run test
asyncio.run(test_waypoint_navigation())
```

**Run SITL tests**:

```bash
# Start SITL in background
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14540 &

# Run test suite
cd ~/Github/DroneProjects/flyby-f11
python3 test/sitl/test_waypoint_navigation.py
python3 test/sitl/test_obstacle_avoidance.py
python3 test/sitl/test_geofence_enforcement.py
python3 test/sitl/test_battery_failsafe.py

# Kill SITL
pkill -9 sim_vehicle.py
```

### Gazebo Simulation Tests

**Automated scenario testing**:

```bash
#!/bin/bash
# test/simulation/run_scenarios.sh

scenarios=(
    "indoor_navigation"
    "obstacle_field"
    "gps_denied_mission"
    "dynamic_replanning"
)

for scenario in "${scenarios[@]}"; do
    echo "Running scenario: $scenario"

    # Launch Gazebo with scenario world
    gazebo --verbose worlds/${scenario}.world &
    GAZEBO_PID=$!
    sleep 5

    # Launch SITL
    cd ~/ardupilot/ArduCopter
    sim_vehicle.py -v ArduCopter -f gazebo-iris --console &
    SITL_PID=$!
    sleep 10

    # Launch ROS 2 autonomy stack
    cd ~/Github/DroneProjects/flyby-f11/ros2_ws
    source install/setup.bash
    ros2 launch flyby_f11_bringup simulation.launch.py &
    ROS_PID=$!
    sleep 5

    # Run test
    python3 ../test/simulation/test_${scenario}.py
    TEST_RESULT=$?

    # Cleanup
    kill $ROS_PID $SITL_PID $GAZEBO_PID
    sleep 2

    if [ $TEST_RESULT -ne 0 ]; then
        echo "✗ Scenario $scenario FAILED"
        exit 1
    else
        echo "✓ Scenario $scenario PASSED"
    fi
done

echo "All scenarios passed!"
```

### Ontology Safety Testing

**Exhaustive constraint verification**:

```python
# test/ontology/test_safety_constraints.py
import pytest
from pyswip import Prolog

class TestSafetyConstraints:
    @pytest.fixture
    def prolog(self):
        p = Prolog()
        p.consult("ontology/execution_mode/compiled_rules.pl")
        return p

    def test_collision_avoidance_enforced(self, prolog):
        """Test that collision avoidance constraints are enforced."""
        # Simulate obstacle at 2m distance
        prolog.assertz("distance(drone, obstacle_1, 2.0)")

        # Query if forward movement allowed
        result = list(prolog.query("canExecute(moveForward)"))

        # Should be blocked due to safety margin (3m)
        assert len(result) == 0  # No solutions = action blocked

    def test_geofence_enforcement(self, prolog):
        """Test that geofence boundaries are enforced."""
        # Set geofence boundary
        prolog.assertz("geofenceBoundary([[-10, -10], [10, 10]])")

        # Test position inside geofence
        prolog.assertz("position(drone, [0, 0, -5])")
        result_inside = list(prolog.query("isWithinGeofence(drone)"))
        assert len(result_inside) > 0  # Inside geofence

        # Test position outside geofence
        prolog.retract("position(drone, [0, 0, -5])")
        prolog.assertz("position(drone, [15, 0, -5])")
        result_outside = list(prolog.query("isWithinGeofence(drone)"))
        assert len(result_outside) == 0  # Outside geofence

    def test_energy_management_triggers_rtl(self, prolog):
        """Test that low battery triggers return-to-home."""
        # Simulate low battery
        prolog.assertz("batteryLevel(18.0)")  # 18% remaining
        prolog.assertz("energyToReturnHome(15.0)")
        prolog.assertz("safetyReserve(5.0)")

        # Query if RTL should trigger
        result = list(prolog.query("mustReturnToHome"))
        assert len(result) > 0  # Should trigger RTL

    def test_all_safety_axioms_loaded(self, prolog):
        """Verify all critical safety rules are present."""
        required_rules = [
            "canExecute/1",
            "mustAvoid/1",
            "isWithinGeofence/1",
            "mustReturnToHome/0",
            "hasSafeClearance/1",
            "violatesSafetyConstraint/1"
        ]

        for rule in required_rules:
            result = list(prolog.query(f"current_predicate({rule})"))
            assert len(result) > 0, f"Missing safety rule: {rule}"
```

**Run ontology tests**:
```bash
pytest test/ontology/ -v --tb=short
```

## Hardware-in-the-Loop (HIL) Testing

### HIL Test Setup

**Hardware configuration**:
- Jetson Orin NX connected to development PC via network
- ArduPilot flight controller connected via USB/serial
- Cameras (T265, D455) connected to Jetson
- Gazebo running on development PC
- MAVLink bridge between SITL and real hardware

**Launch HIL test**:

```bash
# On development PC: Start Gazebo
gazebo --verbose worlds/hil_test.world

# On development PC: Start SITL with HIL
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --out=udp:<jetson-ip>:14550

# On Jetson: Start autonomy stack
cd ~/Github/DroneProjects/flyby-f11/ros2_ws
source install/setup.bash
ros2 launch flyby_f11_bringup hil_test.launch.py

# On development PC: Run HIL test
python3 test/hil/test_perception_latency.py
```

### HIL Test Scenarios

**Test 1: Perception latency**:

```python
# test/hil/test_perception_latency.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import time

class PercepLatencyTest(Node):
    def __init__(self):
        super().__init__('perception_latency_test')
        self.sub = self.create_subscription(
            Header,
            '/perception/detection_header',
            self.callback,
            10
        )
        self.latencies = []

    def callback(self, msg):
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(msg.stamp)
        latency_ms = (now - stamp).nanoseconds / 1e6
        self.latencies.append(latency_ms)

        if len(self.latencies) >= 100:
            avg_latency = sum(self.latencies) / len(self.latencies)
            max_latency = max(self.latencies)

            print(f"Avg latency: {avg_latency:.2f}ms")
            print(f"Max latency: {max_latency:.2f}ms")

            assert avg_latency < 100, "Avg perception latency > 100ms"
            assert max_latency < 200, "Max perception latency > 200ms"

            print("✓ Perception latency test PASSED")
            rclpy.shutdown()

rclpy.init()
node = PercepLatencyTest()
rclpy.spin(node)
```

**Test 2: Memory usage under load**:

```python
# test/hil/test_memory_usage.py
import subprocess
import time

def get_jetson_memory():
    """Get Jetson memory usage via SSH."""
    result = subprocess.run(
        ['ssh', 'jetson@192.168.1.100', 'free -m | grep Mem'],
        capture_output=True, text=True
    )
    parts = result.stdout.split()
    total = int(parts[1])
    used = int(parts[2])
    return used, total

# Start autonomy stack
subprocess.Popen(['ssh', 'jetson@192.168.1.100',
                  'cd ~/ros2_ws && source install/setup.bash && '
                  'ros2 launch flyby_f11_bringup execution_mode.launch.py'])

time.sleep(30)  # Wait for startup

# Monitor memory for 5 minutes
for i in range(60):
    used, total = get_jetson_memory()
    percent = (used / total) * 100

    print(f"[{i*5}s] Memory: {used}MB / {total}MB ({percent:.1f}%)")

    assert percent < 95, f"Memory usage too high: {percent:.1f}%"

    time.sleep(5)

print("✓ Memory usage test PASSED")
```

## Verification and Validation

### Verification (Are we building the system right?)

**Code reviews**:
- All PRs require 1+ reviewer approval
- Automated checks: linting, formatting, tests pass
- Manual review: logic correctness, edge cases, performance

**Static analysis**:
```bash
# Python
ruff check src/
mypy src/

# C++
cppcheck --enable=all src/
clang-tidy src/*.cpp
```

**Ontology verification**:
```bash
# Verify SUMO axioms with Vampire theorem prover
cd ~/flyby-f11/ontology/planning_mode
vampire --mode casc safety_axioms.kif

# Check for inconsistencies
vampire --mode casc --proof on consistency_check.kif
```

### Validation (Are we building the right system?)

**Requirements traceability**:
- Every requirement has associated tests
- Test coverage report shows requirement coverage

**User acceptance testing** (with MCTSSA):
- Mission scenario demonstrations
- Operator feedback on explainability
- Performance validation against success criteria

**Flight testing validation**:
- Progressive testing: tethered → indoor → outdoor
- Safety pilot oversight for all autonomous flights
- Video recording + data logging for post-flight analysis

## Test Coverage Goals

### Coverage Targets

| Component | Target Coverage | Rationale |
|-----------|----------------|-----------|
| Ontology rules | 100% | Safety-critical |
| Perception grounding | 90%+ | Core functionality |
| RL policy wrappers | 85%+ | Integration critical |
| Utilities | 70%+ | Less critical |
| Launch files | Test all scenarios | Integration |

**Measure coverage**:

```bash
# Python coverage
pytest --cov=perception_grounding --cov-report=term-missing

# C++ coverage (gcov/lcov)
colcon build --cmake-args -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage"
colcon test
lcov --capture --directory build --output-file coverage.info
genhtml coverage.info --output-directory coverage_html
```

## Continuous Integration

**GitHub Actions workflow** (`.github/workflows/ci.yml`):

```yaml
name: CI

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3

      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: humble

      - name: Install dependencies
        run: |
          cd ros2_ws
          rosdep install --from-paths src --ignore-src -r -y

      - name: Build workspace
        run: |
          cd ros2_ws
          colcon build --symlink-install

      - name: Run tests
        run: |
          cd ros2_ws
          colcon test
          colcon test-result --verbose

      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@v3
        with:
          name: test-results
          path: ros2_ws/build/*/test_results/**/*.xml

      - name: Check coverage
        run: |
          cd ros2_ws
          colcon test --pytest-args --cov --cov-report=xml

      - name: Upload coverage to Codecov
        uses: codecov/codecov-action@v3
        with:
          files: ./ros2_ws/coverage.xml
```

## Testing Schedule

### Development Phase
- **Daily**: Unit tests on modified code
- **Pre-commit**: Linting, formatting, affected unit tests
- **Pre-PR**: Full unit + integration tests, coverage check
- **Post-merge**: Full test suite + SITL scenarios

### Integration Phase
- **Weekly**: Full SITL test suite (all scenarios)
- **Bi-weekly**: HIL tests on Jetson hardware
- **Monthly**: Performance benchmarking

### Pre-Deployment
- **1 week before**: Stress testing, edge case validation
- **3 days before**: Full regression suite
- **1 day before**: Smoke tests, pre-flight checklist dry run

### Post-Deployment
- **After each flight**: Log analysis, regression test for any issues
- **Weekly**: Review flight data, update test scenarios

## Test Documentation

**Test plan template**:

```markdown
# Test Plan: <Feature Name>

## Objective
<What are we testing?>

## Scope
- In scope: <List>
- Out of scope: <List>

## Test Cases

### TC-001: <Test Case Name>
- **Preconditions**: <Setup required>
- **Steps**:
  1. <Step 1>
  2. <Step 2>
- **Expected Result**: <What should happen>
- **Actual Result**: <What happened>
- **Status**: PASS / FAIL

## Test Environment
- Hardware: <Platform>
- Software: <Versions>

## Risks and Mitigation
- Risk: <Risk>
  - Mitigation: <How to reduce>

## Results Summary
- Total tests: X
- Passed: Y
- Failed: Z
- Coverage: W%
```

---

**Last Updated**: 2025-12-25
**Status**: Comprehensive testing strategy for flyby-f11
