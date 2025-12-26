# Phase 6: Mission Orchestration (Simplified)

## Overview

Implement the orchestration system that coordinates mission planning and execution using the unified Vampire reasoning engine. With the single-reasoner architecture, this phase is significantly simplified—no mode swapping or container orchestration needed.

**Architectural Note:** Phase 3 evaluation determined that Vampire can serve both planning and execution needs. This phase focuses on mission lifecycle management rather than compute mode transitions.

## Human Description

The original Phase 6 planned complex container swapping between "Planning Mode" (heavyweight reasoners) and "Execution Mode" (lightweight Prolog). With the single-reasoner decision:

**Original Plan (Deprecated):**
- Swap 14GB planning containers for execution containers
- Complex memory management for 16GB budget
- Container orchestration between modes

**New Simplified Plan:**
- Single Vampire process handles both planning and tactical queries
- Different query types (planning vs tactical) distinguished by timeout/priority
- Mission lifecycle management (mission receipt → verification → execution → monitoring)

### Tiered Architecture Integration

| Tier | Layer | Function | Orchestration Role |
|------|-------|----------|-------------------|
| 1 | Classical Control | PID, motor control | Not managed here |
| 2 | Pre-computed Safety | Costmaps | Updated before flight |
| 3 | Tactical Reasoning | Safety queries | 20Hz monitoring via Vampire |
| 4 | Mission Planning | Route verification | Pre-flight via Vampire |

## AI Agent Instructions

### Prerequisites
- Phase 4 completed (Vampire runtime with ROS 2 bridge)
- Phase 5 completed (Perception grounding with TPTP facts)
- Understanding of ROS 2 lifecycle nodes
- Familiarity with mission management patterns

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

#### 1. Create Mission Orchestrator Package

**Location:** `ros2_ws/src/mission_orchestrator/`

**Package Structure:**
```
mission_orchestrator/
├── CMakeLists.txt
├── package.xml
├── mission_orchestrator/
│   ├── __init__.py
│   ├── mission_manager_node.py
│   ├── mission_verifier.py
│   ├── safety_monitor.py
│   └── mission_state_machine.py
├── msg/
│   ├── MissionPlan.msg
│   ├── MissionStatus.msg
│   └── SafetyState.msg
├── srv/
│   ├── VerifyMission.srv
│   └── StartMission.srv
├── action/
│   └── ExecuteMission.action
├── config/
│   └── orchestrator_params.yaml
├── launch/
│   └── mission_orchestrator.launch.py
└── test/
    └── test_mission_lifecycle.py
```

#### 2. Implement Mission State Machine

```python
# mission_orchestrator/mission_state_machine.py
from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional, List

class MissionState(Enum):
    IDLE = auto()
    RECEIVING = auto()
    VERIFYING = auto()
    VERIFIED = auto()
    ARMING = auto()
    EXECUTING = auto()
    PAUSED = auto()
    RETURNING = auto()
    LANDING = auto()
    COMPLETED = auto()
    ABORTED = auto()

@dataclass
class MissionContext:
    mission_id: str
    waypoints: List[dict]
    verified: bool = False
    current_waypoint: int = 0
    safety_violations: List[str] = None

class MissionStateMachine:
    """Manages mission lifecycle with Vampire verification."""

    VALID_TRANSITIONS = {
        MissionState.IDLE: [MissionState.RECEIVING],
        MissionState.RECEIVING: [MissionState.VERIFYING, MissionState.IDLE],
        MissionState.VERIFYING: [MissionState.VERIFIED, MissionState.IDLE],
        MissionState.VERIFIED: [MissionState.ARMING, MissionState.IDLE],
        MissionState.ARMING: [MissionState.EXECUTING, MissionState.IDLE],
        MissionState.EXECUTING: [MissionState.PAUSED, MissionState.RETURNING,
                                 MissionState.COMPLETED, MissionState.ABORTED],
        MissionState.PAUSED: [MissionState.EXECUTING, MissionState.RETURNING,
                              MissionState.ABORTED],
        MissionState.RETURNING: [MissionState.LANDING, MissionState.ABORTED],
        MissionState.LANDING: [MissionState.COMPLETED, MissionState.ABORTED],
        MissionState.COMPLETED: [MissionState.IDLE],
        MissionState.ABORTED: [MissionState.IDLE],
    }

    def __init__(self):
        self.state = MissionState.IDLE
        self.context: Optional[MissionContext] = None

    def can_transition_to(self, new_state: MissionState) -> bool:
        return new_state in self.VALID_TRANSITIONS.get(self.state, [])

    def transition_to(self, new_state: MissionState) -> bool:
        if not self.can_transition_to(new_state):
            return False
        self.state = new_state
        return True
```

#### 3. Implement Mission Manager Node

```python
# mission_orchestrator/mission_manager_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from vampire_bridge.srv import VampireQuery
from vampire_bridge.msg import ReasoningQuery, ReasoningResult
from mission_orchestrator.msg import MissionPlan, MissionStatus, SafetyState
from mission_orchestrator.srv import VerifyMission, StartMission
from mission_orchestrator.action import ExecuteMission
from mission_orchestrator.mission_state_machine import MissionStateMachine, MissionState

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.state_machine = MissionStateMachine()

        # Parameters
        self.declare_parameter('planning_timeout_ms', 5000)
        self.declare_parameter('tactical_timeout_ms', 100)
        self.declare_parameter('safety_check_hz', 20.0)

        # Vampire query client
        self.vampire_client = self.create_client(
            VampireQuery,
            '/vampire/query'
        )

        # Mission verification service
        self.verify_srv = self.create_service(
            VerifyMission,
            '/mission/verify',
            self.verify_mission_callback
        )

        # Mission start service
        self.start_srv = self.create_service(
            StartMission,
            '/mission/start',
            self.start_mission_callback
        )

        # Mission execution action
        self.execute_action = ActionServer(
            self,
            ExecuteMission,
            '/mission/execute',
            self.execute_mission_callback
        )

        # Status publisher
        self.status_pub = self.create_publisher(
            MissionStatus,
            '/mission/status',
            10
        )

        # Safety state publisher
        self.safety_pub = self.create_publisher(
            SafetyState,
            '/mission/safety_state',
            10
        )

        # Safety monitoring timer (20Hz)
        safety_period = 1.0 / self.get_parameter('safety_check_hz').value
        self.safety_timer = self.create_timer(safety_period, self.safety_check)

        self.get_logger().info('Mission manager initialized')

    async def verify_mission_callback(self, request, response):
        """
        Verify mission plan using Vampire for planning queries.
        Uses longer timeout (5s) for comprehensive verification.
        """
        self.state_machine.transition_to(MissionState.VERIFYING)

        # Build verification queries
        queries = [
            ('planning', 'mission_feasibility', request.mission.waypoints),
            ('planning', 'regulatory_compliance', []),
            ('planning', 'path_safety', request.mission.waypoints),
        ]

        all_passed = True
        violations = []

        for query_type, query_name, params in queries:
            query = ReasoningQuery()
            query.query_type = query_type
            query.query_name = query_name
            query.timeout_ms = self.get_parameter('planning_timeout_ms').value
            query.use_cache = False  # Always fresh for verification

            result = await self._execute_query(query)

            if not result.success or result.result != 'THEOREM':
                all_passed = False
                violations.extend(result.violated_axioms)

        if all_passed:
            self.state_machine.transition_to(MissionState.VERIFIED)
            response.verified = True
            response.message = "Mission verified successfully"
        else:
            self.state_machine.transition_to(MissionState.IDLE)
            response.verified = False
            response.message = f"Verification failed: {violations}"

        return response

    async def safety_check(self):
        """
        Periodic safety check during execution.
        Uses short timeout (100ms) for tactical queries at 20Hz.
        """
        if self.state_machine.state != MissionState.EXECUTING:
            return

        query = ReasoningQuery()
        query.query_type = 'safety'
        query.query_name = 'safe_state_check'
        query.timeout_ms = self.get_parameter('tactical_timeout_ms').value
        query.use_cache = True  # Use cache for repeated queries

        result = await self._execute_query(query)

        # Publish safety state
        safety_state = SafetyState()
        safety_state.stamp = self.get_clock().now().to_msg()
        safety_state.safe = result.success and result.result == 'THEOREM'
        safety_state.violations = result.violated_axioms
        safety_state.latency_ms = result.latency_ms
        self.safety_pub.publish(safety_state)

        # Handle violations
        if not safety_state.safe:
            self.get_logger().error(f'SAFETY VIOLATION: {result.violated_axioms}')
            await self._handle_safety_violation(result.violated_axioms)

    async def _handle_safety_violation(self, violations: list):
        """Handle safety violations based on severity."""
        critical_violations = [
            'nfz_violation', 'collision_imminent', 'lost_localization'
        ]

        for violation in violations:
            if violation in critical_violations:
                self.get_logger().error(f'CRITICAL: {violation} - Aborting mission')
                self.state_machine.transition_to(MissionState.ABORTED)
                return

        # Non-critical violations: pause and alert
        self.state_machine.transition_to(MissionState.PAUSED)

    async def _execute_query(self, query: ReasoningQuery) -> ReasoningResult:
        """Execute a Vampire query via the bridge."""
        future = self.vampire_client.call_async(VampireQuery.Request(query=query))
        result = await future
        return result.result
```

#### 4. Implement Safety Monitor

```python
# mission_orchestrator/safety_monitor.py
"""
Continuous safety monitoring using Vampire tactical queries.
Runs at 20Hz during mission execution.
"""

import rclpy
from rclpy.node import Node
from vampire_bridge.srv import VampireQuery
from vampire_bridge.msg import ReasoningQuery
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Vampire client
        self.vampire_client = self.create_client(VampireQuery, '/vampire/query')

        # State subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_callback, 10)
        self.fact_sub = self.create_subscription(
            String, '/perception/tptp_facts', self.fact_callback, 10)

        # Current state
        self.current_facts = []
        self.battery_level = 100.0
        self.position = None

        # Safety check timer (20Hz)
        self.timer = self.create_timer(0.05, self.run_safety_queries)

        # Safety violation publisher
        self.violation_pub = self.create_publisher(
            String, '/safety/violations', 10)

        self.get_logger().info('Safety monitor initialized')

    async def run_safety_queries(self):
        """Run priority safety queries at 20Hz."""

        safety_queries = [
            ('geofence_check', self.check_geofence),
            ('nfz_violation', self.check_nfz),
            ('battery_return', self.check_battery),
        ]

        for query_name, check_fn in safety_queries:
            if await check_fn():
                self.get_logger().error(f'SAFETY: {query_name} TRIGGERED')
                self.violation_pub.publish(String(data=query_name))

    async def check_geofence(self) -> bool:
        """Check if drone is within mission geofence."""
        query = ReasoningQuery()
        query.query_type = 'safety'
        query.query_name = 'geofence_check'
        query.timeout_ms = 50
        query.use_cache = True

        result = await self._query(query)
        return result.result == 'COUNTER_SATISFIABLE'  # Violation detected

    async def check_nfz(self) -> bool:
        """Check if drone is in a no-fly zone."""
        query = ReasoningQuery()
        query.query_type = 'safety'
        query.query_name = 'nfz_violation'
        query.timeout_ms = 50
        query.use_cache = True

        result = await self._query(query)
        return result.result == 'THEOREM'  # NFZ violation is a theorem

    async def check_battery(self) -> bool:
        """Check if battery requires return to launch."""
        query = ReasoningQuery()
        query.query_type = 'safety'
        query.query_name = 'battery_return'
        query.timeout_ms = 50
        query.use_cache = True

        result = await self._query(query)
        return result.result == 'THEOREM'  # Battery return is required

    async def _query(self, query):
        future = self.vampire_client.call_async(VampireQuery.Request(query=query))
        return (await future).result
```

#### 5. Create Quadlet for Production Deployment

**Location:** `quadlet/mission-orchestrator.container`

```ini
[Unit]
Description=Mission Orchestrator for Flyby F-11
After=vampire-reasoning.service

[Container]
Image=localhost/flyby-f11-ros2:latest
ContainerName=mission-orchestrator

# Mount configurations
Volume=/home/flyby/ros2_ws:/workspace/ros2_ws:z
Volume=/home/flyby/config/missions:/workspace/missions:z,ro

# ROS 2 networking
Network=host
Environment=ROS_DOMAIN_ID=42
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Run the mission orchestrator node
Exec=/workspace/ros2_ws/install/mission_orchestrator/lib/mission_orchestrator/mission_manager_node

[Service]
Restart=always
TimeoutStartSec=30

[Install]
WantedBy=default.target
```

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `mission_orchestrator` package builds and installs
- [ ] Mission state machine handles all lifecycle transitions
- [ ] Mission verification uses Vampire planning queries (5s timeout)
- [ ] Safety monitoring runs at 20Hz with Vampire tactical queries
- [ ] Safety violations trigger appropriate responses (pause, abort, RTL)
- [ ] Quadlet container starts after vampire-reasoning service
- [ ] Integration tests pass with simulated missions

### Verification

Run automated verification:
```bash
bash .phases/phase-06-phase-transition/verification.sh
```

### Common Pitfalls

- **Query timeouts**: Planning queries need longer timeouts than tactical
- **State machine race conditions**: Ensure atomic state transitions
- **Service dependencies**: vampire-reasoning must be running first
- **Cache invalidation**: State changes must invalidate relevant cached queries
- **Action cancellation**: Handle mission abort cleanly

### References

- [ROS 2 Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS 2 Actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Phase 4 Vampire Bridge](../phase-04-execution-mode/TASK.md)
- [Phase 3 Evaluation Report](../../ontology/evaluation/EVALUATION_REPORT.qmd)

### Dependencies
See `dependencies.json` - requires Phase 4 and 5 completion.

### Next Phase
After completion, proceed to Phase 7: Mission Planner RL Agent
