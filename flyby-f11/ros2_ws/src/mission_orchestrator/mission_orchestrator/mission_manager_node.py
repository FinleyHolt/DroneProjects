#!/usr/bin/env python3
"""
mission_manager_node.py - Main ROS 2 node for mission orchestration

Coordinates mission planning and execution using the Vampire reasoning engine.
Provides services for mission verification and start, plus an action server
for mission execution with progress feedback.

Services:
    /mission/verify (VerifyMission) - Pre-flight verification with 5s timeout
    /mission/start (StartMission) - Start a verified mission

Action Servers:
    /mission/execute (ExecuteMission) - Execute mission with feedback

Publishers:
    /mission/status (MissionStatus) - Periodic status updates (1Hz)
    /mission/safety_state (SafetyState) - Safety monitoring results (20Hz)

Parameters:
    planning_timeout_ms: Timeout for planning queries (default: 5000)
    tactical_timeout_ms: Timeout for tactical queries (default: 100)
    safety_check_hz: Safety monitoring rate (default: 20.0)
    status_publish_hz: Status publish rate (default: 1.0)

Usage:
    ros2 run mission_orchestrator mission_manager_node
"""

import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Time

from mission_orchestrator.mission_state_machine import (
    MissionStateMachine, MissionState, MissionContext, Waypoint
)
from mission_orchestrator.safety_monitor import SafetyMonitor


class MissionManagerNode(Node):
    """
    ROS 2 node for mission lifecycle management.

    Manages mission verification, execution, and safety monitoring using
    the Vampire theorem prover for both planning (pre-flight) and tactical
    (runtime) queries.
    """

    # Mapping from MissionState enum to message constants
    STATE_TO_MSG = {
        MissionState.IDLE: 0,
        MissionState.RECEIVING: 1,
        MissionState.VERIFYING: 2,
        MissionState.VERIFIED: 3,
        MissionState.ARMING: 4,
        MissionState.EXECUTING: 5,
        MissionState.PAUSED: 6,
        MissionState.RETURNING: 7,
        MissionState.LANDING: 8,
        MissionState.COMPLETED: 9,
        MissionState.ABORTED: 10,
    }

    def __init__(self):
        super().__init__('mission_manager')

        # Declare parameters
        self._declare_parameters()

        # Get parameter values
        self._planning_timeout_ms = self.get_parameter('planning_timeout_ms').value
        self._tactical_timeout_ms = self.get_parameter('tactical_timeout_ms').value
        self._safety_check_hz = self.get_parameter('safety_check_hz').value
        self._status_publish_hz = self.get_parameter('status_publish_hz').value

        # Initialize state machine with transition callback
        self._state_machine = MissionStateMachine(
            on_transition=self._on_state_transition
        )

        # Initialize safety monitor
        self._safety_monitor = SafetyMonitor(
            node=self,
            state_machine=self._state_machine,
            tactical_timeout_ms=self._tactical_timeout_ms
        )

        # Callback groups for concurrent service handling
        self._service_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        # Create Vampire query client
        self._init_vampire_client()

        # Create services
        self._init_services()

        # Create action server
        self._init_action_server()

        # Create publishers
        self._init_publishers()

        # Create timers
        self._init_timers()

        # Track active goal handle for action feedback
        self._active_goal: Optional[ServerGoalHandle] = None
        self._goal_lock = threading.Lock()

        # Execution statistics
        self._stats = {
            'missions_verified': 0,
            'missions_started': 0,
            'missions_completed': 0,
            'missions_aborted': 0,
            'verification_failures': 0,
        }

        self.get_logger().info(
            f'Mission manager initialized. '
            f'Planning timeout: {self._planning_timeout_ms}ms, '
            f'Tactical timeout: {self._tactical_timeout_ms}ms, '
            f'Safety check rate: {self._safety_check_hz}Hz'
        )

    def _declare_parameters(self) -> None:
        """Declare all node parameters with defaults."""
        self.declare_parameter('planning_timeout_ms', 5000)
        self.declare_parameter('tactical_timeout_ms', 100)
        self.declare_parameter('safety_check_hz', 20.0)
        self.declare_parameter('status_publish_hz', 1.0)

    def _init_vampire_client(self) -> None:
        """Initialize Vampire query service client."""
        try:
            from vampire_bridge.srv import VampireQuery
            self._VampireQuery = VampireQuery
            self._vampire_client = self.create_client(
                VampireQuery,
                '/vampire/query',
                callback_group=self._service_cb_group
            )
            self.get_logger().info('Vampire query client created')
        except ImportError:
            self.get_logger().warning(
                'vampire_bridge not available - queries will fail'
            )
            self._vampire_client = None
            self._VampireQuery = None

    def _init_services(self) -> None:
        """Create ROS 2 services."""
        try:
            from mission_orchestrator.srv import VerifyMission, StartMission
            self._VerifyMission = VerifyMission
            self._StartMission = StartMission

            self._verify_srv = self.create_service(
                VerifyMission,
                '/mission/verify',
                self._verify_mission_callback,
                callback_group=self._service_cb_group
            )

            self._start_srv = self.create_service(
                StartMission,
                '/mission/start',
                self._start_mission_callback,
                callback_group=self._service_cb_group
            )

            self.get_logger().info('Mission services created')
        except ImportError as e:
            self.get_logger().error(
                f'Failed to import mission services: {e}. '
                'Run colcon build to generate message types.'
            )

    def _init_action_server(self) -> None:
        """Create action server for mission execution."""
        try:
            from mission_orchestrator.action import ExecuteMission
            self._ExecuteMission = ExecuteMission

            self._execute_action = ActionServer(
                self,
                ExecuteMission,
                '/mission/execute',
                execute_callback=self._execute_mission_callback,
                goal_callback=self._execute_goal_callback,
                cancel_callback=self._execute_cancel_callback,
                callback_group=self._service_cb_group
            )

            self.get_logger().info('Mission execution action server created')
        except ImportError as e:
            self.get_logger().error(f'Failed to import action type: {e}')

    def _init_publishers(self) -> None:
        """Create status publishers."""
        try:
            from mission_orchestrator.msg import MissionStatus, SafetyState
            self._MissionStatus = MissionStatus
            self._SafetyState = SafetyState

            self._status_pub = self.create_publisher(
                MissionStatus,
                '/mission/status',
                10
            )

            self._safety_pub = self.create_publisher(
                SafetyState,
                '/mission/safety_state',
                10
            )

            self.get_logger().info('Status publishers created')
        except ImportError as e:
            self.get_logger().error(f'Failed to import message types: {e}')
            self._status_pub = None
            self._safety_pub = None

    def _init_timers(self) -> None:
        """Create periodic timers."""
        # Status publishing timer (1Hz by default)
        status_period = 1.0 / self._status_publish_hz
        self._status_timer = self.create_timer(
            status_period,
            self._publish_status,
            callback_group=self._timer_cb_group
        )

        # Safety monitoring timer (20Hz by default)
        safety_period = 1.0 / self._safety_check_hz
        self._safety_timer = self.create_timer(
            safety_period,
            self._safety_check_callback,
            callback_group=self._timer_cb_group
        )

    def _on_state_transition(self, old_state: MissionState, new_state: MissionState) -> None:
        """Callback invoked on state machine transitions."""
        self.get_logger().info(f'State transition: {old_state.name} -> {new_state.name}')

        # Track statistics
        if new_state == MissionState.COMPLETED:
            self._stats['missions_completed'] += 1
        elif new_state == MissionState.ABORTED:
            self._stats['missions_aborted'] += 1

        # Publish immediate status update on transition
        self._publish_status()

    # -------------------------------------------------------------------------
    # Service Callbacks
    # -------------------------------------------------------------------------

    def _verify_mission_callback(self, request, response):
        """
        Verify mission plan using Vampire planning queries.

        Uses longer timeout (5s) for comprehensive verification of:
        - Mission feasibility
        - Regulatory compliance
        - Path safety
        - Battery sufficiency
        """
        mission = request.mission
        start_time = time.perf_counter()

        self.get_logger().info(
            f'Verifying mission: {mission.mission_id} '
            f'({len(mission.waypoints)} waypoints)'
        )

        # Transition to RECEIVING then VERIFYING
        if not self._state_machine.transition_to(MissionState.RECEIVING):
            response.verified = False
            response.message = f'Cannot verify: invalid state {self._state_machine.state.name}'
            return response

        # Create mission context
        waypoints = [
            Waypoint(
                latitude=wp.latitude,
                longitude=wp.longitude,
                altitude=wp.altitude,
                hold_time_sec=wp.hold_time_sec,
                acceptance_radius_m=wp.acceptance_radius_m
            )
            for wp in mission.waypoints
        ]

        context = MissionContext(
            mission_id=mission.mission_id,
            waypoints=waypoints,
            battery_required_percent=mission.min_battery_percent
        )
        self._state_machine.context = context
        self._state_machine.transition_to(MissionState.VERIFYING)

        # Run verification queries
        issues = []
        feasibility_ok = True
        regulatory_ok = True
        path_safety_ok = True
        battery_ok = True

        if self._vampire_client is not None:
            # Check mission feasibility
            result = self._run_planning_query(
                'planning', 'mission_feasibility',
                [f'mission_id={mission.mission_id}']
            )
            if result is None or result.szs_status != 'Theorem':
                feasibility_ok = False
                issues.append('Mission feasibility check failed')

            # Check regulatory compliance
            result = self._run_planning_query(
                'planning', 'regulatory_compliance',
                [f'max_altitude={mission.max_altitude_m}']
            )
            if result is None or result.szs_status != 'Theorem':
                regulatory_ok = False
                issues.append('Regulatory compliance check failed')

            # Check path safety (geofence, NFZ)
            result = self._run_planning_query(
                'safety', 'path_safety',
                [f'geofence_radius={mission.geofence_radius_m}']
            )
            if result is None or result.szs_status != 'Theorem':
                path_safety_ok = False
                issues.append('Path safety check failed')

            # Check battery sufficiency
            result = self._run_planning_query(
                'safety', 'battery_return',
                [f'min_battery={mission.min_battery_percent}']
            )
            if result is None or result.szs_status != 'Theorem':
                battery_ok = False
                issues.append('Insufficient battery for mission')
        else:
            self.get_logger().warning('Vampire client not available - skipping verification queries')

        # Update context and response
        elapsed_ms = (time.perf_counter() - start_time) * 1000

        all_ok = feasibility_ok and regulatory_ok and path_safety_ok and battery_ok
        context.verified = all_ok
        context.verification_issues = issues

        if all_ok:
            self._state_machine.transition_to(MissionState.VERIFIED)
            response.verified = True
            response.message = 'Mission verified successfully'
            self._stats['missions_verified'] += 1
        else:
            self._state_machine.transition_to(MissionState.IDLE)
            response.verified = False
            response.message = f'Verification failed: {"; ".join(issues)}'
            self._stats['verification_failures'] += 1

        response.issues = issues
        response.feasibility_ok = feasibility_ok
        response.regulatory_ok = regulatory_ok
        response.path_safety_ok = path_safety_ok
        response.battery_ok = battery_ok
        response.verification_latency_ms = elapsed_ms

        self.get_logger().info(
            f'Verification complete: verified={response.verified}, '
            f'latency={elapsed_ms:.1f}ms'
        )

        return response

    def _start_mission_callback(self, request, response):
        """
        Start execution of a verified mission.

        Transitions from VERIFIED -> ARMING -> EXECUTING.
        """
        mission_id = request.mission_id

        self.get_logger().info(f'Start mission request: {mission_id}')

        # Verify we have a verified mission with matching ID
        context = self._state_machine.context
        if context is None:
            response.success = False
            response.message = 'No mission loaded'
            response.state = self.STATE_TO_MSG[self._state_machine.state]
            response.state_name = self._state_machine.state.name
            return response

        if context.mission_id != mission_id:
            response.success = False
            response.message = f'Mission ID mismatch: expected {context.mission_id}'
            response.state = self.STATE_TO_MSG[self._state_machine.state]
            response.state_name = self._state_machine.state.name
            return response

        if self._state_machine.state != MissionState.VERIFIED:
            response.success = False
            response.message = f'Mission not verified (state: {self._state_machine.state.name})'
            response.state = self.STATE_TO_MSG[self._state_machine.state]
            response.state_name = self._state_machine.state.name
            return response

        # Transition through ARMING to EXECUTING
        if not self._state_machine.transition_to(MissionState.ARMING):
            response.success = False
            response.message = 'Failed to enter ARMING state'
            response.state = self.STATE_TO_MSG[self._state_machine.state]
            response.state_name = self._state_machine.state.name
            return response

        # In a real system, we would wait for arming confirmation here
        # For now, immediately transition to EXECUTING
        if not self._state_machine.transition_to(MissionState.EXECUTING):
            response.success = False
            response.message = 'Failed to enter EXECUTING state'
            response.state = self.STATE_TO_MSG[self._state_machine.state]
            response.state_name = self._state_machine.state.name
            return response

        response.success = True
        response.message = f'Mission {mission_id} started'
        response.state = self.STATE_TO_MSG[self._state_machine.state]
        response.state_name = self._state_machine.state.name
        self._stats['missions_started'] += 1

        self.get_logger().info(f'Mission {mission_id} started successfully')

        return response

    # -------------------------------------------------------------------------
    # Action Server Callbacks
    # -------------------------------------------------------------------------

    def _execute_goal_callback(self, goal_request) -> GoalResponse:
        """Accept or reject mission execution goal."""
        mission_id = goal_request.mission_id

        context = self._state_machine.context
        if context is None or context.mission_id != mission_id:
            self.get_logger().warning(f'Rejecting goal: mission {mission_id} not loaded')
            return GoalResponse.REJECT

        if self._state_machine.state not in {MissionState.VERIFIED, MissionState.EXECUTING}:
            self.get_logger().warning(
                f'Rejecting goal: invalid state {self._state_machine.state.name}'
            )
            return GoalResponse.REJECT

        self.get_logger().info(f'Accepting goal for mission {mission_id}')
        return GoalResponse.ACCEPT

    def _execute_cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle mission cancellation request."""
        self.get_logger().info('Mission cancellation requested')

        # Transition to RETURNING on cancel
        if self._state_machine.is_flight_active():
            self._state_machine.transition_to(MissionState.RETURNING, reason='User cancelled')

        return CancelResponse.ACCEPT

    def _execute_mission_callback(self, goal_handle: ServerGoalHandle):
        """
        Execute mission with feedback.

        This is the main action callback that runs during mission execution.
        """
        mission_id = goal_handle.request.mission_id
        self.get_logger().info(f'Executing mission: {mission_id}')

        with self._goal_lock:
            self._active_goal = goal_handle

        try:
            # If not already executing, start the mission
            if self._state_machine.state == MissionState.VERIFIED:
                self._state_machine.transition_to(MissionState.ARMING)
                self._state_machine.transition_to(MissionState.EXECUTING)

            result = self._ExecuteMission.Result()
            feedback = self._ExecuteMission.Feedback()

            start_time = time.time()
            context = self._state_machine.context

            # Main execution loop
            while rclpy.ok():
                state = self._state_machine.state

                # Check for terminal states
                if state == MissionState.COMPLETED:
                    result.success = True
                    result.final_state = self.STATE_TO_MSG[state]
                    result.message = 'Mission completed successfully'
                    result.waypoints_completed = context.current_waypoint_idx + 1
                    result.total_flight_time_sec = time.time() - start_time
                    result.safety_events = context.safety_violations
                    goal_handle.succeed()
                    break

                if state == MissionState.ABORTED:
                    result.success = False
                    result.final_state = self.STATE_TO_MSG[state]
                    result.message = f'Mission aborted: {context.abort_reason or "Unknown"}'
                    result.waypoints_completed = context.current_waypoint_idx
                    result.total_flight_time_sec = time.time() - start_time
                    result.safety_events = context.safety_violations
                    goal_handle.abort()
                    break

                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self._state_machine.transition_to(MissionState.RETURNING)
                    result.success = False
                    result.final_state = self.STATE_TO_MSG[MissionState.RETURNING]
                    result.message = 'Mission cancelled by user'
                    result.waypoints_completed = context.current_waypoint_idx
                    result.total_flight_time_sec = time.time() - start_time
                    result.safety_events = context.safety_violations
                    goal_handle.canceled()
                    break

                # Publish feedback
                feedback.stamp = self.get_clock().now().to_msg()
                feedback.current_waypoint = context.current_waypoint_idx
                feedback.total_waypoints = len(context.waypoints)
                feedback.progress_percent = (
                    (context.current_waypoint_idx / max(len(context.waypoints), 1)) * 100.0
                )
                feedback.distance_to_waypoint_m = 0.0  # Would come from position tracking
                feedback.eta_sec = 0.0  # Would be calculated from speed
                feedback.safety_ok = len(context.safety_violations) == 0
                feedback.active_violations = context.safety_violations[-5:]  # Last 5
                feedback.state = self.STATE_TO_MSG[state]
                feedback.state_name = state.name

                goal_handle.publish_feedback(feedback)

                # Sleep briefly to avoid spinning too fast
                time.sleep(0.1)

        finally:
            with self._goal_lock:
                self._active_goal = None

        return result

    # -------------------------------------------------------------------------
    # Timer Callbacks
    # -------------------------------------------------------------------------

    def _publish_status(self) -> None:
        """Publish mission status (1Hz)."""
        if self._status_pub is None:
            return

        status = self._MissionStatus()
        status.stamp = self.get_clock().now().to_msg()

        context = self._state_machine.context
        state = self._state_machine.state

        status.mission_id = context.mission_id if context else ''
        status.state = self.STATE_TO_MSG[state]

        if context:
            status.current_waypoint_idx = context.current_waypoint_idx
            status.total_waypoints = len(context.waypoints)
            status.mission_progress_percent = (
                (context.current_waypoint_idx / max(len(context.waypoints), 1)) * 100.0
            )
            status.active_violations = context.safety_violations[-5:]
            status.safety_ok = len(context.safety_violations) == 0
        else:
            status.current_waypoint_idx = 0
            status.total_waypoints = 0
            status.mission_progress_percent = 0.0
            status.active_violations = []
            status.safety_ok = True

        # Get safety monitor metrics
        safety_stats = self._safety_monitor.get_statistics()
        status.last_safety_check_latency_ms = safety_stats.get('last_latency_ms', 0.0)
        status.avg_safety_check_latency_ms = safety_stats.get('avg_latency_ms', 0.0)

        status.status_message = f'State: {state.name}'

        self._status_pub.publish(status)

    def _safety_check_callback(self) -> None:
        """Run safety checks at 20Hz during active flight."""
        if not self._state_machine.is_safety_monitoring_active():
            return

        # Delegate to safety monitor
        safety_state = self._safety_monitor.run_safety_check()

        # Publish safety state
        if self._safety_pub is not None and safety_state is not None:
            self._safety_pub.publish(safety_state)

        # Handle safety violations
        if safety_state is not None and not safety_state.safe:
            self._handle_safety_violation(safety_state)

    def _handle_safety_violation(self, safety_state) -> None:
        """Handle detected safety violations."""
        context = self._state_machine.context
        if context is None:
            return

        # Record violations
        for violation in safety_state.violations:
            self._state_machine.record_safety_violation(violation)
            self.get_logger().warning(f'Safety violation: {violation}')

        # Take action based on recommended response
        action = safety_state.recommended_action

        if action == 4:  # ACTION_ABORT
            self.get_logger().error('CRITICAL: Aborting mission')
            self._state_machine.transition_to(
                MissionState.ABORTED,
                reason=f'Critical safety violation: {safety_state.violations}'
            )
        elif action == 3:  # ACTION_RTL
            self.get_logger().warning('Safety violation: Returning to launch')
            self._state_machine.transition_to(MissionState.RETURNING)
        elif action == 2:  # ACTION_PAUSE
            self.get_logger().warning('Safety violation: Pausing mission')
            self._state_machine.transition_to(MissionState.PAUSED)

    # -------------------------------------------------------------------------
    # Vampire Query Helpers
    # -------------------------------------------------------------------------

    def _run_planning_query(self, query_type: str, query_name: str, parameters: list):
        """
        Run a planning query with 5s timeout.

        Args:
            query_type: Query type (planning, safety)
            query_name: Template name
            parameters: Query parameters

        Returns:
            ReasoningResult or None if failed
        """
        if self._vampire_client is None:
            return None

        # Wait for service availability
        if not self._vampire_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Vampire service not available')
            return None

        try:
            from vampire_bridge.msg import ReasoningQuery

            request = self._VampireQuery.Request()
            request.query = ReasoningQuery()
            request.query.query_type = query_type
            request.query.query_name = query_name
            request.query.parameters = parameters
            request.query.timeout_ms = self._planning_timeout_ms
            request.query.use_cache = False  # Always fresh for verification

            future = self._vampire_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=6.0)

            if future.done():
                return future.result().result
            else:
                self.get_logger().warning(f'Planning query timeout: {query_name}')
                return None

        except Exception as e:
            self.get_logger().error(f'Planning query failed: {e}')
            return None

    # -------------------------------------------------------------------------
    # Statistics
    # -------------------------------------------------------------------------

    def get_statistics(self) -> dict:
        """Return node statistics."""
        safety_stats = self._safety_monitor.get_statistics()
        return {
            'state': self._state_machine.state.name,
            'missions': self._stats.copy(),
            'safety_monitor': safety_stats,
        }


def main(args=None):
    """Main entry point for mission_manager_node."""
    rclpy.init(args=args)

    node = MissionManagerNode()

    # Use multi-threaded executor for concurrent service handling
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down mission manager')
    finally:
        stats = node.get_statistics()
        node.get_logger().info(f'Final statistics: {stats}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
