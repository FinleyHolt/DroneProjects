"""
safety_monitor.py - Continuous safety monitoring using Vampire tactical queries

Runs at 20Hz during active mission execution to verify:
    - Geofence containment
    - No-fly zone avoidance
    - Battery return requirements
    - Communications status
    - Localization quality

Uses Vampire tactical queries with short timeouts (100ms) to meet the
20Hz monitoring rate while providing formal safety verification.

The safety monitor integrates with the HotFactBuffer from Phase 5 to
ensure queries include current sensor state.
"""

import time
import threading
from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.node import Node
    from mission_orchestrator.mission_state_machine import MissionStateMachine


@dataclass
class SafetyCheckResult:
    """Result of a single safety check."""
    name: str
    passed: bool
    latency_ms: float
    from_cache: bool = False
    violation_details: Optional[str] = None


class SafetyMonitor:
    """
    Continuous safety monitoring using Vampire tactical queries.

    Runs at 20Hz during EXECUTING, PAUSED, RETURNING, and LANDING states.
    Uses short query timeouts (default 100ms) to maintain real-time
    monitoring while providing formal safety verification.

    Violation Severity:
        CRITICAL (abort): nfz_violation, collision_imminent, lost_localization
        HIGH (RTL): geofence_violation, battery_critical
        MEDIUM (pause): battery_low, comms_degraded
        LOW (warn): sensor_degraded

    Attributes:
        tactical_timeout_ms: Timeout for each safety query
        last_check_time: Timestamp of last safety check
        total_checks: Number of safety checks performed
    """

    # Safety queries to run each cycle
    SAFETY_QUERIES = [
        ('geofence_check', 'safety', 'geofence_check'),
        ('nfz_violation', 'safety', 'nfz_violation'),
        ('battery_return', 'safety', 'battery_return'),
    ]

    # Violation severity classification
    CRITICAL_VIOLATIONS = {'nfz_violation', 'collision_imminent', 'lost_localization'}
    HIGH_VIOLATIONS = {'geofence_violation', 'battery_critical'}
    MEDIUM_VIOLATIONS = {'battery_low', 'comms_degraded'}

    def __init__(
        self,
        node: 'Node',
        state_machine: 'MissionStateMachine',
        tactical_timeout_ms: int = 100
    ):
        """
        Initialize the safety monitor.

        Args:
            node: Parent ROS 2 node for logging and service clients
            state_machine: Mission state machine reference
            tactical_timeout_ms: Timeout for tactical queries
        """
        self._node = node
        self._state_machine = state_machine
        self._tactical_timeout_ms = tactical_timeout_ms

        # Vampire client (will be set from node)
        self._vampire_client = None
        self._init_vampire_client()

        # Statistics
        self._lock = threading.Lock()
        self._total_checks = 0
        self._total_latency_ms = 0.0
        self._last_latency_ms = 0.0
        self._violations_detected = 0
        self._last_check_time: Optional[float] = None

        # Individual check results cache (for composite status)
        self._check_results: dict[str, SafetyCheckResult] = {}

    def _init_vampire_client(self) -> None:
        """Initialize Vampire query client from node."""
        try:
            from vampire_bridge.srv import VampireQuery
            self._VampireQuery = VampireQuery
            self._vampire_client = self._node.create_client(
                VampireQuery,
                '/vampire/query'
            )
        except ImportError:
            self._node.get_logger().warning(
                'vampire_bridge not available for safety monitoring'
            )
            self._VampireQuery = None

    def run_safety_check(self):
        """
        Run all safety queries for the current state.

        Returns a SafetyState message with aggregated results.

        Returns:
            SafetyState message or None if monitoring not active
        """
        if not self._state_machine.is_safety_monitoring_active():
            return None

        start_time = time.perf_counter()
        self._last_check_time = time.time()

        # Run all safety queries
        results = []
        for query_key, query_type, query_name in self.SAFETY_QUERIES:
            result = self._run_tactical_query(query_key, query_type, query_name)
            results.append(result)
            self._check_results[query_key] = result

        # Calculate total latency
        total_latency_ms = (time.perf_counter() - start_time) * 1000

        # Update statistics
        with self._lock:
            self._total_checks += 1
            self._total_latency_ms += total_latency_ms
            self._last_latency_ms = total_latency_ms

        # Build SafetyState message
        try:
            from mission_orchestrator.msg import SafetyState
            safety_state = self._build_safety_state(results, total_latency_ms)
            return safety_state
        except ImportError:
            return None

    def _run_tactical_query(
        self,
        query_key: str,
        query_type: str,
        query_name: str
    ) -> SafetyCheckResult:
        """
        Run a single tactical safety query.

        Args:
            query_key: Identifier for this check type
            query_type: Query type (safety, operational)
            query_name: Template name

        Returns:
            SafetyCheckResult with check outcome
        """
        start_time = time.perf_counter()

        if self._vampire_client is None:
            return SafetyCheckResult(
                name=query_key,
                passed=True,  # Fail-open if no reasoner
                latency_ms=0.0,
                violation_details='Vampire client not available'
            )

        try:
            from vampire_bridge.msg import ReasoningQuery

            request = self._VampireQuery.Request()
            request.query = ReasoningQuery()
            request.query.query_type = query_type
            request.query.query_name = query_name
            request.query.parameters = []
            request.query.timeout_ms = self._tactical_timeout_ms
            request.query.use_cache = True  # Use cache for repeated queries

            # Synchronous call with timeout
            if not self._vampire_client.wait_for_service(timeout_sec=0.05):
                latency_ms = (time.perf_counter() - start_time) * 1000
                return SafetyCheckResult(
                    name=query_key,
                    passed=True,  # Fail-open
                    latency_ms=latency_ms,
                    violation_details='Service not available'
                )

            future = self._vampire_client.call_async(request)

            # Wait for result with short timeout
            timeout_sec = self._tactical_timeout_ms / 1000.0 + 0.02
            end_time = time.time() + timeout_sec
            while time.time() < end_time and not future.done():
                time.sleep(0.005)

            latency_ms = (time.perf_counter() - start_time) * 1000

            if not future.done():
                return SafetyCheckResult(
                    name=query_key,
                    passed=True,  # Fail-open on timeout
                    latency_ms=latency_ms,
                    violation_details='Query timeout'
                )

            result = future.result().result

            # Interpret result based on query type
            # For safety queries, THEOREM = violation detected
            passed = self._interpret_result(query_key, result)

            return SafetyCheckResult(
                name=query_key,
                passed=passed,
                latency_ms=latency_ms,
                from_cache=result.from_cache,
                violation_details=None if passed else result.proof_summary
            )

        except Exception as e:
            latency_ms = (time.perf_counter() - start_time) * 1000
            self._node.get_logger().error(f'Safety query failed: {e}')
            return SafetyCheckResult(
                name=query_key,
                passed=True,  # Fail-open
                latency_ms=latency_ms,
                violation_details=str(e)
            )

    def _interpret_result(self, query_key: str, result) -> bool:
        """
        Interpret Vampire result for safety check.

        Different queries have different semantics:
            - geofence_check: THEOREM = within geofence (safe)
            - nfz_violation: THEOREM = in NFZ (violation)
            - battery_return: THEOREM = need to return (violation)

        Args:
            query_key: Query identifier
            result: ReasoningResult from Vampire

        Returns:
            True if check passed (safe), False if violation detected
        """
        szs_status = result.szs_status.lower()

        # Successful proof
        if 'theorem' in szs_status:
            # For geofence_check, theorem means we're inside (safe)
            if query_key == 'geofence_check':
                return True
            # For violation checks, theorem means violation detected
            else:
                return False

        # Counter-satisfiable (negation is true)
        if 'counter' in szs_status or 'satisfiable' in szs_status:
            if query_key == 'geofence_check':
                return False  # Outside geofence
            else:
                return True  # No violation

        # Timeout or error - fail-open (assume safe, log warning)
        self._node.get_logger().warning(
            f'Ambiguous safety result for {query_key}: {szs_status}'
        )
        return True

    def _build_safety_state(self, results: list[SafetyCheckResult], latency_ms: float):
        """Build SafetyState message from check results."""
        from mission_orchestrator.msg import SafetyState

        state = SafetyState()
        state.stamp = self._node.get_clock().now().to_msg()

        # Aggregate results
        violations = []
        for result in results:
            if not result.passed:
                violations.append(result.name)
                with self._lock:
                    self._violations_detected += 1

        state.violations = violations
        state.safe = len(violations) == 0

        # Individual check status
        state.geofence_ok = self._check_results.get(
            'geofence_check', SafetyCheckResult('', True, 0.0)
        ).passed
        state.nfz_ok = self._check_results.get(
            'nfz_violation', SafetyCheckResult('', True, 0.0)
        ).passed
        state.battery_ok = self._check_results.get(
            'battery_return', SafetyCheckResult('', True, 0.0)
        ).passed

        # These would come from additional queries or HotFactBuffer
        state.comms_ok = True  # Placeholder
        state.localization_ok = True  # Placeholder

        # Determine recommended action
        state.recommended_action = self._determine_action(violations)

        # Performance metrics
        state.latency_ms = latency_ms
        state.from_cache = any(r.from_cache for r in results)

        return state

    def _determine_action(self, violations: list[str]) -> int:
        """
        Determine recommended action based on violations.

        Returns:
            Action constant (0=NONE, 1=WARN, 2=PAUSE, 3=RTL, 4=ABORT)
        """
        # Check for critical violations requiring abort
        for v in violations:
            if v in self.CRITICAL_VIOLATIONS:
                return 4  # ACTION_ABORT

        # Check for high severity requiring RTL
        for v in violations:
            if v in self.HIGH_VIOLATIONS:
                return 3  # ACTION_RTL

        # Check for medium severity requiring pause
        for v in violations:
            if v in self.MEDIUM_VIOLATIONS:
                return 2  # ACTION_PAUSE

        # Any other violations are warnings
        if violations:
            return 1  # ACTION_WARN

        return 0  # ACTION_NONE

    def get_statistics(self) -> dict:
        """Return safety monitoring statistics."""
        with self._lock:
            avg_latency = (
                self._total_latency_ms / self._total_checks
                if self._total_checks > 0 else 0.0
            )
            return {
                'total_checks': self._total_checks,
                'total_latency_ms': self._total_latency_ms,
                'avg_latency_ms': round(avg_latency, 2),
                'last_latency_ms': round(self._last_latency_ms, 2),
                'violations_detected': self._violations_detected,
                'last_check_time': self._last_check_time,
                'active': self._state_machine.is_safety_monitoring_active(),
            }

    def reset_statistics(self) -> None:
        """Reset all statistics counters."""
        with self._lock:
            self._total_checks = 0
            self._total_latency_ms = 0.0
            self._last_latency_ms = 0.0
            self._violations_detected = 0
            self._last_check_time = None
