#!/usr/bin/env python3
"""
vampire_node.py - Main ROS 2 node for Vampire theorem prover integration

Provides a ROS 2 service interface to the Vampire theorem prover for
ontological reasoning queries. Supports caching, timeout management,
query template substitution, and concurrent query execution.

The node uses a ThreadPoolExecutor to execute Vampire subprocess calls
concurrently, allowing multiple service requests to be processed in
parallel. Cache lookups remain synchronous (fast path) while only
Vampire subprocess execution is offloaded to the thread pool.

Usage:
    ros2 run vampire_bridge vampire_node.py

Service:
    /vampire/query (vampire_bridge/srv/VampireQuery)

Parameters:
    vampire_binary: Path to Vampire executable
    ontology_path: Path to UAV domain ontology
    templates_path: Path to benchmark query templates
    default_timeout_ms: Default query timeout
    cache_size: Maximum cache entries
    cache_ttl_safety: TTL for safety queries (seconds)
    cache_ttl_operational: TTL for operational queries (seconds)
    cache_ttl_planning: TTL for planning queries (seconds)
    max_concurrent_queries: Maximum concurrent Vampire subprocess executions (default: 4)
"""

import subprocess
import tempfile
import time
import threading
from concurrent.futures import ThreadPoolExecutor, Future
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

# These imports will work after package is built with colcon
# For now, we import from local modules
from vampire_bridge.cache_manager import QueryCache
from vampire_bridge.query_builder import QueryBuilder
from vampire_bridge.result_parser import ResultParser


class VampireNode(Node):
    """
    ROS 2 node providing Vampire theorem prover query service.

    Handles synchronous reasoning queries for the UAV autonomy system,
    with caching for repeated queries to meet latency requirements.
    """

    def __init__(self):
        super().__init__('vampire_reasoning')

        # Declare parameters
        self._declare_parameters()

        # Get parameter values
        self.vampire_binary = self.get_parameter('vampire_binary').value
        self.ontology_path = self.get_parameter('ontology_path').value
        self.templates_path = self.get_parameter('templates_path').value
        self.default_timeout_ms = self.get_parameter('default_timeout_ms').value

        # Initialize components
        self._init_cache()
        self.query_builder = QueryBuilder(
            templates_path=self.templates_path,
            ontology_path=self.ontology_path
        )
        self.result_parser = ResultParser()

        # Initialize thread pool for concurrent Vampire execution
        self.max_concurrent_queries = self.get_parameter('max_concurrent_queries').value
        self._executor = ThreadPoolExecutor(
            max_workers=self.max_concurrent_queries,
            thread_name_prefix='vampire_worker'
        )

        # Concurrency metrics (thread-safe)
        self._metrics_lock = threading.Lock()
        self._pending_queries = 0
        self._in_flight_queries = 0
        self._completed_queries = 0
        self._total_concurrent_latency_ms = 0.0

        # Verify Vampire binary exists
        if not Path(self.vampire_binary).exists():
            self.get_logger().warning(
                f"Vampire binary not found at {self.vampire_binary}. "
                "Queries will fail until binary is available."
            )

        # Create service
        # Import service type - will be available after colcon build
        try:
            from vampire_bridge.srv import VampireQuery
            from vampire_bridge.msg import ReasoningResult
            self.ReasoningResult = ReasoningResult

            self.query_srv = self.create_service(
                VampireQuery,
                'vampire/query',
                self.handle_query
            )
            self.get_logger().info('VampireQuery service created at /vampire/query')
        except ImportError:
            self.get_logger().error(
                "Failed to import vampire_bridge messages. "
                "Run 'colcon build' to generate message types."
            )
            self.query_srv = None

        # Statistics
        self.queries_processed = 0
        self.total_latency_ms = 0.0

        self.get_logger().info(
            f'Vampire reasoning node initialized. '
            f'Binary: {self.vampire_binary}, '
            f'Cache size: {self.get_parameter("cache_size").value}, '
            f'Max concurrent queries: {self.max_concurrent_queries}'
        )

    def _declare_parameters(self):
        """Declare all node parameters with defaults."""
        self.declare_parameter('vampire_binary', '/usr/local/bin/vampire')
        self.declare_parameter('ontology_path', '/workspace/ontology/planning_mode')
        self.declare_parameter('templates_path', '/workspace/ontology/evaluation/benchmark_queries')
        self.declare_parameter('default_timeout_ms', 100)
        self.declare_parameter('cache_size', 1000)
        self.declare_parameter('cache_ttl_safety', 10.0)
        self.declare_parameter('cache_ttl_operational', 60.0)
        self.declare_parameter('cache_ttl_planning', 300.0)
        self.declare_parameter('max_concurrent_queries', 4)

    def _init_cache(self):
        """Initialize query cache with configured TTL values."""
        ttl_by_type = {
            'safety': self.get_parameter('cache_ttl_safety').value,
            'operational': self.get_parameter('cache_ttl_operational').value,
            'planning': self.get_parameter('cache_ttl_planning').value,
        }
        self.cache = QueryCache(
            max_size=self.get_parameter('cache_size').value,
            ttl_by_type=ttl_by_type
        )

    def handle_query(self, request, response):
        """
        Handle VampireQuery service request.

        Cache lookups are synchronous (fast path). Vampire subprocess execution
        is submitted to a thread pool for concurrent execution. The service
        call blocks until the result is available, but multiple concurrent
        service calls can execute Vampire in parallel.

        Args:
            request: VampireQuery request with ReasoningQuery
            response: VampireQuery response to populate

        Returns:
            Populated response with ReasoningResult
        """
        query = request.query
        start_time = time.perf_counter()

        self.get_logger().debug(
            f"Query received: type={query.query_type}, name={query.query_name}"
        )

        # Check cache first (synchronous fast path)
        if query.use_cache:
            cached = self.cache.get(
                query.query_type,
                query.query_name,
                list(query.parameters)
            )
            if cached is not None:
                elapsed_ms = (time.perf_counter() - start_time) * 1000
                response.result = self._create_result_msg(
                    cached,
                    from_cache=True,
                    latency_ms=elapsed_ms
                )
                self.get_logger().debug(f"Cache hit: {elapsed_ms:.2f}ms")
                return response

        # Submit Vampire query to thread pool for concurrent execution
        with self._metrics_lock:
            self._pending_queries += 1

        try:
            future = self._executor.submit(self._execute_vampire_concurrent, query)

            # Update metrics: pending -> in_flight
            with self._metrics_lock:
                self._pending_queries -= 1
                self._in_flight_queries += 1

            # Wait for result (blocks this service call, but other calls can proceed)
            result = future.result()

        except Exception as e:
            # Update metrics on failure
            with self._metrics_lock:
                if self._pending_queries > 0:
                    self._pending_queries -= 1
                elif self._in_flight_queries > 0:
                    self._in_flight_queries -= 1

            self.get_logger().error(f"Vampire execution failed: {e}")
            response.result = self._create_error_result(str(e))
            return response

        finally:
            # Update metrics: in_flight -> completed
            with self._metrics_lock:
                if self._in_flight_queries > 0:
                    self._in_flight_queries -= 1
                self._completed_queries += 1

        elapsed_ms = (time.perf_counter() - start_time) * 1000

        # Cache successful results
        if result.success and query.use_cache:
            self.cache.put(
                query.query_type,
                query.query_name,
                list(query.parameters),
                result
            )

        # Update statistics (thread-safe)
        with self._metrics_lock:
            self._total_concurrent_latency_ms += elapsed_ms

        self.queries_processed += 1
        self.total_latency_ms += elapsed_ms

        # Create response
        response.result = self._create_result_msg(
            result,
            from_cache=False,
            latency_ms=elapsed_ms
        )

        self.get_logger().debug(
            f"Query completed: status={result.szs_status}, latency={elapsed_ms:.2f}ms, "
            f"in_flight={self._in_flight_queries}"
        )

        return response

    def _execute_vampire(self, query):
        """
        Execute Vampire theorem prover on the query.

        Args:
            query: ReasoningQuery message

        Returns:
            VampireResult from parser
        """
        # Build TPTP query
        try:
            tptp_content = self.query_builder.build_query(
                query.query_type,
                query.query_name,
                list(query.parameters),
                query.raw_tptp if query.raw_tptp else None
            )
        except FileNotFoundError as e:
            self.get_logger().error(f"Template not found: {e}")
            raise

        # Write to temporary file
        with tempfile.NamedTemporaryFile(
            mode='w',
            suffix='.tptp',
            delete=False
        ) as f:
            f.write(tptp_content)
            query_file = f.name

        try:
            # Calculate timeout
            timeout_ms = query.timeout_ms if query.timeout_ms > 0 else self.default_timeout_ms
            timeout_sec = timeout_ms / 1000.0

            # Execute Vampire
            start = time.perf_counter()
            proc = subprocess.run(
                [
                    self.vampire_binary,
                    '--input_syntax', 'tptp',
                    '--time_limit', str(int(timeout_sec) + 1),
                    query_file
                ],
                capture_output=True,
                text=True,
                timeout=timeout_sec + 2  # Allow slightly more than Vampire's limit
            )
            elapsed_ms = (time.perf_counter() - start) * 1000

            # Parse result
            result = self.result_parser.parse(
                proc.stdout,
                proc.stderr,
                execution_time_ms=elapsed_ms
            )
            return result

        except subprocess.TimeoutExpired:
            from vampire_bridge.result_parser import VampireResult
            return VampireResult(
                szs_status='Timeout',
                success=False,
                proof_summary='Query exceeded timeout limit',
                violated_axioms=[],
                latency_ms=timeout_ms,
                raw_output='',
                error_message=f'Timeout after {timeout_ms}ms'
            )

        finally:
            # Clean up temp file
            Path(query_file).unlink(missing_ok=True)

    def _execute_vampire_concurrent(self, query):
        """
        Thread-safe wrapper for Vampire execution.

        This method is designed to be called from the ThreadPoolExecutor.
        It ensures proper temp file cleanup even when multiple queries
        execute concurrently. Each thread gets its own temp file with
        a unique name (via NamedTemporaryFile).

        Args:
            query: ReasoningQuery message

        Returns:
            VampireResult from parser

        Note:
            The underlying _execute_vampire method already uses
            NamedTemporaryFile which generates unique filenames per call,
            so concurrent execution is safe. The finally block ensures
            cleanup even if the thread is interrupted.
        """
        # Delegate to the existing implementation
        # NamedTemporaryFile creates unique files per call, making this thread-safe
        return self._execute_vampire(query)

    def _create_result_msg(self, result, from_cache: bool, latency_ms: float):
        """Create ReasoningResult message from VampireResult."""
        msg = self.ReasoningResult()
        msg.success = result.success
        msg.szs_status = result.szs_status
        msg.proof_summary = result.proof_summary
        msg.violated_axioms = result.violated_axioms
        msg.latency_ms = latency_ms
        msg.from_cache = from_cache
        msg.vampire_output = result.raw_output[:5000]  # Truncate for message
        msg.error_message = result.error_message
        return msg

    def _create_error_result(self, error_message: str):
        """Create error ReasoningResult message."""
        msg = self.ReasoningResult()
        msg.success = False
        msg.szs_status = 'Error'
        msg.proof_summary = 'Execution error'
        msg.violated_axioms = []
        msg.latency_ms = 0.0
        msg.from_cache = False
        msg.vampire_output = ''
        msg.error_message = error_message[:500]
        return msg

    def get_statistics(self) -> dict:
        """Get node performance statistics including concurrency metrics."""
        cache_stats = self.cache.get_stats()
        avg_latency = (
            self.total_latency_ms / self.queries_processed
            if self.queries_processed > 0 else 0.0
        )

        # Get concurrency metrics (thread-safe)
        with self._metrics_lock:
            pending = self._pending_queries
            in_flight = self._in_flight_queries
            completed = self._completed_queries
            total_concurrent_latency = self._total_concurrent_latency_ms

        avg_concurrent_latency = (
            total_concurrent_latency / completed
            if completed > 0 else 0.0
        )

        return {
            'queries_processed': self.queries_processed,
            'average_latency_ms': round(avg_latency, 2),
            'cache': cache_stats,
            'concurrency': {
                'max_concurrent_queries': self.max_concurrent_queries,
                'pending_queries': pending,
                'in_flight_queries': in_flight,
                'completed_queries': completed,
                'average_concurrent_latency_ms': round(avg_concurrent_latency, 2),
            },
        }

    def shutdown(self) -> None:
        """
        Gracefully shutdown the node and thread pool.

        Waits for pending queries to complete before shutting down
        the thread pool executor.
        """
        self.get_logger().info('Shutting down thread pool executor...')

        # Shutdown thread pool, waiting for pending work to complete
        self._executor.shutdown(wait=True)

        with self._metrics_lock:
            self.get_logger().info(
                f'Thread pool shutdown complete. '
                f'Completed queries: {self._completed_queries}'
            )


def main(args=None):
    """Main entry point for vampire_node."""
    rclpy.init(args=args)

    node = VampireNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down vampire_reasoning node')
    finally:
        stats = node.get_statistics()
        node.get_logger().info(f'Final statistics: {stats}')
        node.shutdown()  # Gracefully shutdown thread pool
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
