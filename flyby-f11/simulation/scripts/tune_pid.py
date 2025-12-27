#!/usr/bin/env python3
"""
PID Tuning Tool for ArduPilot SITL

This tool provides automated PID tuning tests for multicopter configurations
running in ArduPilot SITL with Gazebo simulation.

Features:
- Hover stability tests (position hold)
- Waypoint following tests (trajectory tracking)
- Step response analysis
- Performance metrics (overshoot, settling time, steady-state error)

Author: Finley Holt
Date: 2025-12-27

Requirements:
- Python 3.10+
- pymavlink
- numpy
- matplotlib (optional, for plotting)

Usage:
    python tune_pid.py --test hover --duration 30
    python tune_pid.py --test waypoint --waypoints waypoints.yaml
    python tune_pid.py --test step --axis yaw --magnitude 45
    python tune_pid.py --analyze results.json --plot
"""

import argparse
import json
import logging
import math
import socket
import sys
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Optional

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except ImportError:
    PYMAVLINK_AVAILABLE = False
    print("Warning: pymavlink not available. Install with: pip install pymavlink")

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("Warning: numpy not available. Install with: pip install numpy")

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class TestType(Enum):
    """Available test types."""
    HOVER = "hover"
    WAYPOINT = "waypoint"
    STEP = "step"
    RATE = "rate"


@dataclass
class Position:
    """3D position in NED frame."""
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0

    def distance_to(self, other: 'Position') -> float:
        """Calculate Euclidean distance to another position."""
        return math.sqrt(
            (self.north - other.north) ** 2 +
            (self.east - other.east) ** 2 +
            (self.down - other.down) ** 2
        )

    def horizontal_distance_to(self, other: 'Position') -> float:
        """Calculate horizontal distance to another position."""
        return math.sqrt(
            (self.north - other.north) ** 2 +
            (self.east - other.east) ** 2
        )


@dataclass
class Attitude:
    """Vehicle attitude (roll, pitch, yaw in radians)."""
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class VehicleState:
    """Complete vehicle state snapshot."""
    timestamp: float
    position: Position
    attitude: Attitude
    velocity: Position
    angular_velocity: Attitude
    is_armed: bool = False
    flight_mode: str = ""


@dataclass
class PerformanceMetrics:
    """Performance metrics from a test run."""
    test_type: str
    test_name: str
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())

    # Step response metrics
    rise_time: Optional[float] = None  # Time to reach 90% of setpoint
    settling_time: Optional[float] = None  # Time to stay within 2% of setpoint
    overshoot: Optional[float] = None  # Maximum overshoot percentage
    steady_state_error: Optional[float] = None  # Final error from setpoint

    # Position tracking metrics
    max_position_error: Optional[float] = None  # Maximum position error (m)
    mean_position_error: Optional[float] = None  # Mean position error (m)
    rms_position_error: Optional[float] = None  # RMS position error (m)

    # Attitude tracking metrics
    max_attitude_error: Optional[float] = None  # Maximum attitude error (deg)
    mean_attitude_error: Optional[float] = None  # Mean attitude error (deg)

    # Rate metrics
    max_rate: Optional[float] = None  # Maximum angular rate (deg/s)
    mean_rate: Optional[float] = None  # Mean angular rate (deg/s)

    # General metrics
    test_duration: Optional[float] = None  # Total test time (s)
    samples_collected: Optional[int] = None


@dataclass
class PIDConfig:
    """PID controller configuration."""
    p_gain: float = 0.0
    i_gain: float = 0.0
    d_gain: float = 0.0
    i_max: float = 0.0
    filter_hz: float = 0.0


class MAVLinkConnection:
    """Manages MAVLink connection to ArduPilot SITL."""

    def __init__(
        self,
        connection_string: str = "tcp:127.0.0.1:5760",
        timeout: float = 30.0
    ):
        self.connection_string = connection_string
        self.timeout = timeout
        self.connection = None
        self.target_system = 1
        self.target_component = 1

    def connect(self) -> bool:
        """Establish MAVLink connection."""
        if not PYMAVLINK_AVAILABLE:
            logger.error("pymavlink not available")
            return False

        try:
            logger.info(f"Connecting to {self.connection_string}...")
            self.connection = mavutil.mavlink_connection(
                self.connection_string,
                baud=115200
            )

            # Wait for heartbeat
            logger.info("Waiting for heartbeat...")
            msg = self.connection.wait_heartbeat(timeout=self.timeout)
            if msg:
                self.target_system = msg.get_srcSystem()
                self.target_component = msg.get_srcComponent()
                logger.info(
                    f"Connected to system {self.target_system}, "
                    f"component {self.target_component}"
                )
                return True
            else:
                logger.error("Heartbeat timeout")
                return False

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Close MAVLink connection."""
        if self.connection:
            self.connection.close()
            self.connection = None
            logger.info("Disconnected")

    def get_vehicle_state(self) -> Optional[VehicleState]:
        """Get current vehicle state."""
        if not self.connection:
            return None

        # Request attitude
        attitude = self.connection.messages.get('ATTITUDE')
        # Request local position
        local_pos = self.connection.messages.get('LOCAL_POSITION_NED')
        # Request heartbeat for mode info
        heartbeat = self.connection.messages.get('HEARTBEAT')

        if not all([attitude, local_pos, heartbeat]):
            return None

        return VehicleState(
            timestamp=time.time(),
            position=Position(
                north=local_pos.x,
                east=local_pos.y,
                down=local_pos.z
            ),
            attitude=Attitude(
                roll=attitude.roll,
                pitch=attitude.pitch,
                yaw=attitude.yaw
            ),
            velocity=Position(
                north=local_pos.vx,
                east=local_pos.vy,
                down=local_pos.vz
            ),
            angular_velocity=Attitude(
                roll=attitude.rollspeed,
                pitch=attitude.pitchspeed,
                yaw=attitude.yawspeed
            ),
            is_armed=(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0,
            flight_mode=mavutil.mode_string_v10(heartbeat)
        )

    def send_position_target(self, position: Position, yaw: float = 0.0):
        """Send position setpoint in local NED frame."""
        if not self.connection:
            return

        self.connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (position only)
            position.north,
            position.east,
            position.down,
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            yaw,
            0  # yaw_rate
        )

    def set_mode(self, mode: str) -> bool:
        """Set flight mode."""
        if not self.connection:
            return False

        mode_id = self.connection.mode_mapping().get(mode)
        if mode_id is None:
            logger.error(f"Unknown mode: {mode}")
            return False

        self.connection.set_mode(mode_id)
        return True

    def arm(self) -> bool:
        """Arm the vehicle."""
        if not self.connection:
            return False

        self.connection.arducopter_arm()
        return self.connection.motors_armed_wait(timeout=10)

    def disarm(self) -> bool:
        """Disarm the vehicle."""
        if not self.connection:
            return False

        self.connection.arducopter_disarm()
        return self.connection.motors_disarmed_wait(timeout=10)

    def recv_messages(self, timeout: float = 0.1):
        """Receive and process pending messages."""
        if not self.connection:
            return

        while True:
            msg = self.connection.recv_match(blocking=False, timeout=timeout)
            if msg is None:
                break

    def get_param(self, param_name: str) -> Optional[float]:
        """Get parameter value."""
        if not self.connection:
            return None

        self.connection.param_fetch_one(param_name)
        msg = self.connection.recv_match(
            type='PARAM_VALUE',
            condition=f'PARAM_VALUE.param_id=="{param_name}"',
            timeout=5
        )
        if msg:
            return msg.param_value
        return None

    def set_param(self, param_name: str, value: float) -> bool:
        """Set parameter value."""
        if not self.connection:
            return False

        self.connection.param_set_send(param_name, value)
        return True


class PIDTuner:
    """Main PID tuning test controller."""

    def __init__(self, connection: MAVLinkConnection):
        self.connection = connection
        self.state_history: list[VehicleState] = []
        self.target_history: list[tuple[float, Position]] = []

    def collect_state(self, duration: float, sample_rate: float = 50.0):
        """Collect vehicle state for specified duration."""
        self.state_history.clear()
        interval = 1.0 / sample_rate
        start_time = time.time()

        while time.time() - start_time < duration:
            self.connection.recv_messages()
            state = self.connection.get_vehicle_state()
            if state:
                self.state_history.append(state)
            time.sleep(interval)

        logger.info(f"Collected {len(self.state_history)} state samples")

    def run_hover_test(
        self,
        altitude: float = 5.0,
        duration: float = 30.0
    ) -> PerformanceMetrics:
        """
        Run hover stability test.

        The vehicle takes off to specified altitude and holds position.
        Measures position drift and attitude stability.
        """
        logger.info(f"Starting hover test at {altitude}m for {duration}s")

        target = Position(north=0.0, east=0.0, down=-altitude)
        metrics = PerformanceMetrics(
            test_type="hover",
            test_name=f"hover_{altitude}m_{duration}s"
        )

        # Collect data during hover
        self.target_history.append((time.time(), target))
        self.collect_state(duration)

        # Analyze hover stability
        if len(self.state_history) > 0:
            metrics = self._analyze_position_hold(target, metrics)

        return metrics

    def run_waypoint_test(
        self,
        waypoints: list[Position],
        hold_time: float = 5.0
    ) -> PerformanceMetrics:
        """
        Run waypoint following test.

        The vehicle flies to each waypoint in sequence and holds briefly.
        Measures trajectory tracking accuracy.
        """
        logger.info(f"Starting waypoint test with {len(waypoints)} waypoints")

        metrics = PerformanceMetrics(
            test_type="waypoint",
            test_name=f"waypoint_{len(waypoints)}wp"
        )

        all_errors = []

        for i, wp in enumerate(waypoints):
            logger.info(f"Flying to waypoint {i+1}/{len(waypoints)}: {wp}")

            # Send position target
            self.connection.send_position_target(wp)
            self.target_history.append((time.time(), wp))

            # Wait for arrival and hold
            self.collect_state(hold_time)

            # Calculate errors at this waypoint
            if len(self.state_history) > 0:
                for state in self.state_history[-int(hold_time * 50 * 0.8):]:
                    error = state.position.distance_to(wp)
                    all_errors.append(error)

        # Aggregate metrics
        if NUMPY_AVAILABLE and len(all_errors) > 0:
            metrics.max_position_error = float(np.max(all_errors))
            metrics.mean_position_error = float(np.mean(all_errors))
            metrics.rms_position_error = float(np.sqrt(np.mean(np.array(all_errors) ** 2)))
            metrics.samples_collected = len(all_errors)

        return metrics

    def run_step_response_test(
        self,
        axis: str = "yaw",
        magnitude: float = 45.0,
        settle_time: float = 10.0
    ) -> PerformanceMetrics:
        """
        Run step response test.

        Applies a step input and measures response characteristics.

        Args:
            axis: Response axis ("roll", "pitch", "yaw", "altitude")
            magnitude: Step magnitude (degrees for angles, meters for altitude)
            settle_time: Time to wait for settling
        """
        logger.info(f"Starting step response test: {axis} += {magnitude}")

        metrics = PerformanceMetrics(
            test_type="step",
            test_name=f"step_{axis}_{magnitude}"
        )

        # Get initial state
        self.connection.recv_messages()
        initial_state = self.connection.get_vehicle_state()
        if not initial_state:
            logger.error("Could not get initial state")
            return metrics

        # Apply step and collect response
        self.collect_state(settle_time)

        # Analyze step response
        if len(self.state_history) > 0:
            metrics = self._analyze_step_response(
                axis, magnitude, initial_state, metrics
            )

        return metrics

    def run_rate_test(
        self,
        axis: str = "yaw",
        rate: float = 90.0,
        duration: float = 5.0
    ) -> PerformanceMetrics:
        """
        Run angular rate tracking test.

        Commands a constant angular rate and measures tracking accuracy.

        Args:
            axis: Rate axis ("roll", "pitch", "yaw")
            rate: Commanded rate in deg/s
            duration: Test duration
        """
        logger.info(f"Starting rate test: {axis} rate = {rate} deg/s")

        metrics = PerformanceMetrics(
            test_type="rate",
            test_name=f"rate_{axis}_{rate}dps"
        )

        # Collect rate response
        self.collect_state(duration)

        # Analyze rate tracking
        if len(self.state_history) > 0:
            metrics = self._analyze_rate_tracking(axis, rate, metrics)

        return metrics

    def _analyze_position_hold(
        self,
        target: Position,
        metrics: PerformanceMetrics
    ) -> PerformanceMetrics:
        """Analyze position hold performance."""
        if not NUMPY_AVAILABLE:
            logger.warning("numpy not available for analysis")
            return metrics

        position_errors = []
        attitude_errors = []

        for state in self.state_history:
            # Position error
            pos_error = state.position.distance_to(target)
            position_errors.append(pos_error)

            # Attitude error (deviation from level)
            att_error = math.sqrt(
                state.attitude.roll ** 2 +
                state.attitude.pitch ** 2
            )
            attitude_errors.append(math.degrees(att_error))

        pos_array = np.array(position_errors)
        att_array = np.array(attitude_errors)

        metrics.max_position_error = float(np.max(pos_array))
        metrics.mean_position_error = float(np.mean(pos_array))
        metrics.rms_position_error = float(np.sqrt(np.mean(pos_array ** 2)))
        metrics.steady_state_error = float(np.mean(pos_array[-50:]))  # Last 1 second

        metrics.max_attitude_error = float(np.max(att_array))
        metrics.mean_attitude_error = float(np.mean(att_array))

        metrics.test_duration = (
            self.state_history[-1].timestamp - self.state_history[0].timestamp
        )
        metrics.samples_collected = len(self.state_history)

        return metrics

    def _analyze_step_response(
        self,
        axis: str,
        magnitude: float,
        initial_state: VehicleState,
        metrics: PerformanceMetrics
    ) -> PerformanceMetrics:
        """Analyze step response performance."""
        if not NUMPY_AVAILABLE:
            logger.warning("numpy not available for analysis")
            return metrics

        # Extract response values based on axis
        values = []
        times = []
        initial_value = 0.0

        if axis == "altitude":
            initial_value = -initial_state.position.down
            for state in self.state_history:
                values.append(-state.position.down)
                times.append(state.timestamp)
        elif axis == "yaw":
            initial_value = math.degrees(initial_state.attitude.yaw)
            for state in self.state_history:
                values.append(math.degrees(state.attitude.yaw))
                times.append(state.timestamp)
        elif axis == "roll":
            initial_value = math.degrees(initial_state.attitude.roll)
            for state in self.state_history:
                values.append(math.degrees(state.attitude.roll))
                times.append(state.timestamp)
        elif axis == "pitch":
            initial_value = math.degrees(initial_state.attitude.pitch)
            for state in self.state_history:
                values.append(math.degrees(state.attitude.pitch))
                times.append(state.timestamp)

        values = np.array(values)
        times = np.array(times) - times[0]
        setpoint = initial_value + magnitude

        # Calculate metrics
        # Rise time (10% to 90%)
        threshold_10 = initial_value + 0.1 * magnitude
        threshold_90 = initial_value + 0.9 * magnitude

        try:
            idx_10 = np.where(values >= threshold_10)[0][0]
            idx_90 = np.where(values >= threshold_90)[0][0]
            metrics.rise_time = float(times[idx_90] - times[idx_10])
        except IndexError:
            pass

        # Overshoot
        max_value = np.max(values)
        if magnitude > 0:
            overshoot = (max_value - setpoint) / magnitude * 100
        else:
            overshoot = (setpoint - np.min(values)) / abs(magnitude) * 100
        metrics.overshoot = float(max(0, overshoot))

        # Settling time (2% band)
        settling_band = 0.02 * abs(magnitude)
        settled_mask = np.abs(values - setpoint) <= settling_band
        if np.any(settled_mask):
            # Find first index where it stays settled
            for i in range(len(settled_mask)):
                if np.all(settled_mask[i:]):
                    metrics.settling_time = float(times[i])
                    break

        # Steady-state error
        steady_values = values[-50:]  # Last 1 second at 50Hz
        metrics.steady_state_error = float(np.mean(np.abs(steady_values - setpoint)))

        metrics.test_duration = float(times[-1])
        metrics.samples_collected = len(values)

        return metrics

    def _analyze_rate_tracking(
        self,
        axis: str,
        commanded_rate: float,
        metrics: PerformanceMetrics
    ) -> PerformanceMetrics:
        """Analyze rate tracking performance."""
        if not NUMPY_AVAILABLE:
            logger.warning("numpy not available for analysis")
            return metrics

        # Extract rate values based on axis
        rates = []
        for state in self.state_history:
            if axis == "roll":
                rates.append(math.degrees(state.angular_velocity.roll))
            elif axis == "pitch":
                rates.append(math.degrees(state.angular_velocity.pitch))
            elif axis == "yaw":
                rates.append(math.degrees(state.angular_velocity.yaw))

        rates = np.array(rates)

        metrics.max_rate = float(np.max(np.abs(rates)))
        metrics.mean_rate = float(np.mean(np.abs(rates)))
        metrics.steady_state_error = float(np.mean(np.abs(rates - commanded_rate)))

        metrics.test_duration = (
            self.state_history[-1].timestamp - self.state_history[0].timestamp
        )
        metrics.samples_collected = len(rates)

        return metrics


def save_results(metrics: PerformanceMetrics, output_path: Path):
    """Save test results to JSON file."""
    results = asdict(metrics)
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)
    logger.info(f"Results saved to {output_path}")


def load_results(input_path: Path) -> PerformanceMetrics:
    """Load test results from JSON file."""
    with open(input_path, 'r') as f:
        data = json.load(f)
    return PerformanceMetrics(**data)


def print_metrics(metrics: PerformanceMetrics):
    """Print performance metrics in readable format."""
    print("\n" + "=" * 60)
    print(f"Test Results: {metrics.test_name}")
    print(f"Type: {metrics.test_type}")
    print(f"Timestamp: {metrics.timestamp}")
    print("=" * 60)

    if metrics.rise_time is not None:
        print(f"Rise Time (10%-90%):    {metrics.rise_time:.3f} s")
    if metrics.settling_time is not None:
        print(f"Settling Time (2%):     {metrics.settling_time:.3f} s")
    if metrics.overshoot is not None:
        print(f"Overshoot:              {metrics.overshoot:.2f} %")
    if metrics.steady_state_error is not None:
        print(f"Steady-State Error:     {metrics.steady_state_error:.4f}")

    if metrics.max_position_error is not None:
        print(f"\nMax Position Error:     {metrics.max_position_error:.3f} m")
    if metrics.mean_position_error is not None:
        print(f"Mean Position Error:    {metrics.mean_position_error:.3f} m")
    if metrics.rms_position_error is not None:
        print(f"RMS Position Error:     {metrics.rms_position_error:.3f} m")

    if metrics.max_attitude_error is not None:
        print(f"\nMax Attitude Error:     {metrics.max_attitude_error:.2f} deg")
    if metrics.mean_attitude_error is not None:
        print(f"Mean Attitude Error:    {metrics.mean_attitude_error:.2f} deg")

    if metrics.max_rate is not None:
        print(f"\nMax Angular Rate:       {metrics.max_rate:.2f} deg/s")
    if metrics.mean_rate is not None:
        print(f"Mean Angular Rate:      {metrics.mean_rate:.2f} deg/s")

    if metrics.test_duration is not None:
        print(f"\nTest Duration:          {metrics.test_duration:.2f} s")
    if metrics.samples_collected is not None:
        print(f"Samples Collected:      {metrics.samples_collected}")

    print("=" * 60 + "\n")


def evaluate_tuning(metrics: PerformanceMetrics) -> dict:
    """
    Evaluate PID tuning quality based on metrics.

    Returns a dictionary with pass/fail status and recommendations.
    """
    results = {
        "overall": "PASS",
        "issues": [],
        "recommendations": []
    }

    # Position hold targets
    if metrics.mean_position_error is not None:
        if metrics.mean_position_error > 0.5:
            results["issues"].append("Position error too high")
            results["recommendations"].append(
                "Increase position P gain or reduce D gain"
            )
            results["overall"] = "NEEDS_TUNING"

    # Overshoot targets
    if metrics.overshoot is not None:
        if metrics.overshoot > 10:
            results["issues"].append(f"Overshoot {metrics.overshoot:.1f}% exceeds 10%")
            results["recommendations"].append(
                "Reduce P gain or increase D gain"
            )
            results["overall"] = "NEEDS_TUNING"

    # Settling time targets
    if metrics.settling_time is not None:
        if metrics.settling_time > 2.0:
            results["issues"].append(
                f"Settling time {metrics.settling_time:.2f}s exceeds 2s target"
            )
            results["recommendations"].append(
                "Increase P gain for faster response"
            )
            if results["overall"] == "PASS":
                results["overall"] = "ACCEPTABLE"

    # Steady-state error targets
    if metrics.steady_state_error is not None:
        if metrics.steady_state_error > 0.1:
            results["issues"].append("Steady-state error too high")
            results["recommendations"].append(
                "Increase I gain to reduce steady-state error"
            )
            results["overall"] = "NEEDS_TUNING"

    return results


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="PID Tuning Tool for ArduPilot SITL",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --test hover --duration 30
  %(prog)s --test waypoint --waypoints waypoints.yaml
  %(prog)s --test step --axis yaw --magnitude 45
  %(prog)s --analyze results.json
        """
    )

    parser.add_argument(
        "--test",
        type=str,
        choices=["hover", "waypoint", "step", "rate"],
        help="Test type to run"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Test duration in seconds (default: 30)"
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=5.0,
        help="Hover altitude in meters (default: 5)"
    )
    parser.add_argument(
        "--axis",
        type=str,
        choices=["roll", "pitch", "yaw", "altitude"],
        default="yaw",
        help="Axis for step/rate test (default: yaw)"
    )
    parser.add_argument(
        "--magnitude",
        type=float,
        default=45.0,
        help="Step magnitude in degrees/meters (default: 45)"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=90.0,
        help="Rate command in deg/s (default: 90)"
    )
    parser.add_argument(
        "--waypoints",
        type=str,
        help="Path to waypoints YAML file"
    )
    parser.add_argument(
        "--connection",
        type=str,
        default="tcp:127.0.0.1:5760",
        help="MAVLink connection string (default: tcp:127.0.0.1:5760)"
    )
    parser.add_argument(
        "--output",
        type=str,
        help="Output file for results (default: results_<timestamp>.json)"
    )
    parser.add_argument(
        "--analyze",
        type=str,
        help="Analyze existing results file"
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate plots (requires matplotlib)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print test configuration without running"
    )

    args = parser.parse_args()

    # Analyze existing results
    if args.analyze:
        results_path = Path(args.analyze)
        if not results_path.exists():
            logger.error(f"Results file not found: {args.analyze}")
            sys.exit(1)

        metrics = load_results(results_path)
        print_metrics(metrics)

        evaluation = evaluate_tuning(metrics)
        print(f"Evaluation: {evaluation['overall']}")
        if evaluation['issues']:
            print("\nIssues:")
            for issue in evaluation['issues']:
                print(f"  - {issue}")
        if evaluation['recommendations']:
            print("\nRecommendations:")
            for rec in evaluation['recommendations']:
                print(f"  - {rec}")
        return

    # Validate test arguments
    if not args.test:
        parser.print_help()
        sys.exit(1)

    # Dry run mode
    if args.dry_run:
        print("\n=== Dry Run Configuration ===")
        print(f"Test Type: {args.test}")
        print(f"Connection: {args.connection}")
        print(f"Duration: {args.duration}s")
        if args.test == "hover":
            print(f"Altitude: {args.altitude}m")
        elif args.test == "step":
            print(f"Axis: {args.axis}")
            print(f"Magnitude: {args.magnitude}")
        elif args.test == "rate":
            print(f"Axis: {args.axis}")
            print(f"Rate: {args.rate} deg/s")
        return

    # Check dependencies
    if not PYMAVLINK_AVAILABLE:
        logger.error("pymavlink is required. Install with: pip install pymavlink")
        sys.exit(1)

    if not NUMPY_AVAILABLE:
        logger.warning("numpy is not available. Analysis will be limited.")

    # Connect to SITL
    connection = MAVLinkConnection(args.connection)
    if not connection.connect():
        logger.error("Failed to connect to SITL")
        sys.exit(1)

    try:
        tuner = PIDTuner(connection)

        # Run selected test
        if args.test == "hover":
            metrics = tuner.run_hover_test(
                altitude=args.altitude,
                duration=args.duration
            )
        elif args.test == "waypoint":
            if args.waypoints and YAML_AVAILABLE:
                with open(args.waypoints) as f:
                    wp_data = yaml.safe_load(f)
                waypoints = [
                    Position(**wp) for wp in wp_data.get("waypoints", [])
                ]
            else:
                # Default test waypoints
                waypoints = [
                    Position(north=10, east=0, down=-5),
                    Position(north=10, east=10, down=-5),
                    Position(north=0, east=10, down=-5),
                    Position(north=0, east=0, down=-5),
                ]
            metrics = tuner.run_waypoint_test(waypoints)
        elif args.test == "step":
            metrics = tuner.run_step_response_test(
                axis=args.axis,
                magnitude=args.magnitude,
                settle_time=args.duration
            )
        elif args.test == "rate":
            metrics = tuner.run_rate_test(
                axis=args.axis,
                rate=args.rate,
                duration=args.duration
            )

        # Print results
        print_metrics(metrics)

        # Evaluate tuning
        evaluation = evaluate_tuning(metrics)
        print(f"Evaluation: {evaluation['overall']}")

        # Save results
        if args.output:
            output_path = Path(args.output)
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = Path(f"results_{args.test}_{timestamp}.json")

        save_results(metrics, output_path)

    finally:
        connection.disconnect()


if __name__ == "__main__":
    main()
