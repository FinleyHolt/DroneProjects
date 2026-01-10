"""
ROS 2 Ontology Controller Node for Flyby F-11 UAV.

Wraps the OntologyBehaviorController in a ROS 2 node interface.
Monitors UAV state and publishes behavior commands when ontology axioms trigger.

Topics:
    Subscribed:
        /uav/state (flyby_msgs/UAVState) - UAV state from state estimator
        /perception/detections (flyby_msgs/DetectionArray) - Person detections

    Published:
        /autonomy/behavior_command (flyby_msgs/BehaviorCommand) - Active behavior
        /autonomy/flight_phase (flyby_msgs/FlightPhase) - Current flight phase
        /cmd_vel (geometry_msgs/Twist) - Velocity command to flight controller

    Services:
        /autonomy/arm (std_srvs/Trigger) - Arm the drone
        /autonomy/reset (std_srvs/Trigger) - Reset controller state
"""

import numpy as np
import time
from dataclasses import dataclass
from typing import Optional, List
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Point, Vector3
from std_srvs.srv import Trigger

from flyby_msgs.msg import (
    UAVState,
    FlightPhase,
    BehaviorCommand,
    DetectionArray,
)


class OntologyBehavior(Enum):
    """Behaviors triggered by ontology axioms."""
    NONE = auto()
    TAKEOFF = auto()
    RTL = auto()
    EMERGENCY_LAND = auto()
    HOVER = auto()
    GEOFENCE_RECOVERY = auto()
    NFZ_AVOIDANCE = auto()
    PERSON_AVOIDANCE = auto()


class AxiomPriority(Enum):
    """Priority levels for axioms."""
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4


@dataclass
class ControllerConfig:
    """Configuration for ontology controller."""
    # Takeoff
    takeoff_altitude: float = 25.0
    takeoff_ascent_rate: float = 2.0
    takeoff_altitude_tolerance: float = 1.0

    # Battery (distance-based calculation)
    battery_per_km: float = 2.5
    battery_headroom: float = 10.0
    battery_emergency_threshold: float = 15.0
    min_reserve: float = 10.0

    # Person safety
    person_critical_distance: float = 10.0
    person_warning_distance: float = 15.0

    # RTL
    rtl_switch_to_landing_distance: float = 3.0
    rtl_max_horizontal_speed: float = 4.0

    # Velocity limits
    max_horizontal_velocity: float = 5.0
    max_vertical_velocity: float = 3.0


class OntologyControllerNode(Node):
    """ROS 2 node for ontology-driven behavior control."""

    def __init__(self):
        super().__init__('ontology_controller_node')

        # Declare parameters
        self.declare_parameter('takeoff_altitude', 25.0)
        self.declare_parameter('battery_per_km', 2.5)
        self.declare_parameter('battery_headroom', 10.0)
        self.declare_parameter('person_critical_distance', 10.0)
        self.declare_parameter('control_rate', 60.0)

        # Load config
        self.config = ControllerConfig(
            takeoff_altitude=self.get_parameter('takeoff_altitude').value,
            battery_per_km=self.get_parameter('battery_per_km').value,
            battery_headroom=self.get_parameter('battery_headroom').value,
            person_critical_distance=self.get_parameter('person_critical_distance').value,
        )
        control_rate = self.get_parameter('control_rate').value

        # State
        self.latest_uav_state: Optional[UAVState] = None
        self.latest_detections: Optional[DetectionArray] = None
        self.closest_person_distance: float = float('inf')
        self.home_position = np.zeros(3)

        # Behavior state
        self.active_behavior = OntologyBehavior.NONE
        self.behavior_priority = AxiomPriority.LOW
        self.behavior_reason = ""
        self.behavior_start_time: float = 0.0

        # Flight phase state
        self._flight_phase = FlightPhase.PREFLIGHT
        self._armed = False
        self._takeoff_complete = False
        self._rtl_committed = False
        self._emergency_triggered = False

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.uav_state_sub = self.create_subscription(
            UAVState, '/uav/state', self._uav_state_callback, sensor_qos
        )
        self.detection_sub = self.create_subscription(
            DetectionArray, '/perception/detections', self._detection_callback, 10
        )

        # Publishers
        self.behavior_pub = self.create_publisher(BehaviorCommand, '/autonomy/behavior_command', 10)
        self.phase_pub = self.create_publisher(FlightPhase, '/autonomy/flight_phase', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Services
        self.arm_srv = self.create_service(Trigger, '/autonomy/arm', self._arm_callback)
        self.reset_srv = self.create_service(Trigger, '/autonomy/reset', self._reset_callback)

        # Control timer
        self.control_timer = self.create_timer(1.0 / control_rate, self._control_callback)

        self.get_logger().info(
            f'Ontology Controller Node initialized:\n'
            f'  Takeoff altitude: {self.config.takeoff_altitude}m\n'
            f'  Person critical distance: {self.config.person_critical_distance}m\n'
            f'  Control rate: {control_rate}Hz'
        )

    def _uav_state_callback(self, msg: UAVState):
        """Store latest UAV state."""
        self.latest_uav_state = msg

        # Update home position on first valid state
        if np.linalg.norm(self.home_position) < 0.1:
            self.home_position = np.array([msg.position.x, msg.position.y, 0.0])

    def _detection_callback(self, msg: DetectionArray):
        """Process detections for person proximity."""
        self.latest_detections = msg

        # Find closest person
        self.closest_person_distance = float('inf')
        for det in msg.detections:
            if det.ontology_class == 'Person' and det.distance > 0:
                if det.distance < self.closest_person_distance:
                    self.closest_person_distance = det.distance

    def _arm_callback(self, request, response):
        """Handle arm service request."""
        if self._armed:
            response.success = True
            response.message = "Already armed"
        else:
            self._armed = True
            self._flight_phase = FlightPhase.ARMED
            response.success = True
            response.message = "Armed successfully"
            self.get_logger().info("UAV armed via service")
        return response

    def _reset_callback(self, request, response):
        """Reset controller state."""
        self._armed = False
        self._takeoff_complete = False
        self._rtl_committed = False
        self._emergency_triggered = False
        self._flight_phase = FlightPhase.PREFLIGHT
        self.active_behavior = OntologyBehavior.NONE
        response.success = True
        response.message = "Controller reset"
        self.get_logger().info("Controller state reset")
        return response

    def _control_callback(self):
        """Main control loop - check axioms and execute behaviors."""
        if self.latest_uav_state is None:
            return

        state = self.latest_uav_state
        now = time.time()

        # Update internal flight phase from state
        if state.flight_phase:
            self._flight_phase = state.flight_phase.phase

        # Check axioms in priority order
        behavior_cmd = self._check_axioms(state, now)

        # Execute behavior and get velocity command
        velocity = None
        if behavior_cmd is not None:
            velocity = self._execute_behavior(behavior_cmd, state)
            self._publish_behavior(behavior_cmd, velocity)
        else:
            # No active behavior - RL is in control
            self._publish_no_behavior()

        # Publish velocity if computed
        if velocity is not None:
            self._publish_velocity(velocity)

        # Publish flight phase
        self._publish_phase(state)

    def _check_axioms(self, state: UAVState, now: float) -> Optional[BehaviorCommand]:
        """Check ontology axioms and return behavior command if triggered."""

        # 1. Emergency checks (CRITICAL priority)
        if self._check_emergency_conditions(state):
            return self._create_command(
                OntologyBehavior.EMERGENCY_LAND,
                AxiomPriority.CRITICAL,
                "mustLand",
                "Critical failure detected"
            )

        # 2. Ontology-controlled takeoff
        if not self._takeoff_complete and self._armed:
            if state.altitude < self.config.takeoff_altitude - self.config.takeoff_altitude_tolerance:
                return self._create_command(
                    OntologyBehavior.TAKEOFF,
                    AxiomPriority.HIGH,
                    "takeoffSequence",
                    f"Ascending to {self.config.takeoff_altitude}m"
                )
            else:
                self._takeoff_complete = True
                self._flight_phase = FlightPhase.MISSION_EXECUTION
                self.get_logger().info("Takeoff complete, entering MISSION_EXECUTION")

        # 3. RTL hysteresis - once committed, stay committed
        if self._rtl_committed:
            return self._create_command(
                OntologyBehavior.RTL,
                AxiomPriority.HIGH,
                "rtlCommitted",
                "RTL in progress"
            )

        # 4. Battery reserve axiom
        required_reserve = self._compute_battery_reserve(state)
        if state.battery_percent < required_reserve:
            self._rtl_committed = True
            return self._create_command(
                OntologyBehavior.RTL,
                AxiomPriority.HIGH,
                "batteryReserveReturn",
                f"Battery {state.battery_percent:.1f}% < required {required_reserve:.1f}%"
            )

        # 5. Comms denied battery check
        if not state.comms_ok and state.battery_percent < 30.0:
            self._rtl_committed = True
            return self._create_command(
                OntologyBehavior.RTL,
                AxiomPriority.HIGH,
                "commsDeniedBatteryReserve",
                "Comms denied + battery < 30%"
            )

        # 6. Person proximity check
        if self.closest_person_distance < self.config.person_critical_distance:
            return self._create_command(
                OntologyBehavior.HOVER,
                AxiomPriority.CRITICAL,
                "minPersonDistance",
                f"Person at {self.closest_person_distance:.1f}m (critical)"
            )

        return None

    def _check_emergency_conditions(self, state: UAVState) -> bool:
        """Check for conditions requiring immediate landing."""
        if state.battery_percent < self.config.battery_emergency_threshold:
            self._emergency_triggered = True
            return True
        if not state.localization_ok:
            self._emergency_triggered = True
            return True
        return False

    def _compute_battery_reserve(self, state: UAVState) -> float:
        """Compute required battery reserve based on distance to home."""
        distance_km = state.distance_to_home / 1000.0
        altitude_cost = state.altitude * 0.1  # 0.1% per meter altitude

        reserve = (distance_km * self.config.battery_per_km +
                  self.config.battery_headroom +
                  altitude_cost)

        # Add buffer for comms denied
        if not state.comms_ok:
            reserve += 10.0

        return max(reserve, self.config.min_reserve)

    def _execute_behavior(self, cmd: BehaviorCommand, state: UAVState) -> Optional[np.ndarray]:
        """Execute behavior and return velocity setpoint."""
        if cmd.behavior_type == BehaviorCommand.TAKEOFF:
            return self._execute_takeoff(state)
        elif cmd.behavior_type == BehaviorCommand.RTL:
            return self._execute_rtl(state)
        elif cmd.behavior_type == BehaviorCommand.EMERGENCY_LAND:
            return self._execute_emergency_land(state)
        elif cmd.behavior_type == BehaviorCommand.HOVER:
            return np.zeros(4)  # Zero velocity
        return None

    def _execute_takeoff(self, state: UAVState) -> np.ndarray:
        """Execute takeoff - ascend at constant rate."""
        return np.array([0.0, 0.0, self.config.takeoff_ascent_rate, 0.0])

    def _execute_rtl(self, state: UAVState) -> np.ndarray:
        """Execute RTL - return to home position."""
        # Calculate vector to home
        pos = np.array([state.position.x, state.position.y, state.position.z])
        to_home = self.home_position[:2] - pos[:2]
        dist_to_home = np.linalg.norm(to_home)

        if dist_to_home < self.config.rtl_switch_to_landing_distance:
            # Close to home, descend
            self._flight_phase = FlightPhase.LANDING
            return np.array([0.0, 0.0, -self.config.max_vertical_velocity, 0.0])

        # Move toward home
        direction = to_home / (dist_to_home + 1e-6)
        speed = min(self.config.rtl_max_horizontal_speed, dist_to_home)
        vx = direction[0] * speed
        vy = direction[1] * speed

        return np.array([vx, vy, 0.0, 0.0])

    def _execute_emergency_land(self, state: UAVState) -> np.ndarray:
        """Execute emergency landing - immediate descent."""
        return np.array([0.0, 0.0, -self.config.max_vertical_velocity, 0.0])

    def _create_command(
        self,
        behavior: OntologyBehavior,
        priority: AxiomPriority,
        axiom: str,
        reason: str
    ) -> BehaviorCommand:
        """Create behavior command message."""
        cmd = BehaviorCommand()
        cmd.stamp = self.get_clock().now().to_msg()

        # Map enum to message constants
        behavior_map = {
            OntologyBehavior.NONE: BehaviorCommand.NONE,
            OntologyBehavior.TAKEOFF: BehaviorCommand.TAKEOFF,
            OntologyBehavior.RTL: BehaviorCommand.RTL,
            OntologyBehavior.EMERGENCY_LAND: BehaviorCommand.EMERGENCY_LAND,
            OntologyBehavior.HOVER: BehaviorCommand.HOVER,
            OntologyBehavior.GEOFENCE_RECOVERY: BehaviorCommand.GEOFENCE_RECOVERY,
            OntologyBehavior.NFZ_AVOIDANCE: BehaviorCommand.NFZ_AVOIDANCE,
            OntologyBehavior.PERSON_AVOIDANCE: BehaviorCommand.PERSON_AVOIDANCE,
        }
        cmd.behavior_type = behavior_map.get(behavior, BehaviorCommand.NONE)

        priority_map = {
            AxiomPriority.LOW: BehaviorCommand.PRIORITY_LOW,
            AxiomPriority.MEDIUM: BehaviorCommand.PRIORITY_MEDIUM,
            AxiomPriority.HIGH: BehaviorCommand.PRIORITY_HIGH,
            AxiomPriority.CRITICAL: BehaviorCommand.PRIORITY_CRITICAL,
        }
        cmd.priority = priority_map.get(priority, BehaviorCommand.PRIORITY_LOW)

        cmd.axiom_name = axiom
        cmd.trigger_reason = reason
        cmd.active = True

        return cmd

    def _publish_behavior(self, cmd: BehaviorCommand, velocity: Optional[np.ndarray]):
        """Publish behavior command."""
        if velocity is not None:
            cmd.velocity_setpoint.linear.x = velocity[0]
            cmd.velocity_setpoint.linear.y = velocity[1]
            cmd.velocity_setpoint.linear.z = velocity[2]
            if len(velocity) > 3:
                cmd.velocity_setpoint.angular.z = velocity[3]
            cmd.velocity_valid = True

        self.behavior_pub.publish(cmd)

    def _publish_no_behavior(self):
        """Publish that no ontology behavior is active."""
        cmd = BehaviorCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.behavior_type = BehaviorCommand.NONE
        cmd.active = False
        self.behavior_pub.publish(cmd)

    def _publish_velocity(self, velocity: np.ndarray):
        """Publish velocity command."""
        msg = Twist()
        msg.linear.x = float(velocity[0])
        msg.linear.y = float(velocity[1])
        msg.linear.z = float(velocity[2])
        if len(velocity) > 3:
            msg.angular.z = float(velocity[3])
        self.cmd_vel_pub.publish(msg)

    def _publish_phase(self, state: UAVState):
        """Publish current flight phase."""
        msg = FlightPhase()
        msg.stamp = self.get_clock().now().to_msg()
        msg.phase = self._flight_phase
        msg.target_altitude = self.config.takeoff_altitude
        msg.phase_complete = self._takeoff_complete
        self.phase_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OntologyControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
