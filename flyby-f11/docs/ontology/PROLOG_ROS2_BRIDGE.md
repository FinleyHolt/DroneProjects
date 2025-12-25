# Prolog-ROS 2 Bridge Integration Guide

## Overview

This guide explains how to integrate SWI-Prolog with ROS 2 Python nodes using PySwip. The Prolog knowledge base runs onboard the Jetson Orin NX and provides real-time reasoning for the UAV's autonomy stack.

## Architecture

```
ROS 2 Nodes (Python)
    |
    | PySwip
    v
SWI-Prolog Knowledge Base
    |
    +-- Static Rules (compiled from SUMO)
    +-- Dynamic Facts (sensor data, mission state)
```

### Data Flow

1. **Sensors → ROS 2 Topics**: Altitude, battery, GPS, obstacles
2. **ROS 2 Node → Prolog**: Assert sensor data as facts
3. **Prolog Reasoning**: Query safety rules, mission constraints
4. **Prolog → ROS 2 Node**: Return decisions (e.g., "return_to_home")
5. **ROS 2 Node → Action Server**: Execute actions

## Installation

### SWI-Prolog

Already covered in `SUMO_TO_PROLOG.md`. Ensure SWI-Prolog is installed:

```bash
swipl --version
# SWI-Prolog version 9.x.x
```

### PySwip

Install PySwip (Python bridge to SWI-Prolog):

```bash
pip install pyswip
```

Or build from source for ARM:

```bash
git clone https://github.com/yuce/pyswip.git
cd pyswip
pip install .
```

### Verify Installation

```python
from pyswip import Prolog
prolog = Prolog()
print(list(prolog.query("member(X, [1,2,3])")))
# [{'X': 1}, {'X': 2}, {'X': 3}]
```

## Example: Safety Monitor Node

### Prolog Knowledge Base

File: `/home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src/flyby_f11_mission/kb/uav_safety.pl`

```prolog
% uav_safety.pl

% Ontology
subclass(quadcopter, unmanned_aerial_vehicle).
subclass(unmanned_aerial_vehicle, aircraft).

isa(X, Y) :- subclass(X, Y).
isa(X, Z) :- subclass(X, Y), isa(Y, Z).

% Dynamic facts (updated from ROS 2)
:- dynamic altitude/2.
:- dynamic battery_level/2.
:- dynamic ground_speed/2.
:- dynamic distance_to_obstacle/2.
:- dynamic gps_status/2.
:- dynamic current_phase/2.
:- dynamic instance/2.

% UAV instance
instance(flyby_f11, quadcopter).

% Safety rules

% Altitude constraint
violates_safety(UAV, altitude_limit) :-
    instance(UAV, unmanned_aerial_vehicle),
    altitude(UAV, ALT),
    ALT > 120.

% Battery safety
requires_action(UAV, return_to_home, high) :-
    instance(UAV, unmanned_aerial_vehicle),
    battery_level(UAV, LEVEL),
    LEVEL < 20.

requires_action(UAV, return_to_home, medium) :-
    instance(UAV, unmanned_aerial_vehicle),
    battery_level(UAV, LEVEL),
    LEVEL >= 20,
    LEVEL < 30.

% Obstacle proximity
requires_action(UAV, emergency_stop, critical) :-
    instance(UAV, unmanned_aerial_vehicle),
    distance_to_obstacle(UAV, DIST),
    DIST < 2.

requires_action(UAV, slow_down, medium) :-
    instance(UAV, unmanned_aerial_vehicle),
    distance_to_obstacle(UAV, DIST),
    DIST >= 2,
    DIST < 5.

% GPS loss handling
requires_action(UAV, hover, high) :-
    instance(UAV, unmanned_aerial_vehicle),
    gps_status(UAV, lost),
    \+ current_phase(UAV, landing).

% Safe to land
can_land(UAV) :-
    instance(UAV, unmanned_aerial_vehicle),
    ground_speed(UAV, SPEED),
    SPEED < 2,
    distance_to_obstacle(UAV, DIST),
    DIST > 5.

% Overall safety check
is_safe(UAV) :-
    instance(UAV, unmanned_aerial_vehicle),
    \+ violates_safety(UAV, _),
    \+ requires_action(UAV, _, critical).

% Get all required actions
all_required_actions(UAV, ACTIONS) :-
    instance(UAV, unmanned_aerial_vehicle),
    findall(action(ACTION, PRIORITY),
            requires_action(UAV, ACTION, PRIORITY),
            ACTIONS).
```

### ROS 2 Safety Monitor Node

File: `/home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src/flyby_f11_mission/flyby_f11_mission/safety_monitor.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pyswip import Prolog
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import PoseStamped
import os

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Initialize Prolog
        self.prolog = Prolog()
        kb_path = os.path.join(
            os.path.dirname(__file__),
            '../kb/uav_safety.pl'
        )
        self.prolog.consult(kb_path)
        self.get_logger().info(f"Loaded Prolog KB from {kb_path}")

        # Subscriptions
        self.create_subscription(
            Float32,
            '/telemetry/altitude',
            self.altitude_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/telemetry/battery_percent',
            self.battery_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/perception/nearest_obstacle_distance',
            self.obstacle_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/telemetry/gps_ok',
            self.gps_callback,
            10
        )

        self.create_subscription(
            String,
            '/autonomy/flight_phase',
            self.phase_callback,
            10
        )

        # Publishers
        self.safety_status_pub = self.create_publisher(
            Bool,
            '/safety/is_safe',
            10
        )

        self.required_actions_pub = self.create_publisher(
            String,
            '/safety/required_actions',
            10
        )

        # Timer for periodic safety checks
        self.create_timer(0.5, self.check_safety)  # 2 Hz

        self.get_logger().info("Safety monitor initialized")

    def altitude_callback(self, msg):
        """Update altitude in Prolog KB"""
        self.update_prolog_fact('altitude', 'flyby_f11', msg.data)

    def battery_callback(self, msg):
        """Update battery level in Prolog KB"""
        self.update_prolog_fact('battery_level', 'flyby_f11', msg.data)

    def obstacle_callback(self, msg):
        """Update obstacle distance in Prolog KB"""
        self.update_prolog_fact('distance_to_obstacle', 'flyby_f11', msg.data)

    def gps_callback(self, msg):
        """Update GPS status in Prolog KB"""
        status = 'ok' if msg.data else 'lost'
        self.update_prolog_fact('gps_status', 'flyby_f11', status, is_string=True)

    def phase_callback(self, msg):
        """Update current flight phase in Prolog KB"""
        self.update_prolog_fact('current_phase', 'flyby_f11', msg.data, is_string=True)

    def update_prolog_fact(self, predicate, uav, value, is_string=False):
        """
        Update a Prolog fact by retracting old value and asserting new.
        """
        try:
            # Retract old fact
            self.prolog.retractall(f"{predicate}({uav}, _)")

            # Assert new fact
            if is_string:
                self.prolog.assertz(f"{predicate}({uav}, {value})")
            else:
                self.prolog.assertz(f"{predicate}({uav}, {value})")

        except Exception as e:
            self.get_logger().error(f"Failed to update {predicate}: {e}")

    def check_safety(self):
        """
        Query Prolog for safety status and required actions.
        """
        # Check if UAV is safe
        try:
            safe_query = list(self.prolog.query("is_safe(flyby_f11)"))
            is_safe = len(safe_query) > 0

            # Publish safety status
            msg = Bool()
            msg.data = is_safe
            self.safety_status_pub.publish(msg)

            # Get required actions
            actions_query = list(self.prolog.query(
                "all_required_actions(flyby_f11, ACTIONS)"
            ))

            if actions_query:
                actions = actions_query[0]['ACTIONS']
                self.get_logger().info(f"Required actions: {actions}")

                # Publish actions
                actions_msg = String()
                actions_msg.data = str(actions)
                self.required_actions_pub.publish(actions_msg)

                # Execute critical actions
                self.execute_critical_actions(actions)

        except Exception as e:
            self.get_logger().error(f"Safety check failed: {e}")

    def execute_critical_actions(self, actions):
        """
        Execute critical actions (emergency stop, return to home, etc.)
        """
        # Parse actions list
        # actions is a Prolog list: [action(emergency_stop, critical), ...]

        # For now, just log (TODO: call action servers)
        for action in actions:
            self.get_logger().warn(f"CRITICAL ACTION REQUIRED: {action}")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File

File: `/home/finley/Github/DroneProjects/flyby-f11/ros2_ws/src/flyby_f11_bringup/launch/safety_monitor.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flyby_f11_mission',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])
```

## Example: Mission Planner Query Node

### Prolog Knowledge Base

File: `mission_planning.pl`

```prolog
% mission_planning.pl

:- dynamic waypoint_visited/1.
:- dynamic current_waypoint/1.

% Mission waypoints
waypoint(wp1). waypoint(wp2). waypoint(wp3). waypoint(wp4).

% Waypoint sequence
next_waypoint(wp1, wp2).
next_waypoint(wp2, wp3).
next_waypoint(wp3, wp4).

% Transitive closure
can_reach(X, Y) :- next_waypoint(X, Y).
can_reach(X, Z) :- next_waypoint(X, Y), can_reach(Y, Z).

% Mission completion
all_waypoints_visited :-
    forall(waypoint(WP), waypoint_visited(WP)).

% Next unvisited waypoint
next_unvisited(NEXT) :-
    current_waypoint(CURRENT),
    next_waypoint(CURRENT, NEXT),
    \+ waypoint_visited(NEXT).

% No more waypoints
mission_complete :-
    current_waypoint(CURRENT),
    \+ next_waypoint(CURRENT, _).
```

### ROS 2 Mission Planner Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pyswip import Prolog
from std_msgs.msg import String

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')

        # Initialize Prolog
        self.prolog = Prolog()
        self.prolog.consult("mission_planning.pl")

        # Initialize mission state
        self.prolog.assertz("current_waypoint(wp1)")

        # Publisher
        self.waypoint_pub = self.create_publisher(
            String,
            '/mission/next_waypoint',
            10
        )

        # Timer
        self.create_timer(5.0, self.plan_next_waypoint)

        self.get_logger().info("Mission planner initialized")

    def plan_next_waypoint(self):
        """Query Prolog for next waypoint"""
        try:
            # Check if mission complete
            complete = list(self.prolog.query("mission_complete"))
            if complete:
                self.get_logger().info("Mission complete!")
                return

            # Get next unvisited waypoint
            result = list(self.prolog.query("next_unvisited(NEXT)"))
            if result:
                next_wp = result[0]['NEXT']
                self.get_logger().info(f"Next waypoint: {next_wp}")

                # Publish next waypoint
                msg = String()
                msg.data = next_wp
                self.waypoint_pub.publish(msg)

                # Update Prolog state (mark as visited)
                self.prolog.assertz(f"waypoint_visited({next_wp})")
                self.prolog.retractall("current_waypoint(_)")
                self.prolog.assertz(f"current_waypoint({next_wp})")

        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced: Complex Prolog Queries

### Passing ROS Messages to Prolog

```python
from geometry_msgs.msg import Point

def point_to_prolog(point):
    """Convert ROS Point to Prolog term"""
    return f"point({point.x}, {point.y}, {point.z})"

# Assert waypoint location
waypoint_location = Point(x=10.5, y=20.3, z=15.0)
prolog.assertz(f"waypoint_location(wp1, {point_to_prolog(waypoint_location)})")

# Query: is waypoint in no-fly zone?
result = list(prolog.query("is_in_no_fly_zone(wp1)"))
```

### Retrieving Complex Results

```python
# Query: find all safe waypoints
safe_waypoints = []
for result in prolog.query("waypoint(WP), is_safe_waypoint(WP)"):
    safe_waypoints.append(result['WP'])

print(f"Safe waypoints: {safe_waypoints}")
```

## Performance Optimization

### 1. Batch Fact Updates

Instead of updating one fact at a time, batch updates:

```python
def update_sensor_batch(self, altitude, battery, obstacle_dist):
    """Batch update multiple facts"""
    try:
        self.prolog.retractall("altitude(flyby_f11, _)")
        self.prolog.retractall("battery_level(flyby_f11, _)")
        self.prolog.retractall("distance_to_obstacle(flyby_f11, _)")

        self.prolog.assertz(f"altitude(flyby_f11, {altitude})")
        self.prolog.assertz(f"battery_level(flyby_f11, {battery})")
        self.prolog.assertz(f"distance_to_obstacle(flyby_f11, {obstacle_dist})")
    except Exception as e:
        self.get_logger().error(f"Batch update failed: {e}")
```

### 2. Use Tabling for Expensive Queries

In Prolog KB:

```prolog
:- table reachable_from/2.

reachable_from(X, Y) :- edge(X, Y).
reachable_from(X, Z) :- edge(X, Y), reachable_from(Y, Z).
```

### 3. Pre-Compile Prolog KB

```bash
# Compile to Quick Load File (QLF)
swipl -g "qcompile('uav_safety.pl')" -t halt

# Load QLF (faster than .pl)
prolog.consult("uav_safety.qlf")
```

## Debugging

### Enable Prolog Tracing

```python
# In Python
prolog.query("trace")  # Enable trace mode
result = list(prolog.query("is_safe(flyby_f11)"))
prolog.query("notrace")  # Disable trace
```

### Query Current Facts

```python
# List all altitude facts
for result in prolog.query("altitude(UAV, ALT)"):
    print(f"{result['UAV']} altitude: {result['ALT']}")
```

### Test Prolog KB Standalone

```bash
swipl uav_safety.pl
?- assert(altitude(flyby_f11, 50)).
?- assert(battery_level(flyby_f11, 80)).
?- is_safe(flyby_f11).
true.
```

## Error Handling

### Handle Prolog Exceptions

```python
from pyswip.prolog import PrologError

try:
    result = list(prolog.query("is_safe(flyby_f11)"))
except PrologError as e:
    self.get_logger().error(f"Prolog query failed: {e}")
```

### Validate Facts Before Asserting

```python
def assert_altitude(self, uav, altitude):
    """Assert altitude with validation"""
    if not isinstance(altitude, (int, float)):
        self.get_logger().error(f"Invalid altitude: {altitude}")
        return

    if altitude < 0 or altitude > 500:
        self.get_logger().warn(f"Altitude out of range: {altitude}")

    try:
        self.prolog.retractall(f"altitude({uav}, _)")
        self.prolog.assertz(f"altitude({uav}, {altitude})")
    except Exception as e:
        self.get_logger().error(f"Failed to assert altitude: {e}")
```

## Deployment to Jetson Orin NX

### Docker Container

Include SWI-Prolog and PySwip in Dockerfile:

```dockerfile
# In flyby-f11/docker/Dockerfile

FROM ros:humble

# Install SWI-Prolog
RUN apt-get update && apt-get install -y \
    software-properties-common && \
    apt-add-repository ppa:swi-prolog/stable && \
    apt-get update && \
    apt-get install -y swi-prolog

# Install PySwip
RUN pip install pyswip

# Copy ROS 2 workspace
COPY ros2_ws /workspace/ros2_ws

# Build ROS 2 packages
WORKDIR /workspace/ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Source workspace
RUN echo "source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["ros2", "launch", "flyby_f11_bringup", "safety_monitor.launch.py"]
```

### Systemd Service (Alternative)

```ini
# /etc/systemd/system/prolog-safety-monitor.service

[Unit]
Description=Prolog Safety Monitor
After=network.target

[Service]
Type=simple
User=flyby
WorkingDirectory=/home/flyby/ros2_ws
ExecStart=/usr/bin/python3 /home/flyby/ros2_ws/src/flyby_f11_mission/flyby_f11_mission/safety_monitor.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

## Integration with Behavior Trees

Combine Prolog reasoning with BehaviorTree.CPP:

```python
# In BT action node
class CheckSafetyCondition(Node):
    def __init__(self, name, prolog):
        super().__init__(name)
        self.prolog = prolog

    def tick(self):
        # Query Prolog
        safe = list(self.prolog.query("is_safe(flyby_f11)"))
        if safe:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
```

## Next Steps

1. **Create Prolog KB** for flyby-f11 domain (see `examples/compiled_rules.pl`)
2. **Implement safety monitor node** (see `examples/prolog_query_node.py`)
3. **Test in simulation** with Gazebo SITL
4. **Deploy to Jetson** and validate performance
5. **Integrate with behavior trees** for mission execution

## References

- PySwip GitHub: https://github.com/yuce/pyswip
- SWI-Prolog Python Interface: https://www.swi-prolog.org/pldoc/man?section=pyswip
- ROS 2 Python Client: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
