# Grounding Nodes Implementation Guide

## Overview

Grounding nodes are ROS 2 Python nodes that convert perception outputs (subsymbolic) into symbolic facts for the Prolog reasoning engine. They form the critical bridge between the vision pipeline and ontological reasoning.

## Architecture

```
Vision Pipeline (Subsymbolic)
         │
         ▼
    ROS 2 Topics
    /perception/detections
         │
         ▼
  Grounding Nodes
  (Symbol Grounding)
         │
         ▼
   Prolog Facts
   (Symbolic)
         │
         ▼
 Reasoning Engine
```

## Core Concepts

### Symbol Grounding Problem
The challenge of connecting low-level perception data (pixels, bounding boxes) to high-level symbolic representations (concepts, relations, rules).

**Example**:
- **Subsymbolic**: YOLO detection [class=15, bbox=(100,200,150,250), conf=0.87]
- **Symbolic**: `object(person_1, person, [x:125, y:225], 0.87, t:123456)`

### Grounding Pipeline

1. **Perception**: Vision model outputs detections
2. **Filtering**: Remove low-confidence or irrelevant detections
3. **Spatial Reasoning**: Compute spatial relationships (above, left_of, near)
4. **Fact Generation**: Convert to Prolog facts
5. **Knowledge Base Update**: Assert facts via PySwip

## Implementation Pattern

### Basic Grounding Node Structure

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pyswip import Prolog
from flyby_f11_interfaces.msg import PerceptionObject, PerceptionObjectArray

class ObjectGroundingNode(Node):
    """
    Grounding node that converts object detections to Prolog facts.
    """

    def __init__(self):
        super().__init__('object_grounding_node')

        # Initialize Prolog engine
        self.prolog = Prolog()
        self.load_ontology()

        # ROS 2 subscribers
        self.perception_sub = self.create_subscription(
            PerceptionObjectArray,
            '/perception/objects',
            self.perception_callback,
            10
        )

        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_objects', 20)

        self.get_logger().info('Object Grounding Node initialized')

    def load_ontology(self):
        """Load Prolog ontology file."""
        ontology_path = '/path/to/ontology.pl'
        self.prolog.consult(ontology_path)
        self.get_logger().info(f'Loaded ontology: {ontology_path}')

    def perception_callback(self, msg):
        """
        Callback for perception detections.
        Converts detections to Prolog facts.
        """
        # Clear previous object facts
        self.prolog.retractall("detected_object(_, _, _, _, _)")

        # Filter detections by confidence
        conf_threshold = self.get_parameter('confidence_threshold').value
        filtered_objects = [
            obj for obj in msg.objects
            if obj.confidence >= conf_threshold
        ]

        # Ground each object
        for obj in filtered_objects:
            self.ground_object(obj, msg.header.stamp)

        self.get_logger().info(
            f'Grounded {len(filtered_objects)} objects'
        )

    def ground_object(self, obj, timestamp):
        """
        Ground a single object detection to Prolog fact.

        Args:
            obj: PerceptionObject message
            timestamp: Detection timestamp
        """
        # Extract object properties
        obj_id = f"obj_{obj.track_id}"
        obj_class = obj.class_name
        x_center = (obj.bbox.x_min + obj.bbox.x_max) / 2
        y_center = (obj.bbox.y_min + obj.bbox.y_max) / 2
        confidence = obj.confidence

        # Assert Prolog fact
        fact = (
            f"detected_object("
            f"'{obj_id}', "
            f"'{obj_class}', "
            f"[{x_center}, {y_center}], "
            f"{confidence}, "
            f"{timestamp.sec}"
            f")"
        )

        self.prolog.assertz(fact)
        self.get_logger().debug(f'Asserted: {fact}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectGroundingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Grounding Techniques

### 1. Spatial Relation Grounding

Compute spatial relationships between detected objects:

```python
def compute_spatial_relations(self, objects):
    """
    Compute spatial relations between objects.
    Assert facts like: left_of(obj_1, obj_2), above(obj_3, obj_4)
    """
    # Clear previous spatial relations
    self.prolog.retractall("left_of(_, _)")
    self.prolog.retractall("right_of(_, _)")
    self.prolog.retractall("above(_, _)")
    self.prolog.retractall("below(_, _)")

    for i, obj1 in enumerate(objects):
        for obj2 in objects[i+1:]:
            # Compute centers
            x1 = (obj1.bbox.x_min + obj1.bbox.x_max) / 2
            y1 = (obj1.bbox.y_min + obj1.bbox.y_max) / 2
            x2 = (obj2.bbox.x_min + obj2.bbox.x_max) / 2
            y2 = (obj2.bbox.y_min + obj2.bbox.y_max) / 2

            # Horizontal relations
            if x1 < x2 - 50:  # threshold for "left of"
                self.prolog.assertz(
                    f"left_of(obj_{obj1.track_id}, obj_{obj2.track_id})"
                )
            elif x1 > x2 + 50:
                self.prolog.assertz(
                    f"right_of(obj_{obj1.track_id}, obj_{obj2.track_id})"
                )

            # Vertical relations
            if y1 < y2 - 50:  # threshold for "above"
                self.prolog.assertz(
                    f"above(obj_{obj1.track_id}, obj_{obj2.track_id})"
                )
            elif y1 > y2 + 50:
                self.prolog.assertz(
                    f"below(obj_{obj1.track_id}, obj_{obj2.track_id})"
                )
```

### 2. Temporal Grounding

Track object persistence across frames:

```python
class TemporalGroundingNode(Node):
    """
    Grounding node with temporal reasoning.
    Tracks object persistence and state changes.
    """

    def __init__(self):
        super().__init__('temporal_grounding_node')
        self.prolog = Prolog()
        self.object_history = {}  # track_id -> [timestamps]
        self.load_ontology()

        self.perception_sub = self.create_subscription(
            PerceptionObjectArray,
            '/perception/objects',
            self.perception_callback,
            10
        )

    def perception_callback(self, msg):
        """Track temporal object properties."""
        current_time = msg.header.stamp.sec

        # Update object history
        for obj in msg.objects:
            if obj.track_id not in self.object_history:
                self.object_history[obj.track_id] = []
            self.object_history[obj.track_id].append(current_time)

        # Assert temporal facts
        for track_id, timestamps in self.object_history.items():
            duration = timestamps[-1] - timestamps[0]

            # Object has been present for > 5 seconds
            if duration > 5:
                self.prolog.assertz(
                    f"persistent_object(obj_{track_id}, {duration})"
                )

            # Object just appeared (first detection)
            if len(timestamps) == 1:
                self.prolog.assertz(
                    f"new_object(obj_{track_id}, {current_time})"
                )
```

### 3. Scene Graph Grounding

Generate complete scene graph with objects and relations:

```python
def ground_scene_graph(self, msg):
    """
    Ground complete scene graph.

    Generates facts:
    - object(ID, Class, Location, Confidence)
    - spatial_relation(Relation, Obj1, Obj2)
    - scene_context(Context)
    """
    # Clear previous scene
    self.prolog.retractall("object(_, _, _, _)")
    self.prolog.retractall("spatial_relation(_, _, _)")
    self.prolog.retractall("scene_context(_)")

    # Ground objects
    for obj in msg.objects:
        obj_id = f"obj_{obj.track_id}"
        location = [
            (obj.bbox.x_min + obj.bbox.x_max) / 2,
            (obj.bbox.y_min + obj.bbox.y_max) / 2
        ]
        self.prolog.assertz(
            f"object('{obj_id}', '{obj.class_name}', "
            f"{location}, {obj.confidence})"
        )

    # Compute and ground spatial relations
    self.ground_spatial_relations(msg.objects)

    # Infer scene context
    scene_type = self.infer_scene_context(msg.objects)
    self.prolog.assertz(f"scene_context('{scene_type}')")

def infer_scene_context(self, objects):
    """
    Infer scene context based on detected objects.

    Returns:
        str: Scene context (indoor, outdoor, urban, etc.)
    """
    class_counts = {}
    for obj in objects:
        class_counts[obj.class_name] = class_counts.get(obj.class_name, 0) + 1

    # Simple heuristic-based context inference
    if 'car' in class_counts and class_counts['car'] > 3:
        return 'urban_outdoor'
    elif 'chair' in class_counts or 'desk' in class_counts:
        return 'indoor_office'
    elif 'tree' in class_counts:
        return 'outdoor_natural'
    else:
        return 'unknown'
```

## Message Definitions

### PerceptionObject.msg
```
# Single object detection

std_msgs/Header header
uint32 track_id
string class_name
float32 confidence
BoundingBox bbox
geometry_msgs/Point3D world_position  # 3D position in world frame
```

### BoundingBox.msg
```
# 2D bounding box in image coordinates

float32 x_min
float32 y_min
float32 x_max
float32 y_max
```

### PerceptionObjectArray.msg
```
# Array of object detections

std_msgs/Header header
PerceptionObject[] objects
```

## Launch Configuration

### grounding_nodes.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Confidence threshold for object grounding'
        ),

        # Object grounding node
        Node(
            package='flyby_f11_grounding',
            executable='object_grounding_node',
            name='object_grounding',
            output='screen',
            parameters=[{
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'max_objects': 20,
                'ontology_path': '/path/to/ontology.pl'
            }]
        ),

        # Spatial relation grounding node
        Node(
            package='flyby_f11_grounding',
            executable='spatial_grounding_node',
            name='spatial_grounding',
            output='screen'
        ),

        # Temporal grounding node
        Node(
            package='flyby_f11_grounding',
            executable='temporal_grounding_node',
            name='temporal_grounding',
            output='screen'
        )
    ])
```

## Best Practices

### 1. Fact Management
- **Clear old facts**: Use `retractall()` to remove outdated facts before asserting new ones
- **Unique IDs**: Use track IDs from object tracker for consistent object identity
- **Timestamps**: Include timestamps for temporal reasoning

### 2. Performance Optimization
- **Batch assertions**: Group related facts to reduce Prolog overhead
- **Confidence filtering**: Only ground high-confidence detections
- **Spatial indexing**: Use efficient spatial data structures for relation computation

### 3. Error Handling
```python
def safe_assert(self, fact):
    """Safely assert fact with error handling."""
    try:
        self.prolog.assertz(fact)
    except Exception as e:
        self.get_logger().error(f'Failed to assert fact: {fact}. Error: {e}')
```

### 4. Debugging
```python
def debug_knowledge_base(self):
    """Print current knowledge base state."""
    results = list(self.prolog.query("detected_object(ID, Class, Loc, Conf, T)"))
    self.get_logger().info(f'Knowledge base contains {len(results)} objects:')
    for result in results:
        self.get_logger().info(f'  {result}')
```

## Testing

### Unit Testing
```python
import unittest
from object_grounding_node import ObjectGroundingNode

class TestObjectGrounding(unittest.TestCase):
    def setUp(self):
        self.node = ObjectGroundingNode()

    def test_ground_object(self):
        """Test object grounding."""
        obj = create_mock_object(class_name='person', confidence=0.85)
        self.node.ground_object(obj, timestamp=123456)

        # Query Prolog to verify fact was asserted
        results = list(self.node.prolog.query(
            "detected_object(ID, 'person', _, Conf, _)"
        ))
        self.assertEqual(len(results), 1)
        self.assertGreaterEqual(results[0]['Conf'], 0.85)
```

### Integration Testing
```bash
# Launch perception + grounding nodes
ros2 launch flyby_f11_bringup perception_grounding.launch.py

# Verify facts are being asserted
ros2 service call /reasoning/query flyby_f11_interfaces/srv/PrologQuery \
  "{query: 'detected_object(ID, Class, _, _, _)'}"
```

## Common Pitfalls

1. **Memory Leaks**: Always retract old facts before asserting new ones
2. **ID Collisions**: Use unique IDs (track IDs, not detection indices)
3. **Prolog Syntax**: Ensure proper quoting for string literals
4. **Performance**: Minimize Prolog queries in high-frequency callbacks

## References

- ROS 2 Python Client Library: `/home/finley/Github/DroneProjects/flyby-f11/docs/ros2/client_libs/rclpy_api.html`
- PySwip Documentation: `/home/finley/Github/DroneProjects/flyby-f11/docs/ros2/pyswip/pyswip/README.md`
- Custom Message Definitions: `CUSTOM_MESSAGES.md`
