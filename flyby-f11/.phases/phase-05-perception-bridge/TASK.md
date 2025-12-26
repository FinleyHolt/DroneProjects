# Phase 5: Perception-to-Reasoning Bridge

## Overview

Build the symbolic abstraction layer that translates sub-symbolic vision model outputs (bounding boxes, segmentation masks, depth maps) into symbolic TPTP facts for ontological reasoning via Vampire. This bridges the gap between perception and reasoning.

**Architectural Context:** Based on Phase 3 evaluation, we use Vampire directly for all reasoning. Perception outputs are converted to TPTP format for Vampire queries (no Prolog translation).

## Human Description

Vision models output sub-symbolic representations that the Vampire theorem prover cannot directly process. This phase creates ROS 2 "grounding nodes" that:

1. Take perception outputs (YOLO detections, segmentation, depth)
2. Convert them to ontology-aligned TPTP facts
3. Compute spatial relations (distance, between, northOf)
4. Detect temporal events (enters zone, loitering)
5. Publish facts for Vampire queries via the `vampire_bridge` from Phase 4

This is the critical bridge that enables the ontology to reason about what the drone perceives in real-time.

### Integration with Tiered Safety Architecture

| Tier | Layer | Function | Perception Role |
|------|-------|----------|-----------------|
| 1 | Classical Control | PID, motor control | None |
| 2 | Pre-computed Safety | Obstacle buffers | Depth map → costmap (not symbolic) |
| 3 | Tactical Reasoning | NFZ, battery | Grounded facts → Vampire (~50ms) |
| 4 | Mission Planning | Route planning | Full state → Vampire (~1s) |

**This phase handles Tier 3 and 4 perception grounding.**

## AI Agent Instructions

### Prerequisites
- Phase 4 completed (Vampire runtime with ROS 2 bridge)
- Understanding of ROS 2 message types
- Familiarity with computer vision outputs
- Knowledge of TPTP format for Vampire

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

#### 1. Create Perception Grounding Package

**Location:** `ros2_ws/src/perception_grounding/`

**Package Structure:**
```
perception_grounding/
├── CMakeLists.txt
├── package.xml
├── perception_grounding/
│   ├── __init__.py
│   ├── object_grounding_node.py
│   ├── terrain_grounding_node.py
│   ├── spatial_relation_node.py
│   ├── event_detection_node.py
│   └── tptp_fact_builder.py      # TPTP format generation
├── msg/
│   ├── GroundedFact.msg
│   ├── GroundedObject.msg
│   ├── SpatialRelation.msg
│   └── DetectedEvent.msg
├── config/
│   ├── grounding_params.yaml
│   └── ontology_class_mapping.yaml  # YOLO class → ontology concept
├── launch/
│   └── perception_grounding.launch.py
└── test/
    ├── test_object_grounding.py
    └── test_tptp_generation.py
```

#### 2. Implement TPTP Fact Builder

**Core Module for Vampire Integration:**

```python
# perception_grounding/tptp_fact_builder.py
"""
Builds TPTP-format facts from perception data for Vampire queries.
"""

from dataclasses import dataclass
from typing import List, Optional
import hashlib

@dataclass
class TPTPFact:
    """Represents a TPTP fact for Vampire."""
    predicate: str
    arguments: List[str]
    fact_id: str

    def to_tptp(self) -> str:
        """Convert to TPTP format."""
        args = ', '.join(self.arguments)
        return f"fof({self.fact_id}, axiom, {self.predicate}({args}))."

class TPTPFactBuilder:
    """Builds TPTP facts from perception grounding."""

    def __init__(self, ontology_namespace: str = 'uav'):
        self.namespace = ontology_namespace
        self._fact_counter = 0

    def _generate_id(self, prefix: str) -> str:
        self._fact_counter += 1
        return f"{prefix}_{self._fact_counter}"

    def object_type(self, object_id: str, class_name: str) -> TPTPFact:
        """Create objectType fact: objectType(obj_123, person)"""
        return TPTPFact(
            predicate='objectType',
            arguments=[object_id, class_name.lower()],
            fact_id=self._generate_id('obj_type')
        )

    def position(self, entity_id: str, x: float, y: float, z: float) -> TPTPFact:
        """Create position fact: position(obj_123, 45.2, -122.1, 100.0)"""
        return TPTPFact(
            predicate='position',
            arguments=[entity_id, f'{x:.2f}', f'{y:.2f}', f'{z:.2f}'],
            fact_id=self._generate_id('pos')
        )

    def distance(self, entity1: str, entity2: str, dist: float) -> TPTPFact:
        """Create distance fact: distance(drone, obj_123, 5.2)"""
        return TPTPFact(
            predicate='distance',
            arguments=[entity1, entity2, f'{dist:.2f}'],
            fact_id=self._generate_id('dist')
        )

    def in_region(self, entity_id: str, region_id: str) -> TPTPFact:
        """Create inRegion fact: inRegion(drone, nfz_zone_1)"""
        return TPTPFact(
            predicate='inRegion',
            arguments=[entity_id, region_id],
            fact_id=self._generate_id('region')
        )

    def terrain_type(self, region_id: str, terrain: str) -> TPTPFact:
        """Create terrainType fact: terrainType(region_5, water)"""
        return TPTPFact(
            predicate='terrainType',
            arguments=[region_id, terrain.lower()],
            fact_id=self._generate_id('terrain')
        )

    def between(self, entity: str, entity1: str, entity2: str) -> TPTPFact:
        """Create between fact: between(drone, obstacle_1, waypoint_A)"""
        return TPTPFact(
            predicate='between',
            arguments=[entity, entity1, entity2],
            fact_id=self._generate_id('between')
        )

    def enters_zone(self, entity_id: str, zone_id: str, timestamp: float) -> TPTPFact:
        """Create enters event: enters(obj_123, nfz_zone_1)"""
        return TPTPFact(
            predicate='enters',
            arguments=[entity_id, zone_id],
            fact_id=self._generate_id('event_enter')
        )

    def battery_level(self, uav_id: str, percentage: float) -> TPTPFact:
        """Create battery fact: batteryLevel(flyby_f11, 68.5)"""
        return TPTPFact(
            predicate='batteryLevel',
            arguments=[uav_id, f'{percentage:.1f}'],
            fact_id=self._generate_id('battery')
        )

    def build_query_file(self, facts: List[TPTPFact], query: str) -> str:
        """Build complete TPTP file with facts and conjecture."""
        lines = [
            "% Auto-generated perception facts for Vampire query",
            "% Source: perception_grounding package",
            ""
        ]

        # Add all facts
        for fact in facts:
            lines.append(fact.to_tptp())

        lines.append("")
        lines.append(f"% Query conjecture")
        lines.append(query)

        return '\n'.join(lines)
```

#### 3. Implement ObjectGroundingNode

```python
# perception_grounding/object_grounding_node.py
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from perception_grounding.msg import GroundedObject
from perception_grounding.tptp_fact_builder import TPTPFactBuilder
import yaml

class ObjectGroundingNode(Node):
    def __init__(self):
        super().__init__('object_grounding')

        # Load ontology class mapping (YOLO class → ontology concept)
        self.declare_parameter('class_mapping_file', '')
        mapping_file = self.get_parameter('class_mapping_file').value
        with open(mapping_file) as f:
            self.class_mapping = yaml.safe_load(f)

        self.fact_builder = TPTPFactBuilder()
        self.object_counter = 0

        # Subscribe to YOLO detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )

        # Publish grounded objects
        self.grounded_pub = self.create_publisher(
            GroundedObject,
            '/perception/grounded_objects',
            10
        )

        # Publish TPTP facts for vampire_bridge
        self.fact_pub = self.create_publisher(
            String,
            '/perception/tptp_facts',
            10
        )

        self.get_logger().info('Object grounding node initialized')

    def detection_callback(self, msg):
        for detection in msg.detections:
            # Map YOLO class to ontology concept
            yolo_class = detection.results[0].hypothesis.class_id
            if yolo_class not in self.class_mapping:
                continue

            ontology_class = self.class_mapping[yolo_class]

            # Generate stable object ID
            self.object_counter += 1
            object_id = f"obj_{self.object_counter}"

            # Create TPTP facts
            type_fact = self.fact_builder.object_type(object_id, ontology_class)
            confidence = detection.results[0].hypothesis.score

            # Publish grounded object
            grounded = GroundedObject()
            grounded.object_id = object_id
            grounded.ontology_class = ontology_class
            grounded.confidence = confidence
            grounded.tptp_fact = type_fact.to_tptp()
            self.grounded_pub.publish(grounded)

            # Publish TPTP fact
            self.fact_pub.publish(String(data=type_fact.to_tptp()))
```

#### 4. Implement SpatialRelationGroundingNode

```python
# perception_grounding/spatial_relation_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from perception_grounding.msg import SpatialRelation, GroundedObject
from perception_grounding.tptp_fact_builder import TPTPFactBuilder
import numpy as np

class SpatialRelationNode(Node):
    def __init__(self):
        super().__init__('spatial_relation_grounding')

        self.fact_builder = TPTPFactBuilder()
        self.drone_pose = None
        self.objects = {}

        # Subscribe to drone pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        # Subscribe to grounded objects
        self.object_sub = self.create_subscription(
            GroundedObject,
            '/perception/grounded_objects',
            self.object_callback,
            10
        )

        # Publish spatial relations
        self.relation_pub = self.create_publisher(
            SpatialRelation,
            '/perception/spatial_relations',
            10
        )

        # Timer for periodic relation computation (10Hz)
        self.timer = self.create_timer(0.1, self.compute_relations)

        self.get_logger().info('Spatial relation node initialized')

    def compute_relations(self):
        if self.drone_pose is None:
            return

        drone_pos = np.array([
            self.drone_pose.pose.position.x,
            self.drone_pose.pose.position.y,
            self.drone_pose.pose.position.z
        ])

        for obj_id, obj_data in self.objects.items():
            obj_pos = obj_data['position']
            dist = np.linalg.norm(drone_pos - obj_pos)

            # Create TPTP distance fact
            fact = self.fact_builder.distance('drone', obj_id, dist)

            # Publish relation
            relation = SpatialRelation()
            relation.subject = 'drone'
            relation.object = obj_id
            relation.relation_type = 'distance'
            relation.value = dist
            relation.tptp_fact = fact.to_tptp()
            self.relation_pub.publish(relation)

            # Log critical proximity
            if dist < 10.0:
                self.get_logger().warn(f'Object {obj_id} in critical proximity: {dist:.1f}m')
```

#### 5. Implement EventDetectionNode

```python
# perception_grounding/event_detection_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from perception_grounding.msg import DetectedEvent
from perception_grounding.tptp_fact_builder import TPTPFactBuilder
from shapely.geometry import Point, Polygon
import yaml

class EventDetectionNode(Node):
    def __init__(self):
        super().__init__('event_detection')

        self.fact_builder = TPTPFactBuilder()

        # Load zone definitions (geofence, NFZs)
        self.declare_parameter('zones_file', '')
        zones_file = self.get_parameter('zones_file').value
        with open(zones_file) as f:
            self.zones = yaml.safe_load(f)

        # Track zone membership history
        self.zone_history = {}

        # Subscribe to drone pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        # Publish detected events
        self.event_pub = self.create_publisher(
            DetectedEvent,
            '/perception/events',
            10
        )

        self.get_logger().info('Event detection node initialized')

    def pose_callback(self, msg):
        drone_pos = Point(msg.pose.position.x, msg.pose.position.y)

        current_zones = set()
        for zone_id, zone_data in self.zones.items():
            polygon = Polygon(zone_data['vertices'])
            if polygon.contains(drone_pos):
                current_zones.add(zone_id)

        # Detect zone entry/exit events
        prev_zones = self.zone_history.get('drone', set())

        for zone_id in current_zones - prev_zones:
            self.publish_event('enters', 'drone', zone_id)
            if self.zones[zone_id].get('type') == 'no_fly_zone':
                self.get_logger().error(f'SAFETY: Drone entered NFZ {zone_id}!')

        for zone_id in prev_zones - current_zones:
            self.publish_event('exits', 'drone', zone_id)

        self.zone_history['drone'] = current_zones

    def publish_event(self, event_type, entity_id, zone_id):
        fact = self.fact_builder.enters_zone(entity_id, zone_id, 0.0)

        event = DetectedEvent()
        event.event_type = event_type
        event.entity_id = entity_id
        event.zone_id = zone_id
        event.tptp_fact = fact.to_tptp()
        event.stamp = self.get_clock().now().to_msg()

        self.event_pub.publish(event)
        self.get_logger().info(f'Event: {event_type}({entity_id}, {zone_id})')
```

#### 6. Create Ontology Class Mapping

**Configuration for YOLO → Ontology mapping:**

```yaml
# config/ontology_class_mapping.yaml
# Maps YOLO class names to ontology concepts from uav_domain.kif

# People and vehicles (safety-critical)
person: Person
car: Vehicle
truck: Vehicle
bus: Vehicle
motorcycle: Vehicle
bicycle: Vehicle

# Aircraft (critical for airspace safety)
airplane: Aircraft
helicopter: Rotorcraft

# Structures
building: Building
tower: Tower

# Natural features
tree: Vegetation
water: WaterBody

# Animals (collision avoidance)
bird: Bird
dog: Animal
cat: Animal
```

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `perception_grounding` package builds and installs
- [ ] ObjectGroundingNode converts YOLO detections to TPTP facts
- [ ] TerrainGroundingNode processes segmentation masks
- [ ] SpatialRelationGroundingNode computes spatial relations
- [ ] EventDetectionNode detects zone entry/exit events
- [ ] TPTP facts published at 10Hz for Vampire queries
- [ ] Integration tests pass with simulated data
- [ ] Latency from perception to fact publication < 10ms

### Verification

Run automated verification:
```bash
bash .phases/phase-05-perception-bridge/verification.sh
```

### Common Pitfalls

- **Coordinate frames**: Ensure consistent TF frames across nodes
- **Race conditions**: TPTP fact assembly timing
- **Object ID tracking**: Maintain consistent IDs across frames (use tracking)
- **TPTP syntax**: Validate generated TPTP is parseable by Vampire
- **Type mismatches**: YOLO classes vs. ontology concepts (use mapping file)
- **Fact staleness**: Implement TTL for transient facts

### References

- [ROS 2 Message Design](https://design.ros2.org/articles/interface_definition.html)
- [TPTP Syntax](http://www.tptp.org/TPTP/TR/TPTPTR.shtml)
- [Vampire Input Format](https://vprover.github.io/usage.html)
- [Phase 4 Vampire Bridge](../phase-04-execution-mode/TASK.md)
- [Phase 3 Evaluation Report](../../ontology/evaluation/EVALUATION_REPORT.qmd)

### Dependencies
See `dependencies.json` - requires Phase 4 completion.

### Next Phase
After completion, proceed to Phase 6: Simplified Phase Transition (Vampire orchestration)
