# Phase 6d: Perception Pipeline

## Overview

Create a perception pipeline that converts raw camera images into semantic TPTP facts for the Vampire theorem prover. This bridges the gap between visual sensor data and ontological reasoning, enabling the ontology to reason about observed objects, terrain types, and environmental features.

## Human Description

The ontology can express concepts like "UAV is near Building" or "detected obstacle at position X", but raw camera images contain no semantic information. This phase creates:

1. **Object Detection Node** - YOLO-based detection of buildings, vehicles, people, landing zones
2. **Semantic Segmentation Node** - Terrain classification (grass, water, concrete, forest)
3. **TPTP Fact Generator** - Convert detections to ontology-compatible facts
4. **Perception Bridge Node** - ROS 2 node connecting camera to Vampire

The pipeline must run at minimum 10Hz on Jetson Orin NX to feed real-time observations to the ontology reasoner.

## AI Agent Instructions

### Prerequisites
- Phase 6a completed (camera topics available)
- Phase 6c completed (worlds provide objects to detect)
- Understanding of YOLO/ultralytics inference
- Familiarity with TPTP format for Vampire
- Knowledge of ROS 2 image processing

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

#### 1. Create Perception Package Structure

**Location:** `ros2_ws/src/flyby_f11_perception/`

```bash
cd ros2_ws/src
ros2 pkg create flyby_f11_perception --build-type ament_python \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge
```

**Package Structure:**
```
flyby_f11_perception/
├── package.xml
├── setup.py
├── setup.cfg
├── flyby_f11_perception/
│   ├── __init__.py
│   ├── object_detector_node.py
│   ├── semantic_segmenter_node.py
│   ├── tptp_fact_generator.py
│   ├── perception_bridge_node.py
│   └── models/
│       └── .gitkeep
├── config/
│   ├── detection_classes.yaml
│   ├── segmentation_classes.yaml
│   └── tptp_templates.yaml
├── launch/
│   └── perception.launch.py
├── msg/
│   ├── Detection.msg
│   ├── DetectionArray.msg
│   ├── SemanticMask.msg
│   └── TPTPFact.msg
└── test/
    ├── test_detector.py
    ├── test_tptp_generator.py
    └── test_integration.py
```

#### 2. Define Detection Classes

Map detected objects to ontology concepts:

```yaml
# config/detection_classes.yaml
# YOLO class ID -> Ontology concept mapping
detection_classes:
  # Structures
  0: { name: "building", ontology_class: "StaticObstacle", confidence_threshold: 0.6 }
  1: { name: "house", ontology_class: "StaticObstacle", confidence_threshold: 0.6 }
  2: { name: "warehouse", ontology_class: "StaticObstacle", confidence_threshold: 0.6 }
  3: { name: "tower", ontology_class: "StaticObstacle", confidence_threshold: 0.7 }

  # Vehicles
  10: { name: "car", ontology_class: "DynamicObstacle", confidence_threshold: 0.5 }
  11: { name: "truck", ontology_class: "DynamicObstacle", confidence_threshold: 0.5 }
  12: { name: "boat", ontology_class: "DynamicObstacle", confidence_threshold: 0.5 }

  # People
  20: { name: "person", ontology_class: "Person", confidence_threshold: 0.4 }
  21: { name: "crowd", ontology_class: "PersonGroup", confidence_threshold: 0.5 }

  # Landing zones
  30: { name: "helipad", ontology_class: "LandingZone", confidence_threshold: 0.7 }
  31: { name: "open_field", ontology_class: "LandingZone", confidence_threshold: 0.5 }
  32: { name: "rooftop_flat", ontology_class: "LandingZone", confidence_threshold: 0.6 }

  # Hazards
  40: { name: "power_line", ontology_class: "StaticObstacle", confidence_threshold: 0.6 }
  41: { name: "crane", ontology_class: "StaticObstacle", confidence_threshold: 0.6 }

# Model configuration
model:
  name: "yolov8n"  # Nano for speed on Jetson
  input_size: [640, 640]
  device: "cuda:0"  # GPU inference
  half_precision: true  # FP16 for Jetson
```

#### 3. Define Segmentation Classes

```yaml
# config/segmentation_classes.yaml
segmentation_classes:
  0: { name: "background", ontology_class: null }
  1: { name: "grass", ontology_class: "OpenTerrain" }
  2: { name: "concrete", ontology_class: "HardSurface" }
  3: { name: "asphalt", ontology_class: "HardSurface" }
  4: { name: "water", ontology_class: "WaterSurface" }
  5: { name: "forest", ontology_class: "VegetationDense" }
  6: { name: "building", ontology_class: "Structure" }
  7: { name: "vehicle", ontology_class: "VehicleArea" }
  8: { name: "sand", ontology_class: "SoftTerrain" }

model:
  name: "deeplabv3_mobilenet"  # Lightweight for real-time
  input_size: [512, 512]
  device: "cuda:0"
  half_precision: true
```

#### 4. Define TPTP Fact Templates

```yaml
# config/tptp_templates.yaml
# Templates for converting detections to TPTP facts

fact_templates:
  # Object observation: observedObject(id, class, x, y, z, confidence, timestamp)
  observed_object: |
    fof(obs_{id}, axiom,
      observedObject({object_id}, {ontology_class}, position({x}, {y}, {z}),
                     confidence({confidence}), time({timestamp}))).

  # Distance to object: distanceTo(uav, object, distance)
  distance_to: |
    fof(dist_{id}, axiom,
      distanceTo(f11_uav, {object_id}, {distance})).

  # Terrain classification: terrainAt(x, y, class)
  terrain_at: |
    fof(terrain_{id}, axiom,
      terrainType(region({x_min}, {y_min}, {x_max}, {y_max}), {terrain_class})).

  # Safe landing zone detected
  landing_zone_detected: |
    fof(lz_{id}, axiom,
      (and
        (instance({lz_id}, LandingZone)
        (hasPosition({lz_id}, position({x}, {y}, {z})))
        (hasRadius({lz_id}, {radius}))))).

  # Person detected (triggers safety constraint)
  person_detected: |
    fof(person_{id}, axiom,
      (and
        (instance({person_id}, Person)
        (hasPosition({person_id}, position({x}, {y}, {z})))
        (distanceTo(f11_uav, {person_id}, {distance}))))).

# Batching configuration
batching:
  max_facts_per_message: 50
  publish_rate_hz: 10
  deduplicate_window_ms: 500
```

#### 5. Implement Object Detector Node

```python
# flyby_f11_perception/object_detector_node.py
"""
YOLO-based object detection for UAV perception.
Publishes detected objects with 3D position estimates.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import yaml
import os

# Defer ultralytics import for systems without it
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

from flyby_f11_perception.msg import Detection, DetectionArray


class ObjectDetectorNode(Node):
    """
    Detects objects in camera images and estimates 3D positions.
    """

    def __init__(self):
        super().__init__('object_detector')

        # Load configuration
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value
        if config_path and os.path.exists(config_path):
            with open(config_path) as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._default_config()

        self.detection_classes = self.config.get('detection_classes', {})

        # Initialize YOLO model
        if YOLO_AVAILABLE:
            model_name = self.config.get('model', {}).get('name', 'yolov8n')
            self.model = YOLO(f"{model_name}.pt")
            self.get_logger().info(f'Loaded YOLO model: {model_name}')
        else:
            self.model = None
            self.get_logger().warn('ultralytics not installed, using mock detections')

        # ROS interfaces
        self.bridge = CvBridge()
        self.current_pose = None
        self.camera_info = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/f11/camera/image_raw',
            self.image_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/f11/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            DetectionArray,
            '/perception/detections',
            10
        )

        # Statistics
        self.frame_count = 0
        self.detection_count = 0

        self.get_logger().info('Object detector initialized')

    def _default_config(self):
        """Default configuration if file not provided."""
        return {
            'detection_classes': {
                0: {'name': 'building', 'ontology_class': 'StaticObstacle', 'confidence_threshold': 0.5},
                1: {'name': 'person', 'ontology_class': 'Person', 'confidence_threshold': 0.4},
                2: {'name': 'vehicle', 'ontology_class': 'DynamicObstacle', 'confidence_threshold': 0.5},
            },
            'model': {'name': 'yolov8n'}
        }

    def pose_callback(self, msg: PoseStamped):
        """Update current UAV pose."""
        self.current_pose = msg

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics for depth estimation."""
        self.camera_info = msg

    def image_callback(self, msg: Image):
        """Process incoming camera image."""
        self.frame_count += 1

        # Convert ROS image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # Run detection
        detections = self._detect_objects(cv_image, msg.header.stamp)

        # Publish results
        if detections:
            det_array = DetectionArray()
            det_array.header = msg.header
            det_array.detections = detections
            self.detection_pub.publish(det_array)

            self.detection_count += len(detections)

        # Log statistics periodically
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Processed {self.frame_count} frames, '
                f'{self.detection_count} total detections'
            )

    def _detect_objects(self, image: np.ndarray, timestamp) -> list:
        """Run YOLO detection and convert to Detection messages."""
        detections = []

        if self.model is None:
            # Mock detection for testing without YOLO
            return self._mock_detections(timestamp)

        # Run inference
        results = self.model(image, verbose=False)

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for box in boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                # Get class configuration
                class_config = self.detection_classes.get(class_id, {})
                if not class_config:
                    continue

                threshold = class_config.get('confidence_threshold', 0.5)
                if confidence < threshold:
                    continue

                # Create detection message
                det = Detection()
                det.header.stamp = timestamp
                det.class_id = class_id
                det.class_name = class_config.get('name', 'unknown')
                det.ontology_class = class_config.get('ontology_class', 'Object')
                det.confidence = confidence

                # Bounding box (normalized)
                xyxy = box.xyxyn[0].cpu().numpy()
                det.bbox_x = float(xyxy[0])
                det.bbox_y = float(xyxy[1])
                det.bbox_width = float(xyxy[2] - xyxy[0])
                det.bbox_height = float(xyxy[3] - xyxy[1])

                # Estimate 3D position (simplified - uses center + assumed depth)
                det.position = self._estimate_position(
                    (xyxy[0] + xyxy[2]) / 2,
                    (xyxy[1] + xyxy[3]) / 2,
                    det.class_name
                )

                detections.append(det)

        return detections

    def _estimate_position(self, u_norm, v_norm, class_name):
        """
        Estimate 3D world position from normalized image coordinates.
        Uses camera projection and assumed object height/size.
        """
        from geometry_msgs.msg import Point

        pos = Point()

        # Default assumed distances by class
        default_distances = {
            'building': 50.0,
            'person': 15.0,
            'vehicle': 30.0,
        }
        assumed_distance = default_distances.get(class_name, 30.0)

        if self.current_pose and self.camera_info:
            # Project using camera intrinsics
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            # Convert normalized to pixel coordinates
            img_width = self.camera_info.width
            img_height = self.camera_info.height
            u = u_norm * img_width
            v = v_norm * img_height

            # Ray direction in camera frame
            x_cam = (u - cx) / fx * assumed_distance
            y_cam = (v - cy) / fy * assumed_distance
            z_cam = assumed_distance

            # Transform to world frame (simplified - assumes camera pointing down)
            uav_pos = self.current_pose.pose.position
            pos.x = uav_pos.x + x_cam
            pos.y = uav_pos.y + y_cam
            pos.z = 0.0  # Assume objects on ground

        else:
            # Fallback: relative position estimate
            pos.x = (u_norm - 0.5) * assumed_distance * 2
            pos.y = (v_norm - 0.5) * assumed_distance * 2
            pos.z = 0.0

        return pos

    def _mock_detections(self, timestamp):
        """Generate mock detections for testing."""
        import random

        detections = []
        if random.random() > 0.3:  # 70% chance of detection
            det = Detection()
            det.header.stamp = timestamp
            det.class_id = 0
            det.class_name = 'building'
            det.ontology_class = 'StaticObstacle'
            det.confidence = random.uniform(0.6, 0.95)
            det.bbox_x = random.uniform(0.2, 0.6)
            det.bbox_y = random.uniform(0.2, 0.6)
            det.bbox_width = random.uniform(0.1, 0.3)
            det.bbox_height = random.uniform(0.1, 0.4)

            from geometry_msgs.msg import Point
            det.position = Point()
            det.position.x = random.uniform(-50, 50)
            det.position.y = random.uniform(-50, 50)
            det.position.z = 0.0

            detections.append(det)

        return detections


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 6. Implement TPTP Fact Generator

```python
# flyby_f11_perception/tptp_fact_generator.py
"""
Converts perception outputs to TPTP facts for Vampire theorem prover.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import yaml
import os
from collections import deque
import hashlib
import time

from flyby_f11_perception.msg import DetectionArray, Detection


class TPTPFactGenerator(Node):
    """
    Generates TPTP facts from perception detections.
    """

    def __init__(self):
        super().__init__('tptp_fact_generator')

        # Load TPTP templates
        self.declare_parameter('template_file', '')
        template_path = self.get_parameter('template_file').value
        if template_path and os.path.exists(template_path):
            with open(template_path) as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._default_templates()

        self.templates = self.config.get('fact_templates', {})
        self.batching = self.config.get('batching', {})

        # Current UAV pose
        self.current_pose = None

        # Deduplication: track recent facts by hash
        self.recent_facts = deque(maxlen=1000)
        self.dedup_window_ms = self.batching.get('deduplicate_window_ms', 500)

        # Fact counter for unique IDs
        self.fact_counter = 0

        # Subscribers
        self.detection_sub = self.create_subscription(
            DetectionArray,
            '/perception/detections',
            self.detection_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        # Publisher - TPTP facts as strings
        self.fact_pub = self.create_publisher(
            String,
            '/perception/tptp_facts',
            10
        )

        # Batch publishing timer
        publish_rate = self.batching.get('publish_rate_hz', 10)
        self.pending_facts = []
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_batch)

        self.get_logger().info('TPTP Fact Generator initialized')

    def _default_templates(self):
        """Default TPTP templates."""
        return {
            'fact_templates': {
                'observed_object': (
                    "fof(obs_{id}, axiom, "
                    "observedObject({object_id}, {ontology_class}, "
                    "position({x}, {y}, {z}), confidence({confidence}), "
                    "time({timestamp})))."
                ),
                'distance_to': (
                    "fof(dist_{id}, axiom, "
                    "distanceTo(f11_uav, {object_id}, {distance}))."
                ),
                'person_detected': (
                    "fof(person_{id}, axiom, "
                    "(and (instance({person_id}, Person) "
                    "(hasPosition({person_id}, position({x}, {y}, {z}))) "
                    "(distanceTo(f11_uav, {person_id}, {distance})))))."
                ),
            },
            'batching': {
                'max_facts_per_message': 50,
                'publish_rate_hz': 10,
                'deduplicate_window_ms': 500,
            }
        }

    def pose_callback(self, msg: PoseStamped):
        """Update current UAV pose."""
        self.current_pose = msg

    def detection_callback(self, msg: DetectionArray):
        """Convert detections to TPTP facts."""
        timestamp = self._get_timestamp(msg.header.stamp)

        for det in msg.detections:
            facts = self._detection_to_facts(det, timestamp)
            for fact in facts:
                if not self._is_duplicate(fact):
                    self.pending_facts.append(fact)

    def _detection_to_facts(self, det: Detection, timestamp: float) -> list:
        """Convert a single detection to TPTP facts."""
        facts = []
        self.fact_counter += 1
        fact_id = self.fact_counter

        # Generate unique object ID
        object_id = f"{det.class_name}_{fact_id}"

        # Position
        x = round(det.position.x, 2)
        y = round(det.position.y, 2)
        z = round(det.position.z, 2)

        # Basic observation fact
        template = self.templates.get('observed_object', '')
        if template:
            fact = template.format(
                id=fact_id,
                object_id=object_id,
                ontology_class=det.ontology_class,
                x=x, y=y, z=z,
                confidence=round(det.confidence, 3),
                timestamp=round(timestamp, 3)
            )
            facts.append(fact)

        # Distance fact
        if self.current_pose:
            uav_pos = self.current_pose.pose.position
            distance = self._calculate_distance(
                uav_pos.x, uav_pos.y, uav_pos.z,
                x, y, z
            )

            dist_template = self.templates.get('distance_to', '')
            if dist_template:
                fact = dist_template.format(
                    id=f"{fact_id}_dist",
                    object_id=object_id,
                    distance=round(distance, 2)
                )
                facts.append(fact)

        # Special handling for persons (safety critical)
        if det.ontology_class == 'Person':
            person_template = self.templates.get('person_detected', '')
            if person_template and self.current_pose:
                uav_pos = self.current_pose.pose.position
                distance = self._calculate_distance(
                    uav_pos.x, uav_pos.y, uav_pos.z,
                    x, y, z
                )
                fact = person_template.format(
                    id=f"{fact_id}_person",
                    person_id=object_id,
                    x=x, y=y, z=z,
                    distance=round(distance, 2)
                )
                facts.append(fact)

        return facts

    def _calculate_distance(self, x1, y1, z1, x2, y2, z2) -> float:
        """Calculate Euclidean distance."""
        import math
        return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

    def _get_timestamp(self, ros_time) -> float:
        """Convert ROS time to float seconds."""
        return ros_time.sec + ros_time.nanosec * 1e-9

    def _is_duplicate(self, fact: str) -> bool:
        """Check if fact was recently published."""
        fact_hash = hashlib.md5(fact.encode()).hexdigest()[:16]
        current_time = time.time() * 1000  # ms

        # Clean old entries
        while self.recent_facts and (current_time - self.recent_facts[0][0]) > self.dedup_window_ms:
            self.recent_facts.popleft()

        # Check for duplicate
        for _, h in self.recent_facts:
            if h == fact_hash:
                return True

        # Add new fact
        self.recent_facts.append((current_time, fact_hash))
        return False

    def publish_batch(self):
        """Publish accumulated facts."""
        if not self.pending_facts:
            return

        max_per_msg = self.batching.get('max_facts_per_message', 50)

        # Batch facts into messages
        while self.pending_facts:
            batch = self.pending_facts[:max_per_msg]
            self.pending_facts = self.pending_facts[max_per_msg:]

            msg = String()
            msg.data = '\n'.join(batch)
            self.fact_pub.publish(msg)

            self.get_logger().debug(f'Published {len(batch)} TPTP facts')


def main(args=None):
    rclpy.init(args=args)
    node = TPTPFactGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 7. Implement Perception Bridge Node

This node orchestrates the complete perception pipeline:

```python
# flyby_f11_perception/perception_bridge_node.py
"""
Main perception bridge connecting camera to ontology reasoner.
Orchestrates detection, segmentation, and TPTP fact generation.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import time


class PerceptionBridgeNode(Node):
    """
    Main perception pipeline orchestrator.

    Coordinates:
    - Object detection
    - Semantic segmentation
    - TPTP fact generation
    - Performance monitoring
    """

    def __init__(self):
        super().__init__('perception_bridge')

        # Parameters
        self.declare_parameter('target_fps', 10.0)
        self.declare_parameter('enable_segmentation', True)
        self.declare_parameter('enable_detection', True)

        self.target_fps = self.get_parameter('target_fps').value
        self.enable_seg = self.get_parameter('enable_segmentation').value
        self.enable_det = self.get_parameter('enable_detection').value

        # Callback groups for parallel processing
        self.cb_group = ReentrantCallbackGroup()

        # State tracking
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.fps_history = []
        self.is_active = True

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/f11/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.cb_group
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        # Control subscriber (enable/disable pipeline)
        self.control_sub = self.create_subscription(
            Bool,
            '/perception/enable',
            self.control_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/perception/status',
            10
        )

        # Performance monitoring timer
        self.stats_timer = self.create_timer(5.0, self.publish_stats)

        self.get_logger().info(
            f'Perception bridge initialized: '
            f'detection={self.enable_det}, segmentation={self.enable_seg}, '
            f'target_fps={self.target_fps}'
        )

    def pose_callback(self, msg: PoseStamped):
        """Track UAV pose for position estimation."""
        self.current_pose = msg

    def control_callback(self, msg: Bool):
        """Enable/disable perception pipeline."""
        self.is_active = msg.data
        self.get_logger().info(f'Perception pipeline {"enabled" if self.is_active else "disabled"}')

    def image_callback(self, msg: Image):
        """Process incoming camera image."""
        if not self.is_active:
            return

        current_time = time.time()
        self.frame_count += 1

        # Calculate actual FPS
        dt = current_time - self.last_frame_time
        if dt > 0:
            fps = 1.0 / dt
            self.fps_history.append(fps)
            if len(self.fps_history) > 100:
                self.fps_history.pop(0)

        self.last_frame_time = current_time

        # Note: Actual detection/segmentation is handled by separate nodes
        # This node primarily monitors and orchestrates

    def publish_stats(self):
        """Publish performance statistics."""
        if self.fps_history:
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            min_fps = min(self.fps_history)
            max_fps = max(self.fps_history)
        else:
            avg_fps = min_fps = max_fps = 0.0

        status = {
            'active': self.is_active,
            'frame_count': self.frame_count,
            'avg_fps': round(avg_fps, 2),
            'min_fps': round(min_fps, 2),
            'max_fps': round(max_fps, 2),
            'target_fps': self.target_fps,
            'performance': 'OK' if avg_fps >= self.target_fps * 0.9 else 'DEGRADED'
        }

        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)

        if avg_fps < self.target_fps * 0.9 and self.frame_count > 100:
            self.get_logger().warn(
                f'Perception running below target: {avg_fps:.1f} FPS '
                f'(target: {self.target_fps})'
            )


def main(args=None):
    rclpy.init(args=args)

    node = PerceptionBridgeNode()

    # Use multi-threaded executor for parallel processing
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 8. Create Message Definitions

```
# msg/Detection.msg
# Single object detection result

std_msgs/Header header

# Classification
uint32 class_id
string class_name
string ontology_class  # Mapped to UAV ontology
float32 confidence

# 2D bounding box (normalized 0-1)
float32 bbox_x
float32 bbox_y
float32 bbox_width
float32 bbox_height

# Estimated 3D position (world frame)
geometry_msgs/Point position
```

```
# msg/DetectionArray.msg
# Array of detections from single frame

std_msgs/Header header
flyby_f11_perception/Detection[] detections
```

```
# msg/SemanticMask.msg
# Semantic segmentation result

std_msgs/Header header

# Segmentation mask (encoded as image)
sensor_msgs/Image mask

# Class labels present in mask
string[] class_names
uint8[] class_ids
```

```
# msg/TPTPFact.msg
# Single TPTP fact for Vampire

std_msgs/Header header
string fact_id
string fact_content
float32 confidence
string source  # "detection", "segmentation", "derived"
```

#### 9. Create Launch File

```python
# launch/perception.launch.py
"""Launch perception pipeline nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('flyby_f11_perception')
    config_dir = os.path.join(pkg_dir, 'config')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'enable_detection',
            default_value='true',
            description='Enable object detection'
        ),
        DeclareLaunchArgument(
            'enable_segmentation',
            default_value='false',
            description='Enable semantic segmentation'
        ),
        DeclareLaunchArgument(
            'target_fps',
            default_value='10.0',
            description='Target processing frame rate'
        ),

        # Object detector node
        Node(
            package='flyby_f11_perception',
            executable='object_detector_node',
            name='object_detector',
            output='screen',
            parameters=[{
                'config_file': os.path.join(config_dir, 'detection_classes.yaml')
            }],
            condition=LaunchConfiguration('enable_detection')
        ),

        # TPTP fact generator
        Node(
            package='flyby_f11_perception',
            executable='tptp_fact_generator',
            name='tptp_fact_generator',
            output='screen',
            parameters=[{
                'template_file': os.path.join(config_dir, 'tptp_templates.yaml')
            }]
        ),

        # Perception bridge (orchestrator)
        Node(
            package='flyby_f11_perception',
            executable='perception_bridge_node',
            name='perception_bridge',
            output='screen',
            parameters=[{
                'target_fps': LaunchConfiguration('target_fps'),
                'enable_detection': LaunchConfiguration('enable_detection'),
                'enable_segmentation': LaunchConfiguration('enable_segmentation'),
            }]
        ),
    ])
```

#### 10. Create Integration Tests

```python
# test/test_tptp_generator.py
"""Test TPTP fact generation."""
import pytest
import rclpy
from flyby_f11_perception.msg import Detection, DetectionArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


def test_fact_format():
    """Verify TPTP facts have correct syntax."""
    # This would be a unit test for the fact generator
    # Actual implementation would import and test TPTPFactGenerator

    # Example expected fact format
    expected_pattern = r"fof\(obs_\d+, axiom, observedObject\(.+\)\)\."

    # Verify fact matches pattern
    sample_fact = (
        "fof(obs_1, axiom, "
        "observedObject(building_1, StaticObstacle, "
        "position(10.5, 20.3, 0.0), confidence(0.85), time(12345.67)))."
    )

    import re
    assert re.match(expected_pattern, sample_fact)


def test_person_safety_fact():
    """Verify person detections generate safety-critical facts."""
    # Person detections should include distance for safety reasoning
    sample_fact = (
        "fof(person_1_person, axiom, "
        "(and (instance(person_1, Person) "
        "(hasPosition(person_1, position(5.0, 10.0, 0.0))) "
        "(distanceTo(f11_uav, person_1, 12.5)))))."
    )

    assert "Person" in sample_fact
    assert "distanceTo" in sample_fact


def test_deduplication():
    """Verify duplicate facts are filtered."""
    # Same detection appearing in consecutive frames should not
    # produce duplicate facts within the dedup window
    pass  # Implementation would test actual dedup logic
```

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `flyby_f11_perception` package builds successfully
- [ ] Object detector node runs without errors
- [ ] TPTP fact generator produces valid syntax
- [ ] Facts published to `/perception/tptp_facts` topic
- [ ] Pipeline maintains >= 10 Hz on Jetson Orin NX
- [ ] Person detections include distance facts
- [ ] Detection -> TPTP mapping works for all defined classes
- [ ] Deduplication prevents redundant facts
- [ ] Performance monitoring reports FPS correctly

### Verification

Run automated verification:
```bash
bash .phases/phase-06d-perception-pipeline/verification.sh
```

### Common Pitfalls

- **YOLO model loading**: Ensure model weights are downloaded before first run
- **CUDA availability**: Check GPU is accessible in container
- **Frame rate drops**: Monitor GPU memory usage, reduce model size if needed
- **TPTP syntax**: Vampire is strict - verify fact format with simple queries first
- **Image encoding**: Ensure cv_bridge handles Gazebo image format correctly
- **Topic names**: Verify camera topics match Phase 6a configuration

### References

- [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- [TPTP Syntax](http://tptp.org/TPTP/SyntaxBNF.html)
- [ROS 2 Image Pipeline](https://docs.ros.org/en/humble/p/image_pipeline/)
- [Vampire Theorem Prover](https://vprover.github.io/)
- [Phase 6a Sensor Topics](../phase-06a-isr-sensor-integration/TASK.md)

### Dependencies
See `dependencies.json` - requires Phase 6a and Phase 6c completion.

### Next Phase
After completion, proceed to Phase 6b: Simulation & Training Environment (now depends on 6c and 6d)
