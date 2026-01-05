"""
ROS 2 YOLO Detector Node for Flyby F-11 UAV.

Subscribes to camera images, runs YOLOv11 with ByteTrack tracking,
and publishes detections. Designed for simulation-to-deployment parity.

Topics:
    Subscribed:
        /camera/image_raw (sensor_msgs/Image) - Input camera image
        /uav/state (flyby_msgs/UAVState) - UAV state for distance calculation

    Published:
        /perception/detections (flyby_msgs/DetectionArray) - Detected objects
        /perception/debug_image (sensor_msgs/Image) - Annotated debug image
"""

import numpy as np
import time
from dataclasses import dataclass
from typing import Optional, Set, Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge

from flyby_msgs.msg import Detection, DetectionArray, UAVState


# =============================================================================
# ISR-Relevant COCO Class IDs
# =============================================================================
ISR_RELEVANT_COCO_CLASSES: Set[int] = {
    0,  # person - highest priority for safety
    1,  # bicycle
    2,  # car
    3,  # motorcycle
    5,  # bus
    7,  # truck
}


@dataclass
class InternalDetection:
    """Internal detection representation before conversion to ROS message."""
    class_id: int
    class_name: str
    ontology_class: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # xyxy pixel coords
    position_3d: Optional[np.ndarray] = None
    distance: float = 0.0
    bearing: float = 0.0
    velocity: Optional[np.ndarray] = None
    priority: int = 10
    safety_distance: float = 0.0
    track_id: int = -1
    image_size: Tuple[int, int] = (640, 480)


class YOLODetectorNode(Node):
    """ROS 2 node wrapping YOLO detector with ByteTrack tracking."""

    def __init__(self):
        super().__init__('yolo_detector_node')

        # Declare parameters
        self.declare_parameter('model_path', '/workspace/models/yolo11x.pt')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('isr_filter_enabled', True)
        self.declare_parameter('inference_rate', 10.0)  # Hz
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/perception/detections')

        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.isr_filter_enabled = self.get_parameter('isr_filter_enabled').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        image_topic = self.get_parameter('image_topic').value
        detection_topic = self.get_parameter('detection_topic').value

        # Initialize YOLO model
        self.model = None
        self._init_yolo()

        # Class mappings (COCO -> internal)
        self.coco_to_internal = {
            0: 1,   # person
            1: 8,   # bicycle
            2: 2,   # car
            3: 9,   # motorcycle
            5: 5,   # bus
            7: 7,   # truck
        }

        self.class_config = {
            1: {'name': 'person', 'ontology_class': 'Person', 'priority': 1, 'safety_distance': 15.0},
            2: {'name': 'car', 'ontology_class': 'DynamicObstacle', 'priority': 2, 'safety_distance': 10.0},
            5: {'name': 'bus', 'ontology_class': 'DynamicObstacle', 'priority': 2, 'safety_distance': 10.0},
            7: {'name': 'truck', 'ontology_class': 'DynamicObstacle', 'priority': 2, 'safety_distance': 10.0},
            8: {'name': 'bicycle', 'ontology_class': 'DynamicObstacle', 'priority': 3, 'safety_distance': 8.0},
            9: {'name': 'motorcycle', 'ontology_class': 'DynamicObstacle', 'priority': 2, 'safety_distance': 10.0},
        }

        # CV bridge for image conversion
        self.bridge = CvBridge()

        # State
        self.latest_image: Optional[np.ndarray] = None
        self.latest_image_stamp = None
        self.latest_uav_state: Optional[UAVState] = None
        self.inference_times: List[float] = []

        # Tracking stats
        self.tracking_stats = {
            'active_tracks': 0,
            'new_tracks': 0,
            'lost_tracks': 0,
        }

        # QoS for sensor data (best effort for real-time)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            sensor_qos
        )

        self.uav_state_sub = self.create_subscription(
            UAVState,
            '/uav/state',
            self._uav_state_callback,
            sensor_qos
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            DetectionArray,
            detection_topic,
            10
        )

        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/perception/debug_image',
                10
            )

        # Timer for inference (runs at specified rate)
        inference_period = 1.0 / self.inference_rate
        self.inference_timer = self.create_timer(inference_period, self._inference_callback)

        self.get_logger().info(
            f'YOLO Detector Node initialized:\n'
            f'  Model: {self.model_path}\n'
            f'  Tracking: {self.enable_tracking}\n'
            f'  ISR Filter: {self.isr_filter_enabled}\n'
            f'  Rate: {self.inference_rate} Hz\n'
            f'  Image topic: {image_topic}\n'
            f'  Detection topic: {detection_topic}'
        )

    def _init_yolo(self):
        """Initialize YOLO model."""
        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'Loaded YOLO model from {self.model_path}')
        except ImportError:
            self.get_logger().error('ultralytics not installed, using mock detections')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')

    def _image_callback(self, msg: Image):
        """Store latest image for processing."""
        try:
            # Convert ROS Image to OpenCV/numpy
            if msg.encoding in ['rgb8', 'bgr8']:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            elif msg.encoding == 'rgba8':
                rgba = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgba8')
                self.latest_image = rgba[:, :, :3]  # Drop alpha
            else:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            self.latest_image_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def _uav_state_callback(self, msg: UAVState):
        """Store latest UAV state."""
        self.latest_uav_state = msg

    def _inference_callback(self):
        """Run YOLO inference on latest image."""
        if self.latest_image is None:
            return

        start_time = time.perf_counter()

        # Get UAV position if available
        uav_position = None
        if self.latest_uav_state is not None:
            uav_position = np.array([
                self.latest_uav_state.position.x,
                self.latest_uav_state.position.y,
                self.latest_uav_state.position.z
            ])

        # Run detection
        detections = self._run_yolo(self.latest_image, uav_position)

        # Calculate timing
        inference_time = (time.perf_counter() - start_time) * 1000
        self.inference_times.append(inference_time)
        if len(self.inference_times) > 100:
            self.inference_times.pop(0)

        # Publish detections
        self._publish_detections(detections, inference_time)

        # Publish debug image if enabled
        if self.publish_debug_image:
            self._publish_debug_image(self.latest_image, detections)

    def _run_yolo(self, image: np.ndarray, uav_position: Optional[np.ndarray]) -> List[InternalDetection]:
        """Run YOLO inference with optional tracking."""
        if self.model is None:
            return self._mock_detections(image.shape[:2])

        img_h, img_w = image.shape[:2]
        classes_filter = list(ISR_RELEVANT_COCO_CLASSES) if self.isr_filter_enabled else None

        # Run inference
        if self.enable_tracking:
            results = self.model.track(
                image,
                verbose=False,
                conf=self.confidence_threshold,
                iou=self.nms_threshold,
                persist=True,
                tracker="bytetrack.yaml",
                classes=classes_filter,
            )
        else:
            results = self.model(
                image,
                verbose=False,
                conf=self.confidence_threshold,
                iou=self.nms_threshold,
                classes=classes_filter,
            )

        detections = []
        track_ids_seen = set()

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for box in boxes:
                coco_class = int(box.cls[0])
                internal_class = self.coco_to_internal.get(coco_class)

                if internal_class is None:
                    continue

                config = self.class_config.get(internal_class, {})

                # Get bbox
                xyxy = box.xyxy[0].cpu().numpy()
                x1 = max(0, min(float(xyxy[0]), img_w))
                y1 = max(0, min(float(xyxy[1]), img_h))
                x2 = max(0, min(float(xyxy[2]), img_w))
                y2 = max(0, min(float(xyxy[3]), img_h))

                if x1 >= x2 or y1 >= y2:
                    continue

                # Get track ID
                track_id = -1
                if self.enable_tracking and box.id is not None:
                    track_id = int(box.id[0])
                    track_ids_seen.add(track_id)

                det = InternalDetection(
                    class_id=internal_class,
                    class_name=config.get('name', 'unknown'),
                    ontology_class=config.get('ontology_class', 'Object'),
                    confidence=float(box.conf[0]),
                    bbox=(x1, y1, x2, y2),
                    priority=config.get('priority', 10),
                    safety_distance=config.get('safety_distance', 0.0),
                    track_id=track_id,
                    image_size=(img_w, img_h),
                )

                # Calculate distance if UAV position available
                if uav_position is not None and det.position_3d is not None:
                    det.distance = float(np.linalg.norm(det.position_3d - uav_position))
                    rel_pos = det.position_3d - uav_position
                    det.bearing = float(np.arctan2(rel_pos[1], rel_pos[0]))

                detections.append(det)

        # Update tracking stats
        self.tracking_stats['active_tracks'] = len(track_ids_seen)

        # Sort by priority
        detections.sort(key=lambda d: (d.priority, -d.confidence))

        return detections

    def _mock_detections(self, image_shape: Tuple[int, int]) -> List[InternalDetection]:
        """Generate mock detections for testing without YOLO."""
        import random
        detections = []

        if random.random() < 0.3:
            detections.append(InternalDetection(
                class_id=1,
                class_name='person',
                ontology_class='Person',
                confidence=random.uniform(0.5, 0.95),
                bbox=(100, 100, 200, 300),
                priority=1,
                safety_distance=15.0,
                track_id=random.randint(1, 100),
                image_size=(image_shape[1], image_shape[0]),
            ))

        return detections

    def _publish_detections(self, detections: List[InternalDetection], inference_time: float):
        """Publish detections as ROS message."""
        msg = DetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'

        if detections and detections[0].image_size:
            msg.image_width = detections[0].image_size[0]
            msg.image_height = detections[0].image_size[1]

        msg.num_detections = len(detections)
        msg.inference_time_ms = inference_time
        msg.total_latency_ms = inference_time  # Could add encoding time
        msg.source = 'yolo_v11'
        msg.is_ground_truth = False

        msg.active_tracks = self.tracking_stats['active_tracks']
        msg.new_tracks = self.tracking_stats['new_tracks']
        msg.lost_tracks = self.tracking_stats['lost_tracks']

        for det in detections:
            d = Detection()
            d.stamp = msg.header.stamp
            d.class_id = det.class_id
            d.class_name = det.class_name
            d.ontology_class = det.ontology_class
            d.confidence = det.confidence

            d.bbox_x1 = det.bbox[0]
            d.bbox_y1 = det.bbox[1]
            d.bbox_x2 = det.bbox[2]
            d.bbox_y2 = det.bbox[3]

            d.image_width = det.image_size[0]
            d.image_height = det.image_size[1]

            if det.position_3d is not None:
                d.position_3d = Point(x=det.position_3d[0], y=det.position_3d[1], z=det.position_3d[2])
                d.has_position_3d = True
            else:
                d.has_position_3d = False

            d.distance = det.distance
            d.bearing = det.bearing

            if det.velocity is not None:
                d.velocity = Vector3(x=det.velocity[0], y=det.velocity[1], z=det.velocity[2])
                d.has_velocity = True
            else:
                d.has_velocity = False

            d.track_id = det.track_id
            d.priority = det.priority
            d.safety_distance = det.safety_distance

            msg.detections.append(d)

        self.detection_pub.publish(msg)

    def _publish_debug_image(self, image: np.ndarray, detections: List[InternalDetection]):
        """Publish annotated debug image."""
        try:
            import cv2

            debug_img = image.copy()

            for det in detections:
                x1, y1, x2, y2 = [int(v) for v in det.bbox]

                # Color by class (person=red, vehicle=blue)
                if det.ontology_class == 'Person':
                    color = (255, 0, 0)  # Red
                else:
                    color = (0, 0, 255)  # Blue

                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)

                label = f'{det.class_name} {det.confidence:.2f}'
                if det.track_id >= 0:
                    label += f' #{det.track_id}'

                cv2.putText(debug_img, label, (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # Convert to ROS message
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='rgb8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'camera_link'

            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().warn(f'Failed to publish debug image: {e}')

    def reset_tracking(self):
        """Reset ByteTrack state (call between episodes)."""
        if self.model is not None and self.enable_tracking:
            self.model.predictor = None
        self.tracking_stats = {'active_tracks': 0, 'new_tracks': 0, 'lost_tracks': 0}
        self.get_logger().info('Tracking state reset')


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
