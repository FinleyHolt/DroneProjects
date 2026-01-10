"""
ROS 2 Perception Encoder Node for Flyby F-11 UAV.

Subscribes to detection arrays and encodes them into a fixed-size
observation vector (516 dims) for RL policy input.

Topics:
    Subscribed:
        /perception/detections (flyby_msgs/DetectionArray) - Input detections
        /uav/state (flyby_msgs/UAVState) - UAV state for spatial encoding

    Published:
        /perception/encoded (flyby_msgs/PerceptionState) - Encoded observation
"""

import numpy as np
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from flyby_msgs.msg import DetectionArray, PerceptionState, UAVState


class PerceptionEncoderNode(Node):
    """ROS 2 node for encoding detections into RL observation vectors."""

    # Observation dimensions
    NUM_PRIORITY_DETECTIONS = 10
    FEATURES_PER_DETECTION = 10
    GRID_SIZE = 8
    GRID_CHANNELS = 6
    SCENE_STATS_DIM = 32
    TOTAL_DIM = (NUM_PRIORITY_DETECTIONS * FEATURES_PER_DETECTION +
                 GRID_SIZE * GRID_SIZE * GRID_CHANNELS +
                 SCENE_STATS_DIM)  # 100 + 384 + 32 = 516

    def __init__(self):
        super().__init__('perception_encoder_node')

        # Parameters
        self.declare_parameter('detection_topic', '/perception/detections')
        self.declare_parameter('output_topic', '/perception/encoded')
        self.declare_parameter('spatial_range', 100.0)  # meters

        detection_topic = self.get_parameter('detection_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.spatial_range = self.get_parameter('spatial_range').value

        # State
        self.latest_uav_state: Optional[UAVState] = None

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.detection_sub = self.create_subscription(
            DetectionArray,
            detection_topic,
            self._detection_callback,
            10
        )

        self.uav_state_sub = self.create_subscription(
            UAVState,
            '/uav/state',
            self._uav_state_callback,
            sensor_qos
        )

        # Publishers
        self.encoded_pub = self.create_publisher(
            PerceptionState,
            output_topic,
            10
        )

        self.get_logger().info(
            f'Perception Encoder Node initialized:\n'
            f'  Input: {detection_topic}\n'
            f'  Output: {output_topic}\n'
            f'  Observation dim: {self.TOTAL_DIM}'
        )

    def _uav_state_callback(self, msg: UAVState):
        """Store latest UAV state."""
        self.latest_uav_state = msg

    def _detection_callback(self, msg: DetectionArray):
        """Encode detections into observation vector."""
        import time
        start_time = time.perf_counter()

        # Get UAV position
        uav_pos = np.array([0.0, 0.0, 0.0])
        if self.latest_uav_state is not None:
            uav_pos = np.array([
                self.latest_uav_state.position.x,
                self.latest_uav_state.position.y,
                self.latest_uav_state.position.z
            ])

        # Encode detections
        observation = self._encode_detections(msg, uav_pos)

        # Calculate stats
        encoding_time = (time.perf_counter() - start_time) * 1000

        closest_person = float('inf')
        person_count = 0
        for det in msg.detections:
            if det.ontology_class == 'Person':
                person_count += 1
                if det.distance > 0 and det.distance < closest_person:
                    closest_person = det.distance

        # Publish
        out_msg = PerceptionState()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = 'base_link'

        out_msg.observation = observation.tolist()
        out_msg.num_detections = msg.num_detections
        out_msg.from_ground_truth = msg.is_ground_truth
        out_msg.encoding_time_ms = encoding_time
        out_msg.closest_person_distance = closest_person if closest_person != float('inf') else -1.0
        out_msg.person_count = person_count

        self.encoded_pub.publish(out_msg)

    def _encode_detections(self, msg: DetectionArray, uav_position: np.ndarray) -> np.ndarray:
        """Encode detection array into 516-dim observation vector."""
        observation = np.zeros(self.TOTAL_DIM, dtype=np.float32)

        # 1. Priority detections (top 10 x 10 features = 100 dims)
        priority_start = 0
        for i, det in enumerate(msg.detections[:self.NUM_PRIORITY_DETECTIONS]):
            base = priority_start + i * self.FEATURES_PER_DETECTION

            # Normalize bbox to center format
            if det.image_width > 0 and det.image_height > 0:
                cx = ((det.bbox_x1 + det.bbox_x2) / 2) / det.image_width
                cy = ((det.bbox_y1 + det.bbox_y2) / 2) / det.image_height
                bw = abs(det.bbox_x2 - det.bbox_x1) / det.image_width
                bh = abs(det.bbox_y2 - det.bbox_y1) / det.image_height
            else:
                cx, cy, bw, bh = 0.5, 0.5, 0.0, 0.0

            # Calculate relative position
            rel_x, rel_y, rel_z = 0.0, 0.0, 0.0
            if det.has_position_3d:
                rel = np.array([det.position_3d.x, det.position_3d.y, det.position_3d.z]) - uav_position
                rel_x = np.clip(rel[0] / self.spatial_range, -1, 1)
                rel_y = np.clip(rel[1] / self.spatial_range, -1, 1)
                rel_z = np.clip(rel[2] / self.spatial_range, -1, 1)

            observation[base:base + 10] = [
                det.class_id / 10.0,           # Normalized class
                det.confidence,                 # Confidence [0-1]
                rel_x, rel_y, rel_z,           # Relative position
                np.clip(det.distance / self.spatial_range, 0, 1),  # Distance
                np.clip((det.bearing + np.pi) / (2 * np.pi), 0, 1),  # Bearing
                det.velocity.x if det.has_velocity else 0.0,
                det.velocity.y if det.has_velocity else 0.0,
                bw * bh,                       # Bbox size (normalized area)
            ]

        # 2. Spatial grid (8x8x6 = 384 dims)
        grid_start = self.NUM_PRIORITY_DETECTIONS * self.FEATURES_PER_DETECTION
        grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE, self.GRID_CHANNELS), dtype=np.float32)

        for det in msg.detections:
            if not det.has_position_3d:
                continue

            # Map position to grid cell
            rel = np.array([det.position_3d.x, det.position_3d.y, det.position_3d.z]) - uav_position
            gx = int((rel[0] / self.spatial_range + 1) * self.GRID_SIZE / 2)
            gy = int((rel[1] / self.spatial_range + 1) * self.GRID_SIZE / 2)

            gx = np.clip(gx, 0, self.GRID_SIZE - 1)
            gy = np.clip(gy, 0, self.GRID_SIZE - 1)

            # Channel 0: Person count
            if det.ontology_class == 'Person':
                grid[gx, gy, 0] += 1

            # Channel 1: Obstacle count
            if det.ontology_class in ['DynamicObstacle', 'StaticObstacle']:
                grid[gx, gy, 1] += 1

            # Channel 2: Nearest distance (inverted, closer = higher)
            dist_inv = 1.0 / (det.distance + 1.0)
            if dist_inv > grid[gx, gy, 2]:
                grid[gx, gy, 2] = dist_inv

            # Channel 3-4: Average motion
            if det.has_velocity:
                grid[gx, gy, 3] += det.velocity.x
                grid[gx, gy, 4] += det.velocity.y

            # Channel 5: Highest priority
            priority_val = 1.0 - (det.priority / 10.0)
            if priority_val > grid[gx, gy, 5]:
                grid[gx, gy, 5] = priority_val

        observation[grid_start:grid_start + 384] = grid.flatten()

        # 3. Scene statistics (32 dims)
        stats_start = grid_start + 384
        stats = np.zeros(self.SCENE_STATS_DIM, dtype=np.float32)

        # Object counts by class
        class_counts = {}
        for det in msg.detections:
            class_counts[det.ontology_class] = class_counts.get(det.ontology_class, 0) + 1

        stats[0] = class_counts.get('Person', 0) / 10.0
        stats[1] = class_counts.get('DynamicObstacle', 0) / 10.0
        stats[2] = class_counts.get('StaticObstacle', 0) / 10.0
        stats[3] = len(msg.detections) / 20.0

        # Distance statistics
        if msg.detections:
            distances = [d.distance for d in msg.detections if d.distance > 0]
            if distances:
                stats[4] = min(distances) / self.spatial_range
                stats[5] = np.mean(distances) / self.spatial_range
                stats[6] = max(distances) / self.spatial_range

        # Person proximity
        person_distances = [d.distance for d in msg.detections
                          if d.ontology_class == 'Person' and d.distance > 0]
        if person_distances:
            stats[7] = min(person_distances) / self.spatial_range
            stats[8] = 1.0  # Person present flag

        # Tracking stats
        stats[9] = msg.active_tracks / 20.0
        stats[10] = msg.new_tracks / 5.0
        stats[11] = msg.lost_tracks / 5.0

        # Confidence statistics
        if msg.detections:
            confs = [d.confidence for d in msg.detections]
            stats[12] = np.mean(confs)
            stats[13] = np.std(confs)

        observation[stats_start:stats_start + self.SCENE_STATS_DIM] = stats

        return observation


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionEncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
