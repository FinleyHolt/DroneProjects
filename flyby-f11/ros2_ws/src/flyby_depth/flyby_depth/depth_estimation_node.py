"""
ROS 2 Depth Estimation Node for Flyby F-11 UAV.

Runs DepthAnything V2 for monocular depth estimation and uses
rangefinder readings to provide metric scale correction.

Topics:
    Subscribed:
        /camera/image_raw (sensor_msgs/Image) - Input RGB image
        /rangefinder/range (sensor_msgs/Range) - Laser rangefinder distance
        /camera/camera_info (sensor_msgs/CameraInfo) - Camera intrinsics (optional)

    Published:
        /depth/metric (sensor_msgs/Image) - Metric depth map (32FC1)
        /depth/depth_map (flyby_msgs/DepthMap) - Full depth map with metadata
        /depth/points (sensor_msgs/PointCloud2) - 3D point cloud
        /depth/debug_image (sensor_msgs/Image) - Colorized depth visualization
"""

import numpy as np
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Range, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge

from flyby_msgs.msg import DepthMap
from flyby_depth.scale_corrector import ScaleCorrector


class DepthEstimationNode(Node):
    """ROS 2 node for monocular depth estimation with rangefinder scale correction."""

    def __init__(self):
        super().__init__('depth_estimation_node')

        # Declare parameters
        self._declare_parameters()

        # Get parameters
        self.model_size = self.get_parameter('model_size').value
        self.model_path = self.get_parameter('model_path').value
        self.device = self.get_parameter('device').value
        self.inference_resolution = self.get_parameter('inference_resolution').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.scale_filter_alpha = self.get_parameter('scale_filter_alpha').value
        self.min_valid_range = self.get_parameter('min_valid_range').value
        self.max_valid_range = self.get_parameter('max_valid_range').value
        self.scale_center_region = self.get_parameter('scale_center_region').value

        self.max_depth = self.get_parameter('max_depth').value
        self.min_depth = self.get_parameter('min_depth').value

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.focal_length_mm = self.get_parameter('focal_length_mm').value
        self.sensor_width_mm = self.get_parameter('sensor_width_mm').value

        self.publish_pointcloud = self.get_parameter('publish_pointcloud').value
        self.pointcloud_decimation = self.get_parameter('pointcloud_decimation').value
        self.pointcloud_max_range = self.get_parameter('pointcloud_max_range').value

        self.publish_debug_image = self.get_parameter('publish_debug_image').value

        # Topics
        image_topic = self.get_parameter('image_topic').value
        rangefinder_topic = self.get_parameter('rangefinder_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_map_topic = self.get_parameter('depth_map_topic').value
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value

        # Initialize depth model
        self.model = None
        self.transform = None
        self._init_depth_model()

        # Scale corrector
        self.scale_corrector = ScaleCorrector(
            alpha=self.scale_filter_alpha,
            min_valid_range=self.min_valid_range,
            max_valid_range=self.max_valid_range,
            center_region=self.scale_center_region,
        )

        # Camera intrinsics (computed from parameters, updated by camera_info)
        self.fx, self.fy, self.cx, self.cy = self._compute_intrinsics()

        # CV bridge
        self.bridge = CvBridge()

        # State
        self.latest_image: Optional[np.ndarray] = None
        self.latest_image_stamp = None
        self.latest_rangefinder: Optional[float] = None
        self.rangefinder_valid: bool = False
        self.inference_times: list = []

        # QoS for sensor data
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

        self.rangefinder_sub = self.create_subscription(
            Range,
            rangefinder_topic,
            self._rangefinder_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            sensor_qos
        )

        # Publishers
        self.depth_pub = self.create_publisher(Image, depth_topic, 10)
        self.depth_map_pub = self.create_publisher(DepthMap, depth_map_topic, 10)

        if self.publish_pointcloud:
            self.pointcloud_pub = self.create_publisher(
                PointCloud2, pointcloud_topic, 10
            )

        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image, debug_image_topic, 10
            )

        # Inference timer
        inference_period = 1.0 / self.publish_rate
        self.inference_timer = self.create_timer(inference_period, self._inference_callback)

        self.get_logger().info(
            f'Depth Estimation Node initialized:\n'
            f'  Model: DepthAnything V2 {self.model_size}\n'
            f'  Device: {self.device}\n'
            f'  Rate: {self.publish_rate} Hz\n'
            f'  Max depth: {self.max_depth} m\n'
            f'  Image topic: {image_topic}\n'
            f'  Rangefinder topic: {rangefinder_topic}'
        )

    def _declare_parameters(self):
        """Declare all node parameters."""
        self.declare_parameter('model_size', 'small')
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('inference_resolution', 518)
        self.declare_parameter('publish_rate', 15.0)

        self.declare_parameter('scale_filter_alpha', 0.3)
        self.declare_parameter('min_valid_range', 0.5)
        self.declare_parameter('max_valid_range', 1200.0)
        self.declare_parameter('scale_center_region', 0.1)

        self.declare_parameter('max_depth', 100.0)
        self.declare_parameter('min_depth', 0.5)
        self.declare_parameter('depth_confidence_threshold', 0.3)

        self.declare_parameter('image_width', 3840)
        self.declare_parameter('image_height', 2160)
        self.declare_parameter('focal_length_mm', 4.4)
        self.declare_parameter('sensor_width_mm', 6.4)

        self.declare_parameter('publish_pointcloud', True)
        self.declare_parameter('pointcloud_decimation', 4)
        self.declare_parameter('pointcloud_max_range', 50.0)

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('rangefinder_topic', '/rangefinder/range')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('depth_topic', '/depth/metric')
        self.declare_parameter('depth_map_topic', '/depth/depth_map')
        self.declare_parameter('pointcloud_topic', '/depth/points')

        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/depth/debug_image')

    def _init_depth_model(self):
        """Initialize DepthAnything V2 model."""
        try:
            import torch
            from transformers import pipeline

            # Model name mapping
            model_names = {
                'small': 'depth-anything/Depth-Anything-V2-Small-hf',
                'base': 'depth-anything/Depth-Anything-V2-Base-hf',
                'large': 'depth-anything/Depth-Anything-V2-Large-hf',
            }

            model_name = model_names.get(self.model_size, model_names['small'])
            if self.model_path:
                model_name = self.model_path

            self.get_logger().info(f'Loading depth model: {model_name}')

            # Create depth estimation pipeline
            self.model = pipeline(
                task='depth-estimation',
                model=model_name,
                device=0 if self.device == 'cuda' and torch.cuda.is_available() else -1,
            )

            self.get_logger().info(f'Depth model loaded successfully on {self.device}')

        except ImportError as e:
            self.get_logger().error(
                f'Failed to import depth model dependencies: {e}\n'
                'Install with: pip install transformers torch'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load depth model: {e}')

    def _compute_intrinsics(self) -> Tuple[float, float, float, float]:
        """Compute camera intrinsics from parameters."""
        # Focal length in pixels: fx = (focal_mm / sensor_width_mm) * image_width
        fx = (self.focal_length_mm / self.sensor_width_mm) * self.image_width
        fy = fx  # Assume square pixels
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0
        return fx, fy, cx, cy

    def _image_callback(self, msg: Image):
        """Store latest image for processing."""
        try:
            if msg.encoding in ['rgb8', 'bgr8']:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            elif msg.encoding == 'rgba8':
                rgba = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgba8')
                self.latest_image = rgba[:, :, :3]
            else:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            self.latest_image_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def _rangefinder_callback(self, msg: Range):
        """Store latest rangefinder reading."""
        self.latest_rangefinder = msg.range
        self.rangefinder_valid = (
            msg.range >= msg.min_range
            and msg.range <= msg.max_range
        )

    def _camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics from camera_info."""
        if msg.k[0] > 0:  # fx
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.image_width = msg.width
            self.image_height = msg.height

    def _inference_callback(self):
        """Run depth inference on latest image."""
        if self.latest_image is None:
            return

        if self.model is None:
            self.get_logger().warn_once('Depth model not loaded, skipping inference')
            return

        start_time = time.perf_counter()

        # Run depth inference
        try:
            result = self.model(self.latest_image)
            relative_depth = np.array(result['depth'])
        except Exception as e:
            self.get_logger().error(f'Depth inference failed: {e}')
            return

        inference_time = (time.perf_counter() - start_time) * 1000

        # Update scale from rangefinder
        rangefinder_reading = self.latest_rangefinder or 0.0
        scale_estimate = self.scale_corrector.update(
            relative_depth,
            rangefinder_reading,
            self.rangefinder_valid,
        )

        # Apply scale to get metric depth
        metric_depth = self.scale_corrector.apply_scale(
            relative_depth,
            min_depth=self.min_depth,
            max_depth=self.max_depth,
        )

        # Compute statistics
        valid_mask = (metric_depth > self.min_depth) & (metric_depth < self.max_depth)
        valid_count = int(valid_mask.sum())
        if valid_count > 0:
            min_depth_val = float(metric_depth[valid_mask].min())
            max_depth_val = float(metric_depth[valid_mask].max())
            mean_depth_val = float(metric_depth[valid_mask].mean())
        else:
            min_depth_val = 0.0
            max_depth_val = 0.0
            mean_depth_val = 0.0

        # Publish depth image
        self._publish_depth_image(metric_depth)

        # Publish full DepthMap message
        self._publish_depth_map(
            metric_depth,
            scale_estimate,
            min_depth_val,
            max_depth_val,
            mean_depth_val,
            valid_count,
            inference_time,
        )

        # Publish point cloud
        if self.publish_pointcloud:
            self._publish_pointcloud(metric_depth)

        # Publish debug image
        if self.publish_debug_image:
            self._publish_debug_image(metric_depth, scale_estimate)

        # Track timing
        self.inference_times.append(inference_time)
        if len(self.inference_times) > 100:
            self.inference_times.pop(0)

    def _publish_depth_image(self, metric_depth: np.ndarray):
        """Publish metric depth as sensor_msgs/Image (32FC1)."""
        msg = self.bridge.cv2_to_imgmsg(metric_depth, encoding='32FC1')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth_frame'
        self.depth_pub.publish(msg)

    def _publish_depth_map(
        self,
        metric_depth: np.ndarray,
        scale_estimate,
        min_depth: float,
        max_depth: float,
        mean_depth: float,
        valid_count: int,
        inference_time: float,
    ):
        """Publish full DepthMap message with metadata."""
        msg = DepthMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth_frame'

        # Depth image
        msg.depth_image = self.bridge.cv2_to_imgmsg(metric_depth, encoding='32FC1')
        msg.depth_image.header = msg.header

        # Scale info
        msg.scale_factor = scale_estimate.scale_factor
        msg.scale_confidence = scale_estimate.confidence
        msg.rangefinder_reading = scale_estimate.rangefinder_reading
        msg.rangefinder_valid = scale_estimate.valid

        # Statistics
        msg.min_depth = min_depth
        msg.max_depth = max_depth
        msg.mean_depth = mean_depth
        msg.valid_pixel_count = valid_count

        # Camera intrinsics
        msg.fx = self.fx
        msg.fy = self.fy
        msg.cx = self.cx
        msg.cy = self.cy

        # Metadata
        msg.inference_time_ms = inference_time
        msg.model_name = f'depth_anything_v2_{self.model_size}'

        self.depth_map_pub.publish(msg)

    def _publish_pointcloud(self, metric_depth: np.ndarray):
        """Generate and publish point cloud from depth map."""
        h, w = metric_depth.shape
        dec = self.pointcloud_decimation

        # Downsample
        depth_ds = metric_depth[::dec, ::dec]
        h_ds, w_ds = depth_ds.shape

        # Generate pixel coordinates
        u = np.arange(0, w, dec)
        v = np.arange(0, h, dec)
        u, v = np.meshgrid(u, v)

        # Valid mask
        valid = (depth_ds > self.min_depth) & (depth_ds < self.pointcloud_max_range)

        # Back-project to 3D (camera frame: Z forward, X right, Y down)
        z = depth_ds[valid]
        x = (u[valid] - self.cx) * z / self.fx
        y = (v[valid] - self.cy) * z / self.fy

        # Stack points
        points = np.stack([x, y, z], axis=-1).astype(np.float32)

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth_frame'

        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()

        self.pointcloud_pub.publish(msg)

    def _publish_debug_image(self, metric_depth: np.ndarray, scale_estimate):
        """Publish colorized depth visualization."""
        try:
            import cv2

            # Normalize depth for visualization
            depth_vis = metric_depth.copy()
            depth_vis = np.clip(depth_vis, 0, self.max_depth)
            depth_vis = (depth_vis / self.max_depth * 255).astype(np.uint8)

            # Apply colormap
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)

            # Add text overlay
            cv2.putText(
                depth_colored,
                f'Scale: {scale_estimate.scale_factor:.2f} (conf: {scale_estimate.confidence:.2f})',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                depth_colored,
                f'Range: {scale_estimate.rangefinder_reading:.1f}m',
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            if self.inference_times:
                avg_time = np.mean(self.inference_times)
                cv2.putText(
                    depth_colored,
                    f'Inference: {avg_time:.1f}ms',
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )

            # Publish
            debug_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'camera_depth_frame'
            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().warn(f'Failed to publish debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
