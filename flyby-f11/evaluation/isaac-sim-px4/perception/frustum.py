"""
Camera Frustum Utilities for Ground Truth Detection

Provides fast geometric computations for determining which objects
are visible to the camera based on frustum intersection. Used in
Training Mode for 1000+ env steps/second without rendering.

Key concepts:
- Frustum: 3D pyramid defined by camera FOV and near/far planes
- AABB: Axis-Aligned Bounding Box for objects
- Visibility: Object intersects frustum AND is not occluded

Performance: Pure numpy, no rendering required.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from scipy.spatial.transform import Rotation


@dataclass
class CameraParams:
    """Camera intrinsic and extrinsic parameters."""
    # Resolution
    width: int = 640
    height: int = 480

    # Field of view (degrees)
    fov_horizontal: float = 90.0
    fov_vertical: float = 67.5  # Computed from horizontal FOV and aspect ratio

    # Depth range (meters)
    near_clip: float = 0.5
    far_clip: float = 200.0

    # Focal length (for projection)
    focal_length: float = 4.5  # mm

    # Sensor size (for focal length based FOV)
    sensor_width: float = 6.4  # mm (typical 1/2.3" sensor)

    def __post_init__(self):
        """Compute derived parameters."""
        aspect = self.width / self.height
        if self.fov_vertical == 67.5:  # Default, compute from horizontal
            self.fov_vertical = 2 * np.degrees(np.arctan(
                np.tan(np.radians(self.fov_horizontal / 2)) / aspect
            ))


@dataclass
class AABB:
    """Axis-Aligned Bounding Box."""
    min_point: np.ndarray  # (x, y, z) minimum corner
    max_point: np.ndarray  # (x, y, z) maximum corner

    @property
    def center(self) -> np.ndarray:
        return (self.min_point + self.max_point) / 2

    @property
    def size(self) -> np.ndarray:
        return self.max_point - self.min_point

    @classmethod
    def from_center_size(cls, center: np.ndarray, size: np.ndarray) -> 'AABB':
        half_size = size / 2
        return cls(center - half_size, center + half_size)

    def get_corners(self) -> np.ndarray:
        """Get all 8 corners of the AABB."""
        corners = np.array([
            [self.min_point[0], self.min_point[1], self.min_point[2]],
            [self.max_point[0], self.min_point[1], self.min_point[2]],
            [self.min_point[0], self.max_point[1], self.min_point[2]],
            [self.max_point[0], self.max_point[1], self.min_point[2]],
            [self.min_point[0], self.min_point[1], self.max_point[2]],
            [self.max_point[0], self.min_point[1], self.max_point[2]],
            [self.min_point[0], self.max_point[1], self.max_point[2]],
            [self.max_point[0], self.max_point[1], self.max_point[2]],
        ])
        return corners


class CameraFrustum:
    """
    Camera frustum for visibility testing.

    Uses 6-plane representation for fast intersection tests.
    Planes are stored as (normal, distance) pairs in camera frame.

    Usage:
        frustum = CameraFrustum(camera_params)

        # Check single point
        visible = frustum.is_visible(point, camera_pose)

        # Get visible objects from list
        visible_indices = frustum.get_visible_objects(objects, camera_pose)

        # Project to image coordinates
        img_x, img_y = frustum.project_to_image(point, camera_pose)
    """

    def __init__(self, params: CameraParams):
        self.params = params

        # Precompute frustum plane normals in camera frame
        # Camera convention: +Z forward, +X right, +Y down
        self._compute_frustum_planes()

        # Precompute projection matrix
        self._compute_projection_matrix()

    def _compute_frustum_planes(self):
        """Compute 6 frustum planes in camera frame."""
        fov_h_rad = np.radians(self.params.fov_horizontal / 2)
        fov_v_rad = np.radians(self.params.fov_vertical / 2)

        # Near and far planes (normals pointing inward)
        self.near_plane = (np.array([0, 0, 1]), self.params.near_clip)
        self.far_plane = (np.array([0, 0, -1]), -self.params.far_clip)

        # Side planes (computed from FOV angles)
        # Left plane: normal points right (+X component)
        cos_h = np.cos(fov_h_rad)
        sin_h = np.sin(fov_h_rad)
        self.left_plane = (np.array([cos_h, 0, sin_h]), 0)
        self.right_plane = (np.array([-cos_h, 0, sin_h]), 0)

        # Top/bottom planes
        cos_v = np.cos(fov_v_rad)
        sin_v = np.sin(fov_v_rad)
        self.top_plane = (np.array([0, cos_v, sin_v]), 0)
        self.bottom_plane = (np.array([0, -cos_v, sin_v]), 0)

        self.planes = [
            self.near_plane, self.far_plane,
            self.left_plane, self.right_plane,
            self.top_plane, self.bottom_plane
        ]

    def _compute_projection_matrix(self):
        """Compute perspective projection matrix."""
        fov_h_rad = np.radians(self.params.fov_horizontal)
        aspect = self.params.width / self.params.height
        near = self.params.near_clip
        far = self.params.far_clip

        # Focal lengths in pixels
        self.fx = self.params.width / (2 * np.tan(fov_h_rad / 2))
        self.fy = self.fx  # Square pixels

        # Principal point (image center)
        self.cx = self.params.width / 2
        self.cy = self.params.height / 2

        # 3x3 intrinsic matrix
        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

    def world_to_camera(
        self,
        point_world: np.ndarray,
        camera_pose: np.ndarray
    ) -> np.ndarray:
        """
        Transform world point to camera frame.

        Args:
            point_world: Point in world frame (3,) or (N, 3)
            camera_pose: 4x4 camera-to-world transform

        Returns:
            Point in camera frame (3,) or (N, 3)
        """
        # Camera-to-world inverse = world-to-camera
        world_to_cam = np.linalg.inv(camera_pose)

        if point_world.ndim == 1:
            point_h = np.append(point_world, 1.0)
            point_cam = world_to_cam @ point_h
            return point_cam[:3]
        else:
            # Batch transform
            points_h = np.hstack([point_world, np.ones((len(point_world), 1))])
            points_cam = (world_to_cam @ points_h.T).T
            return points_cam[:, :3]

    def is_in_frustum_camera_frame(self, point_cam: np.ndarray) -> bool:
        """
        Check if point (in camera frame) is inside frustum.

        Uses half-space intersection test with all 6 planes.
        """
        for normal, dist in self.planes:
            # Point is outside if dot(normal, point) < dist
            if np.dot(normal, point_cam) < dist:
                return False
        return True

    def is_visible(
        self,
        point_world: np.ndarray,
        camera_pose: np.ndarray
    ) -> bool:
        """
        Check if world point is visible from camera.

        Args:
            point_world: 3D point in world frame
            camera_pose: 4x4 camera-to-world transform

        Returns:
            True if point is inside camera frustum
        """
        point_cam = self.world_to_camera(point_world, camera_pose)
        return self.is_in_frustum_camera_frame(point_cam)

    def project_to_image(
        self,
        point_world: np.ndarray,
        camera_pose: np.ndarray
    ) -> Tuple[Optional[float], Optional[float]]:
        """
        Project 3D world point to 2D image coordinates.

        Args:
            point_world: 3D point in world frame
            camera_pose: 4x4 camera-to-world transform

        Returns:
            (x, y) normalized image coordinates (0-1, 0-1) or (None, None) if behind camera
        """
        point_cam = self.world_to_camera(point_world, camera_pose)

        # Check if behind camera
        if point_cam[2] <= 0:
            return None, None

        # Perspective projection
        x_proj = self.fx * point_cam[0] / point_cam[2] + self.cx
        y_proj = self.fy * point_cam[1] / point_cam[2] + self.cy

        # Normalize to 0-1
        x_norm = x_proj / self.params.width
        y_norm = y_proj / self.params.height

        return x_norm, y_norm

    def project_to_image_batch(
        self,
        points_world: np.ndarray,
        camera_pose: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Project multiple 3D points to image coordinates.

        Args:
            points_world: (N, 3) array of world points
            camera_pose: 4x4 camera-to-world transform

        Returns:
            x_norm: (N,) normalized x coordinates
            y_norm: (N,) normalized y coordinates
            valid: (N,) boolean mask of valid projections
        """
        points_cam = self.world_to_camera(points_world, camera_pose)

        # Valid if in front of camera
        valid = points_cam[:, 2] > 0

        x_norm = np.zeros(len(points_world))
        y_norm = np.zeros(len(points_world))

        if np.any(valid):
            z_valid = points_cam[valid, 2]
            x_proj = self.fx * points_cam[valid, 0] / z_valid + self.cx
            y_proj = self.fy * points_cam[valid, 1] / z_valid + self.cy

            x_norm[valid] = x_proj / self.params.width
            y_norm[valid] = y_proj / self.params.height

        return x_norm, y_norm, valid

    def get_visible_objects(
        self,
        objects: List[AABB],
        camera_pose: np.ndarray
    ) -> List[int]:
        """
        Get indices of objects that intersect the camera frustum.

        Uses conservative AABB-frustum intersection test.

        Args:
            objects: List of AABBs to test
            camera_pose: 4x4 camera-to-world transform

        Returns:
            List of indices for visible objects
        """
        visible_indices = []

        for i, aabb in enumerate(objects):
            if self._aabb_intersects_frustum(aabb, camera_pose):
                visible_indices.append(i)

        return visible_indices

    def _aabb_intersects_frustum(
        self,
        aabb: AABB,
        camera_pose: np.ndarray
    ) -> bool:
        """
        Conservative AABB-frustum intersection test.

        Tests if any corner of the AABB is inside the frustum,
        OR if the frustum might intersect the AABB.
        """
        # Transform AABB corners to camera frame
        corners_world = aabb.get_corners()
        corners_cam = self.world_to_camera(corners_world, camera_pose)

        # Quick reject: all corners behind camera
        if np.all(corners_cam[:, 2] <= 0):
            return False

        # Quick accept: any corner inside frustum
        for corner in corners_cam:
            if corner[2] > 0 and self.is_in_frustum_camera_frame(corner):
                return True

        # Conservative: check if AABB center is reasonably close
        center_cam = self.world_to_camera(aabb.center, camera_pose)
        if center_cam[2] > 0:
            # Project center and check if within expanded bounds
            x_proj = center_cam[0] / center_cam[2]
            y_proj = center_cam[1] / center_cam[2]

            # Expand bounds by object size / distance
            max_dim = np.max(aabb.size)
            angular_size = max_dim / max(center_cam[2], 0.1)

            fov_h_rad = np.radians(self.params.fov_horizontal / 2)
            fov_v_rad = np.radians(self.params.fov_vertical / 2)

            if (abs(x_proj) < np.tan(fov_h_rad) + angular_size and
                abs(y_proj) < np.tan(fov_v_rad) + angular_size and
                center_cam[2] < self.params.far_clip + max_dim):
                return True

        return False

    def compute_bbox_from_aabb(
        self,
        aabb: AABB,
        camera_pose: np.ndarray
    ) -> Optional[Tuple[float, float, float, float]]:
        """
        Compute 2D bounding box from 3D AABB projection.

        Args:
            aabb: 3D bounding box
            camera_pose: 4x4 camera-to-world transform

        Returns:
            (cx, cy, w, h) normalized bbox or None if not visible
        """
        corners_world = aabb.get_corners()
        x_coords, y_coords, valid = self.project_to_image_batch(corners_world, camera_pose)

        if not np.any(valid):
            return None

        # Get bounds of valid projections
        x_valid = x_coords[valid]
        y_valid = y_coords[valid]

        x_min = np.clip(np.min(x_valid), 0, 1)
        x_max = np.clip(np.max(x_valid), 0, 1)
        y_min = np.clip(np.min(y_valid), 0, 1)
        y_max = np.clip(np.max(y_valid), 0, 1)

        # Skip if too small or outside image
        width = x_max - x_min
        height = y_max - y_min

        if width < 0.005 or height < 0.005:  # Minimum 0.5% of image
            return None

        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2

        return (cx, cy, width, height)


def camera_pose_from_position_orientation(
    position: np.ndarray,
    orientation: np.ndarray,
    camera_offset: np.ndarray = None,
    camera_rotation: np.ndarray = None
) -> np.ndarray:
    """
    Construct camera pose matrix from UAV state.

    Args:
        position: UAV position (x, y, z) in world frame
        orientation: UAV orientation as quaternion (w, x, y, z)
        camera_offset: Camera position offset from UAV center (default: forward and down)
        camera_rotation: Camera rotation relative to UAV body (default: 45deg down pitch)

    Returns:
        4x4 camera-to-world transformation matrix
    """
    if camera_offset is None:
        camera_offset = np.array([0.1, 0.0, -0.05])  # Forward and slightly below

    if camera_rotation is None:
        # 45 degree downward pitch for ISR
        camera_rotation = Rotation.from_euler('ZYX', [0, 45, 0], degrees=True).as_quat()

    # UAV body-to-world rotation
    uav_rot = Rotation.from_quat(orientation[[1, 2, 3, 0]])  # Convert wxyz to xyzw

    # Camera-to-body rotation
    cam_rot = Rotation.from_quat(camera_rotation[[1, 2, 3, 0]] if camera_rotation[0] > 0.5
                                  else camera_rotation)

    # Combined rotation: camera-to-world
    total_rot = uav_rot * cam_rot

    # Camera position in world frame
    cam_pos = position + uav_rot.apply(camera_offset)

    # Build 4x4 transform
    pose = np.eye(4)
    pose[:3, :3] = total_rot.as_matrix()
    pose[:3, 3] = cam_pos

    return pose


# Default object sizes for different classes (meters)
DEFAULT_OBJECT_SIZES = {
    1: np.array([0.5, 0.5, 1.8]),    # person
    2: np.array([4.5, 2.0, 1.5]),    # vehicle/car
    3: np.array([10.0, 10.0, 5.0]),  # building (small)
    4: np.array([1.0, 1.0, 1.0]),    # POI target
    5: np.array([5.0, 5.0, 0.1]),    # landing zone
    6: np.array([3.0, 3.0, 8.0]),    # tree
    7: np.array([0.3, 0.3, 10.0]),   # power line pole
}


def create_aabb_for_object(
    position: np.ndarray,
    class_id: int,
    size_override: np.ndarray = None
) -> AABB:
    """
    Create AABB for an object given its position and class.

    Args:
        position: Object center position (x, y, z)
        class_id: Object class (1=person, 2=vehicle, etc.)
        size_override: Optional custom size

    Returns:
        AABB for the object
    """
    size = size_override if size_override is not None else DEFAULT_OBJECT_SIZES.get(
        class_id, np.array([1.0, 1.0, 1.0])
    )
    return AABB.from_center_size(np.array(position), size)
