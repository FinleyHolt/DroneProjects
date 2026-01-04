"""
Perception pipeline for Flyby F-11 UAV RL training.

DUAL-MODE ARCHITECTURE:
This package supports two perception modes for sim-to-real transfer:

1. TRAINING MODE ("gt"):
   - GroundTruthDetector uses frustum math (no rendering)
   - Runs at 1000+ env steps/second
   - Configurable noise matches real detector behavior

2. E2E VALIDATION MODE ("full"):
   - YOLODetector runs real inference on camera images
   - Built-in ByteTrack tracking via model.track()
   - Runs at ~20 Hz (real-time)
   - Validates full pipeline before deployment

Both modes produce IDENTICAL 516-dimensional observation vectors.

Components:
- DualModePerception: Top-level interface with mode switching
- GroundTruthDetector: Fast frustum-based detection for training
- YOLODetector: Real YOLO inference with ByteTrack tracking
- PerceptionEncoder: Shared encoding (detections → 516-dim obs)
- SpatialGridEncoder: 8x8 grid encoding (384 dims)
- SceneStatistics: Aggregate scene metrics (32 dims)
- TPTPGenerator: Vampire ATP fact generation for safety checking

Output: 516-dimensional observation vector
- Priority detections (top-10): 100 dims
- Spatial grid (8x8x6): 384 dims
- Scene statistics: 32 dims

Performance targets:
- Training mode: <1ms per observation
- E2E mode: <20ms per observation (50ms control loop budget)
"""

from .detector import YOLODetector, Detection
from .spatial_encoder import SpatialGridEncoder, SpatialGridConfig
from .scene_statistics import SceneStatistics, SceneStatisticsConfig
from .perception_encoder import PerceptionEncoder, PerceptionConfig, OUTPUT_DIM
from .tptp_generator import TPTPGenerator, TPTPConfig
from .frustum import CameraFrustum, CameraParams, AABB, camera_pose_from_position_orientation
from .ground_truth_detector import (
    GroundTruthDetector, GroundTruthConfig, WorldObject,
    create_world_objects_from_sim
)
from .dual_mode_perception import (
    DualModePerception, DualModeConfig,
    create_dual_mode_perception
)
from .viewport_camera import (
    ViewportCamera, ViewportCameraConfig,
    DetectionLogger
)

__all__ = [
    # Dual-mode top-level interface
    'DualModePerception',
    'DualModeConfig',
    'create_dual_mode_perception',
    # Ground truth detector (training mode)
    'GroundTruthDetector',
    'GroundTruthConfig',
    'WorldObject',
    'create_world_objects_from_sim',
    # Frustum utilities
    'CameraFrustum',
    'CameraParams',
    'AABB',
    'camera_pose_from_position_orientation',
    # YOLO detector with ByteTrack (E2E mode)
    'YOLODetector',
    'Detection',
    # Main encoder
    'PerceptionEncoder',
    'PerceptionConfig',
    'OUTPUT_DIM',
    # Spatial encoder
    'SpatialGridEncoder',
    'SpatialGridConfig',
    # Scene statistics
    'SceneStatistics',
    'SceneStatisticsConfig',
    # TPTP generator
    'TPTPGenerator',
    'TPTPConfig',
    # Viewport camera (Isaac Sim camera API workaround)
    'ViewportCamera',
    'ViewportCameraConfig',
    'DetectionLogger',
]

__version__ = '0.2.0'
