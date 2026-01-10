# DepthMap.msg
# Metric depth map with rangefinder-based scale correction
#
# Published by flyby_depth node at 10-15 Hz
# Used by local_avoidance node for obstacle detection

std_msgs/Header header

# Metric depth image (32FC1 encoding, values in meters)
# Invalid pixels are set to 0.0 or max_depth
sensor_msgs/Image depth_image

# Scale correction from rangefinder
float32 scale_factor              # Current scale: metric_depth = relative_depth * scale_factor
float32 scale_confidence          # Confidence in scale estimate [0-1]
                                  # Low when rangefinder reading is invalid or depth at center is unreliable

# Depth statistics
float32 min_depth                 # Closest valid depth in image (meters)
float32 max_depth                 # Farthest valid depth in image (meters)
float32 mean_depth                # Mean depth of valid pixels (meters)
int32 valid_pixel_count           # Number of pixels with valid depth

# Rangefinder data used for scale correction
float32 rangefinder_reading       # Raw rangefinder distance (meters)
bool rangefinder_valid            # True if rangefinder reading is usable

# Camera intrinsics (for point cloud generation)
float32 fx                        # Focal length x (pixels)
float32 fy                        # Focal length y (pixels)
float32 cx                        # Principal point x (pixels)
float32 cy                        # Principal point y (pixels)

# Processing metadata
float32 inference_time_ms         # Depth model inference time
string model_name                 # e.g., "depth_anything_v2_small"
