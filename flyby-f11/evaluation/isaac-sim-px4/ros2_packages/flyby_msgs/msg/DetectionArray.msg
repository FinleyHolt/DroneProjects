# DetectionArray.msg
# Array of detections from a single frame
#
# Published by flyby_perception node on /perception/detections

std_msgs/Header header              # Frame timestamp and ID

# Detections sorted by priority (safety-critical first)
Detection[] detections

# Frame metadata
int32 image_width                   # Source image width
int32 image_height                  # Source image height
int32 num_detections                # Total detection count

# Tracking statistics
int32 active_tracks                 # Number of currently tracked objects
int32 new_tracks                    # New tracks this frame
int32 lost_tracks                   # Lost tracks this frame

# Performance
float32 inference_time_ms           # YOLO inference time
float32 total_latency_ms            # Total processing latency

# Source information
string source                       # Image source (isaac_sim, realsense, etc.)
bool is_ground_truth                # True if using GT labels instead of inference
