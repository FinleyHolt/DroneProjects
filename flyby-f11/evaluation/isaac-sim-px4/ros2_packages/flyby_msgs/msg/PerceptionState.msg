# PerceptionState.msg
# Encoded perception state for RL observation space
#
# 516-dimensional fixed-size vector suitable for neural network input
# Combines priority detections, spatial grid, and scene statistics

std_msgs/Header header

# Full encoded observation (516 dims)
# Layout:
#   [0:100]   - Priority detections (top 10 x 10 features)
#   [100:484] - Spatial grid (8x8x6 channels)
#   [484:516] - Scene statistics (32 dims)
float32[516] observation

# Component breakdowns for debugging
float32[] priority_detections       # Top detections encoding
float32[] spatial_grid              # 8x8x6 grid encoding
float32[] scene_statistics          # Aggregate stats

# Source metadata
int32 num_detections                # Number of detections used
bool from_ground_truth              # True if GT, false if YOLO
float32 encoding_time_ms            # Time to encode

# Person proximity (extracted for safety)
float32 closest_person_distance     # Nearest person (meters, inf if none)
int32 person_count                  # Number of people detected
