# ObstacleMap.msg
# 3D obstacle representation and avoidance recommendations from VFH+
#
# Published by local_avoidance node at 10-20 Hz
# Used for visualization and by trajectory controller for path modification

std_msgs/Header header

# Obstacle point cloud (filtered, in body frame)
sensor_msgs/PointCloud2 obstacles

# Closest obstacle information
float32 closest_obstacle_distance       # Distance to nearest obstacle (meters)
geometry_msgs/Point closest_obstacle_point  # Position of closest obstacle (body frame)
bool obstacle_detected                  # True if any obstacle within safety radius

# VFH+ avoidance recommendation
bool path_blocked                       # True if current heading toward goal is blocked
float32 recommended_heading             # Recommended yaw change (radians, body frame)
float32 recommended_pitch               # Recommended pitch change (radians, body frame)
float32 recommendation_confidence       # Confidence in recommendation [0-1]

# Goal information (input to VFH+)
geometry_msgs/Point goal_direction      # Unit vector toward goal (body frame)
float32 goal_distance                   # Distance to goal (meters)

# Histogram summary (for debugging/visualization)
int32 histogram_azimuth_bins            # Number of azimuth bins (e.g., 36)
int32 histogram_elevation_bins          # Number of elevation bins (e.g., 18)
int32 blocked_bins                      # Number of bins marked as blocked
int32 candidate_directions              # Number of viable directions found

# Safety state
uint8 safety_level                      # 0=CLEAR, 1=CAUTION, 2=WARNING, 3=CRITICAL
uint8 SAFETY_CLEAR = 0                  # No obstacles within lookahead
uint8 SAFETY_CAUTION = 1                # Obstacles detected, avoidance active
uint8 SAFETY_WARNING = 2                # Obstacles within safety radius
uint8 SAFETY_CRITICAL = 3               # Obstacles within critical radius, emergency stop

# Octomap metadata
float32 octomap_resolution              # Resolution of underlying octomap (meters)
int32 octomap_size                      # Number of occupied voxels

# Processing metadata
float32 processing_time_ms              # VFH+ computation time
