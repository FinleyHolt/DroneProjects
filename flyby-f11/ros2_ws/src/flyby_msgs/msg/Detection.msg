# Detection.msg
# Single object detection from YOLO or ground truth
#
# Used for ISR (Intelligence, Surveillance, Reconnaissance) perception
# Published by flyby_perception node at 10Hz

builtin_interfaces/Time stamp

# Detection identification
int32 class_id                      # Internal class ID (1=person, 2=car, etc.)
string class_name                   # Human-readable name (person, car, truck)
string ontology_class               # Ontology class (Person, DynamicObstacle, etc.)
float32 confidence                  # Detection confidence [0-1]

# Bounding box in pixel coordinates (xyxy format)
float32 bbox_x1                     # Top-left x pixel
float32 bbox_y1                     # Top-left y pixel
float32 bbox_x2                     # Bottom-right x pixel
float32 bbox_y2                     # Bottom-right y pixel

# Image dimensions for normalization
int32 image_width
int32 image_height

# 3D position in world frame (meters, NED or ENU depending on config)
geometry_msgs/Point position_3d
bool has_position_3d                # True if position_3d is valid

# Distance and bearing from UAV
float32 distance                    # Distance from UAV (meters)
float32 bearing                     # Bearing angle from UAV heading (radians)

# Velocity estimate (if tracking enabled)
geometry_msgs/Vector3 velocity      # Estimated velocity (m/s)
bool has_velocity                   # True if velocity estimate available

# Tracking
int32 track_id                      # Persistent track ID (-1 if not tracked)

# Priority and safety
int32 priority                      # Priority level (1=highest, 10=lowest)
float32 safety_distance             # Required safety distance (meters)
