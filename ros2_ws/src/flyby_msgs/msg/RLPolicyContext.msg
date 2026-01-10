# RLPolicyContext.msg
# Context information passed to RL policy nodes in the behavior tree
#
# Contains all information needed by SearchPolicy and DwellPolicy

# Header
std_msgs/Header header

# Policy identification
string policy_type                           # "search" or "dwell"
bool is_active                               # True if policy is currently executing

# UAV State (from state estimator)
geometry_msgs/PoseStamped current_pose       # Current position and orientation
geometry_msgs/TwistStamped current_velocity  # Current linear and angular velocity
float64 battery_pct                          # Current battery percentage
float64 flight_time_sec                      # Time since takeoff

# Gimbal state
float64 gimbal_pan_rad                       # Current gimbal pan angle
float64 gimbal_tilt_rad                      # Current gimbal tilt angle
float64 gimbal_pan_rate_limit               # Maximum pan rate (rad/s)
float64 gimbal_tilt_rate_limit              # Maximum tilt rate (rad/s)

# Search context (for SearchPolicy)
AreaOfOperations search_area                 # Current search area
float64 coverage_pct                         # Current coverage percentage
geometry_msgs/Point[] frontier_points        # Uncovered frontier points
int32 targets_detected                       # Number of targets detected this mission

# Dwell context (for DwellPolicy)
TargetOfInterest current_target              # Target being tracked (if any)
geometry_msgs/Point target_image_position    # Target position in image frame [u, v, depth]
float64 track_confidence                     # Current track confidence [0.0, 1.0]
float64 dwell_time_remaining_sec             # Remaining dwell time required

# Constraints
geometry_msgs/Polygon[] active_nfz           # Currently active NFZ polygons
ThreatZone[] nearby_threats                  # Threats within sensor range
float64 max_velocity_mps                     # Maximum allowed velocity
float64 min_altitude_m                       # Minimum allowed altitude
float64 max_altitude_m                       # Maximum allowed altitude

# Termination conditions
bool should_terminate                        # True if policy should terminate
string termination_reason                    # Reason for termination if applicable
