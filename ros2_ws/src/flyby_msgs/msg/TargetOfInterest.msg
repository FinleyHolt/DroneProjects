# TargetOfInterest.msg
# Defines a target of interest for ISR dwell operations
#
# Represents a point or area requiring detailed observation

# Identification
string target_id                             # Unique target identifier
string target_type                           # "vehicle", "structure", "person", "unknown"
uint8 priority                               # Target priority (1=highest, 255=lowest)

# Position
geometry_msgs/PoseStamped position           # Target position and orientation
geometry_msgs/Vector3 velocity               # Estimated target velocity (for moving targets)
float64 position_uncertainty_m               # Position uncertainty radius in meters

# Observation requirements
float64 required_dwell_time_sec              # Required dwell time in seconds
float64 accumulated_dwell_time_sec           # Dwell time accumulated so far
float64 min_gsd_cm                           # Minimum ground sample distance (image quality)
string[] required_sensors                    # Required sensors ("eo", "ir", "sar")

# Classification
float64 confidence                           # Target classification confidence [0.0, 1.0]
string classification                        # Current classification result
bool is_confirmed                            # True if target has been confirmed by operator

# Status
bool is_active                               # True if target is currently being tracked
bool dwell_complete                          # True if dwell requirements are met
builtin_interfaces/Time first_detected       # When target was first detected
builtin_interfaces/Time last_observed        # Last observation time
int32 observation_count                      # Number of times target has been observed
