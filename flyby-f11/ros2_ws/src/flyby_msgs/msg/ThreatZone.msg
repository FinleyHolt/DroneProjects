# ThreatZone.msg
# Defines a threat zone with associated risk level
#
# Used for soft-constraint path planning (prefer avoidance but allow transit if necessary)

# Identification
string zone_id                               # Unique zone identifier
string zone_type                             # "sam", "aaa", "small_arms", "electronic", "unknown"

# Geometry
geometry_msgs/Polygon boundary               # 2D boundary polygon
float64 min_altitude_m                       # Minimum effective altitude AGL
float64 max_altitude_m                       # Maximum effective altitude AGL

# Threat characteristics
uint8 threat_level                           # 1=low, 2=medium, 3=high, 4=critical
float64 detection_range_m                    # Estimated detection range
float64 engagement_range_m                   # Estimated engagement range
float64 lethality                            # Estimated lethality [0.0, 1.0]

# Path planning costs
float64 cost_multiplier                      # Cost multiplier for path planning (1.0 = normal)
float64 exposure_penalty                     # Penalty per second of exposure

# Status
bool is_active                               # True if threat is currently active
bool is_confirmed                            # True if threat has been confirmed
builtin_interfaces/Time last_updated         # Last update time
string source                                # Intelligence source ("sigint", "humint", "imint", "estimated")
