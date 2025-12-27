# Waypoint.msg
# Single waypoint in a mission plan

float64 latitude                    # Latitude in degrees
float64 longitude                   # Longitude in degrees
float64 altitude                    # Altitude in meters (AMSL or AGL based on frame)

# Waypoint behavior
float64 hold_time_sec 0.0           # Time to hold at waypoint (default 0)
float64 acceptance_radius_m 5.0     # Radius to consider waypoint reached (default 5m)
float64 speed_m_s 0.0               # Target speed to waypoint (0 = use default)

# Waypoint type
uint8 TYPE_NAVIGATE = 0             # Navigate to waypoint
uint8 TYPE_LOITER = 1               # Loiter at waypoint
uint8 TYPE_LAND = 2                 # Land at waypoint
uint8 type 0                        # Waypoint type (default: navigate)
