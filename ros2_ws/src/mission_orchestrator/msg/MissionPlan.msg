# MissionPlan.msg
# Mission plan containing waypoints and parameters for execution
#
# This message defines a complete mission that can be verified and executed
# by the mission orchestrator.

# Mission identification
string mission_id                   # Unique mission identifier
builtin_interfaces/Time created_at  # Mission creation timestamp

# Waypoints defining the mission path
mission_orchestrator/Waypoint[] waypoints

# Mission boundaries
float64 geofence_radius_m 500.0     # Maximum distance from home (default 500m)
float64 max_altitude_m 120.0        # Maximum altitude AGL (default 120m FAA limit)

# Safety parameters
float64 min_battery_percent 30.0    # Minimum battery to start mission (default 30%)
float64 rtl_battery_percent 20.0    # Battery level to trigger RTL (default 20%)

# Home position (if different from current)
float64 home_latitude 0.0           # Home latitude (0 = use current)
float64 home_longitude 0.0          # Home longitude (0 = use current)
float64 home_altitude 0.0           # Home altitude AMSL

# Optional no-fly zone references
string[] nfz_ids                    # IDs of NFZs to check against
