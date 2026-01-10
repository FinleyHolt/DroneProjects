# MissionTasking.msg
# Mission tasking information for ISR operations
#
# Defines the current mission objectives, areas of operation, and priorities

# Header
std_msgs/Header header

# Mission identification
string mission_id                            # Unique mission identifier
string mission_type                          # "area_search", "point_surveillance", "route_reconnaissance"
uint8 priority                               # Mission priority (1=highest, 255=lowest)

# Area of Operations
AreaOfOperations area_of_operations          # Primary AO definition

# Points of Interest
TargetOfInterest[] targets_of_interest       # Known targets requiring attention

# Constraints
geometry_msgs/Polygon[] no_fly_zones         # Hard NFZ constraints
ThreatZone[] threat_zones                    # Soft threat constraints

# Mission parameters
float64 min_coverage_pct                     # Minimum coverage percentage required
float64 min_dwell_time_sec                   # Minimum dwell time per target
float64 max_mission_time_sec                 # Maximum mission duration
float64 battery_reserve_pct                  # Battery reserve for RTB

# Home base
geometry_msgs/PoseStamped home_base          # RTB destination

# Status
bool is_active                               # True if mission is currently active
builtin_interfaces/Time start_time           # Mission start time
builtin_interfaces/Time deadline             # Mission completion deadline
