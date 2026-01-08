# AreaOfOperations.msg
# Defines an area of operations for ISR missions
#
# Represents a polygonal region with altitude bounds and priority

# Identification
string ao_id                                 # Unique AO identifier
string ao_name                               # Human-readable name
uint8 priority                               # AO priority (1=highest, 255=lowest)

# Geometry
geometry_msgs/Polygon boundary               # 2D boundary polygon (vertices in order)
float64 min_altitude_m                       # Minimum altitude AGL in meters
float64 max_altitude_m                       # Maximum altitude AGL in meters

# Entry/exit points
geometry_msgs/Point[] entry_points           # Suggested entry points
geometry_msgs/Point[] exit_points            # Suggested exit points

# Coverage requirements
float64 coverage_target_pct                  # Target coverage percentage
float64 current_coverage_pct                 # Current coverage percentage
float64 grid_resolution_m                    # Coverage grid resolution in meters

# Search pattern
string recommended_pattern                   # "lawnmower", "spiral", "expanding_square", "rl_adaptive"
float64 recommended_altitude_m               # Recommended search altitude
float64 recommended_speed_mps                # Recommended search speed

# Status
bool is_searched                             # True if AO has been fully searched
builtin_interfaces/Time last_visited         # Last time this AO was visited
