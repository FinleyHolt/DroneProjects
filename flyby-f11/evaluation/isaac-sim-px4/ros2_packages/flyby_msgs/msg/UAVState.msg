# UAVState.msg
# Complete UAV state for autonomy decision-making
#
# Published at 60Hz by state estimator
# Combines IMU, GPS, battery, and flight controller data

std_msgs/Header header

# Position (NED frame relative to home, meters)
geometry_msgs/Point position
float32 altitude                    # Above home (positive up)
float32 altitude_agl                # Above ground level

# Velocity (NED frame, m/s)
geometry_msgs/Vector3 velocity
float32 ground_speed                # Horizontal speed magnitude

# Attitude (quaternion and euler for convenience)
geometry_msgs/Quaternion orientation
float32 roll                        # Roll angle (radians)
float32 pitch                       # Pitch angle (radians)
float32 yaw                         # Yaw angle (radians, 0=North)

# Angular rates (rad/s)
geometry_msgs/Vector3 angular_velocity

# Battery state
float32 battery_percent             # Battery level [0-100]
float32 battery_voltage             # Voltage (V)
float32 battery_current             # Current draw (A)
float32 estimated_remaining_sec     # Estimated flight time remaining

# Flight state
FlightPhase flight_phase            # Current flight phase

# Navigation state
float32 distance_to_home            # Distance to home position (meters)
float32 heading_to_home             # Bearing to home (radians)

# Comms state
bool comms_ok                       # True if comms with GCS operational
float32 comms_denied_duration_sec   # Time since comms lost (0 if ok)

# Localization state
bool localization_ok                # True if position estimate reliable
float32 position_uncertainty        # Position std dev (meters)

# Flags
bool armed                          # True if motors armed
bool in_air                         # True if airborne
bool home_position_valid            # True if home position set
