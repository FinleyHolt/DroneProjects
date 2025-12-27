# GimbalState.msg
# Current state of the F-11 ISR 3-axis gimbal
# Includes current angles and limits from ontology (isr_extensions.kif)

# Standard header with timestamp
std_msgs/Header header

# Current yaw angle in radians
float64 yaw

# Current pitch angle in radians
float64 pitch

# Current roll angle in radians
float64 roll

# Yaw limits [min, max] in radians (-6.02, +6.02)
float64[2] yaw_limits

# Pitch limits [min, max] in radians (-2.09, +2.09)
float64[2] pitch_limits

# Roll limits [min, max] in radians (-0.785, +0.785)
float64[2] roll_limits

# Gimbal operational status
bool operational
