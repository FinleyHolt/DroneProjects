# GimbalCommand.msg
# Gimbal angle command for F-11 ISR 3-axis gimbal
# Angles in radians, limits from ontology (isr_extensions.kif)

# Yaw angle command: -6.02 to +6.02 rad (±345 degrees)
float64 yaw

# Pitch angle command: -2.09 to +2.09 rad (±120 degrees)
float64 pitch

# Roll angle command: -0.785 to +0.785 rad (±45 degrees)
float64 roll
