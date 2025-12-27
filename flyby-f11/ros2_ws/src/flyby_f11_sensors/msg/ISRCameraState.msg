# ISRCameraState.msg
# State of the F-11 ISR camera payload (Sony A7R-class)

# Standard header with timestamp
std_msgs/Header header

# Camera active status
bool camera_active

# Current imaging mode
string mode  # "rgb", "thermal", "hybrid"

# Zoom level (1.0 to 20.0 for optical zoom)
float64 zoom_level

# Current exposure setting
float64 exposure

# Camera operational status
bool operational

# Image dimensions (from isr_extensions.kif: 1920x1080)
uint32 image_width
uint32 image_height

# Frame rate (from isr_extensions.kif: 30 Hz)
float64 frame_rate
