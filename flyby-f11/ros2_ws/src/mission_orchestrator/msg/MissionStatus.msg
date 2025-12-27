# MissionStatus.msg
# Current mission execution status
#
# Published periodically (1Hz) and on state transitions

builtin_interfaces/Time stamp       # Timestamp of this status

# Mission identification
string mission_id                   # Active mission ID (empty if none)

# State machine status
uint8 STATE_IDLE = 0
uint8 STATE_RECEIVING = 1
uint8 STATE_VERIFYING = 2
uint8 STATE_VERIFIED = 3
uint8 STATE_ARMING = 4
uint8 STATE_EXECUTING = 5
uint8 STATE_PAUSED = 6
uint8 STATE_RETURNING = 7
uint8 STATE_LANDING = 8
uint8 STATE_COMPLETED = 9
uint8 STATE_ABORTED = 10
uint8 state                         # Current mission state

# Execution progress
uint32 current_waypoint_idx         # Index of current target waypoint
uint32 total_waypoints              # Total waypoints in mission
float32 mission_progress_percent    # Overall mission progress (0-100)

# Safety status
bool safety_ok                      # True if no active safety violations
string[] active_violations          # List of active safety violations

# Performance metrics
float64 last_safety_check_latency_ms  # Latency of most recent safety check
float64 avg_safety_check_latency_ms   # Average safety check latency

# Additional info
string status_message               # Human-readable status message
