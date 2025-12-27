# SafetyState.msg
# Safety monitoring status from Vampire tactical queries
#
# Published at 20Hz during active flight states

builtin_interfaces/Time stamp       # Timestamp of safety check

# Overall safety status
bool safe                           # True if all safety checks passed
string[] violations                 # List of detected violations

# Individual safety check results
bool geofence_ok                    # True if within geofence
bool nfz_ok                         # True if not in no-fly zone
bool battery_ok                     # True if battery above threshold
bool comms_ok                       # True if communications operational
bool localization_ok                # True if position estimate reliable

# Recommended action
uint8 ACTION_NONE = 0               # Continue mission
uint8 ACTION_WARN = 1               # Log warning, continue
uint8 ACTION_PAUSE = 2              # Pause mission, await operator
uint8 ACTION_RTL = 3                # Return to launch
uint8 ACTION_ABORT = 4              # Immediate abort (land in place)
uint8 recommended_action            # Recommended action based on violations

# Performance
float64 latency_ms                  # Safety query execution latency
bool from_cache                     # True if result from cache
