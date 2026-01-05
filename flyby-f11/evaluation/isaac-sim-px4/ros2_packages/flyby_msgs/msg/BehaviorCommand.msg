# BehaviorCommand.msg
# Ontology-driven behavior command that preempts RL control
#
# Published by flyby_autonomy when ontology axioms trigger behavior

builtin_interfaces/Time stamp

# Behavior types (from ontology axioms)
uint8 NONE = 0                      # No behavior active, RL in control
uint8 TAKEOFF = 1                   # Ontology-controlled takeoff
uint8 RTL = 2                       # Return to launch
uint8 EMERGENCY_LAND = 3            # Immediate landing
uint8 HOVER = 4                     # Station keep
uint8 GEOFENCE_RECOVERY = 5         # Return to safe area
uint8 NFZ_AVOIDANCE = 6             # Back away from no-fly zone
uint8 PERSON_AVOIDANCE = 7          # Maintain distance from person

uint8 behavior_type                 # Active behavior

# Priority levels (determines preemption)
uint8 PRIORITY_LOW = 1              # Soft constraint (penalized but allowed)
uint8 PRIORITY_MEDIUM = 2           # Should trigger soon
uint8 PRIORITY_HIGH = 3             # Must trigger soon
uint8 PRIORITY_CRITICAL = 4         # Immediate action required

uint8 priority                      # Command priority

# Triggering axiom (for logging/debugging)
string axiom_name                   # Ontology axiom that triggered (e.g. "batteryReserveReturn")
string trigger_reason               # Human-readable explanation

# Target (if applicable)
geometry_msgs/Point target_position # Target position for behavior
float32 target_altitude             # Target altitude (meters)
bool has_target                     # True if target_position valid

# Velocity command (output of behavior executor)
geometry_msgs/Twist velocity_setpoint   # Velocity command (m/s, rad/s)
bool velocity_valid                     # True if velocity_setpoint computed

# Behavior status
bool active                         # True if behavior currently executing
bool completed                      # True if behavior objective achieved
float32 duration_sec                # Time behavior has been active
