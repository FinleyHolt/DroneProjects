# FlightPhase.msg
# Current flight phase in the ontology state machine
#
# Maps to OntologyBehaviorController phases

builtin_interfaces/Time stamp

# Flight phases (hierarchical state machine)
uint8 PREFLIGHT = 0                 # Ground checks, pre-arm
uint8 ARMED = 1                     # Armed, ready for takeoff
uint8 TAKEOFF = 2                   # Ascending to mission altitude
uint8 MISSION_EXECUTION = 3         # Active mission (RL/waypoint)
uint8 RTL = 4                       # Return to launch
uint8 LANDING = 5                   # Final descent
uint8 DISARMED = 6                  # Post-flight, disarmed
uint8 EMERGENCY_LAND = 7            # Emergency landing in progress

uint8 phase                         # Current phase

# Phase timing
float32 phase_duration_sec          # Time in current phase
float32 mission_elapsed_sec         # Total mission time

# Phase-specific data
float32 target_altitude             # Target altitude for current phase (meters)
bool phase_complete                 # True if phase objective achieved

# Transition info
string transition_reason            # Reason for last phase transition
