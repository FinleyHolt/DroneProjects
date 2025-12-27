# ReasoningQuery.msg
# Query message for the Vampire theorem prover reasoning service
#
# Supports three query types for tiered safety architecture:
#   - "safety": High-priority tactical queries (~50ms target)
#   - "operational": Medium-priority operational queries (~50ms target)
#   - "planning": Lower-priority mission planning queries (~200ms acceptable)

# Query classification
string query_type               # "safety", "operational", "planning"
string query_name               # Template name e.g., "geofence_check", "nfz_violation"

# Query parameters (template substitution values)
string[] parameters             # Query-specific parameters as key=value strings

# Execution options
uint32 timeout_ms 100           # Max execution time in milliseconds (default 100ms)
bool use_cache true             # Whether to check cache first (default true)

# Optional raw TPTP content (overrides query_name if provided)
string raw_tptp                 # Direct TPTP content, bypasses template lookup
