# ReasoningResult.msg
# Result message from the Vampire theorem prover reasoning service
#
# SZS Status codes (subset used):
#   - THEOREM: Conjecture is proven (safety violation detected, condition satisfied)
#   - COUNTER_SATISFIABLE: Conjecture is disproven (no violation, condition not met)
#   - SATISFIABLE: Axioms are consistent but conjecture neither proven nor disproven
#   - TIMEOUT: Query exceeded time limit
#   - ERROR: Execution or parsing error

# Execution status
bool success                    # True if query executed without errors
string szs_status               # SZS status: "THEOREM", "COUNTER_SATISFIABLE", "TIMEOUT", etc.

# Result details
string proof_summary            # Human-readable explanation of the result
string[] violated_axioms        # Names of axioms involved in proving violation (if applicable)

# Performance metrics
float64 latency_ms              # Actual execution time in milliseconds
bool from_cache                 # True if result was served from cache

# Debug information
string vampire_output           # Raw output from Vampire (truncated if too long)
string error_message            # Error details if success is false
