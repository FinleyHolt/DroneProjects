# ADR-002: State Persistence Architecture for Vampire Bridge

**Status:** Accepted
**Date:** 2024-12-26
**Decision Makers:** Finley Holt

## Context

The Flyby F-11 UAV autonomy platform uses the Vampire theorem prover for runtime safety reasoning via the `vampire_bridge` ROS 2 node. Per ADR-001, Vampire handles tactical reasoning at the 20Hz navigation rate with target latency of 50ms (p95).

### Current Architecture

The `vampire_node.py` component:
- Receives reasoning queries via ROS 2 service interface
- Builds TPTP queries from templates with parameter substitution
- Executes Vampire via subprocess with temporary TPTP files
- Parses results and returns to caller with caching

### Problem Statement

**State facts** (battery level, position, sensor status, comms status, geofence containment) must be grounded as TPTP facts for Vampire reasoning. The current architecture has no defined protocol for state fact management:

1. Template files contain hardcoded state facts (e.g., `fof(battery_state, axiom, has_battery_status(drone_alpha, battery_critical)).`)
2. Dynamic state cannot be injected at runtime without modifying templates
3. No mechanism to update facts between queries at 20Hz
4. Potential consistency issues if state changes mid-query

### Requirements

| Requirement | Constraint |
|-------------|------------|
| Query rate | 20Hz (50ms period) |
| Target latency | <50ms p95 per query |
| Deployment target | Jetson Orin NX 16GB |
| State update rate | Variable (10-50Hz from sensors) |
| Concurrent queries | Up to 3 (safety, operational, planning) |
| Memory budget | <100MB for vampire_bridge |

### State Fact Categories

| Category | Update Frequency | Example Facts |
|----------|------------------|---------------|
| Hot (volatile) | 10-50Hz | position, velocity, obstacle_distance |
| Warm (periodic) | 1-5Hz | battery_percent, sensor_status, comms_status |
| Cold (rare) | <0.1Hz | geofence_boundary, nfz_definitions, mission_parameters |

## Options Analyzed

### Option 1: File-Based State Persistence

**Approach:** Write state facts to a dedicated `.tptp` file on each update. Include this file in all queries via TPTP `include()` directive.

```
state_facts.tptp (written by state monitor)
├── fof(battery_state, axiom, has_battery_status(drone_alpha, battery_low)).
├── fof(position_state, axiom, position(drone_alpha, pos_47_122_100)).
├── fof(comms_state, axiom, has_comms_status(drone_alpha, comms_denied)).
└── fof(obstacle_dist, axiom, min_obstacle_distance(drone_alpha, 8.5)).

query.tptp (generated per query)
├── include('state_facts.tptp').
├── include('common_axioms.tptp').
└── fof(check_safety, conjecture, safe_state(drone_alpha)).
```

**Analysis:**

| Criterion | Assessment |
|-----------|------------|
| Latency impact | **High negative.** File I/O adds 1-5ms per write at 20Hz. Jetson eMMC write amplification degrades over time. Filesystem sync overhead unpredictable. |
| Memory footprint | **Minimal.** State file <10KB. No in-process memory growth. |
| Consistency guarantees | **Weak.** Race condition between state write and query read. No atomic update mechanism. State file may be partially written when included. |
| Implementation complexity | **Low.** Simple file write/read. Leverages existing TPTP include mechanism. |

**Latency Breakdown:**
- File write (4KB): ~1-3ms (Jetson eMMC)
- File sync: ~1-5ms (varies by filesystem state)
- Vampire include processing: ~0.5ms
- Total overhead: 2.5-8.5ms per query cycle

**Verdict:** Rejected. File I/O latency inconsistent and degrades SSD lifespan. Race conditions unacceptable for safety-critical queries.

---

### Option 2: In-Memory Buffer with Template Injection

**Approach:** Maintain state facts in Python dictionary. Inject facts directly into TPTP query string before each Vampire invocation.

```python
class StateBuffer:
    def __init__(self):
        self._facts: dict[str, str] = {}
        self._lock = threading.RLock()

    def update(self, fact_name: str, tptp_axiom: str) -> None:
        with self._lock:
            self._facts[fact_name] = tptp_axiom

    def get_facts_block(self) -> str:
        with self._lock:
            return '\n'.join(self._facts.values())

# Query construction
def build_query(template: str, state_buffer: StateBuffer) -> str:
    state_block = state_buffer.get_facts_block()
    return f"{STATE_HEADER}\n{state_block}\n{STATE_FOOTER}\n{template}"
```

**Analysis:**

| Criterion | Assessment |
|-----------|------------|
| Latency impact | **Negligible.** String concatenation <0.1ms. No I/O. Dictionary access O(1). |
| Memory footprint | **Low.** ~50-100 facts at ~200 bytes each = ~20KB steady state. Thread-safe lock adds minimal overhead. |
| Consistency guarantees | **Strong within query.** Lock ensures atomic snapshot of state at query time. State may change between queries (acceptable for 20Hz). |
| Implementation complexity | **Moderate.** Requires state buffer class, thread synchronization, integration with QueryBuilder. |

**Latency Breakdown:**
- Lock acquisition: ~0.01ms
- Dictionary iteration: ~0.02ms (100 facts)
- String join: ~0.05ms
- Total overhead: <0.1ms per query

**Verdict:** Strong candidate. Meets all latency requirements. Simple mental model.

---

### Option 3: Fact Delta Tracking

**Approach:** Track which facts changed since last query. Only include changed facts plus a compact "base state" reference. Vampire caches unchanged axioms internally.

```python
class DeltaTracker:
    def __init__(self):
        self._current: dict[str, str] = {}
        self._last_sent: dict[str, str] = {}
        self._base_state_hash: str = ""

    def update(self, fact_name: str, tptp_axiom: str) -> None:
        self._current[fact_name] = tptp_axiom

    def get_delta(self) -> tuple[str, list[str]]:
        changed = []
        for name, axiom in self._current.items():
            if self._last_sent.get(name) != axiom:
                changed.append(axiom)
        self._last_sent = self._current.copy()
        return (self._base_state_hash, changed)
```

**Analysis:**

| Criterion | Assessment |
|-----------|------------|
| Latency impact | **Marginal improvement.** Vampire reparses full file regardless; internal caching minimal. Query size reduction ~20-50% but parsing not linear with size. |
| Memory footprint | **Higher.** Requires both current and last-sent copies. ~40KB steady state. |
| Consistency guarantees | **Weak.** Delta assumes Vampire maintains state between invocations. Vampire is stateless per invocation; each subprocess starts fresh. |
| Implementation complexity | **High.** Delta tracking logic, hash computation, state reconstruction on Vampire side (not possible with subprocess model). |

**Critical Flaw:** Vampire is invoked as a subprocess for each query. There is no persistent Vampire process to maintain state between invocations. Deltas require the receiver to have prior state, which Vampire does not retain.

**Verdict:** Rejected. Fundamentally incompatible with subprocess invocation model. Would require persistent Vampire daemon (not supported in current architecture).

---

### Option 4: Hybrid Hot/Cold Partitioning

**Approach:** Partition facts by volatility. Hot facts (position, velocity) in memory buffer. Cold facts (geofence, NFZ definitions) in static files loaded once at startup.

```
Architecture:
┌────────────────────────────────────────────────────────┐
│                    QueryBuilder                         │
├─────────────────────┬──────────────────────────────────┤
│   Cold Facts        │         Hot Facts                │
│   (file-based)      │         (memory buffer)          │
├─────────────────────┼──────────────────────────────────┤
│ - geofence.tptp     │ StateBuffer:                     │
│ - nfz_zones.tptp    │ - position (50Hz)                │
│ - mission_params    │ - velocity (50Hz)                │
│ - common_axioms     │ - battery_status (5Hz)           │
│                     │ - comms_status (1Hz)             │
│ Loaded once at init │ - sensor_health (10Hz)           │
│ Cached in memory    │ Updated via ROS 2 subscribers    │
└─────────────────────┴──────────────────────────────────┘
                      │
                      v
              ┌───────────────────┐
              │  Combined Query   │
              │  cold + hot +     │
              │  query template   │
              └───────────────────┘
```

**Analysis:**

| Criterion | Assessment |
|-----------|------------|
| Latency impact | **Minimal.** Cold facts cached in memory after first load. Hot facts from buffer. No per-query I/O. |
| Memory footprint | **Moderate.** Cold facts ~50-100KB (cached once). Hot facts ~20KB. Total ~120KB. |
| Consistency guarantees | **Strong.** Cold facts immutable during mission. Hot facts atomic snapshot per query. Clear separation of concerns. |
| Implementation complexity | **Moderate-High.** Two subsystems (file cache + memory buffer). Clear API boundaries. Requires cold fact reload mechanism for mission changes. |

**Latency Breakdown:**
- Cold facts: 0ms (pre-cached)
- Hot facts: <0.1ms (memory buffer)
- Query assembly: ~0.1ms
- Total overhead: ~0.2ms per query

**Verdict:** Strong candidate. Optimal for Jetson deployment. Separates concerns cleanly.

---

## Decision

**We adopt Option 4: Hybrid Hot/Cold Partitioning** with the following architecture:

### State Fact Manager Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        StatefactManager                              │
├──────────────────────────────────┬──────────────────────────────────┤
│         ColdFactCache            │          HotFactBuffer           │
├──────────────────────────────────┼──────────────────────────────────┤
│ load_cold_facts(mission_config)  │ update(fact_name, tptp_axiom)   │
│ reload_on_mission_change()       │ get_snapshot() -> dict          │
│ get_cached_block() -> str        │ clear()                         │
├──────────────────────────────────┼──────────────────────────────────┤
│ Sources:                         │ Sources (ROS 2 subscribers):     │
│ - geofence_boundary.tptp         │ - /mavros/battery (5Hz)         │
│ - nfz_definitions.tptp           │ - /mavros/local_position (50Hz) │
│ - mission_parameters.tptp        │ - /mavros/state (1Hz)           │
│ - common_axioms.tptp             │ - /perception/obstacle_dist     │
│                                  │ - /comms/status                 │
│ Loaded: Mission init             │ - /sensors/health               │
│ Reloaded: Mission change event   │                                  │
│ Memory: ~100KB                   │ Memory: ~20KB                   │
└──────────────────────────────────┴──────────────────────────────────┘
                                   │
                                   v
                    ┌──────────────────────────────┐
                    │       QueryBuilder           │
                    │  build_query(template,       │
                    │              statefact_mgr)  │
                    │                              │
                    │  Output: cold_block +        │
                    │          hot_snapshot +      │
                    │          query_template      │
                    └──────────────────────────────┘
```

### Implementation Specification

**HotFactBuffer Class:**

```python
class HotFactBuffer:
    """Thread-safe buffer for volatile state facts updated at sensor rate."""

    FACT_SCHEMA = {
        'battery_status': 'has_battery_status({uav}, {status})',
        'position': 'position({uav}, {pos_id})',
        'comms_status': 'has_comms_status({uav}, {status})',
        'obstacle_distance': 'min_obstacle_distance({uav}, {dist})',
        'sensor_health': 'sensor_operational({uav}, {sensor})',
        'localization_quality': 'has_position_quality({uav}, {quality})',
    }

    def __init__(self, uav_id: str = 'drone_alpha'):
        self._uav_id = uav_id
        self._facts: dict[str, str] = {}
        self._timestamps: dict[str, float] = {}
        self._lock = threading.RLock()

    def update(self, fact_type: str, **kwargs) -> None:
        """Update a hot fact with new values."""
        with self._lock:
            template = self.FACT_SCHEMA.get(fact_type)
            if template:
                axiom = f"fof({fact_type}_state, axiom, {template.format(uav=self._uav_id, **kwargs)})."
                self._facts[fact_type] = axiom
                self._timestamps[fact_type] = time.time()

    def get_snapshot(self) -> str:
        """Return atomic snapshot of all hot facts as TPTP block."""
        with self._lock:
            return '\n'.join(self._facts.values())

    def get_stale_facts(self, max_age_sec: float = 1.0) -> list[str]:
        """Return list of facts older than max_age_sec."""
        now = time.time()
        with self._lock:
            return [k for k, ts in self._timestamps.items()
                    if now - ts > max_age_sec]
```

**ColdFactCache Class:**

```python
class ColdFactCache:
    """Cache for static/rarely-changing facts loaded from files."""

    def __init__(self, base_path: Path):
        self._base_path = base_path
        self._cached_block: str = ""
        self._file_hashes: dict[str, str] = {}

    def load(self, file_list: list[str]) -> None:
        """Load and cache cold fact files."""
        blocks = []
        for filename in file_list:
            filepath = self._base_path / filename
            if filepath.exists():
                content = filepath.read_text()
                self._file_hashes[filename] = hashlib.md5(content.encode()).hexdigest()
                blocks.append(f"% Included from {filename}\n{content}")
        self._cached_block = '\n'.join(blocks)

    def get_cached_block(self) -> str:
        """Return cached cold facts."""
        return self._cached_block

    def needs_reload(self) -> bool:
        """Check if any source file changed."""
        for filename, old_hash in self._file_hashes.items():
            filepath = self._base_path / filename
            if filepath.exists():
                new_hash = hashlib.md5(filepath.read_text().encode()).hexdigest()
                if new_hash != old_hash:
                    return True
        return False
```

### Query Assembly Flow

```
1. ROS 2 subscribers update HotFactBuffer (async, 10-50Hz)
2. Query request received via /vampire/query service
3. QueryBuilder invoked:
   a. cold_block = cold_cache.get_cached_block()      # 0ms (cached)
   b. hot_block = hot_buffer.get_snapshot()           # <0.1ms (atomic)
   c. template = load_template(query_type, query_name) # <0.1ms (cached)
   d. query = assemble(cold_block, hot_block, template)
4. Write assembled query to temp file
5. Invoke Vampire subprocess
6. Parse result, cache, return
```

### Consistency Model

**Guarantee:** Each query sees an atomic snapshot of hot facts at query construction time. State may change between query submission and result return (acceptable for 20Hz tactical reasoning).

**Stale Fact Detection:** HotFactBuffer tracks timestamps. If a fact exceeds `max_age_sec` (configurable, default 1.0s), it is flagged as stale. Stale facts trigger a warning but do not block queries (fail-open for availability).

**Cold Fact Immutability:** Cold facts are immutable during mission execution. Reload occurs only on explicit mission change events (not during active flight).

## Consequences

### Positive

1. **Latency:** Query overhead <0.2ms, well within 50ms budget
2. **Memory:** Total ~120KB, negligible on 16GB Jetson
3. **Consistency:** Atomic snapshots prevent partial state reads
4. **Separation of concerns:** Hot/cold partitioning matches natural data lifecycle
5. **No SSD wear:** Hot facts never written to disk during operation
6. **Debuggability:** Cold facts in files are human-readable and version-controlled

### Negative

1. **Complexity:** Two subsystems to maintain (ColdFactCache, HotFactBuffer)
2. **Staleness risk:** If ROS 2 subscriber fails, hot facts become stale silently
3. **Cold fact rigidity:** Cannot update geofence/NFZ during mission without reload

### Mitigations

| Risk | Mitigation |
|------|------------|
| Stale facts | `get_stale_facts()` method with configurable threshold. Warning logged if critical facts stale >1s. |
| Subscriber failure | Health check via `/diagnostics`. Fallback to safe state if position fact stale >2s. |
| Cold fact changes | Mission replanning triggers `reload_on_mission_change()`. Not permitted during active flight. |

## Implementation Notes

### Integration with Existing QueryBuilder

Modify `QueryBuilder.build_query()` to accept `StateFactManager` instance:

```python
def build_query(
    self,
    query_type: str,
    query_name: str,
    parameters: list,
    statefact_manager: StateFactManager,  # NEW
    raw_tptp: Optional[str] = None
) -> str:
    if raw_tptp:
        return self._wrap_raw_query(raw_tptp)

    # Load template
    template = self._load_template(query_type, query_name)

    # Get state facts
    cold_block = statefact_manager.cold_cache.get_cached_block()
    hot_block = statefact_manager.hot_buffer.get_snapshot()

    # Substitute parameters
    query = self._substitute_placeholders(template, self._parse_parameters(parameters))

    # Assemble final query
    return f"{cold_block}\n\n% === Hot State Facts ===\n{hot_block}\n\n{query}"
```

### ROS 2 Subscriber Setup

```python
# In VampireNode.__init__()
self.statefact_manager = StateFactManager(
    cold_facts_path=Path(self.ontology_path),
    uav_id='drone_alpha'
)

# Battery subscriber
self.create_subscription(
    BatteryState,
    '/mavros/battery',
    self._on_battery,
    qos_profile_sensor_data
)

def _on_battery(self, msg: BatteryState):
    if msg.percentage < 0.20:
        status = 'battery_critical'
    elif msg.percentage < 0.30:
        status = 'battery_low'
    else:
        status = 'battery_nominal'
    self.statefact_manager.hot_buffer.update('battery_status', status=status)
```

## References

- [ADR-001: Single-Reasoner Architecture](ADR-001-vampire-only-reasoner.md)
- [APPROACH.qmd](../../APPROACH.qmd) - Tiered safety architecture
- [vampire_node.py](../../ros2_ws/src/vampire_bridge/vampire_bridge/vampire_node.py) - Current implementation
- [query_builder.py](../../ros2_ws/src/vampire_bridge/vampire_bridge/query_builder.py) - Query construction

## Appendix: Latency Budget Analysis

```
Total 50ms budget allocation:

Query Construction:
  - Cold facts retrieval:     0.0ms (cached)
  - Hot facts snapshot:       0.1ms
  - Template loading:         0.1ms (cached)
  - Parameter substitution:   0.05ms
  - String assembly:          0.05ms
  Subtotal:                   0.3ms

File I/O:
  - Write temp file:          1.0ms
  Subtotal:                   1.0ms

Vampire Execution:
  - Process spawn:            2.0ms
  - TPTP parsing:             3.0ms
  - Theorem proving:          40.0ms (p95)
  - Result output:            0.5ms
  Subtotal:                   45.5ms

Result Processing:
  - Parse output:             0.5ms
  - Cache update:             0.1ms
  Subtotal:                   0.6ms

TOTAL:                        47.4ms (within 50ms budget)
MARGIN:                       2.6ms
```

The hybrid architecture leaves sufficient margin for variance in Vampire execution time while eliminating file I/O overhead for hot facts.
