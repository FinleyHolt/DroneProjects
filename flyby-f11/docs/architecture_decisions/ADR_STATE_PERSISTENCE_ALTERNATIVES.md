# ADR-002 Addendum: State Persistence Implementation Alternatives

**Status:** Reference (documents alternatives to adopted approach)
**Date:** 2024-12-26
**Related:** ADR-002 State Persistence Architecture

## Context

After implementing ADR-002's Hot/Cold fact partitioning, we measured actual performance and evaluated whether the architectural complexity was justified.

## Measured Performance

| Operation | Requirement | Actual |
|-----------|-------------|--------|
| HotFactBuffer.get_snapshot() | <0.5ms | **0.0047ms** |
| ColdFactCache.get_cached_block() | <0.5ms | **~0ms** (cached string return) |
| Combined fact injection | <0.5ms | **0.0168ms** |
| Total query construction | part of 50ms | **<1ms** |

**Key observation:** At these speeds, the hot/cold separation provides no measurable performance benefit. The separation is purely architectural.

## Alternative Approaches (Not Implemented)

### Alternative A: Unified FactBuffer

**Approach:** Single buffer for all facts, regardless of update frequency.

```python
class FactBuffer:
    """Unified buffer for all state facts."""

    def __init__(self):
        self._facts: dict[str, FactEntry] = {}
        self._lock = threading.RLock()

    def update(self, name: str, axiom: str, ttl: Optional[float] = None):
        """Update any fact - hot or cold."""
        self._facts[name] = FactEntry(axiom, time.time(), ttl)

    def load_file(self, path: Path):
        """Load facts from TPTP file (cold facts)."""
        for axiom in parse_tptp(path):
            self.update(axiom.name, axiom.content, ttl=None)  # No expiry

    def get_snapshot(self) -> str:
        """Return all non-expired facts."""
        with self._lock:
            return '\n'.join(
                f.axiom for f in self._facts.values()
                if not f.is_expired()
            )
```

**Pros:**
- Simpler mental model (one class, one API)
- Less code to maintain
- Same performance (dict operations are O(1))

**Cons:**
- Loses semantic distinction between "load once at init" vs "update continuously"
- `needs_reload()` for file change detection becomes awkward
- Staleness thresholds less intuitive (cold facts don't go stale)

**When to prefer:** If the codebase grows and the hot/cold distinction causes confusion, or if a future developer finds the two-class pattern unintuitive.

---

### Alternative B: Fact Categories with Single Manager

**Approach:** One manager class with internal categorization.

```python
class StateFactManager:
    """Manages all state facts with internal hot/cold categorization."""

    def __init__(self):
        self._hot = {}      # Sensor-updated facts
        self._cold = {}     # File-loaded facts
        self._lock = threading.RLock()

    # Unified external API
    def update_sensor(self, name: str, axiom: str): ...
    def load_mission_facts(self, files: list[str]): ...
    def get_all_facts(self) -> str: ...

    # Internal categorization preserved for staleness/reload
    def get_stale_sensors(self) -> list[str]: ...
    def mission_files_changed(self) -> bool: ...
```

**Pros:**
- Single entry point for consumers (QueryBuilder only needs one object)
- Internal structure still supports different lifecycles
- Easier to pass around in ROS node initialization

**Cons:**
- Similar complexity to current approach, just differently organized
- May hide the hot/cold distinction too much (unclear what `update_sensor` vs `load_mission_facts` actually does)

**When to prefer:** If passing two objects (hot_buffer, cold_cache) to QueryBuilder feels awkward, or if you want a facade pattern.

---

### Alternative C: No Separation (YAGNI Approach)

**Approach:** Just use a dict. Add complexity only when proven necessary.

```python
# In QueryBuilder or VampireNode
self._state_facts: dict[str, str] = {}

def update_fact(self, name: str, axiom: str):
    self._state_facts[name] = axiom

def build_query(self, template: str) -> str:
    facts = '\n'.join(self._state_facts.values())
    return f"{facts}\n\n{template}"
```

**Pros:**
- Minimal code
- Zero abstraction overhead
- Easy to understand

**Cons:**
- No staleness detection
- No file change detection for cold facts
- No thread safety (would need to add lock)
- Debugging harder (no metadata about fact sources)

**When to prefer:** If this were a prototype or if the UAV system had simpler state management needs.

---

## Why We Kept Hot/Cold Separation

Despite the performance being fast enough for a unified approach, we retained the separation because:

1. **Semantic clarity**: Cold facts (geofence, NFZ) genuinely have a different lifecycle than hot facts (battery, position). The code reflects this reality.

2. **Staleness detection**: Only makes sense for hot facts. A unified buffer would need awkward "this fact doesn't expire" flags.

3. **File change detection**: Only makes sense for cold facts. `needs_reload()` is a meaningful operation for mission transitions.

4. **Future-proofing**: If we add more sophisticated state management (e.g., fact provenance, confidence scores), the separation provides natural extension points.

5. **Testing**: Easier to unit test hot and cold behaviors independently.

## Recommendation for Future Work

If revisiting this design:

1. **Measure first** - If performance becomes an issue, profile before refactoring
2. **Consider Alternative B** if the two-object API (hot_buffer, cold_cache) becomes unwieldy
3. **Don't simplify to Alternative C** unless removing staleness/reload features entirely

The current implementation is slightly over-engineered for the measured performance, but provides good separation of concerns for a safety-critical system where understanding data lifecycles matters.
