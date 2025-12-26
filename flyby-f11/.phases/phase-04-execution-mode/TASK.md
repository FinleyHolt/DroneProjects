# Phase 4: Vampire Runtime Integration

## Overview

Integrate the Vampire theorem prover as the single reasoning engine for both offline planning and runtime tactical queries. This phase establishes ARM64 cross-compilation, ROS 2 integration, and query caching strategies for the Jetson Orin NX deployment target.

**Prerequisite Decision:** Phase 3 evaluation determined that Vampire (~50ms latency) is acceptable for tactical reasoning at the 20Hz navigation rate, eliminating the need for Prolog translation or OWL conversion.

## Human Description

Based on Phase 3 findings, we use Vampire directly for all ontological reasoning:

1. **No translation needed** - KIF/SUMO remains single source of truth
2. **Full expressivity** - All safety axioms preserved (0% translation loss)
3. **Acceptable latency** - ~50ms fits within 20Hz navigation loop
4. **Simplified architecture** - One reasoner for planning and runtime

This phase creates:
1. ARM64 cross-compiled Vampire binary for Jetson Orin NX
2. ROS 2 reasoning service node for Vampire queries
3. Query caching/memoization for repeated tactical queries
4. Quadlet container for production deployment

### Tiered Safety Architecture Context

| Tier | Layer | Latency | Function | Handled By |
|------|-------|---------|----------|------------|
| 1 | Classical Control | <1ms | PID, motor control, attitude | PX4/ArduPilot |
| 2 | Pre-computed Safety | <10ms | Obstacle buffers, geofence | Costmaps |
| 3 | Tactical Reasoning | ~50ms | NFZ violation, battery critical | **Vampire (this phase)** |
| 4 | Mission Planning | ~1s | Route planning, compliance | **Vampire (this phase)** |

## AI Agent Instructions

### Prerequisites
- Phase 1 completed (SUMO toolchain with Vampire container)
- Phase 2 completed (UAV domain ontology)
- Phase 3 completed (evaluation confirming Vampire as single reasoner)
- Understanding of ARM64 cross-compilation
- Familiarity with ROS 2 service/action patterns

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

#### 1. ARM64 Cross-Compilation of Vampire

**Objective:** Build Vampire theorem prover for aarch64 (Jetson Orin NX).

```bash
# Location: ontology/arm64_build/
mkdir -p ontology/arm64_build

# Containerfile.vampire-arm64
FROM ubuntu:22.04 AS builder

# Install cross-compilation toolchain
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    g++-aarch64-linux-gnu \
    git \
    zlib1g-dev

# Clone and build Vampire for ARM64
RUN git clone https://github.com/vprover/vampire.git /vampire
WORKDIR /vampire
RUN mkdir build && cd build && \
    cmake .. \
      -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
      -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
      -DCMAKE_SYSTEM_NAME=Linux \
      -DCMAKE_SYSTEM_PROCESSOR=aarch64 && \
    make -j$(nproc)

# Final ARM64 container
FROM arm64v8/ubuntu:22.04
COPY --from=builder /vampire/build/bin/vampire /usr/local/bin/
ENTRYPOINT ["vampire"]
```

**Verification:**
- Binary executes on Jetson (or QEMU ARM64 emulation)
- Benchmark queries produce same results as x86 version
- Performance within 2x of x86 baseline

#### 2. Create ROS 2 Vampire Bridge Package

**Location:** `ros2_ws/src/vampire_bridge/`

**Package Structure:**
```
vampire_bridge/
├── CMakeLists.txt
├── package.xml
├── vampire_bridge/
│   ├── __init__.py
│   ├── vampire_node.py       # Main ROS 2 node
│   ├── query_builder.py      # TPTP query construction
│   ├── result_parser.py      # Vampire output parsing
│   └── cache_manager.py      # Query result caching
├── msg/
│   ├── ReasoningQuery.msg
│   └── ReasoningResult.msg
├── srv/
│   └── VampireQuery.srv
├── action/
│   └── VampireProof.action   # For long-running planning queries
├── config/
│   └── vampire_params.yaml
├── launch/
│   └── vampire_bridge.launch.py
└── test/
    └── test_vampire_bridge.py
```

**Message Definitions:**

```
# msg/ReasoningQuery.msg
string query_type        # "safety", "operational", "planning"
string query_name        # e.g., "geofence_check", "nfz_violation"
string[] parameters      # Query-specific parameters
uint32 timeout_ms        # Max execution time (default 100ms)
bool use_cache           # Whether to check cache first
```

```
# msg/ReasoningResult.msg
bool success             # Query executed successfully
string result            # "THEOREM", "COUNTER_SATISFIABLE", "TIMEOUT"
string proof_summary     # Human-readable explanation
float64 latency_ms       # Actual execution time
bool from_cache          # Whether result came from cache
string[] violated_axioms # For violation detection queries
```

```
# srv/VampireQuery.srv
ReasoningQuery query
---
ReasoningResult result
```

#### 3. Implement VampireNode

**Core Node Implementation:**

```python
# vampire_bridge/vampire_node.py
import subprocess
import tempfile
from pathlib import Path
import rclpy
from rclpy.node import Node
from vampire_bridge.msg import ReasoningQuery, ReasoningResult
from vampire_bridge.srv import VampireQuery
from vampire_bridge.cache_manager import QueryCache

class VampireNode(Node):
    def __init__(self):
        super().__init__('vampire_reasoning')

        # Parameters
        self.declare_parameter('vampire_binary', '/usr/local/bin/vampire')
        self.declare_parameter('ontology_path', '/workspace/ontology/planning_mode/uav_domain.kif')
        self.declare_parameter('tptp_queries_path', '/workspace/ontology/evaluation/benchmark_queries/')
        self.declare_parameter('default_timeout_ms', 100)
        self.declare_parameter('cache_size', 1000)
        self.declare_parameter('cache_ttl_seconds', 60)

        # Initialize cache
        self.cache = QueryCache(
            max_size=self.get_parameter('cache_size').value,
            ttl=self.get_parameter('cache_ttl_seconds').value
        )

        # Service for synchronous queries
        self.query_srv = self.create_service(
            VampireQuery,
            'vampire/query',
            self.handle_query
        )

        # Publisher for tactical reasoning results
        self.result_pub = self.create_publisher(
            ReasoningResult,
            'vampire/results',
            10
        )

        self.get_logger().info('Vampire reasoning node initialized')

    def handle_query(self, request, response):
        query = request.query

        # Check cache first
        if query.use_cache:
            cached = self.cache.get(query)
            if cached:
                response.result = cached
                response.result.from_cache = True
                return response

        # Execute Vampire query
        result = self._execute_vampire(query)

        # Cache result
        if result.success and query.use_cache:
            self.cache.put(query, result)

        response.result = result
        return response

    def _execute_vampire(self, query):
        # Build TPTP query file
        tptp_content = self._build_query(query)

        with tempfile.NamedTemporaryFile(mode='w', suffix='.tptp', delete=False) as f:
            f.write(tptp_content)
            query_file = f.name

        try:
            # Execute Vampire
            timeout_sec = query.timeout_ms / 1000.0
            result = subprocess.run(
                [
                    self.get_parameter('vampire_binary').value,
                    '--input_syntax', 'tptp',
                    '--time_limit', str(int(timeout_sec + 1)),
                    query_file
                ],
                capture_output=True,
                text=True,
                timeout=timeout_sec + 2
            )

            return self._parse_result(result, query)

        except subprocess.TimeoutExpired:
            return ReasoningResult(
                success=False,
                result='TIMEOUT',
                proof_summary='Query exceeded timeout',
                latency_ms=query.timeout_ms
            )
        finally:
            Path(query_file).unlink()
```

#### 4. Implement Query Caching Strategy

**Cache Manager:**

```python
# vampire_bridge/cache_manager.py
from collections import OrderedDict
import time
import hashlib

class QueryCache:
    """LRU cache with TTL for Vampire query results."""

    def __init__(self, max_size=1000, ttl=60):
        self.max_size = max_size
        self.ttl = ttl
        self.cache = OrderedDict()
        self.timestamps = {}

    def _key(self, query):
        """Generate cache key from query parameters."""
        content = f"{query.query_type}:{query.query_name}:{':'.join(query.parameters)}"
        return hashlib.md5(content.encode()).hexdigest()

    def get(self, query):
        key = self._key(query)
        if key not in self.cache:
            return None

        # Check TTL
        if time.time() - self.timestamps[key] > self.ttl:
            del self.cache[key]
            del self.timestamps[key]
            return None

        # Move to end (LRU)
        self.cache.move_to_end(key)
        return self.cache[key]

    def put(self, query, result):
        key = self._key(query)

        # Evict oldest if at capacity
        while len(self.cache) >= self.max_size:
            oldest_key = next(iter(self.cache))
            del self.cache[oldest_key]
            del self.timestamps[oldest_key]

        self.cache[key] = result
        self.timestamps[key] = time.time()
```

**Caching Strategy:**
- **Safety queries:** Short TTL (5-10s), high cache priority
- **Operational queries:** Medium TTL (30-60s), state-dependent
- **Planning queries:** Long TTL (5-10 min), rarely repeated

#### 5. Create Quadlet Container for Jetson Deployment

**Location:** `quadlet/vampire-reasoning.container`

```ini
[Unit]
Description=Vampire Reasoning Service for Flyby F-11
After=network.target

[Container]
Image=localhost/flyby-f11-vampire-arm64:latest
ContainerName=vampire-reasoning

# Mount ontology and ROS 2 workspace
Volume=/home/flyby/ontology:/workspace/ontology:z,ro
Volume=/home/flyby/ros2_ws:/workspace/ros2_ws:z

# ROS 2 networking
Network=host
Environment=ROS_DOMAIN_ID=42
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Resource limits
Memory=512M
CPUQuota=100%

[Service]
Restart=always
TimeoutStartSec=30

[Install]
WantedBy=default.target
```

#### 6. Benchmark ARM64 Performance

Create ARM64-specific benchmarks to validate deployment target:

```python
# ontology/arm64_build/benchmark_arm64.py
"""
Benchmark Vampire on ARM64 to validate deployment target.
Should run on Jetson Orin NX or QEMU ARM64 emulation.
"""

import subprocess
import time
import json
from pathlib import Path

QUERIES = [
    ('safety', 'geofence_check'),
    ('safety', 'nfz_violation'),
    ('safety', 'battery_return'),
    ('operational', 'valid_waypoint'),
    ('operational', 'hover_capability'),
    ('planning', 'mission_feasibility'),
]

def benchmark_query(query_type, query_name, iterations=100):
    query_path = f'/workspace/ontology/evaluation/benchmark_queries/{query_type}_{query_name}.tptp'

    times = []
    for _ in range(iterations):
        start = time.perf_counter()
        result = subprocess.run(
            ['vampire', '--input_syntax', 'tptp', '--time_limit', '5', query_path],
            capture_output=True
        )
        elapsed = (time.perf_counter() - start) * 1000  # ms
        times.append(elapsed)

    return {
        'query': f'{query_type}_{query_name}',
        'iterations': iterations,
        'mean_ms': sum(times) / len(times),
        'min_ms': min(times),
        'max_ms': max(times),
        'p95_ms': sorted(times)[int(0.95 * len(times))],
    }

if __name__ == '__main__':
    results = []
    for query_type, query_name in QUERIES:
        print(f'Benchmarking {query_type}_{query_name}...')
        results.append(benchmark_query(query_type, query_name))

    with open('/workspace/arm64_benchmark_results.json', 'w') as f:
        json.dump(results, f, indent=2)
```

**Expected ARM64 Performance:**
- Safety queries: 50-100ms (within 2x of x86)
- Operational queries: 50-100ms
- Planning queries: 100-200ms

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] Vampire ARM64 binary builds and runs on Jetson (or QEMU emulation)
- [ ] ROS 2 `vampire_bridge` package builds and installs
- [ ] VampireQuery service responds within 100ms for safety queries
- [ ] Query caching reduces repeated query latency by >90%
- [ ] Quadlet container starts automatically on Jetson boot
- [ ] ARM64 benchmarks show <2x slowdown vs x86
- [ ] All 15 benchmark queries pass on ARM64

### Verification

Run automated verification:
```bash
bash .phases/phase-04-execution-mode/verification.sh
```

### Common Pitfalls

- **ARM64 build issues**: Ensure all dependencies have ARM packages
- **Subprocess latency**: Process startup adds overhead; keep Vampire warm
- **Cache invalidation**: State changes must invalidate relevant cached queries
- **ROS 2 serialization**: Large proof outputs may need truncation
- **Memory limits**: Vampire can use significant memory for complex proofs
- **TPTP syntax**: Ensure query templates match ontology format

### References

- [Vampire Theorem Prover](https://vprover.github.io/)
- [Vampire GitHub (build instructions)](https://github.com/vprover/vampire)
- [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Quadlet Container Guide](https://docs.podman.io/en/latest/markdown/podman-systemd.unit.5.html)
- [Phase 3 Evaluation Report](../ontology/evaluation/EVALUATION_REPORT.qmd)

### Dependencies
See `dependencies.json` - requires Phase 1, 2, and 3 completion.

### Next Phase
After completion, proceed to Phase 5: Perception-to-Reasoning Bridge (updated for Vampire)
