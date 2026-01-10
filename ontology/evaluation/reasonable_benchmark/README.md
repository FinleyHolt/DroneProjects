# Reasonable OWL 2 RL Reasoner Benchmark

Benchmark suite for evaluating the Reasonable OWL 2 RL reasoner against the UAV domain ontology.

## Overview

[Reasonable](https://github.com/gtfierro/reasonable) is a high-performance Rust-based OWL 2 RL reasoner. It implements forward chaining (materialization) for the OWL 2 RL profile, which is a tractable subset of OWL 2 designed for rule-based reasoning.

**Key Claims:**
- 7x faster than Allegro GraphDB for certain workloads
- Native Rust implementation (memory-safe, fast)
- Python bindings via PyO3

## Directory Contents

```
reasonable_benchmark/
├── Containerfile           # Multi-stage Podman/Docker build
├── benchmark.py            # Python benchmark script
├── run_benchmarks.sh       # Shell wrapper for benchmarks
├── results.json            # Benchmark results (placeholder until run)
├── OWL2_RL_LIMITATIONS.md  # OWL 2 RL profile documentation
└── README.md               # This file
```

## Prerequisites

The benchmark requires an OWL ontology at `../owl_export/uav_domain.owl`. This is created by the ontology translation phase.

## Running the Benchmark

### Option 1: Using Container (Recommended)

```bash
# Build the container
cd /home/finley/Github/DroneProjects/flyby-f11/ontology/evaluation/reasonable_benchmark
podman build -t reasonable-benchmark .

# Run benchmarks (mount ontology directory)
podman run --rm \
    -v ../owl_export:/home/reasoner/ontology:ro \
    -v $(pwd):/home/reasoner/results:rw \
    reasonable-benchmark

# Results will be in results.json
```

### Option 2: Local Python Environment

```bash
# Install dependencies
pip install rdflib owlrl psutil

# Optionally install Reasonable Python bindings
pip install reasonable  # If available on PyPI

# Run benchmark
./run_benchmarks.sh 100  # 100 iterations per query
```

### Option 3: Native Rust Binary

```bash
# Clone and build Reasonable
git clone https://github.com/gtfierro/reasonable
cd reasonable
cargo build --release

# Add to PATH
export PATH="$PATH:$(pwd)/target/release"

# Run benchmarks
cd /home/finley/Github/DroneProjects/flyby-f11/ontology/evaluation/reasonable_benchmark
./run_benchmarks.sh
```

## ARM64 (Jetson) Cross-Compilation

For deployment on Jetson Orin NX (ARM64):

### Build Natively on ARM64 (Recommended)
```bash
# On Jetson device
podman build -t reasonable-benchmark .
```

### Cross-Compile from x86_64
```bash
# Using QEMU emulation
podman build --platform linux/arm64 -t reasonable-benchmark:arm64 .

# Or using Rust cross toolchain
cargo install cross
cross build --release --target aarch64-unknown-linux-gnu
```

See `Containerfile` for detailed cross-compilation notes.

## Benchmark Metrics

The benchmark collects the same metrics as the Vampire benchmark for comparison:

| Metric | Description |
|--------|-------------|
| `cold_start_ms` | First query latency (includes JIT warmup) |
| `mean_ms` | Average query latency |
| `median_ms` | Median query latency |
| `p95_ms` | 95th percentile latency |
| `p99_ms` | 99th percentile latency |
| `success_rate` | Percentage of successful queries |
| `memory_mb` | Peak memory usage |

### Query Categories

| Category | Requirement | Description |
|----------|-------------|-------------|
| Safety | <10ms p95 | Real-time safety checks |
| Operational | <100ms p95 | Operational decisions |
| Planning | <1000ms p95 | Mission planning queries |

## OWL 2 RL Limitations

OWL 2 RL is a restricted profile that trades expressivity for performance. Key limitations:

1. **No existential quantification in class expressions (head position)**
2. **Limited universal quantification**
3. **No complex disjunctions in superclass position**
4. **Limited negation support**

See `OWL2_RL_LIMITATIONS.md` for detailed documentation on:
- Which ontology axioms translate cleanly
- Expected translation losses
- Workarounds for unsupported constructs
- Comparison with Vampire (full FOL)

## Expected Performance

Based on OWL 2 RL characteristics:

| vs Vampire | Expected |
|------------|----------|
| Simple class queries | 10-100x faster |
| Property chains | 5-20x faster |
| Complex joins | 2-10x faster |
| Negation queries | Comparable or slower |

**Trade-off:** Faster queries but reduced expressivity. Some TPTP axioms may not be fully expressible in OWL 2 RL.

## Integration with Evaluation Pipeline

This benchmark is part of the reasoner evaluation pipeline:

```
benchmark_queries/       # TPTP query definitions
    ├── safety_*.tptp
    ├── operational_*.tptp
    └── planning_*.tptp

owl_export/              # Translated OWL ontology
    └── uav_domain.owl   # Input for this benchmark

reasonable_benchmark/    # This directory
    └── results.json     # Output

vampire_benchmark/       # FOL theorem prover
    └── results.json     # Comparison baseline

cross_platform/          # Cross-platform comparison
    └── comparison.json  # Aggregated results
```

## Troubleshooting

### Container Build Fails
- Check Rust version compatibility (requires 1.70+)
- Ensure network access for crates.io

### Python Bindings Not Available
- The benchmark falls back to rdflib + owlrl
- Native Reasonable is faster; install if possible

### Ontology Not Found
- Ensure owl_export phase has completed
- Check path: `../owl_export/uav_domain.owl`

### Memory Issues
- Reduce ontology size or query complexity
- Monitor with `psutil` metrics in results
