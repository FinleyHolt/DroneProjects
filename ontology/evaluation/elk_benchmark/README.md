# ELK Reasoner Benchmark

This directory contains the ELK OWL 2 EL reasoner benchmark for the UAV domain ontology.

## Overview

ELK (EL = Existential Language) is a high-performance OWL 2 EL reasoner written in Java. It provides polynomial-time classification, making it suitable for large ontologies where complete DL reasoning would be intractable.

**Key Characteristics:**
- **Profile**: OWL 2 EL (existential fragment)
- **Complexity**: Polynomial time (PTIME-complete)
- **Strengths**: Fast classification, large ontology support
- **Limitations**: Cannot express disjunction, negation, universal restrictions, or cardinality

## Files

| File | Description |
|------|-------------|
| `Containerfile` | Podman/Docker container definition |
| `ElkBenchmark.java` | Java benchmark program using OWLAPI + ELK |
| `run_benchmark.sh` | Benchmark runner script |
| `results.json` | Benchmark output (generated) |

## Quick Start

### Build the Container

```bash
cd ontology/evaluation/elk_benchmark
podman build -t elk-benchmark -f Containerfile .
```

### Run the Benchmark

```bash
# From the ontology/evaluation directory
cd ontology/evaluation
podman run --rm -v $(pwd):/workspace:Z elk-benchmark
```

Results will be written to `elk_benchmark/results.json`.

### Custom Ontology

```bash
podman run --rm -v $(pwd):/workspace:Z elk-benchmark \
    /workspace/owl_export/custom_ontology.owl \
    /workspace/elk_benchmark/custom_results.json
```

## Benchmark Queries

The benchmark runs the following query types:

1. **Subclass Queries**: Get all subclasses of UAV, Multirotor, UAVMission, etc.
2. **Superclass Queries**: Get superclasses of FlybyF11
3. **Subsumption Checks**: Is FlybyF11 a Quadcopter? Is Quadcopter a UAV?
4. **Instance Queries**: Get instances of FlightPhase, MissionStatus, EmergencyCondition
5. **Unsatisfiability Check**: Find classes that cannot have instances
6. **Taxonomy Depth**: Calculate max depth of UAV class hierarchy

## Output Format

```json
{
  "reasoner": "ELK",
  "reasoner_version": "0.6.0",
  "owl_profile": "OWL 2 EL",
  "timestamp": "2024-12-25T...",
  "ontology_file": "/workspace/owl_export/uav_domain.owl",
  "load_time_ms": 150.5,
  "reasoner_init_time_ms": 45.2,
  "classification_time_ms": 12.8,
  "is_consistent": true,
  "ontology_metrics": {
    "class_count": 85,
    "object_property_count": 35,
    "data_property_count": 12,
    "individual_count": 28,
    "axiom_count": 245
  },
  "queries": [...],
  "summary": {
    "total_queries": 15,
    "total_query_time_ms": 45.3,
    "avg_query_time_ms": 3.02
  }
}
```

## ARM64/aarch64 Deployment (Jetson)

This container is fully compatible with ARM64 platforms including:

- **NVIDIA Jetson Orin NX** (flyby-f11 target platform)
- **NVIDIA Jetson Orin Nano Super** (project-drone platform)
- Apple Silicon (M1/M2/M3)
- AWS Graviton

### Why ARM64 Works

1. **Eclipse Temurin JDK**: The base image uses Eclipse Temurin (formerly AdoptOpenJDK), which provides native ARM64 builds with full JIT support.

2. **Pure Java Stack**: ELK and OWLAPI are 100% Java with no native (JNI) dependencies. No cross-compilation or native library concerns.

3. **Multi-arch Base Image**: The `eclipse-temurin:17-jdk-jammy` image automatically selects the correct architecture.

### Building for ARM64

On an ARM64 machine (e.g., Jetson):
```bash
podman build -t elk-benchmark:arm64 -f Containerfile .
```

Cross-platform build (requires emulation or buildx):
```bash
podman build --platform linux/arm64 -t elk-benchmark:arm64 -f Containerfile .
```

### Performance Considerations

1. **JIT Warmup**: ARM64 JIT compilation characteristics differ from x86_64. First-run classification may be slower. Consider running multiple iterations for benchmarking.

2. **Memory**: Jetson devices have unified memory (shared CPU/GPU). The JVM heap competes with GPU memory. Default `-Xmx` may need adjustment.

3. **Recommended JVM Flags for Jetson**:
   ```bash
   podman run --rm -v $(pwd):/workspace:Z \
       -e JAVA_OPTS="-Xmx2g -XX:+UseG1GC" \
       elk-benchmark
   ```

### Multi-Architecture Manifest

To push a multi-arch image to a registry:

```bash
# Build both architectures
podman build --platform linux/amd64 -t elk-benchmark:amd64 .
podman build --platform linux/arm64 -t elk-benchmark:arm64 .

# Create and push manifest
podman manifest create elk-benchmark:latest
podman manifest add elk-benchmark:latest elk-benchmark:amd64
podman manifest add elk-benchmark:latest elk-benchmark:arm64
podman manifest push elk-benchmark:latest docker://registry.example.com/elk-benchmark:latest
```

## Comparison with Vampire

| Aspect | ELK | Vampire |
|--------|-----|---------|
| Input Format | OWL 2 EL | TPTP/FOF/KIF |
| Expressivity | Limited (EL profile) | Full FOL |
| Safety Axioms | Cannot express | Full support |
| Performance | Polynomial | Decidable for DL, semi-decidable for FOL |
| Classification | Optimized | N/A (theorem proving) |
| Best Use | Taxonomy queries | Safety verification |

**Recommendation**: Use ELK for ontology exploration and classification queries. Use Vampire with the KIF ontology for safety axiom verification.

## Troubleshooting

### ELK JAR Not Found

If ELK fails to download:
```bash
# Manual download
wget https://github.com/liveontologies/elk-reasoner/releases/download/v0.6.0/elk-standalone-0.6.0.zip
unzip elk-standalone-0.6.0.zip
```

### OutOfMemoryError

Increase JVM heap:
```bash
podman run --rm -v $(pwd):/workspace:Z \
    -e JAVA_OPTS="-Xmx4g" \
    elk-benchmark
```

### Ontology Parsing Errors

Ensure the OWL file is valid:
```bash
# Test with OWLAPI only (no reasoning)
podman run --rm -v $(pwd):/workspace:Z elk-benchmark \
    /workspace/owl_export/uav_domain.owl \
    /workspace/test_parse.json
```

## References

- [ELK Reasoner](https://github.com/liveontologies/elk-reasoner)
- [OWL API](https://github.com/owlcs/owlapi)
- [OWL 2 EL Profile](https://www.w3.org/TR/owl2-profiles/#OWL_2_EL)
- [Eclipse Temurin](https://adoptium.net/)
