# Phase 4: Execution Mode Runtime

## Overview

Create the lightweight Prolog execution runtime that runs during flight. This phase establishes the SWI-Prolog container environment optimized for real-time constraint checking with <10ms query latency and <100MB memory footprint.

## Human Description

While planning mode (Phase 1-3) uses heavyweight reasoners for mission verification, execution mode runs during flight and must be extremely lightweight. This phase creates:

1. An ARM-compatible SWI-Prolog container for Jetson deployment
2. Query interface optimized for real-time performance
3. Memory-optimized Prolog engine configuration
4. ROS 2 bridge for Prolog integration
5. Benchmarking infrastructure to validate performance requirements

The execution runtime loads the compiled Prolog rules from Phase 3 and provides real-time constraint checking during autonomous flight.

## AI Agent Instructions

### Prerequisites
- Phase 1 completed (planning toolchain)
- Phase 2 completed (UAV ontology)
- Phase 3 completed (SUMO to Prolog translation)
- Understanding of SWI-Prolog optimization
- Familiarity with ROS 2 service/topic patterns

### Input Requirements
See `inputs.json` for machine-readable specification.

### Task Steps

1. **Create Execution Mode Container**
   - Location: `ontology/Containerfile.execution`
   - Base image: Ubuntu 22.04 (both x86 and ARM builds)
   - Install SWI-Prolog with optimizations
   - Configure TCMalloc for reduced memory footprint
   - Set up pyswip for Python-Prolog bridge
   - Target: <100MB runtime memory

2. **Implement Prolog Query Interface**
   - Location: `ontology/execution_mode/query_interface.py`
   - Load compiled rules from Phase 3
   - Provide synchronous query API
   - Implement caching for frequent queries
   - Add query timeout protection

3. **Create ROS 2 Prolog Bridge**
   - Location: `ros2_ws/src/prolog_bridge/`
   - ROS 2 service for Prolog queries
   - Topic for continuous constraint monitoring
   - Action for complex reasoning tasks
   - Parameter server for runtime configuration

4. **Optimize Query Performance**
   - Profile critical safety queries
   - Implement query indexing
   - Add first-argument indexing hints
   - Benchmark: ensure <10ms for all queries

5. **Build Multi-Architecture Images**
   - x86_64 for development
   - ARM64 for Jetson deployment
   - Test on both architectures
   - Validate performance on Jetson Orin NX

6. **Create Benchmark Suite**
   - Location: `ontology/execution_mode/benchmarks/`
   - Query latency benchmarks
   - Memory footprint measurements
   - Concurrent query stress tests
   - Document results in benchmark report

### Expected Outputs
See `outputs.json` for machine-readable specification.

### Success Criteria

- [ ] `Containerfile.execution` builds for both x86 and ARM
- [ ] SWI-Prolog loads compiled rules without errors
- [ ] Query latency < 10ms for all safety-critical queries
- [ ] Memory footprint < 100MB during operation
- [ ] ROS 2 bridge publishes/subscribes correctly
- [ ] Benchmarks pass on both x86 dev machine and ARM Jetson
- [ ] pyswip integration working with Python 3.10+

### Verification

Run automated verification:
```bash
bash .phases/phase-04-execution-mode/verification.sh
```

### Time Estimate
6-8 hours (includes multi-arch builds and performance optimization)

### Common Pitfalls

- **ARM build issues**: Cross-compilation may require qemu-user-static
- **Memory leaks**: Prolog engines must be properly cleaned up
- **Query timeouts**: Set reasonable timeouts to prevent hangs
- **Indexing**: Unoptimized queries can be orders of magnitude slower
- **Thread safety**: SWI-Prolog requires careful thread management
- **pyswip versions**: Ensure compatible pyswip version for SWI-Prolog version

### References

- [SWI-Prolog Performance Optimization](https://www.swi-prolog.org/pldoc/man?section=profile)
- [pyswip Documentation](https://github.com/yuce/pyswip)
- [ROS 2 Service Design](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Jetson Container Builds](https://github.com/dusty-nv/jetson-containers)

### Dependencies
See `dependencies.json` - requires Phase 1, 2, and 3 completion.

### Next Phase
After completion, proceed to Phase 5: Perception-to-Reasoning Bridge
