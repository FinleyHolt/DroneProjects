# ARM64 Validation Plan: Vampire Theorem Prover on Jetson Orin NX

**Status:** Pending
**Hardware Availability Date:** January 5, 2025
**Author:** Finley Holt
**Last Updated:** December 26, 2024

---

## Executive Summary

The entire Flyby F-11 reasoning architecture rests on a critical assumption: Vampire theorem prover can achieve acceptable latency on ARM64 hardware. Current x86_64 benchmarks show ~48ms p95 for safety queries, which fits within the 20Hz navigation loop (50ms period). If ARM64 performance degrades 2-3x, the architecture becomes infeasible.

This document defines the validation protocol for testing Vampire on the Jetson Orin NX 16GB when hardware becomes available on January 5, 2025.

### Critical Thresholds

| Metric | x86 Baseline | Acceptable ARM | Degraded ARM | Unacceptable |
|--------|--------------|----------------|--------------|--------------|
| Safety p95 | 48.69 ms | < 100 ms | 100-150 ms | > 150 ms |
| Navigation fit | 20 Hz | 10 Hz | 5 Hz | < 5 Hz |
| Decision | Proceed | Proceed | Fallback evaluation | Architecture pivot |

---

## 1. Pre-Arrival Preparation (Before January 5)

### 1.1 Cross-Compilation Environment

**Objective:** Prepare ARM64-compatible Vampire binaries before hardware arrives.

| Task | Description | Status | Due Date |
|------|-------------|--------|----------|
| Install ARM64 cross-compiler | `aarch64-linux-gnu-g++` toolchain | Pending | Jan 2 |
| Cross-compile Vampire | Build Vampire 5.0 for aarch64 target | Pending | Jan 3 |
| Create ARM container | Containerfile for Jetson L4T base image | Pending | Jan 3 |
| Test with QEMU | Validate binary executes under emulation | Pending | Jan 4 |
| Prepare benchmark scripts | Port `benchmark_x86.py` for ARM environment | Pending | Jan 4 |

**Cross-Compilation Commands:**
```bash
# Install cross-compiler
sudo apt install g++-aarch64-linux-gnu

# Clone Vampire
git clone https://github.com/vprover/vampire.git
cd vampire

# Configure for ARM64
cmake -B build-arm64 \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
    -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build-arm64 -j$(nproc)
```

### 1.2 Benchmark Artifacts

**Objective:** Ensure all benchmark inputs are identical to Phase 3 evaluation.

| Artifact | Location | Verification |
|----------|----------|--------------|
| TPTP query files | `ontology/evaluation/benchmark_queries/*.tptp` | 8 files present |
| UAV domain ontology | `ontology/planning_mode/uav_domain.kif` | 1,213 lines |
| Benchmark script | `ontology/evaluation/benchmark_x86.py` | Port to `benchmark_arm64.py` |
| Results schema | Match `decision_matrix.json` format | Identical metrics |

### 1.3 Baseline Documentation

**Objective:** Document exact x86 benchmark conditions for reproducibility.

**x86 Benchmark Environment (Reference):**
- Platform: x86_64 Linux
- CPU: [Document specific CPU model]
- Memory: [Document RAM configuration]
- Vampire version: 5.0.0
- Iterations per query: 100
- Statistical methodology: p95 percentile, 95% CI

**Expected ARM Environment:**
- Platform: aarch64 (ARM64)
- SoC: NVIDIA Jetson Orin NX 16GB
- CPU: 8-core ARM Cortex-A78AE @ 2.0 GHz
- Memory: 16GB LPDDR5 (unified with GPU)
- AI Performance: 50 TOPS (not relevant for Vampire)

---

## 2. Day-1 Validation Checklist (January 5)

### 2.1 Hardware Setup (Morning)

| Step | Task | Verification | Time Est. |
|------|------|--------------|-----------|
| 1 | Unbox and power on Jetson | LED indicators, HDMI output | 15 min |
| 2 | Verify JetPack version | `cat /etc/nv_tegra_release` | 5 min |
| 3 | Install Podman | `sudo apt install podman` | 10 min |
| 4 | Configure CDI for GPU | `sudo nvidia-ctk cdi generate` | 5 min |
| 5 | Transfer container image | `scp` or USB transfer | 30 min |
| 6 | Verify Vampire binary | `./vampire --version` | 2 min |

### 2.2 Smoke Test (Early Afternoon)

**Objective:** Confirm Vampire executes correctly before full benchmark.

```bash
# Single query execution test
./vampire --input_syntax tptp --time_limit 60 \
    /workspace/ontology/evaluation/benchmark_queries/safety_geofence_check.tptp

# Expected output: SZS status Theorem or CounterSatisfiable
# Timeout: If no result within 60 seconds, FAIL
```

**Smoke Test Pass Criteria:**
- [ ] Binary executes without segfault
- [ ] Query returns valid SZS status
- [ ] Execution completes within 60 seconds
- [ ] No memory errors (check with `dmesg`)

### 2.3 Thermal Baseline (Before Benchmark)

**Objective:** Establish thermal baseline to detect throttling during benchmark.

```bash
# Record idle temperature
cat /sys/devices/virtual/thermal/thermal_zone*/temp

# Monitor during benchmark
watch -n 1 'cat /sys/devices/virtual/thermal/thermal_zone*/temp'

# Check power mode
sudo nvpmodel -q
# Should be: MAXN (Mode 0) for maximum performance
```

---

## 3. Benchmark Protocol

### 3.1 Benchmark Configuration

The ARM64 benchmark must match Phase 3 methodology exactly:

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Iterations per query | 100 | Statistical validity (n=100 for z-distribution) |
| Warm-up iterations | 5 | Exclude JIT/cache warm-up effects |
| Timing method | `time.perf_counter()` | High-resolution wall clock |
| Query set | 8 TPTP files | 3 safety, 3 operational, 2 planning |
| Timeout per query | 30 seconds | Match x86 configuration |
| Power mode | MAXN (Mode 0) | Maximum CPU performance |
| Thermal state | Monitor continuously | Detect throttling |

### 3.2 Query Set

Execute all Phase 3 benchmark queries:

**Safety-Critical Queries (Target: <100ms p95 for ARM acceptance):**
1. `safety_geofence_check.tptp` - Geofence boundary validation
2. `safety_nfz_violation.tptp` - No-fly zone detection
3. `safety_battery_return.tptp` - Battery reserve check

**Operational Queries (Target: <200ms p95):**
1. `operational_valid_waypoint.tptp` - Waypoint sequence validation
2. `operational_comms_status.tptp` - Communications status check
3. `operational_threat_zone.tptp` - Threat zone assessment

**Planning Queries (Target: <500ms p95):**
1. `planning_mission_feasibility.tptp` - Mission feasibility analysis
2. `planning_target_prioritization.tptp` - Target prioritization

### 3.3 Metrics Collection

For each query, collect:

| Metric | Description | Formula |
|--------|-------------|---------|
| `mean_ms` | Average latency | `sum(times) / n` |
| `min_ms` | Minimum latency | `min(times)` |
| `max_ms` | Maximum latency | `max(times)` |
| `p50_ms` | Median latency | `times[n * 0.50]` |
| `p95_ms` | 95th percentile | `times[n * 0.95]` |
| `p99_ms` | 99th percentile | `times[n * 0.99]` |
| `stddev_ms` | Standard deviation | `sqrt(variance)` |
| `cv_percent` | Coefficient of variation | `stddev / mean * 100` |
| `success_rate` | Query success percentage | `successes / n * 100` |
| `szs_status` | Theorem prover result | THEOREM, COUNTER_SAT, etc. |

**System Metrics:**
- Peak memory usage (MB)
- CPU temperature at start/end
- Thermal throttling events (if any)
- Power consumption (if measurable)

### 3.4 Execution Script

```bash
#!/bin/bash
# arm64_benchmark.sh - Execute on Jetson Orin NX

set -e

# Ensure maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Set environment
export WORKSPACE=/workspace
export RESULTS_DIR=$WORKSPACE/ontology/evaluation/cross_platform

# Record system state
echo "Recording system baseline..."
cat /etc/nv_tegra_release > $RESULTS_DIR/system_info.txt
nvpmodel -q >> $RESULTS_DIR/system_info.txt
cat /proc/cpuinfo | grep "model name" | head -1 >> $RESULTS_DIR/system_info.txt

# Run benchmark
echo "Starting ARM64 benchmark..."
python3 $WORKSPACE/ontology/evaluation/benchmark_arm64.py \
    --iterations 100 \
    --output $RESULTS_DIR/arm64_results.json

# Record thermal state
cat /sys/devices/virtual/thermal/thermal_zone*/temp >> $RESULTS_DIR/thermal_log.txt

echo "Benchmark complete. Results in $RESULTS_DIR/arm64_results.json"
```

---

## 4. Pass/Fail Criteria

### 4.1 Primary Acceptance Criteria

**PASS - Proceed with Vampire-only architecture:**

| Criterion | Threshold | Rationale |
|-----------|-----------|-----------|
| Safety p95 | < 100 ms | Fits 10Hz minimum navigation rate |
| Operational p95 | < 200 ms | Acceptable for operational queries |
| Planning p95 | < 500 ms | Acceptable for planning phase |
| Success rate | 100% | All queries must complete |
| Memory peak | < 200 MB | Within Jetson memory budget |
| Thermal throttle | None | Sustained operation required |

### 4.2 Degraded Acceptance Criteria

**CONDITIONAL PASS - Proceed with architectural modifications:**

| Criterion | Threshold | Required Mitigation |
|-----------|-----------|---------------------|
| Safety p95 | 100-150 ms | Reduce navigation rate to 5Hz |
| Variability (CV) | > 20% | Implement query timeout + retry |
| Thermal throttle | Detected | Add heatsink, reduce duty cycle |

### 4.3 Failure Criteria

**FAIL - Trigger fallback decision tree:**

| Criterion | Threshold | Consequence |
|-----------|-----------|-------------|
| Safety p95 | > 150 ms | Vampire unsuitable for runtime |
| Success rate | < 95% | Reliability unacceptable |
| Memory peak | > 500 MB | Exceeds safe memory budget |
| Crash/segfault | Any | Binary incompatibility |

---

## 5. Fallback Decision Tree

If ARM64 validation fails, execute the following decision process:

```
ARM64 Benchmark Results
         |
         v
  Safety p95 < 100ms?
    |           |
   YES          NO
    |           |
    v           v
  PASS      Safety p95 < 150ms?
    |           |           |
    |          YES          NO
    |           |           |
    |           v           v
    |    Evaluate      Evaluate
    |    Prolog        Hybrid
    |    Translation   Architecture
    |           |           |
    |           v           v
    +---------> Fallback Option Selection
                        |
                        v
              +--------------------+
              | Option A: Prolog   |
              | Option B: Hybrid   |
              | Option C: Custom   |
              +--------------------+
```

### 5.1 Fallback Option A: Prolog Translation

**Trigger:** Safety p95 100-200ms on ARM

**Description:** Accept the 12-15 hour translation cost to use SWI-Prolog for runtime safety queries while retaining Vampire for offline planning.

| Aspect | Detail |
|--------|--------|
| Effort | 12-15 hours initial translation |
| Ongoing cost | Maintain dual KIF + Prolog representations |
| Semantic loss | ~30% (negation-as-failure vs classical) |
| Expected latency | <1ms (based on x86 benchmarks) |
| ARM availability | Official ARM64 packages available |

**Action Items:**
1. Complete Prolog translation of safety axioms
2. Validate semantic equivalence with Vampire
3. Implement dual-reasoner architecture
4. Create synchronization tooling for KIF/Prolog

### 5.2 Fallback Option B: Hybrid Architecture

**Trigger:** Safety p95 150-300ms on ARM, or Prolog translation unacceptable

**Description:** Pre-compute safety boundaries offline with Vampire, use lightweight runtime monitor for real-time checks.

| Aspect | Detail |
|--------|--------|
| Effort | 20-30 hours (custom monitor implementation) |
| Runtime component | C++ safety monitor with pre-computed lookup tables |
| Vampire role | Offline planning + boundary computation |
| Expected latency | <5ms for monitor queries |
| Complexity | High (two-system coordination) |

**Architecture:**
```
                    +------------------+
                    |  Vampire (Offline) |
                    | - Mission planning |
                    | - Boundary compute |
                    +--------+---------+
                             |
                             v
                    +------------------+
                    | Pre-computed     |
                    | Safety Boundaries|
                    +--------+---------+
                             |
                             v
+------------------+         |         +------------------+
|  Perception      |-------->|-------->|  C++ Safety      |
|  Pipeline        |         |         |  Monitor         |
+------------------+         |         | - Geofence check |
                             |         | - NFZ lookup     |
                             |         | - Battery thresh |
                             |         +------------------+
```

### 5.3 Fallback Option C: Custom C++ Safety Module

**Trigger:** All reasoner options exceed 300ms, or reliability issues

**Description:** Abandon formal reasoning for runtime safety; implement hand-coded safety rules in C++ with Vampire validation.

| Aspect | Detail |
|--------|--------|
| Effort | 40-60 hours (full implementation) |
| Validation | Vampire validates C++ implementation offline |
| Runtime latency | <1ms (native C++ checks) |
| Semantic fidelity | Manual translation, potential for drift |
| Maintainability | Low (code changes require re-validation) |

**This option is last resort.** It sacrifices the benefits of formal reasoning for runtime performance.

---

## 6. Timeline

### Week 1: Hardware Arrival and Initial Validation (Jan 5-11)

| Date | Activity | Deliverable |
|------|----------|-------------|
| Jan 5 | Hardware setup, smoke tests | System operational |
| Jan 6 | Full benchmark execution | `arm64_results.json` |
| Jan 7 | Results analysis, comparison with x86 | Benchmark report |
| Jan 8 | Pass/fail determination | Decision documented |
| Jan 9-10 | If PASS: Proceed with Phase 4 integration | Integration plan |
| Jan 9-10 | If FAIL: Begin fallback evaluation | Fallback selection |

### Week 2: Integration or Fallback (Jan 12-18)

**If PASS:**
| Date | Activity | Deliverable |
|------|----------|-------------|
| Jan 12-14 | ROS 2 integration of Vampire on ARM | Working ROS node |
| Jan 15-16 | Navigation loop integration testing | Timing validation |
| Jan 17-18 | Performance optimization | Optimized deployment |

**If FAIL (Prolog Fallback):**
| Date | Activity | Deliverable |
|------|----------|-------------|
| Jan 12-14 | Complete Prolog translation | `uav_rules_complete.pl` |
| Jan 15-16 | Semantic validation against Vampire | Equivalence report |
| Jan 17-18 | ARM Prolog benchmark | Prolog ARM results |

### Week 3: Finalization (Jan 19-25)

| Date | Activity | Deliverable |
|------|----------|-------------|
| Jan 19-21 | Full integration testing | End-to-end validation |
| Jan 22-23 | Documentation updates | Updated ADRs |
| Jan 24-25 | Phase 4 verification | Verification script pass |

---

## 7. Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Vampire ARM build fails | Low | High | QEMU pre-validation, static linking |
| ARM 3x slower than x86 | Medium | High | Fallback options documented |
| Thermal throttling | Medium | Medium | Monitor temps, add cooling |
| Memory pressure | Low | Medium | 16GB unified memory sufficient |
| Hardware delivery delay | Low | Medium | Continue x86 development |
| Prolog translation errors | Medium | Medium | Vampire-based validation |

---

## 8. Success Metrics

### Primary Success (Vampire PASS)

- [ ] Safety p95 < 100ms on Jetson Orin NX
- [ ] All 8 benchmark queries complete successfully
- [ ] No thermal throttling during sustained operation
- [ ] Memory usage < 200MB peak
- [ ] Vampire integrated with ROS 2 navigation loop
- [ ] 20Hz navigation rate achieved

### Secondary Success (Fallback Required)

- [ ] Clear documentation of ARM performance characteristics
- [ ] Fallback option selected with rationale
- [ ] Fallback implementation complete within 2 weeks
- [ ] End-to-end system operational by Jan 25

---

## References

- [ADR-001: Vampire-Only Architecture](ADR-001-vampire-only-reasoner.md)
- [EVALUATION_REPORT.qmd](../../ontology/evaluation/EVALUATION_REPORT.qmd)
- [decision_matrix.json](../../ontology/evaluation/decision_matrix.json)
- [benchmark_x86.py](../../ontology/evaluation/benchmark_x86.py)
- [Phase 3 Verification Script](../../.phases/phase-03-evaluation/verification.sh)

---

*Document prepared for Flyby F-11 UAV Autonomy Project*
*Phase 4 Pre-requisite: ARM64 Hardware Validation*
