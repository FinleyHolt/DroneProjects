# Phase 5 - Dynamic Replanning

**Status**: Not Started
**Dependencies**: Phase 4 (LLM mission planning and execution)

## Objective
Enable the drone to adapt mission plans in real-time when faced with obstacles, failures, or changing conditions. The LLM becomes not just an initial planner but a **continuous reasoning engine** that monitors execution and replans when necessary.

## Rationale
Phases 1-4 give us:
- Working simulation (Phase 1)
- Defined missions (Phase 2)
- Action primitives (Phase 3)
- Natural-language planning (Phase 4)

But real-world operations require adaptability:
- **Obstacles discovered mid-flight** → replan path around them
- **Action failures** (GPS loss, detection failure) → recover or adjust plan
- **Environmental changes** (wind, visibility) → modify approach
- **Unexpected discoveries** (additional targets) → extend mission
- **Battery constraints** → prioritize objectives and RTH early

Phase 5 adds the **adaptive reasoning loop**: sense → analyze → replan → execute.

## Goals
1. **Event Detection**: Monitor for triggers that require replanning (obstacles, failures, discoveries)
2. **LLM Replanning**: Call LLM mid-mission with updated context to generate new plan
3. **Partial Plan Modification**: Update remaining actions without restarting mission
4. **Obstacle Avoidance**: Integrate obstacle detection and path replanning
5. **Graceful Degradation**: Handle failures by adjusting mission scope
6. **Curveball Testing**: Throw unexpected events at Mission 1/2 and verify adaptive response

## Success Criteria
- [ ] Detect mid-mission events (simulated obstacles, action failures)
- [ ] LLM generates replans within <500ms when triggered
- [ ] Successfully navigate around obstacle injected during Mission 1
- [ ] Handle action failure (e.g., detection unavailable) and adapt plan
- [ ] Prioritize objectives when battery becomes constrained
- [ ] Complete Mission 1 with 3 different curveballs successfully
- [ ] Document replanning triggers and responses

## Tasks

### 5.1 Event Detection & Monitoring

#### Mission Monitoring Node
- [ ] Create `mission_monitor_node` in `llmdrone_llm/`:
  - Subscribe to `/drone/state`, `/mission/status`, `/drone/detections`
  - Track mission progress and environmental state
  - Detect events that may require replanning
  - Publish to `/mission/events` when replanning needed

#### Replanning Triggers
- [ ] Implement trigger detection:
  - **Obstacle detected**: New obstacle in planned path
  - **Action failure**: Action completes with error (timeout, sensor failure)
  - **Resource constraint**: Battery below threshold for planned actions
  - **Unexpected discovery**: Detection of high-priority target outside plan
  - **Geofence approach**: Approaching boundary, need to adjust path
  - **Time constraint**: Mission taking longer than expected
  - **External command**: Operator updates mission mid-flight

- [ ] For each trigger, create event message:
  - Event type
  - Timestamp
  - Relevant data (obstacle location, battery level, etc.)
  - Severity (advisory, warning, critical)

#### Simulation Event Injection
- [ ] Create tools to inject test events:
  - Spawn Gazebo obstacles mid-flight
  - Simulate sensor failures (disable camera, GPS glitch)
  - Modify battery level programmatically
  - Send external command updates

**Deliverable**: Event detection system publishing replanning triggers

### 5.2 LLM Replanning Integration

#### Replanning Prompt Design
- [ ] Create `llm/prompts/mission_replanner_system.md`:
```
You are a real-time mission replanner for an autonomous drone. A mission is currently in progress, and conditions have changed. Your task is to adapt the BEHAVIOR TREE for the remaining mission to handle the new situation.

**Current Mission State**:
- Original objective: [goal]
- Completed BT nodes: [list with results]
- Current position: [GPS]
- Current BT node executing: [node name]
- Remaining BT subtree: [JSON]

**Event Requiring Replan**:
[Event description, obstacle location, failure type, battery level, etc.]

**Available Actions & Conditions**:
[Phase 3 action vocabulary + BT node types]

**Constraints**:
- Battery remaining: [X]% (CRITICAL: validate sufficiency for each action + RTH reserve)
- Time on station: [X] minutes (mission start + elapsed)
- Geofence: [boundary] (verify all new waypoints inside)
- Safety: Avoid collisions, maintain altitude, graceful degradation if objective unreachable

**Your task**:
1. Assess situation criticality (obstacle blocks path? battery insufficient? sensor failed?)
2. Determine if original objective achievable with constraints
3. Generate updated BEHAVIOR TREE for **remaining** mission:
   - If minor (local obstacle): adjust path, continue objective
   - If major (low battery): prioritize high-value targets, simplify plan, ensure RTH
   - If critical (sensor failure): graceful degradation, safe return
4. Include CheckBattery conditions before EACH action
5. Add Fallback nodes for failure recovery

**Output Format**:
[BT JSON schema from Phase 4]

**Example - Battery-Constrained Replanning**:
Original objective: Survey 3 areas (A, B, C)
Event: Battery at 35% (after surveying A)
Replan:
{
  "tree": {
    "type": "Fallback",
    "children": [
      {
        "type": "Sequence",
        "name": "PrioritizedMission",
        "children": [
          {"type": "Condition", "name": "CheckBattery", "params": {"min_percent": 30}},
          {"type": "Action", "name": "SEARCH_AREA", "params": {"boundary": area_B}},  // Skip low-priority C
          {"type": "Condition", "name": "CheckBattery", "params": {"min_percent": 25}},
          {"type": "Action", "name": "RTL"}
        ]
      },
      {"type": "Action", "name": "EMERGENCY_RTL"}  // If battery check fails
    ]
  }
}
```

- [ ] Include 4-5 replanning scenario examples:
  - Obstacle avoidance (local path adjustment)
  - Battery constraint (objective prioritization, skip low-value tasks)
  - Sensor failure (graceful degradation, partial mission)
  - Unexpected discovery (extend mission if battery allows)
  - Multi-failure (simultaneous battery + sensor → immediate RTL)

#### Replanner Service
- [ ] Extend `mission_parser_node` with `/llm/replan_mission` service:
  - Input: Current mission state + event trigger
  - Output: Modified action sequence
  - Build replanning context from current state
  - Call LLM with replanning prompt + context + event
  - Parse and validate modified plan
  - Return updated action sequence

- [ ] Optimize for latency:
  - Reuse LLM session (no cold start)
  - Streamlined context (only relevant info—don't resend full action vocab)
  - Target <500ms response time
  - **This is 4× faster than Phase 4's <2s requirement—validate this is achievable**:
    - Replanning context is shorter (remaining BT, not full mission from scratch)
    - But LLM must reason about constraints, assess criticality
    - If <500ms impossible → adjust to <1s and document impact on real-time response

#### Plan Merger
- [ ] Create `plan_merger` function:
  - Input: Original plan, completed actions, new plan
  - Output: Combined plan with seamless transition
  - Verify new plan starts from current state
  - Ensure no contradictory actions
  - Preserve mission metadata (objectives, constraints)

**Deliverable**: Replanning service generating adapted plans from events

### 5.3 Dynamic Execution Loop

#### Adaptive Executor
- [ ] Modify `mission_executor_node` to support dynamic replanning:
  - Monitor `/mission/events` topic during execution
  - Pause current action when replan triggered (if safe)
  - Call `/llm/replan_mission` service
  - Integrate new plan into execution queue
  - Resume execution with updated actions
  - Log all replanning events

- [ ] Handle execution state transitions:
  - In-flight action completion before switching plans
  - Safe pausing (hover if necessary)
  - Rollback on replanning failure (continue original or RTL)

#### Replanning Decision Logic
- [ ] Implement decision tree for when to replan:
  - **Critical events** (collision imminent, geofence breach) → immediate replan or emergency stop
  - **Warnings** (battery declining, time pressure) → replan at next action boundary
  - **Advisories** (minor detection issue) → log but continue
  - **Multiple events** → prioritize most critical (battery > sensor > minor obstacles)

- [ ] Add **adaptive replan throttling** (not fixed 30s):
  - **Situational throttling**:
    - Normal operations: max 1 replan per 60s (avoid churn)
    - Obstacle-rich environment: allow replan every 10s (responsive to dynamic obstacles)
    - Battery critical (<30%): allow immediate replans (safety priority)
  - Track recent replan history (timestamps, triggers, outcomes)
  - Detect oscillation: if same trigger causes replan 3× in 2 minutes → escalate to failsafe (RTL)
  - **Hysteresis**: If battery trigger at 35%, don't replan again until 30% (avoid repeated replans for small fluctuations)
  - Fallback to RTL if:
    - Replanning fails 2× consecutively
    - Oscillation detected
    - No valid plan found within constraints

#### Feedback Loop
- [ ] Integrate replanning results into LLM context:
  - Track replanning history
  - Include "lessons learned" in subsequent replan context
  - Adapt behavior based on prior replan success/failure

**Deliverable**: Mission executor with adaptive replanning loop

### 5.4 Obstacle Avoidance & Path Replanning

#### Obstacle Detection
- [ ] Enhance perception for obstacle detection:
  - Add depth camera or lidar simulation in Gazebo
  - Or use camera + simple depth estimation
  - Publish detected obstacles to `/drone/obstacles`
  - Include position, size, confidence

#### Local Path Replanning
- [ ] Extend `waypoint_navigator_node` for obstacle avoidance:
  - Maintain occupancy grid or costmap
  - Update costmap with detected obstacles
  - Replan path using A* when obstacle in current path
  - Generate detour waypoints
  - Publish revised path

- [ ] This is **local replanning** (tactical):
  - Navigator handles immediate obstacles
  - LLM replanning handles strategic changes (mission objectives)

#### Integration with LLM Replanning
- [ ] Coordinate local and strategic replanning:
  - Local planner handles simple detours (minor deviations)
  - Trigger LLM replan if detour makes objective unreachable
  - LLM can reassign objectives or modify search area

**Deliverable**: Obstacle detection and avoidance integrated with replanning

### 5.5 Curveball Testing

Comprehensive test scenarios including adversarial and multi-failure cases to validate adaptive autonomy.

#### Test Scenario 1: Mid-Flight Obstacle
- [ ] Mission 1 (Urban Vehicle Search) with injected obstacle:
  - Start mission normally
  - At 50% progress, spawn large obstacle in planned path
  - **Expected behavior**:
    - Detect obstacle via costmap update
    - Trigger replanning (tactical planner attempts local detour first)
    - If local detour adds >2 min → trigger LLM strategic replan
    - LLM generates revised BT with adjusted search pattern
    - Continue search with adjusted path
    - Complete mission successfully
  - **Measure**:
    - Time to detect and replan (<2s detection, <1s replan)
    - Deviation from original path (meters)
    - Mission success (vehicles still detected)
    - Battery usage increase (<15% overhead)

#### Test Scenario 2: Sensor Failure
- [ ] Mission 1 with simulated camera failure:
  - Start mission normally
  - Disable camera feed at 25% progress
  - **Expected behavior**:
    - Detect perception node failure (no detections for 10s)
    - LLM replan with graceful degradation:
      - "Complete geometric coverage for post-mission analysis"
      - OR "Return imagery for offline processing"
      - OR "Abort mission and RTL" (if detection was critical)
    - Continue with degraded mission or safe return
  - **Evaluate**: LLM's reasoning quality—does it choose appropriate fallback?

#### Test Scenario 3: Battery Constraint
- [ ] Mission 1 with accelerated battery drain:
  - Start with 60% battery (lower than normal)
  - Simulate faster drain mid-mission (simulate headwind increasing power draw)
  - **Expected behavior**:
    - CheckBattery condition fails (e.g., at 35% instead of expected 50%)
    - Trigger replanning with battery-constrained prompt
    - LLM prioritizes: "Search highest-probability area first, skip low-priority zones"
    - Partial mission completion with safe RTH (>20% remaining)
  - **Measure**: Does LLM make intelligent prioritization? Does battery reserve hold?

#### Test Scenario 4: Extended Mission
- [ ] Mission 1 with additional discovery:
  - Complete initial search (battery ~40%)
  - Simulate operator command: "Investigate secondary area Bravo"
  - **Expected behavior**:
    - LLM checks battery sufficiency (Bravo survey + RTH)
    - If sufficient (>35%): extend BT with Bravo survey subtree
    - If insufficient: "Recommend follow-up mission, returning home"
    - Execute decision appropriately
  - **Evaluate**: Battery-aware decision making under dynamic tasking

#### Test Scenario 5: GPS Jamming (Adversarial)
- [ ] Mission 1 with simulated GPS loss:
  - Start mission normally
  - At 40% progress, simulate GPS jamming (disable GPS fix for 30s)
  - **Expected behavior**:
    - Detect GPS loss (position covariance spikes)
    - Tactical layer: switch to vision-based position hold or IMU dead reckoning
    - Strategic layer: LLM replan "Hold position, await GPS recovery"
    - If GPS doesn't recover in 60s → "Execute emergency RTL using last known position + IMU"
  - **Measure**: Does system maintain stability? Safe recovery?

#### Test Scenario 6: Multi-Failure (Simultaneous Battery + Sensor)
- [ ] Mission 1 with cascading failures:
  - At 30% progress: camera fails
  - At 50% progress: battery drains to 30%
  - **Expected behavior**:
    - First replan: adapt to sensor loss (continue coverage for post-analysis)
    - Second replan: detect battery insufficient for remaining mission
    - LLM decision: "Abort mission, prioritize safe RTL immediately"
    - Execute RTL with >20% battery margin
  - **Evaluate**: Does LLM handle compounding failures? Prioritize safety?

#### Test Scenario 7: Dynamic Obstacle (Moving Vehicle)
- [ ] Mission 1 with moving obstacle:
  - Spawn ground vehicle that drives through search area
  - **Expected behavior**:
    - Tactical planner detects obstacle, replans path continuously
    - If vehicle blocks critical area for >30s → LLM strategic replan
    - LLM: "Search remaining areas, return to blocked area after timeout"
    - Dynamic scheduling of search sub-areas

#### Test Scenario 8: Communication Loss (Full Autonomy Test)
- [ ] Mission 1 with simulated comm loss:
  - Disable ground station comms at 20% progress
  - **Expected behavior**:
    - System continues mission autonomously (no operator intervention)
    - Complete mission using onboard reasoning
    - Execute RTL on schedule or battery trigger
    - Log all decisions for post-mission review
  - **This validates true communications-denied autonomy**

#### Statistical Validation
- [ ] Run each scenario 10 times:
  - Success rate (mission completed safely)
  - Replanning latency
  - Quality of adaptations (subjective evaluation)
  - Failure modes (document when/why fails)

**Deliverable**: Validated adaptive behavior across multiple failure scenarios

## Deliverables Checklist
- [ ] Event detection and monitoring system
- [ ] LLM replanning service with <500ms latency
- [ ] Adaptive mission executor
- [ ] Obstacle detection and avoidance
- [ ] Curveball test scenarios implemented
- [ ] 4 test scenarios passed with >80% success rate
- [ ] Documentation: `docs/adaptive_replanning.md`, test results

## Known Risks and Mitigation

### Risk: Replanning latency too high for real-time response (<500ms target may be unrealistic)
**Impact**: High
**Mitigation**: Benchmark in Phase 4 to validate <500ms achievable; if not, adjust to <1s and document tactical-layer reliance for fast responses; implement local rule-based fallbacks for critical events

### Risk: LLM generates unsafe replans (violates geofence, battery)
**Impact**: Critical
**Mitigation**: Strict validation of replans, safety constraints in prompt, fallback to RTL on violation

### Risk: Oscillation between plans (replan → replan → replan)
**Impact**: High (battery drain, mission failure)
**Mitigation**: Adaptive throttling (situational, not fixed 30s), hysteresis in battery/sensor triggers (5% deadband), oscillation detection (3 replans same trigger → RTL failsafe)

### Risk: Integration complexity (local + strategic planning)
**Impact**: Medium
**Mitigation**: Clear interface between navigator and LLM, well-defined responsibilities

## Phase Exit Criteria
Before moving to Phase 6 (final validation/deployment), verify:

**Functional Requirements**:
1. ✓ Event detection system working for all 8 trigger types (including adversarial)
2. ✓ LLM replanning service responds in <1s (500ms if achievable, documented otherwise)
3. ✓ Adaptive throttling prevents oscillation (<3 replans per 2 minutes in normal conditions)
4. ✓ All 8 curveball scenarios pass with >70% success rate (10 runs each)
5. ✓ No safety violations during adaptive missions (geofence, altitude, battery reserve maintained)
6. ✓ System maintains stability under replanning (no crashes, graceful degradation)

**Adversarial Robustness**:
1. ✓ GPS jamming scenario handled with safe fallback (Scenario 5)
2. ✓ Multi-failure scenario prioritizes safety over mission completion (Scenario 6)
3. ✓ Full autonomy validated under communications-denied operation (Scenario 8)

**Performance**:
1. ✓ Replanning adds <20% battery overhead in obstacle scenarios
2. ✓ LLM makes contextually appropriate decisions (qualitative eval of 50 replans)
3. ✓ System logs all replan decisions for post-mission audit

## Next Phase
Once Phase 5 is complete, proceed to [Phase 6 - Validation & Deployment](Phase-6-Validation-Deployment.md) for comprehensive testing, edge deployment, containerization, and live-flight preparation.

**Key Achievement**: The drone now has adaptive autonomy—it can handle unexpected situations and "think on its feet" using LLM reasoning. This is the core capability that distinguishes this system from traditional waypoint-following drones.

## Notes and Findings
_Use this section to document replanning strategies, challenges with real-time LLM inference, lessons from curveball testing, and observations about LLM decision quality under pressure._

---

**Phase Started**: [Date]
**Phase Completed**: [Date]

**Technical Decisions**:
- [Replanning trigger thresholds]
- [Coordination between local and strategic planning]
- [Replan throttling strategy]

**Challenges Encountered**:
- [Replanning latency issues]
- [Safety validation complexity]
- [Oscillation between plans]

**Curveball Test Insights**:
- [How LLM handled obstacles]
- [Adaptation quality under constraints]
- [Failure mode analysis]

**Recommendations for Phase 6**:
- [What needs further testing]
- [Edge deployment considerations]
- [Live-flight safety measures]
