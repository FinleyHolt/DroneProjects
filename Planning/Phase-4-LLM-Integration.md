# Phase 4 - LLM Integration

**Status**: Not Started
**Dependencies**: Phase 3 (action vocabulary and working primitives)

## Objective
Integrate a locally-hosted LLM to translate natural-language mission commands into **behavior trees** (not fixed action sequences) using the action vocabulary from Phase 3. The LLM becomes the **mission planner** that converts intent into adaptive, conditional execution plans.

**Architecture Revision**: LLM outputs behavior trees (BT) instead of JSON action sequences to support conditional logic (`IF detection confidence low THEN descend and re-scan`) and iterative refinement required for real missions.

## Rationale
Now that we have:
- Working action primitives (Phase 3)
- Mission definitions (Phase 2)
- Simulation environment (Phase 1)

We need the **reasoning layer** that bridges natural language to actions. The challenge is ensuring the LLM:
- Outputs valid behavior trees (not just linear sequences)
- Supports conditional logic for adaptive missions
- Runs locally on edge hardware (no cloud dependency)
- Responds fast enough for mission planning (<2 seconds initial, <500ms replanning in Phase 5)

**Why Behavior Trees over JSON sequences**:
- **Conditional execution**: `IF battery < 30% THEN prioritize RTH ELSE continue mission`
- **Reactive responses**: `WHILE detecting targets DO capture images`
- **Failure recovery**: `TRY action FALLBACK alternate_action`
- **Composability**: Subtrees can be reused and parameterized

## Goals
1. **LLM Selection & Benchmarking**: Deploy quantized model on development machine; measure latency before committing
2. **Behavior Tree Schema**: Design BT XML/JSON format for LLM output
3. **Structured Generation**: Use constrained generation (Outlines, guidance) to enforce BT schema
4. **Mission Parser**: Natural-language command → behavior tree
5. **BT Executor**: ROS 2 node executing behavior trees with Phase 3 actions
6. **Per-Action Battery Validation**: Ensure LLM checks battery sufficiency for each action (not mission-level)
7. **Mission Execution**: Fly Mission 1 from natural-language tasking with conditional logic

## Success Criteria
- [ ] LLM running locally (MoE or quantized model on GPU)
- [ ] Natural-language input → valid action sequence output
- [ ] <2 second latency for initial mission parse
- [ ] Mission 1 executed successfully from natural-language command
- [ ] LLM outputs conform to action vocabulary 95%+ of the time
- [ ] Safety constraints enforced (geofencing, battery limits)

## Tasks

### 4.1 LLM Selection and Setup

#### Model Selection
- [ ] Research locally-hostable LLM options:
  - **Quantized models** (primary focus): Llama 3.1 8B, Qwen2.5 7B/14B, Mistral 7B
  - **NOT Mixture-of-Experts**: Mixtral 8x7B activates all experts at inference (doesn't save compute as assumed)—too large for Jetson Orin at <2s latency
  - **Function-calling tuned**: Hermes 3, OpenHermes, NousResearch Hermes models
  - Consider model size vs. edge compute constraints (Jetson Orin: 32GB RAM total, need <20GB for LLM after autonomy stack)

- [ ] Evaluation criteria:
  - **Structured output quality**: Test BT generation accuracy with few-shot prompting
  - Context window (need 8k+ tokens for action vocab + BT examples + mission context)
  - **Measured inference latency** on development GPU (not marketing claims): target <2s
  - Quantization support (4-bit GPTQ/AWQ for edge deployment)

- [ ] **Benchmark before committing**:
  - Test candidate models with sample BT generation task
  - Measure actual latency on dev machine and extrapolate to Jetson Orin
  - If no model meets <2s target at acceptable quality → escalate architectural decision (cloud fallback? smaller model? reduced context?)

- [ ] Select model based on:
  - Structured output quality
  - Speed on available hardware
  - Documentation and tooling support

#### Inference Framework
- [ ] Choose inference backend:
  - **llama.cpp**: CPU/GPU, quantization, good for edge
  - **vLLM**: Fast GPU inference, good for development
  - **TensorRT-LLM**: NVIDIA optimized, Jetson compatibility
  - **Ollama**: Easy local hosting, model management

- [ ] Install and configure chosen framework
- [ ] Test inference with sample prompts
- [ ] Benchmark latency and throughput

#### Model Deployment
- [ ] Set up model hosting:
  - Download model weights
  - Configure inference server (REST API or Python bindings)
  - Test API endpoint or library calls
  - Document model path and configuration

- [ ] Create `llm/configs/llm_config.yaml`:
  - Model name and path
  - Inference parameters (temperature, top_p, max_tokens)
  - Hardware settings (GPU layers, threads)
  - API endpoint or local config

**Deliverable**: LLM running locally with API or Python interface

### 4.2 Behavior Tree Schema Design

#### BT Schema Definition
- [ ] Design behavior tree XML/JSON schema in `llm/schemas/behavior_tree_schema.json`:
```xml
<!-- Example BT structure -->
<BehaviorTree>
  <Root>
    <Sequence name="Mission: Survey NAI 3">
      <!-- Initialization -->
      <Action name="TAKEOFF" altitude="30"/>
      <Action name="GOTO_WAYPOINT" lat="38.123" lon="-76.567"/>

      <!-- Main mission logic with conditionals -->
      <ReactiveSequence>
        <Action name="SEARCH_AREA" boundary="..." altitude="30"/>
        <Fallback name="DetectionStrategy">
          <Sequence>
            <Condition name="CheckDetectionConfidence" threshold="0.8"/>
            <Action name="CAPTURE_IMAGES"/>
          </Sequence>
          <Sequence name="LowConfidenceRecovery">
            <Action name="DESCEND" altitude="20"/>
            <Action name="SEARCH_AREA" boundary="..." altitude="20"/>
          </Sequence>
        </Fallback>
      </ReactiveSequence>

      <!-- Return logic with battery check -->
      <Fallback>
        <Sequence>
          <Condition name="CheckBattery" min_percent="25"/>
          <Action name="RTL"/>
        </Sequence>
        <Action name="EMERGENCY_RTL"/>
      </Fallback>
    </Sequence>
  </Root>
</BehaviorTree>
```

- [ ] Define BT node types:
  - **Control nodes**: Sequence, Fallback, Parallel, ReactiveSequence
  - **Condition nodes**: CheckBattery, CheckDetectionConfidence, IsInGeofence
  - **Action nodes**: All Phase 3 strategic actions
  - **Decorator nodes**: Retry, Timeout, ForceSuccess

- [ ] Create JSON-equivalent schema for easier LLM generation (XML verbose)
- [ ] Ensure all Phase 3 actions mappable to BT action nodes
- [ ] Add validation for BT structure (no cycles, valid node types)

#### Constrained Generation Strategy
- [ ] Research constrained generation approaches for BT output:
  - **Outlines library** (recommended): Structured generation with FSM, supports complex schemas
  - **Guidance library**: Grammar-based constrained generation
  - **LangChain with Pydantic**: May struggle with nested BT structure
  - **Custom parsing**: JSON + validation + retry (fallback if libraries fail)

- [ ] Implement chosen approach:
  - Define BT JSON schema with Pydantic models
  - Configure LLM to output valid BT JSON (Outlines can enforce schema during generation)
  - Add BT structure validation (no cycles, valid parent-child relationships)
  - Implement retry logic for invalid outputs (max 3 attempts with error feedback)

- [ ] Test with edge cases:
  - Ambiguous commands ("search the area" without specifying which area)
  - Missing parameters (action without required coords)
  - Invalid BT structure (Fallback with single child—should have ≥2)
  - Out-of-bounds coordinates or altitude violations

- [ ] Validate **per-action battery checks in BT**:
  - Prompt must instruct LLM to insert CheckBattery conditions before power-intensive actions
  - Example: Before SEARCH_AREA, add `<Condition name="CheckBattery" required_percent="40"/>`
  - Validate LLM outputs include battery guards (not just mission-level RTH)

**Deliverable**: LLM consistently outputs valid behavior trees matching schema with battery validation

### 4.3 Mission Parser & Prompt Engineering

#### System Prompt Design
- [ ] Create `llm/prompts/mission_planner_system.md`:
```
You are a mission planning system for an autonomous drone. Your task is to convert natural-language mission objectives into executable BEHAVIOR TREES using ONLY the actions in the provided vocabulary.

**Action Vocabulary**:
[Include Phase 3 strategic actions with parameters and battery estimates]

**Behavior Tree Nodes**:
- Control: Sequence, Fallback, ReactiveSequence
- Conditions: CheckBattery(min_percent), CheckDetectionConfidence(threshold), IsInGeofence
- Actions: [Phase 3 actions]
- Decorators: Retry(max_attempts), Timeout(seconds)

**Safety Requirements**:
- CRITICAL: Insert CheckBattery conditions before EACH power-intensive action
  - SEARCH_AREA requires 40% battery minimum
  - INVESTIGATE_LOCATION requires 25% battery minimum
  - Always check battery allows RTH reserve (20%)
- Maximum altitude: 50m AGL
- All actions must stay within geofence
- Include Fallback nodes for failure recovery

**Output Format**:
[BT JSON schema from 4.2]

**Example - Conditional Logic**:
User: "Survey NAI 3 and identify vehicles. If detection confidence is low, descend and re-scan."
Assistant:
{
  "tree": {
    "type": "Sequence",
    "children": [
      {"type": "Action", "name": "TAKEOFF", "params": {"altitude": 30}},
      {"type": "Action", "name": "GOTO_WAYPOINT", "params": {"lat": 38.123, "lon": -76.567}},
      {
        "type": "ReactiveSequence",
        "children": [
          {
            "type": "Sequence",
            "children": [
              {"type": "Condition", "name": "CheckBattery", "params": {"min_percent": 40}},
              {"type": "Action", "name": "SEARCH_AREA", "params": {"boundary": [...], "altitude": 30}}
            ]
          },
          {
            "type": "Fallback",
            "name": "DetectionStrategy",
            "children": [
              {
                "type": "Sequence",
                "children": [
                  {"type": "Condition", "name": "CheckDetectionConfidence", "params": {"threshold": 0.8}},
                  {"type": "Action", "name": "CAPTURE_IMAGES"}
                ]
              },
              {
                "type": "Sequence",
                "name": "LowConfidenceRecovery",
                "children": [
                  {"type": "Action", "name": "DESCEND", "params": {"altitude": 20}},
                  {"type": "Condition", "name": "CheckBattery", "params": {"min_percent": 35}},
                  {"type": "Action", "name": "SEARCH_AREA", "params": {"boundary": [...], "altitude": 20}}
                ]
              }
            ]
          }
        ]
      },
      {
        "type": "Fallback",
        "children": [
          {
            "type": "Sequence",
            "children": [
              {"type": "Condition", "name": "CheckBattery", "params": {"min_percent": 25}},
              {"type": "Action", "name": "RTL"}
            ]
          },
          {"type": "Action", "name": "EMERGENCY_RTL"}
        ]
      }
    ]
  }
}
```

- [ ] Include 3-5 few-shot BT examples for different mission patterns:
  - Simple linear mission (no conditionals)
  - Battery-constrained mission (prioritization)
  - Adaptive detection mission (conditional descent)
  - Multi-target investigation (parallel subtasks)
- [ ] Add Chain-of-Thought reasoning before BT output
- [ ] **Validate prompt doesn't exceed context window** (may need to trim action vocab descriptions)

#### Mission Context Integration
- [ ] Design context injection:
  - Current drone state (position, battery, mode)
  - Environment info (world name, known landmarks)
  - Available actions with current feasibility
  - Mission history (previous actions, results)

- [ ] Create `mission_context_builder` function:
  - Query ROS topics for drone state
  - Format context for LLM prompt
  - Include relevant constraints

#### Parser Implementation
- [ ] Create `mission_parser_node` in `llmdrone_llm/`:
  - ROS 2 service: `/llm/parse_mission`
  - Input: Natural-language command string
  - Output: JSON action sequence (or error)
  - Call LLM with system prompt + context + user command
  - Validate output against schema
  - Return parsed mission plan

- [ ] Add error handling:
  - Retry on parsing failures (up to 3 attempts)
  - Request clarification for ambiguous commands
  - Log all LLM interactions for debugging

**Deliverable**: Mission parser service converting language to action sequences

### 4.4 Mission Executor

#### Sequence Execution
- [ ] Create `mission_executor_node` in `llmdrone_llm/`:
  - Subscribe to `/llm/mission_plan` topic
  - Execute actions sequentially using Phase 3 action library
  - Call `/drone/execute_action` service for each action
  - Monitor execution status
  - Handle action failures (retry or abort)

- [ ] Add execution monitoring:
  - Publish progress: current action, completion percentage
  - Publish to `/mission/status` topic
  - Log execution timeline
  - Detect and report errors

#### Safety Checks
- [ ] Implement pre-flight validation:
  - Verify all actions are within geofence
  - Check altitude constraints
  - Validate waypoints are reachable
  - Ensure battery sufficient for mission + reserve

- [ ] Add runtime safety monitoring:
  - Battery level checks before each action
  - Geofence violation detection → RTL
  - Loss of GPS → hover and alert
  - Communication timeout → fallback behavior

#### Execution Feedback
- [ ] After mission completion, generate report:
  - Mission summary (actions executed, time taken)
  - Results (detections, images, data collected)
  - Issues encountered (failed actions, warnings)
  - Publish to `/mission/report` topic

**Deliverable**: End-to-end executor from LLM plan to completed mission

### 4.5 Integration Testing

#### Mission 1 from Natural Language
- [ ] Test command: **"Survey NAI 3 and identify vehicle movement"**
  - Launch simulation with `urban_search.world`
  - Start all ROS 2 nodes (bridge, control, perception, actions, LLM)
  - Send command to `/llm/parse_mission` service
  - Verify LLM generates correct action sequence
  - Execute mission via mission_executor
  - Validate mission success criteria met

- [ ] Measure metrics:
  - LLM parse latency
  - Mission execution time
  - Vehicle detection accuracy
  - Success rate over 10 runs

#### Varied Commands
- [ ] Test variations:
  - "Search NAI 3 for trucks and return home"
  - "Fly to waypoint ALPHA, survey the area, and report findings"
  - "Conduct reconnaissance of sector 4 at 40 meters altitude"

- [ ] Evaluate robustness:
  - How does LLM handle variations?
  - Does it maintain safety constraints?
  - Are action sequences efficient?

#### Edge Cases
- [ ] Test failure modes:
  - Command: "Fly to the moon" (impossible request)
  - Command: "Survey for 5 hours" (exceeds battery)
  - Command: "Search outside geofence" (safety violation)
  - Verify LLM refuses or requests clarification

**Deliverable**: Mission 1 executing successfully from natural-language tasking

## Deliverables Checklist
- [ ] LLM deployed locally with inference API
- [ ] JSON schema for action sequences
- [ ] Structured output generation (LangChain/guidance/custom)
- [ ] Mission parser ROS 2 service
- [ ] Mission executor ROS 2 node
- [ ] Safety constraint validation
- [ ] Mission 1 completed from natural-language command
- [ ] Documentation: `docs/llm_integration.md`, prompt engineering guide

## Known Risks and Mitigation

### Risk: LLM outputs invalid behavior trees frequently
**Impact**: High
**Mitigation**: Use Outlines library for schema-constrained generation, validate BT structure, retry with error feedback (max 3), include diverse BT examples in prompt

### Risk: MoE models don't save compute as expected (all experts activated)
**Impact**: Critical (Jetson Orin can't run Mixtral 8x7B at <2s)
**Mitigation**: Benchmark actual latency before committing; focus on quantized 7B-14B models; have cloud fallback contingency

### Risk: Behavior tree complexity exceeds LLM capability
**Impact**: High
**Mitigation**: Start with simpler BT structures (2-3 levels deep), validate with manual examples, simplify if needed

### Risk: Inference latency too high (>2s initial planning)
**Impact**: Critical (Phase 5 requires <500ms replanning—impossible if initial >2s)
**Mitigation**: Benchmark models before selection, use 4-bit quantization, optimize prompt length, cache frequent subtrees; escalate if no model meets target

### Risk: Context window insufficient for action vocab + BT examples + mission state
**Impact**: High
**Mitigation**: Trim action descriptions, use abbreviated notation, paginate action vocab if needed; require ≥8k context model

### Risk: LLM misinterprets commands
**Impact**: High
**Mitigation**: Few-shot prompting, request clarification for ambiguous inputs, human-in-the-loop confirmation

### Risk: Model too large for Jetson deployment
**Impact**: Medium
**Mitigation**: Use 4-bit quantization, smaller MoE models, or consider cloud fallback for development

## Phase Exit Criteria
Before moving to Phase 5, verify:
1. ✓ LLM running locally with <2s parse latency (measured, not estimated)
2. ✓ Natural-language command → valid behavior tree (95%+ success rate over 20 test commands)
3. ✓ BTs include per-action battery validation (not just mission-level)
4. ✓ Mission 1 executed successfully from language tasking with conditional logic
5. ✓ Safety constraints enforced (geofence, altitude, battery checks in BT)
6. ✓ At least 5 different command variations handled correctly (including ambiguous inputs)
7. ✓ BT executor handles Fallback, Sequence, Condition nodes correctly
8. ✓ System documented for deployment with resource consumption baseline
9. ✓ Phase 3 performance budgets still met after LLM integration (CPU, GPU, RAM)

## Next Phase
Once Phase 4 is complete, proceed to [Phase 5 - Dynamic Replanning](Phase-5-Dynamic-Replanning.md) to enable mid-flight adaptive behavior when conditions change or obstacles are encountered.

**Key Achievement**: The drone now understands natural language and can autonomously execute missions without pre-programmed scripts. Phase 5 adds the ability to adapt plans on the fly.

## Notes and Findings
_Use this section to document LLM selection rationale, prompt engineering insights, structured generation challenges, and lessons from natural-language mission testing._

---

**Phase Started**: [Date]
**Phase Completed**: [Date]

**Technical Decisions**:
- [LLM model choice and rationale]
- [Structured output approach]
- [Inference framework selection]

**Prompt Engineering Insights**:
- [What prompting strategies worked best]
- [How many examples needed]
- [How to handle ambiguity]

**Challenges Encountered**:
- [LLM consistency issues]
- [Latency optimization]
- [Integration with ROS 2]

**Natural-Language Understanding**:
- [What command phrasings work well]
- [Common misinterpretations]
- [How to improve parsing accuracy]
