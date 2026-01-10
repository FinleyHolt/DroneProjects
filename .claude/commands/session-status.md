---
allowed-tools: Read, Glob, Bash
---

# Session Status

Check the current multi-session coordination state.

## Instructions

1. **Read Coordination Files**
   - `.claude/tasks/registry.json` - active task claims
   - `.claude/sessions/active/*.json` - active session manifests
   - `.claude/coordination.log` - recent coordination events (last 20 lines)

2. **Identify Current Session**
   - Check if running in an active session (look for session manifest matching current work)
   - If not in a session, note that

3. **Check Session Health**
   For each active session:
   - Calculate time since last_heartbeat
   - Mark as "active" (<5 min), "idle" (5-30 min), or "stale" (>30 min)

4. **Output Status Report**

```
# Multi-Session Coordination Status

## Current Session
- ID: {session-id} (or "Not in a coordinated session")
- Task: {task-description}
- Branch: {branch-name}
- Claimed Paths: {count} patterns

## All Active Sessions

| Session | Task | Status | Last Active | Paths |
|---------|------|--------|-------------|-------|
| swift-falcon-a3c2 | Depth calibration | Active | 2 min | 2 |
| bold-horizon-f17e | BT refactor | Idle | 15 min | 3 |

## Path Claims

| Path Pattern | Claimed By | Since |
|--------------|------------|-------|
| ros2_ws/src/flyby_depth/** | swift-falcon-a3c2 | 1h ago |
| ros2_ws/src/behavior_trees/** | bold-horizon-f17e | 45m ago |

## Stale Sessions (Recovery Available)
- {session-id}: {task} - inactive {X} min - Branch: {branch}

## Recent Activity (from coordination.log)
- {timestamp} {session} {action} {task}
```

$ARGUMENTS

If "json" argument provided, output raw JSON instead of formatted report.
