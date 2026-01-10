---
allowed-tools: Read, Write, Bash, Glob
---

# Claim Task

Claim a task and its associated paths for the current session.

## Instructions

1. **Verify Session**
   - Confirm current session ID from `.claude/sessions/active/`
   - If not in a session, prompt to run `/session-start` first

2. **Parse Task Details**
   - Task slug (kebab-case): e.g., `depth-calibration`
   - Description: Brief description of work
   - Paths: Glob patterns for files this task will modify

3. **Check for Conflicts**
   - Read `.claude/tasks/registry.json`
   - For each path to claim:
     - Check if any existing task claims overlap
     - If conflict with active session: STOP and report conflict
     - If conflict with stale session (>2h no commits): offer to recover

4. **Create Branch**
   ```bash
   git checkout -b {session-id}/{task-slug}
   ```

5. **Update Registry**
   - Add task to `.claude/tasks/registry.json`:
   ```json
   {
     "task-slug": {
       "claimed_by": "session-id",
       "claimed_at": "ISO-timestamp",
       "description": "Task description",
       "paths": ["path/pattern/**"],
       "status": "in_progress"
     }
   }
   ```

6. **Update Session Manifest**
   - Set branch, task_description, claimed_paths
   - Update status to "active"
   - Update last_heartbeat

7. **Log Claim**
   - Append to `.claude/coordination.log`:
   ```
   {timestamp} {session-id} CLAIM {task-slug}
   ```

8. **Confirm**
   ```
   # Task Claimed

   - Task: {task-slug}
   - Branch: {session-id}/{task-slug}
   - Paths Claimed:
     - {path-pattern}

   You now have exclusive access to these paths. Other sessions will see your claim.

   Remember to commit with prefixes: WIP:, CHECKPOINT:, COMPLETE:
   ```

## Conflict Resolution

If paths conflict with another session:
```
# Path Conflict Detected

The following paths are already claimed:
- {path}: claimed by {other-session} for "{task}"

Options:
1. Choose different paths
2. Coordinate with other session (see /session-status)
3. If other session is stale, use /claim-task --recover {task-slug}
```

$ARGUMENTS

Required: task-slug "Description" path1 [path2 ...]

Example: `/claim-task depth-calibration "Fix D455 depth scaling" ros2_ws/src/flyby_depth/**`

Optional flag: `--recover` to take over a stale task
