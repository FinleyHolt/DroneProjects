---
allowed-tools: Read, Write, Bash, Glob, mcp__github__search_issues, mcp__github__issue_write, mcp__github__add_issue_comment
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

5. **GitHub Issue Integration**

   Read project config from `.claude/project-config.json` for IDs.

   a. **Search for Existing Issue**
      - Search GitHub issues for one matching the task-slug or description
      - Use `mcp__github__search_issues` with query like task description
      - If found, use that issue number

   b. **If No Issue Exists, Create One**
      - Use `mcp__github__issue_write` with method "create":
        - title: task description
        - body: Include session ID, task slug, claimed paths
        - labels: ["claude-task"] (optional)
      - Capture the issue number from the response

   c. **Add Issue to Project Board**
      ```bash
      GITHUB_TOKEN= gh project item-add 1 --owner FinleyHolt \
        --url "https://github.com/FinleyHolt/DroneProjects/issues/{issue-number}"
      ```
      - Capture the item ID from the output

   d. **Move to "In Progress" Column**
      - Read IDs from `.claude/project-config.json`
      ```bash
      GITHUB_TOKEN= gh project item-edit --id {item-id} \
        --project-id "PVT_kwHOCPfoCc4BMVJy" \
        --field-id "PVTSSF_lAHOCPfoCc4BMVJyzg7pOKE" \
        --single-select-option-id "47fc9ee4"
      ```

   e. **Store GitHub References**
      - Save `github_issue` (issue number) and `github_project_item_id` in registry

6. **Update Registry**
   - Add task to `.claude/tasks/registry.json`:
   ```json
   {
     "task-slug": {
       "claimed_by": "session-id",
       "claimed_at": "ISO-timestamp",
       "description": "Task description",
       "paths": ["path/pattern/**"],
       "status": "in_progress",
       "github_issue": 42,
       "github_project_item_id": "PVTI_xxx..."
     }
   }
   ```

7. **Update Session Manifest**
   - Set branch, task_description, claimed_paths
   - Update status to "active"
   - Update last_heartbeat

8. **Log Claim**
   - Append to `.claude/coordination.log`:
   ```
   {timestamp} {session-id} CLAIM {task-slug}
   ```

9. **Confirm**
   ```
   # Task Claimed

   - Task: {task-slug}
   - Branch: {session-id}/{task-slug}
   - Issue: #{github_issue} (moved to "In Progress")
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
