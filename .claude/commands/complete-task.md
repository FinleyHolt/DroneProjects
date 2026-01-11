---
allowed-tools: Read, Write, Bash, Glob, mcp__github__create_pull_request
---

# Complete Task

Mark the current task as complete, create a PR, and move to Done on the project board.

## Instructions

1. **Verify Current Task**
   - Read current session manifest from `.claude/sessions/active/`
   - Confirm there is an active task
   - Read `.claude/tasks/registry.json` for task details and GitHub references

2. **Commit Final State**
   - Stage all changes
   - Commit with `COMPLETE:` prefix and issue reference:
   ```bash
   git add -A
   git commit -m "COMPLETE: {task-slug} - {summary}

   [Session: {session-id}]
   [Task: {task-slug}]
   [Closes #{github_issue}]
   "
   ```
   - Push branch:
   ```bash
   git push -u origin {branch}
   ```

3. **Create Pull Request**
   - Use `mcp__github__create_pull_request`:
     - owner: "FinleyHolt"
     - repo: "DroneProjects"
     - title: task description
     - head: {branch}
     - base: "main"
     - body: Include summary, changes, and "Closes #{github_issue}"

4. **Move to "Done" on Project Board**
   - Read IDs from `.claude/project-config.json`
   ```bash
   GITHUB_TOKEN= gh project item-edit --id {github_project_item_id} \
     --project-id "PVT_kwHOCPfoCc4BMVJy" \
     --field-id "PVTSSF_lAHOCPfoCc4BMVJyzg7pOKE" \
     --single-select-option-id "98236657"
   ```

5. **Update Registry**
   - Set task status to "completed"
   - Keep GitHub references for audit trail

6. **Update Session Manifest**
   - Set status to "completed"
   - Clear claimed_paths

7. **Log Completion**
   - Append to `.claude/coordination.log`:
   ```
   {timestamp} {session-id} COMPLETE {task-slug}
   ```

8. **Confirm**
   ```
   # Task Completed

   - Task: {task-slug}
   - Issue: #{github_issue} (moved to "Done")
   - PR: {pr_url}
   - Branch: {branch}

   The PR is ready for review and merge.
   When merged, the issue will be automatically closed.
   ```

$ARGUMENTS

None required. Operates on current session's active task.

Optional: `--no-pr` to skip PR creation (just move to Done)

Example: `/complete-task`
Example: `/complete-task --no-pr`
