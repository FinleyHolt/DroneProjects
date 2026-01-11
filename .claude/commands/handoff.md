---
allowed-tools: Read, Write, Bash, mcp__github__add_issue_comment
---

# Handoff Task

Hand off the current task to another session or make it available for pickup.

## Instructions

1. **Verify Current Task**
   - Read current session manifest from `.claude/sessions/active/`
   - Confirm there is an active task to hand off

2. **Commit Current State**
   - Stage all changes
   - Commit with `HANDOFF:` prefix:
   ```bash
   git add -A
   git commit -m "HANDOFF: {task-slug} - {summary}

   [Session: {session-id}]
   [Task: {task-slug}]

   Handing off to: {target-session or 'any'}
   "
   ```
   - Push branch

3. **Create Handoff Document**
   - Create `.claude/handoffs/{task-slug}.md`:

   ```markdown
   # Handoff: {task-slug}

   ## From/To
   - From: {session-id}
   - Branch: {session-id}/{task-slug}
   - Time: {ISO-timestamp}
   - To: {target-session-id or "any"}

   ## Task Status
   - [ ] Remaining item 1
   - [ ] Remaining item 2
   - [x] Completed item 1

   ## Context
   {Key decisions, current state, blockers}

   ## Files Modified
   - path/to/file1.py - {what changed}
   - path/to/file2.cpp - {what changed}

   ## Next Steps
   1. {Specific next action}
   2. {Second action}

   ## Known Issues / Blockers
   - {Any blockers or issues to be aware of}
   ```

4. **Update Registry**
   - Set task status to "handoff_pending"
   - Add "handoff_to" field (session-id or "any")

5. **Update GitHub Project Board**

   If task has `github_issue` and `github_project_item_id` in registry:

   a. **Move to "Todo" Column**
      - Read IDs from `.claude/project-config.json`
      ```bash
      GITHUB_TOKEN= gh project item-edit --id {github_project_item_id} \
        --project-id "PVT_kwHOCPfoCc4BMVJy" \
        --field-id "PVTSSF_lAHOCPfoCc4BMVJyzg7pOKE" \
        --single-select-option-id "f75ad846"
      ```

   b. **Add Handoff Comment to Issue**
      - Use `mcp__github__add_issue_comment`:
        - issue_number: {github_issue}
        - owner: "FinleyHolt"
        - repo: "DroneProjects"
        - body: "Handed off by session {session-id}.\n\nSee handoff document: `.claude/handoffs/{task-slug}.md`\n\nBranch: {branch} (pushed)\nAvailable to: {target or 'any'}"

6. **Update Session Manifest**
   - Set status to "handed_off"
   - Clear claimed_paths (release the paths)

7. **Log Handoff**
   - Append to `.claude/coordination.log`:
   ```
   {timestamp} {session-id} HANDOFF {task-slug} -> {target or "any"}
   ```

8. **Confirm**
   ```
   # Task Handed Off

   - Task: {task-slug}
   - Branch: {branch} (pushed)
   - Issue: #{github_issue} (moved to "Todo")
   - Handoff doc: .claude/handoffs/{task-slug}.md
   - Available to: {target-session or "any session"}

   The task paths are now released. Another session can claim them.

   To pick up this task in a new session:
   1. /session-start
   2. /claim-task --recover {task-slug}
   ```

$ARGUMENTS

Optional: target session ID to hand off to specifically.
If no argument, task becomes available to any session.

Example: `/handoff` (available to any)
Example: `/handoff bold-horizon-f17e` (specific session)
