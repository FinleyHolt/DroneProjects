---
allowed-tools: Read, Write, Bash
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

5. **Update Session Manifest**
   - Set status to "handed_off"
   - Clear claimed_paths (release the paths)

6. **Log Handoff**
   - Append to `.claude/coordination.log`:
   ```
   {timestamp} {session-id} HANDOFF {task-slug} -> {target or "any"}
   ```

7. **Confirm**
   ```
   # Task Handed Off

   - Task: {task-slug}
   - Branch: {branch} (pushed)
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
