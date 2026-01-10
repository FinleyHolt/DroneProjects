---
allowed-tools: Read, Write, Bash, Glob
---

# Session Start

Initialize a new coordination session for multi-agent work on this repository.

## Instructions

1. **Generate Session ID**
   - Format: `{adjective}-{noun}-{4-hex}` (e.g., `swift-falcon-a3c2`)
   - Use random adjective from: swift, bold, calm, keen, bright, quick, steady, sharp
   - Use random noun from: falcon, vector, horizon, pulse, cipher, beacon, spark, drift
   - Generate 4 random hex characters

2. **Read Coordination State**
   - Read `.claude/tasks/registry.json` to see active task claims
   - Read `.claude/sessions/active/*.json` to see active sessions
   - Check for stale sessions (last_heartbeat > 30 minutes ago)

3. **Report Status**
   ```
   # Session Coordination Status

   ## Your Session
   - ID: {generated-session-id}
   - Started: {timestamp}

   ## Active Sessions
   - {session-id}: {task} (paths: X) - last active: Y min ago

   ## Claimed Paths
   - {path-glob}: claimed by {session-id}

   ## Stale Sessions (recoverable)
   - {session-id}: {task} - inactive for X min

   ## Available Work
   Based on CLAUDE.md roadmap, unclaimed areas include:
   - {area}: {description}
   ```

4. **Create Session Manifest**
   - Create `.claude/sessions/active/{session-id}.json`
   - Set status to "initializing" (will become "active" after claiming task)

5. **Ask User**
   - "What would you like to work on?"
   - Suggest unclaimed areas from roadmap

## Session Manifest Format

```json
{
  "session_id": "swift-falcon-a3c2",
  "started_at": "2026-01-10T14:30:00Z",
  "last_heartbeat": "2026-01-10T14:30:00Z",
  "branch": null,
  "task_description": null,
  "claimed_paths": [],
  "status": "initializing"
}
```

$ARGUMENTS

If arguments provided (e.g., "depth calibration"), immediately claim that task area after initialization.
