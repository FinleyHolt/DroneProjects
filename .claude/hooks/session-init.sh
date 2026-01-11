#!/bin/bash
# Auto-initialize session coordination on conversation start
# Hook: SessionStart
#
# This hook:
# 1. Creates a session manifest for tracking
# 2. Queries GitHub for open issues (if gh authenticated)
# 3. Provides context about active sessions and available work

# Don't exit on error - we want graceful degradation
set +e

# Ensure we're in the project directory
if [ -z "$CLAUDE_PROJECT_DIR" ]; then
    exit 0
fi

cd "$CLAUDE_PROJECT_DIR" || exit 0

# Read input from stdin
INPUT=$(cat)

# Parse session_id using Python (more portable than jq)
SESSION_ID=$(echo "$INPUT" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('session_id',''))" 2>/dev/null)

if [ -z "$SESSION_ID" ]; then
    exit 0
fi

MANIFEST=".claude/sessions/active/${SESSION_ID}.json"

# If manifest already exists, we're resuming - just update heartbeat
if [ -f "$MANIFEST" ]; then
    TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
    python3 -c "
import json, sys
with open('$MANIFEST', 'r') as f:
    data = json.load(f)
data['last_heartbeat'] = '$TIMESTAMP'
with open('$MANIFEST', 'w') as f:
    json.dump(data, f, indent=2)
" 2>/dev/null || true
    exit 0
fi

# Ensure directories exist
mkdir -p .claude/sessions/active

# Create session manifest
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
cat > "$MANIFEST" << EOF
{
  "session_id": "${SESSION_ID}",
  "started_at": "${TIMESTAMP}",
  "last_heartbeat": "${TIMESTAMP}",
  "claimed_paths": [],
  "github_issue": null,
  "status": "active"
}
EOF

# Log the session start
echo "${TIMESTAMP} ${SESSION_ID} SESSION_START" >> .claude/coordination.log

# Count active sessions
ACTIVE_COUNT=$(ls -1 .claude/sessions/active/*.json 2>/dev/null | grep -v '.gitkeep' | wc -l)

# Check for claimed tasks in registry
CLAIMED_TASKS=""
if [ -f ".claude/tasks/registry.json" ]; then
    CLAIMED_TASKS=$(python3 -c "
import json
try:
    with open('.claude/tasks/registry.json') as f:
        data = json.load(f)
    for name, task in data.get('tasks', {}).items():
        paths = ', '.join(task.get('paths', []))
        claimed_by = task.get('claimed_by', 'unknown')
        print(f'- {name}: {paths} (by {claimed_by})')
except:
    pass
" 2>/dev/null)
fi

# Try to get open GitHub issues (if gh is authenticated)
GITHUB_ISSUES=""
if command -v gh &> /dev/null; then
    if gh auth status &>/dev/null 2>&1; then
        GITHUB_ISSUES=$(gh issue list --repo FinleyHolt/DroneProjects --state open --limit 5 --json number,title 2>/dev/null | python3 -c "
import json, sys
try:
    issues = json.load(sys.stdin)
    for i in issues:
        print(f\"- #{i['number']}: {i['title']}\")
except:
    pass
" 2>/dev/null)
    fi
fi

# Build context message
CONTEXT="Session ${SESSION_ID} initialized. ${ACTIVE_COUNT} active session(s)."

if [ -n "$CLAIMED_TASKS" ]; then
    CONTEXT="${CONTEXT}

Claimed paths by other sessions:
${CLAIMED_TASKS}"
fi

if [ -n "$GITHUB_ISSUES" ]; then
    CONTEXT="${CONTEXT}

Open GitHub issues:
${GITHUB_ISSUES}

Run /claim-task to claim a task and integrate with GitHub."
fi

# Output context for Claude (escape for JSON)
python3 -c "
import json
context = '''$CONTEXT'''
output = {
    'hookSpecificOutput': {
        'hookEventName': 'SessionStart',
        'additionalContext': context
    }
}
print(json.dumps(output))
" 2>/dev/null || echo '{"hookSpecificOutput":{"hookEventName":"SessionStart","additionalContext":"Session initialized"}}'
