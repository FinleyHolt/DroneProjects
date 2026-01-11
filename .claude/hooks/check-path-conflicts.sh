#!/bin/bash
# Check for path conflicts before Edit/Write operations
# Hook: PreToolUse (matcher: Edit|Write)
#
# This hook:
# 1. Checks if the file path is claimed by another session
# 2. Warns (exit 1) if conflict but allows operation to proceed
# 3. Auto-tracks edited paths in session manifest

# Don't exit on first error - we want graceful handling
set +e

# Ensure we're in the project directory
if [ -z "$CLAUDE_PROJECT_DIR" ]; then
    exit 0
fi

cd "$CLAUDE_PROJECT_DIR" || exit 0

# Parse input from stdin using Python
INPUT=$(cat)

# Extract tool name, session ID, and file path
read TOOL SESSION_ID FILE_PATH <<< $(echo "$INPUT" | python3 -c "
import sys, json
try:
    d = json.load(sys.stdin)
    tool = d.get('tool_name', '')
    session = d.get('session_id', '')
    file_path = d.get('tool_input', {}).get('file_path', '')
    print(f'{tool} {session} {file_path}')
except:
    print('  ')
" 2>/dev/null)

# Only process Edit and Write operations
if [ "$TOOL" != "Edit" ] && [ "$TOOL" != "Write" ]; then
    exit 0
fi

if [ -z "$SESSION_ID" ] || [ -z "$FILE_PATH" ]; then
    exit 0
fi

# Make path relative to project (remove project dir prefix)
REL_PATH="${FILE_PATH#$CLAUDE_PROJECT_DIR/}"

# Skip .claude/ directory from conflict checking (coordination files)
if [[ "$REL_PATH" == .claude/* ]]; then
    exit 0
fi

# Check registry for conflicts using Python
REGISTRY=".claude/tasks/registry.json"
CONFLICTS=""

if [ -f "$REGISTRY" ]; then
    CONFLICTS=$(python3 -c "
import json
import sys

rel_path = '$REL_PATH'
session_id = '$SESSION_ID'

try:
    with open('$REGISTRY') as f:
        data = json.load(f)

    for task_name, task in data.get('tasks', {}).items():
        # Skip if claimed by same session
        if task.get('claimed_by') == session_id:
            continue
        # Skip if not in progress
        if task.get('status') != 'in_progress':
            continue

        for pattern in task.get('paths', []):
            # Simple glob matching: check if path starts with pattern prefix
            prefix = pattern.rstrip('*').rstrip('/')
            if rel_path.startswith(prefix):
                print(f'{task_name} (claimed by {task.get(\"claimed_by\", \"unknown\")})')
                break
except Exception as e:
    pass
" 2>/dev/null)
fi

if [ -n "$CONFLICTS" ]; then
    # Warn but allow (exit 1 = non-blocking warning)
    echo "PATH CONFLICT: $REL_PATH overlaps with claimed paths:" >&2
    echo "$CONFLICTS" >&2
    echo "Run /session-status to see active sessions, or /handoff to coordinate." >&2

    # Log the conflict
    TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
    echo "${TIMESTAMP} ${SESSION_ID} CONFLICT_WARNING ${REL_PATH}" >> .claude/coordination.log

    exit 1
fi

# No conflict - update session manifest with this path
MANIFEST=".claude/sessions/active/${SESSION_ID}.json"
if [ -f "$MANIFEST" ]; then
    TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
    python3 -c "
import json

manifest_path = '$MANIFEST'
rel_path = '$REL_PATH'
timestamp = '$TIMESTAMP'

try:
    with open(manifest_path, 'r') as f:
        data = json.load(f)

    paths = data.get('claimed_paths', [])
    if rel_path not in paths:
        paths.append(rel_path)
    data['claimed_paths'] = paths
    data['last_heartbeat'] = timestamp

    with open(manifest_path, 'w') as f:
        json.dump(data, f, indent=2)
except:
    pass
" 2>/dev/null || true
fi

exit 0
