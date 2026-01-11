---
allowed-tools: Bash, Read
---

# What's Next

Check the GitHub project board for the next task to work on.

## Instructions

1. **Check Items Currently In Progress**
   ```bash
   GITHUB_TOKEN= gh project item-list 1 --owner FinleyHolt --format json \
     --jq '.items[] | select(.status == "In Progress") | "\(.content.number): \(.title)"'
   ```

2. **List Todo Items (Available Work)**
   ```bash
   GITHUB_TOKEN= gh project item-list 1 --owner FinleyHolt --format json \
     --jq '.items[] | select(.status == "Todo") | "\(.content.number): \(.title) [\(.labels | join(", "))]"'
   ```

3. **Check for Unassigned High-Priority Issues**
   ```bash
   gh issue list --state open --limit 20 --json number,title,labels,assignees \
     --jq '.[] | select(.assignees | length == 0) | "\(.number): \(.title) [\(.labels | map(.name) | join(", "))]"'
   ```

4. **Suggest Next Task**
   - Recommend highest priority unassigned issue from Todo column
   - Show issue details if user wants more info: `gh issue view <number>`

## Workflow

When the user selects an issue to work on:

1. **Use the coordination system:**
   ```
   /claim-task {issue-number}-{short-slug} "Issue title" paths/to/modify/**
   ```
   This will:
   - Create a branch
   - Create/link a GitHub issue
   - Move issue to "In Progress" on the board
   - Lock the paths for exclusive access

2. **Implement the changes**

3. **When done, use:**
   ```
   /complete-task
   ```
   This will:
   - Commit with COMPLETE: prefix
   - Create a PR that closes the issue
   - Move issue to "Done" on the board

## Priority Labels

- `priority:critical` - P0, drop everything
- `priority:high` - P1, do next
- `priority:medium` - P2, normal queue
- `priority:low` - P3, backlog

## Platform Labels

- `platform:isaac-sim` - Isaac Sim simulation
- `platform:project-drone` - Personal dev drone
- `platform:flyby-f11` - Production platform
- `platform:ontology` - Ontology reasoning
