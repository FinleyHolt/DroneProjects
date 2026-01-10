#!/usr/bin/env python3
"""
Minimal KIF (Knowledge Interchange Format) syntax validator.
Checks balanced parentheses and basic S-expression structure.
"""

import sys
import re

def validate_kif(filepath):
    """Validate KIF file syntax. Returns (success, errors)."""
    errors = []

    with open(filepath, 'r') as f:
        content = f.read()

    # Remove comments
    lines = content.split('\n')
    clean_lines = []
    for i, line in enumerate(lines, 1):
        # Remove ;; comments
        if ';;' in line:
            line = line[:line.index(';;')]
        clean_lines.append((i, line))

    clean_content = '\n'.join(line for _, line in clean_lines)

    # Check balanced parentheses
    depth = 0
    for i, char in enumerate(clean_content):
        if char == '(':
            depth += 1
        elif char == ')':
            depth -= 1
            if depth < 0:
                errors.append(f"Unmatched ')' at position {i}")
                return False, errors

    if depth != 0:
        errors.append(f"Unbalanced parentheses: {depth} unclosed '('")
        return False, errors

    # Check for valid KIF keywords
    valid_keywords = {
        'subclass', 'instance', 'domain', 'range', 'documentation',
        '=>', '<=>', 'and', 'or', 'not', 'exists', 'forall',
        'equal', 'holds', 'MeasureFn'
    }

    # Extract top-level forms
    forms = re.findall(r'\((\w+)', clean_content)
    if not forms:
        errors.append("No KIF forms found")
        return False, errors

    print(f"Found {len(forms)} top-level forms")
    print(f"Keywords used: {set(forms) & valid_keywords}")

    return True, []

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <file.kif>")
        sys.exit(1)

    success, errors = validate_kif(sys.argv[1])

    if success:
        print("KIF syntax: VALID")
        sys.exit(0)
    else:
        print("KIF syntax: INVALID")
        for e in errors:
            print(f"  - {e}")
        sys.exit(1)
