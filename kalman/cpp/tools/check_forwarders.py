#!/usr/bin/env python3
import os
import re

root = os.path.join(os.path.dirname(__file__), '..')
root = os.path.abspath(root)

issues = []
for dirpath, dirnames, filenames in os.walk(os.path.join(root, 'src')):
    for fn in filenames:
        if not fn.endswith('.cpp'):
            continue
        path = os.path.join(dirpath, fn)
        with open(path, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.read().splitlines()
        # detect include that references MEUKF implementation in Lib (migration forwarders)
        includes = [l for l in lines if re.search(r"#\s*include\s*\".*(Lib/|../Lib/).*MEUKF.*(\\.cpp|\\.hpp)\"", l)]
        nonblank = [l for l in lines if l.strip()!='']
        # If file includes a Lib .cpp/.hpp but has more than just include+comments, flag
        if includes:
            # Count code-like lines excluding includes and comments
            code_lines = 0
            for l in nonblank:
                ls = l.strip()
                if ls.startswith('//') or ls.startswith('/*') or ls.startswith('*'):
                    continue
                if re.match(r"#\s*include", ls):
                    continue
                code_lines += 1
            if code_lines > 5:
                issues.append((path, len(includes), code_lines))

# Also flag any src/*.cpp that do NOT contain a function header (heuristic):
# If the file contains 'for (' at top before any function, could be broken.
for dirpath, dirnames, filenames in os.walk(os.path.join(root, 'src')):
    for fn in filenames:
        if not fn.endswith('.cpp'):
            continue
        path = os.path.join(dirpath, fn)
        with open(path, 'r', encoding='utf-8', errors='ignore') as f:
            text = f.read()
        # If '#include' then many tokens that look like free-standing code near top
        lines = text.splitlines()
        head = '\n'.join(lines[:40])
        if re.search(r"\n\s*for\s*\(|\n\s*if\s*\(|\n\s*return\s*", head):
            # But ignore if inside a function declaration - heuristic: presence of '::' before
            if '::' not in head:
                issues.append((path, 'suspicious_top_level_code'))

if not issues:
    print('OK: forwarder check passed')
else:
    print('WARN: found potential forwarder issues:')
    for it in issues:
        print(it)

