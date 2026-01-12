#!/usr/bin/env python3
import os, re
ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
headers = []
for dp, dn, fn in os.walk(ROOT):
    for f in fn:
        if f.endswith('.hpp') or f.endswith('.h'):
            headers.append(os.path.join(dp, f))

pat_ifndef = re.compile(r'^\s*#ifndef\s+([A-Za-z0-9_]+)')
pat_define = re.compile(r'^\s*#define\s+([A-Za-z0-9_]+)')
pat_pragma = re.compile(r'^\s*#pragma\s+once')
pat_endif = re.compile(r'^\s*#endif')

macro_map = {}
issues = []

for h in sorted(headers):
    try:
        with open(h, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except Exception as e:
        issues.append((h, 'read_error', str(e)))
        continue
    has_pragma = any(pat_pragma.search(l) for l in lines)
    ifndef = None
    define = None
    endif = False
    for i,l in enumerate(lines):
        if ifndef is None:
            m = pat_ifndef.search(l)
            if m:
                ifndef = (m.group(1), i)
                continue
        if define is None and ifndef is not None:
            m = pat_define.search(l)
            if m:
                define = (m.group(1), i)
                continue
        if pat_endif.search(l):
            endif = True
    rel = os.path.relpath(h, ROOT).replace('\\','/')
    # record macro usage
    if ifndef:
        macro = ifndef[0]
        macro_map.setdefault(macro, []).append(rel)
    # checks
    if not has_pragma:
        issues.append((rel, 'missing_pragma'))
    if ifndef is None and define is None:
        issues.append((rel, 'missing_ifndef_define'))
    elif ifndef and define and ifndef[0] != define[0]:
        issues.append((rel, 'mismatched_macro', ifndef[0], define[0]))
    if not endif:
        issues.append((rel, 'missing_endif'))

# find duplicate macros used in multiple files
for macro, files in macro_map.items():
    if len(files) > 1:
        issues.append((', '.join(files), 'duplicate_macro', macro))

# summary
print('Scanned headers:', len(headers))
print('Issues found:', len(issues))
for it in issues:
    if it[1] == 'missing_pragma':
        print('[MISSING PRAGMA ]', it[0])
    elif it[1] == 'missing_ifndef_define':
        print('[MISSING IFNDEF/DEFINE]', it[0])
    elif it[1] == 'mismatched_macro':
        print('[MISMATCHED MACRO ]', it[0], 'ifndef=', it[2], 'define=', it[3])
    elif it[1] == 'missing_endif':
        print('[MISSING ENDIF   ]', it[0])
    elif it[1] == 'duplicate_macro':
        print('[DUPLICATE MACRO ]', it[2], 'used in:', it[0])
    elif it[1] == 'read_error':
        print('[READ ERROR      ]', it[0], it[2])

# exit code
if issues:
    print('\nRecommendation: review listed files and fix header guards to follow Phase6 conventions.')
else:
    print('\nAll header guards look consistent.')
