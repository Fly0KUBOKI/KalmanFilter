#!/usr/bin/env python3
import os, re, shutil
ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))

def make_macro_from_path(relpath):
    # e.g. Lib/ESKF/inc/eskf_core.hpp -> LIB_ESKF_INC_ESKF_CORE_HPP
    s = relpath.replace('\\','/').upper()
    s = re.sub(r'[^A-Z0-9]', '_', s)
    return s

headers = []
for dp, dn, fn in os.walk(ROOT):
    for f in fn:
        if f.endswith('.hpp') or f.endswith('.h'):
            headers.append(os.path.join(dp, f))

# track used macros to avoid duplicates
used = {}
fixed = []
for h in sorted(headers):
    rel = os.path.relpath(h, ROOT).replace('\\','/')
    with open(h, 'rb') as fh:
        raw = fh.read()
    # strip BOM if present
    if raw.startswith(b'\xef\xbb\xbf'):
        raw = raw[3:]
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError:
        # skip binary or other encodings
        continue
    orig = text
    lines = text.splitlines()
    # remove leading empty lines
    while lines and lines[0].strip() == '':
        lines.pop(0)
    # ensure #pragma once at top
    pragma_added = False
    if not lines or not re.match(r'^\s*#pragma\s+once', lines[0]):
        lines.insert(0, '#pragma once')
        pragma_added = True
    # find existing ifndef/define
    ifndef_idx = None; define_idx = None; endif_idx = None
    for i,l in enumerate(lines[:40]):
        ifndef_m = re.match(r'^\s*#ifndef\s+([A-Za-z0-9_]+)', l)
        if ifndef_m and ifndef_idx is None:
            ifndef_idx = i; ifndef_name = ifndef_m.group(1)
        define_m = re.match(r'^\s*#define\s+([A-Za-z0-9_]+)', l)
        if define_m and define_idx is None:
            define_idx = i; define_name = define_m.group(1)
    # find last endif
    for i in range(len(lines)-1, -1, -1):
        if re.match(r'^\s*#endif', lines[i]):
            endif_idx = i; break
    # decide guard name
    macro = make_macro_from_path(rel)
    # avoid duplicates
    if macro in used:
        suffix = 1
        while f"{macro}_{suffix}" in used:
            suffix += 1
        macro = f"{macro}_{suffix}"
    used[macro] = rel
    # insert ifndef/define after pragma (index 1)
    if ifndef_idx is None or define_idx is None or ifndef_name != macro or define_name != macro:
        # remove existing mismatched ifndef/define
        # simplest: remove any existing ifndef/define within first 40 lines
        new_head = []
        skip_range = set()
        for i,l in enumerate(lines[:40]):
            if re.match(r'^\s*#ifndef\b', l) or re.match(r'^\s*#define\b', l):
                skip_range.add(i)
        for i,l in enumerate(lines[:40]):
            if i in skip_range: continue
            new_head.append(l)
        # ensure pragma is first line
        if not new_head or not re.match(r'^\s*#pragma\s+once', new_head[0]):
            new_head.insert(0, '#pragma once')
        # insert guards after pragma
        new_head.insert(1, f'#ifndef {macro}')
        new_head.insert(2, f'#define {macro}')
        # splice back remainder
        lines = new_head + lines[40:]
        # ensure endif at end
        if not any(re.match(r'^\s*#endif', l) for l in lines[-3:]):
            lines.append('')
            lines.append(f'#endif // {macro}')
    else:
        # existing correct define/ifndef present; ensure endif comment
        if endif_idx is None:
            lines.append('')
            lines.append(f'#endif // {macro}')
        else:
            # replace last endif with comment version
            lines[endif_idx] = f'#endif // {macro}'
    new_text = '\n'.join(lines) + ('\n' if orig.endswith('\n') else '')
    if new_text != orig:
        bak = h + '.bakhdr'
        shutil.copy2(h, bak)
        with open(h, 'w', encoding='utf-8') as fh:
            fh.write(new_text)
        fixed.append(rel)

print(f"Scanned {len(headers)} headers, fixed {len(fixed)} headers")
for p in fixed:
    print("Fixed:", p)
