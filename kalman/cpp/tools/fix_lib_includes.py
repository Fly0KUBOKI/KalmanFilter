#!/usr/bin/env python3
import os
import re

root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
lib_root = os.path.join(root, 'kalman', 'cpp', 'Lib')

pattern = re.compile(r'#include\s+"([^"]*Lib/[^\"]+)"')

changed_files = []

for dirpath, dirnames, filenames in os.walk(lib_root):
    for fn in filenames:
        if not fn.endswith(('.hpp', '.h', '.cpp', '.cxx', '.cc')):
            continue
        fp = os.path.join(dirpath, fn)
        with open(fp, 'r', encoding='utf-8') as f:
            s = f.read()
        new_s = s
        matches = list(pattern.finditer(s))
        if not matches:
            continue
        for m in matches:
            inc_path = m.group(1)  # e.g. ../../Lib/Common/inc/...
            # Find suffix after the first 'Lib/' occurrence
            idx = inc_path.find('Lib/')
            suffix = inc_path[idx+len('Lib/'):]
            target = os.path.join(lib_root, suffix.replace('/', os.sep))
            if not os.path.exists(target):
                # try without 'inc' prefix or alternative formations
                alt = os.path.join(lib_root, suffix)
                if os.path.exists(alt):
                    target = alt
                else:
                    # Can't resolve: skip
                    print('Warning: target not found for', inc_path, 'from', fp)
                    continue
            rel = os.path.relpath(target, start=os.path.dirname(fp)).replace('\\','/')
            new_include = '#include "' + rel + '"'
            # replace the specific include occurrence
            new_s = new_s.replace(m.group(0), new_include)
        if new_s != s:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(new_s)
            changed_files.append(fp)

print('Updated {} files'.format(len(changed_files)))
for cf in changed_files:
    print(' -', cf)
