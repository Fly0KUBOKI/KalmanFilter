#!/usr/bin/env python3
import os
import re
from collections import defaultdict

root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
lib_root = os.path.join(root, 'kalman', 'cpp')

pattern = re.compile(r'^\s*#ifndef\s+([A-Z0-9_]+)')
occ = defaultdict(list)

for dirpath, dirnames, filenames in os.walk(lib_root):
    for fn in filenames:
        if not fn.endswith(('.hpp', '.h', '.cpp', '.cxx', '.cc')):
            continue
        fp = os.path.join(dirpath, fn)
        try:
            with open(fp, 'r', encoding='utf-8') as f:
                for i, line in enumerate(f, start=1):
                    m = pattern.match(line)
                    if m:
                        occ[m.group(1)].append((fp, i))
        except Exception as e:
            print('Error reading', fp, e)

# report duplicates (guards that appear more than once)
dups = {k:v for k,v in occ.items() if len(v)>1}
print('Found {} unique guards, {} duplicates'.format(len(occ), len(dups)))
for k,v in sorted(dups.items(), key=lambda x: -len(x[1])):
    print('\nGuard:', k, 'occurs', len(v), 'times')
    for fp,i in v:
        print(' -', fp, 'line', i)
