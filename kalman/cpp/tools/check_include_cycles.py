#!/usr/bin/env python3
import os
import re
from collections import defaultdict, deque

root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
search_root = os.path.join(root, 'kalman', 'cpp')

inc_pattern = re.compile(r'#include\s+"([^"]+)"')
files = []
for dirpath, dirnames, filenames in os.walk(search_root):
    for fn in filenames:
        if fn.endswith(('.hpp', '.h', '.cpp')):
            files.append(os.path.join(dirpath, fn))

# Build map from relative path to absolute
rel_map = {}
for f in files:
    rel = os.path.relpath(f, start=search_root).replace('\\','/')
    rel_map[rel] = f

# Build include graph (using relative paths normalized)
graph = defaultdict(set)
for rel, absf in rel_map.items():
    with open(absf, 'r', encoding='utf-8') as fh:
        for line in fh:
            m = inc_pattern.search(line)
            if m:
                inc = m.group(1)
                # normalize path relative to current file dir
                inc_abs = os.path.normpath(os.path.join(os.path.dirname(absf), inc))
                if os.path.exists(inc_abs):
                    inc_rel = os.path.relpath(inc_abs, start=search_root).replace('\\','/')
                    graph[rel].add(inc_rel)

# detect cycles via DFS
visited = set()
stack = []
cycles = []

def dfs(node, path, onstack):
    if node in onstack:
        idx = path.index(node)
        cycles.append(path[idx:]+[node])
        return
    if node in visited:
        return
    visited.add(node)
    onstack.add(node)
    for nxt in graph.get(node, []):
        dfs(nxt, path+[nxt], onstack)
    onstack.remove(node)

for n in list(graph.keys()):
    dfs(n, [n], set())

print('Files scanned:', len(rel_map))
print('Include graph nodes:', len(graph))
print('Cycles found:', len(cycles))
for c in cycles:
    print(' -> '.join(c))
