#!/usr/bin/env python3
import re
import os
import argparse
from pathlib import Path

def make_guard_name(root: Path, path: Path) -> str:
    rel = path.relative_to(root).as_posix()
    name = re.sub(r'[^0-9A-Za-z]+', '_', rel).upper()
    # remove leading/trailing underscores
    name = name.strip('_')
    if not name.endswith('_HPP'):
        name = name + '_HPP'
    return name

def process_file(path: Path, root: Path, apply: bool):
    text = path.read_text(encoding='utf-8')
    orig = text
    lines = text.splitlines()
    # Ensure #pragma once at top (before includes/guards)
    i = 0
    while i < len(lines) and lines[i].strip() == '':
        i += 1
    if i < len(lines) and lines[i].strip() != '#pragma once':
        lines.insert(i, '#pragma once')
    # Normalize include order: quoted includes first, then system includes
    include_pattern = re.compile(r'^(\s*#include\s+["<].+[">])\s*$')
    includes = []
    include_idxs = []
    for idx, ln in enumerate(lines):
        if include_pattern.match(ln):
            includes.append(ln)
            include_idxs.append(idx)
    if includes:
        # Determine new order
        quoted = [inc for inc in includes if '"' in inc]
        system = [inc for inc in includes if '<' in inc and '"' not in inc]
        new_includes = quoted + system
        # Replace in place: remove old include lines, insert new_includes at first include index
        first_idx = include_idxs[0]
        # remove by indices descending
        for idx in sorted(include_idxs, reverse=True):
            del lines[idx]
        for offset, inc in enumerate(new_includes):
            lines.insert(first_idx + offset, inc)
    # Header guard pair #ifndef/#define
    guard_name = make_guard_name(root, path)
    has_ifndef = any(re.match(r'\s*#ifndef\s+\w+', ln) for ln in lines)
    if has_ifndef:
        # replace existing guard name occurrences for #ifndef and #define
        lines = [re.sub(r'^(\s*#ifndef\s+)\w+', r"\1" + guard_name, ln) if re.match(r'\s*#ifndef\s+\w+', ln) else ln for ln in lines]
        lines = [re.sub(r'^(\s*#define\s+)\w+', r"\1" + guard_name, ln) if re.match(r'\s*#define\s+\w+', ln) else ln for ln in lines]
    else:
        # insert #ifndef/#define after #pragma once line
        inserted = False
        for idx, ln in enumerate(lines):
            if ln.strip() == '#pragma once':
                lines.insert(idx+1, '')
                lines.insert(idx+2, f'#ifndef {guard_name}')
                lines.insert(idx+3, f'#define {guard_name}')
                inserted = True
                break
        if not inserted:
            lines.insert(0, f'#ifndef {guard_name}')
            lines.insert(1, f'#define {guard_name}')

    # Ensure trailing #endif with comment
    if not any(re.match(r'\s*#endif', ln) for ln in lines[-3:]):
        lines.append('')
        lines.append(f'#endif // {guard_name}')
    else:
        # replace last #endif comment to include guard
        for i in range(len(lines)-1, -1, -1):
            if re.match(r'\s*#endif', lines[i]):
                lines[i] = f'#endif // {guard_name}'
                break

    new_text = '\n'.join(lines) + '\n'
    changed = new_text != orig
    if changed and apply:
        bak = path.with_suffix(path.suffix + '.bak')
        if not bak.exists():
            bak.write_text(orig, encoding='utf-8')
        else:
            i = 1
            while True:
                bak2 = path.with_suffix(path.suffix + f'.bak{i}')
                if not bak2.exists():
                    bak2.write_text(orig, encoding='utf-8')
                    break
                i += 1
        path.write_text(new_text, encoding='utf-8')
    return changed


def walk_and_process(root_dir: str, apply: bool):
    root = Path(root_dir)
    stats = {"files":0, "changed":0}
    for p in sorted(root.rglob('*.hpp')):
        stats['files'] += 1
        try:
            if process_file(p, root, apply):
                stats['changed'] += 1
                print(f"Modified: {p}")
        except Exception as e:
            print(f"Error processing {p}: {e}")
    print(f"Scanned {stats['files']} header files, modified {stats['changed']} files")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Normalize header guards and include order per Phase6')
    parser.add_argument('--root', default='..', help='Root directory to scan (default: one level up)')
    parser.add_argument('--apply', action='store_true', help='Apply changes (writes files). Default is dry-run')
    args = parser.parse_args()
    root_dir = os.path.abspath(args.root)
    print('Root:', root_dir)
    walk_and_process(root_dir, args.apply)
