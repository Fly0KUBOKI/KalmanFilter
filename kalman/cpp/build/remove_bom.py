#!/usr/bin/env python3
import os
from pathlib import Path

def remove_bom_from_file(path: Path):
    try:
        content = path.read_bytes()
        if content.startswith(b'\xef\xbb\xbf'):
            # BOM detected, remove it
            content = content[3:]
            path.write_bytes(content)
            return True
    except Exception as e:
        print(f"Error processing {path}: {e}")
    return False

def walk_and_fix(root_dir: str):
    root = Path(root_dir)
    stats = {"files": 0, "fixed": 0}
    for p in root.rglob('*.hpp'):
        stats['files'] += 1
        if remove_bom_from_file(p):
            stats['fixed'] += 1
            print(f"Fixed BOM: {p}")
    for p in root.rglob('*.cpp'):
        stats['files'] += 1
        if remove_bom_from_file(p):
            stats['fixed'] += 1
            print(f"Fixed BOM: {p}")
    print(f"Scanned {stats['files']} files, fixed {stats['fixed']} files")

if __name__ == '__main__':
    root_dir = os.path.abspath('../..')
    walk_and_fix(root_dir)
