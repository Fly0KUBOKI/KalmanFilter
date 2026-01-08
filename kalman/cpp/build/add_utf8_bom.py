#!/usr/bin/env python3
import sys
from pathlib import Path

FILES = []

# Legacy explicit list (kept for files that must always be covered)
EXPLICIT = [
    "../MEX/mex_eskf_initializer.cpp",
    "../Lib/MEUKF/src/meukf_core.cpp",
]

ROOT = Path(__file__).resolve().parent

# Directories to recursively scan for source/header files to ensure BOM
RECURSE_DIRS = [
    "../Lib",
    "../Src",
    "../MEX",
    "../Inc",
    "../examples",
    "../../../tests",
]

EXTENSIONS = {'.cpp', '.c', '.hpp', '.h', '.cc', '.hh'}

def discover_files(root: Path):
    found = set()
    for d in RECURSE_DIRS:
        base = root.joinpath(d)
        if not base.exists():
            continue
        for p in base.rglob('*'):
            if p.is_file() and p.suffix.lower() in EXTENSIONS:
                found.add(p)
    return sorted(found)

def add_bom(path: Path) -> bool:
    p = path.resolve()
    if not p.exists():
        print(f"Missing: {p}")
        return False
    data = p.read_bytes()
    bom = b"\xef\xbb\xbf"
    if data.startswith(bom):
        print(f"OK: {p}")
        return True
    # Prepend BOM
    p.write_bytes(bom + data)
    print(f"Added BOM: {p}")
    return True

def main():
    ok = True
    # run explicit list first
    for f in EXPLICIT:
        ok = add_bom(ROOT.joinpath(f)) and ok

    # then discover and apply to all relevant source/header files under project dirs
    for p in discover_files(ROOT):
        ok = add_bom(p) and ok
    if not ok:
        sys.exit(2)

if __name__ == '__main__':
    main()
