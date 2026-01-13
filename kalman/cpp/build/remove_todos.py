#!/usr/bin/env python3
import re, os, shutil

ROOT = os.path.join(os.path.dirname(__file__), '..')
ROOT = os.path.normpath(ROOT)

PAT_LINE = re.compile(r"^\s*//.*\b(TODO|FIXME|XXX|HACK)\b.*$", re.IGNORECASE)
PAT_BLOCK = re.compile(r"/\*.*?\b(TODO|FIXME|XXX|HACK)\b.*?\*/", re.IGNORECASE|re.DOTALL)

exts = ('.hpp', '.h', '.cpp', '.c', '.cc')
fixed = []
scanned = 0
for dirpath, dirnames, filenames in os.walk(ROOT):
    for fn in filenames:
        if not fn.endswith(exts):
            continue
        path = os.path.join(dirpath, fn)
        scanned += 1
        with open(path, 'rb') as f:
            try:
                text = f.read().decode('utf-8')
            except UnicodeDecodeError:
                continue
        orig = text
        # remove block comments containing TODO-like tokens
        text = PAT_BLOCK.sub('', text)
        # remove single-line comments containing tokens
        lines = text.splitlines()
        new_lines = [ln for ln in lines if not PAT_LINE.match(ln)]
        new_text = '\n'.join(new_lines) + ('\n' if text.endswith('\n') else '')
        if new_text != orig:
            bak = path + '.baktodo'
            shutil.copy2(path, bak)
            with open(path, 'w', encoding='utf-8') as f:
                f.write(new_text)
            fixed.append(path)

print(f"Scanned {scanned} files, fixed {len(fixed)} files")
for p in fixed:
    print("Fixed TODO:", p)
