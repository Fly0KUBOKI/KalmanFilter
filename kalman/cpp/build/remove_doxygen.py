#!/usr/bin/env python3
import re
import os
import argparse
from pathlib import Path

def process_file(path: Path, apply: bool):
    text = path.read_text(encoding='utf-8')
    orig = text
    # Remove Doxygen style block comments /** ... */
    text = re.sub(r'/\*\*.*?\*/', '', text, flags=re.DOTALL)
    # Remove block comments that look like file headers containing File: or Author:
    text = re.sub(r'/\*.*?(File:|Author:).*?\*/', '', text, flags=re.DOTALL)
    # Remove single-line @ tags: // @brief, // @param, // @return, etc.
    text = re.sub(r'^\s*//\s*@\w+.*\n', '', text, flags=re.MULTILINE)
    # Remove lines containing @brief/@param/@return anywhere
    text = re.sub(r'^.*@brief.*$\n', '', text, flags=re.MULTILINE)
    text = re.sub(r'^.*@param.*$\n', '', text, flags=re.MULTILINE)
    text = re.sub(r'^.*@return.*$\n', '', text, flags=re.MULTILINE)
    # Remove TODO/FIXME/XXX/HACK lines
    text = re.sub(r'^\s*//.*\b(TODO|FIXME|XXX|HACK)\b.*$\n', '', text, flags=re.IGNORECASE|re.MULTILINE)
    # Remove redundant separators like // ==== or // --- or /*** lines
    text = re.sub(r'^\s*//[-=]{3,}.*$\n', '', text, flags=re.MULTILINE)
    text = re.sub(r'^\s*/\*\*\*+.*$\n', '', text, flags=re.MULTILINE)
    # Collapse 3+ blank lines to 2
    text = re.sub(r'\n{3,}', '\n\n', text)

    changed = (text != orig)
    if changed:
        if apply:
            bak = path.with_suffix(path.suffix + '.bak')
            if not bak.exists():
                bak.write_text(orig, encoding='utf-8')
            else:
                # If backup exists, append a numeric suffix
                i = 1
                while True:
                    bak2 = path.with_suffix(path.suffix + f'.bak{i}')
                    if not bak2.exists():
                        bak2.write_text(orig, encoding='utf-8')
                        break
                    i += 1
            path.write_text(text, encoding='utf-8')
    return changed


def walk_and_process(root_dir: str, apply: bool):
    root = Path(root_dir)
    stats = {"files":0, "changed":0}
    for p in sorted(root.rglob('*.hpp')):
        stats['files'] += 1
        try:
            if process_file(p, apply):
                stats['changed'] += 1
                print(f"Modified: {p}")
        except Exception as e:
            print(f"Error processing {p}: {e}")
    for p in sorted(root.rglob('*.cpp')):
        stats['files'] += 1
        try:
            if process_file(p, apply):
                stats['changed'] += 1
                print(f"Modified: {p}")
        except Exception as e:
            print(f"Error processing {p}: {e}")
    print(f"Scanned {stats['files']} files, modified {stats['changed']} files")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remove Doxygen blocks and TODOs per Phase6')
    parser.add_argument('--root', default='..', help='Root directory to scan (default: one level up)')
    parser.add_argument('--apply', action='store_true', help='Apply changes (writes files). Default is dry-run')
    args = parser.parse_args()
    root_dir = os.path.abspath(args.root)
    print('Root:', root_dir)
    walk_and_process(root_dir, args.apply)
