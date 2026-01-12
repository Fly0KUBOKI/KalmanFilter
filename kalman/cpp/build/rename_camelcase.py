#!/usr/bin/env python3
import re
import os
import argparse
from pathlib import Path

def to_snake(name: str) -> str:
    s1 = re.sub('(.)([A-Z][a-z]+)', r"\1_\2", name)
    s2 = re.sub('([a-z0-9])([A-Z])', r"\1_\2", s1)
    return s2.lower()

identifier_pattern = re.compile(r"\b([a-z0-9]+(?:[A-Z][A-Za-z0-9]+)+)\b")

def process_file(path: Path, apply: bool):
    text = path.read_text(encoding='utf-8')
    orig = text
    replacements = {}
    def repl(m):
        name = m.group(1)
        # skip if looks like PascalCase (starts with uppercase)
        if name[0].isupper():
            return name
        new = to_snake(name)
        if new != name:
            replacements[name] = new
            return new
        return name
    new_text = identifier_pattern.sub(repl, text)
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
    return changed, replacements


def walk_and_process(root_dir: str, apply: bool):
    root = Path(root_dir)
    total_files = 0
    total_changed = 0
    all_repls = {}
    for p in sorted(root.rglob('*.hpp')) + sorted(root.rglob('*.cpp')):
        total_files += 1
        try:
            changed, repls = process_file(p, apply)
            if changed:
                total_changed += 1
                print(f"Would modify: {p}")
                for k,v in repls.items():
                    print(f"  {k} -> {v}")
                all_repls[str(p)] = repls
        except Exception as e:
            print(f"Error processing {p}: {e}")
    print(f"Scanned {total_files} files, {total_changed} would change")
    # write a summary
    summary = Path(root_dir) / 'build' / 'camelcase_replacements_summary.txt'
    summary.write_text('\n'.join(['{}: {}'.format(k, v) for k,v_dict in all_repls.items() for k,v in v_dict.items()]), encoding='utf-8')
    print('Summary written to', summary)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Conservative camelCase -> snake_case replacer (dry-run default)')
    parser.add_argument('--root', default='..', help='Root directory to scan')
    parser.add_argument('--apply', action='store_true', help='Apply changes')
    args = parser.parse_args()
    walk_and_process(os.path.abspath(args.root), args.apply)
