#!/usr/bin/env python3
import re
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATTERNS = [
    r"\bnormalize_quat\b",
    r"\bnormalize_quaternion\b",
    r"\bquat_normalize\b",
    r"\bsymmetrize_covariance\b",
    r"\bsymmetrizeCov\b",
    r"\bforce_symmetric\b",
    r"\bmake_symmetric\b",
    r"\bmahalanobis_distance\b",
    r"\bmahalanobis_distance_squared\b",
    r"\bcompute_innovation_and_S\b",
]

def find_files(root):
    for p in root.rglob('*'):
        if p.is_file() and p.suffix in ['.cpp','.hpp','.h','.c','.cc','.md']:
            yield p

results = {pat: [] for pat in PATTERNS}

for f in find_files(ROOT):
    try:
        text = f.read_text(encoding='utf-8', errors='ignore')
    except Exception:
        continue
    for pat in PATTERNS:
        if re.search(pat, text):
            for i, line in enumerate(text.splitlines(), start=1):
                if re.search(pat, line):
                    results[pat].append((str(f.relative_to(ROOT)), i, line.strip()))

out = []
for pat, occ in results.items():
    out.append(f"PATTERN: {pat} -> {len(occ)} occurrences")
    for fn, ln, line in occ:
        out.append(f"  {fn}:{ln}: {line}")
    out.append("")

report = '\n'.join(out)
print(report)

# Also write to file
report_file = ROOT / 'tools' / 'duplicates_report.txt'
report_file.write_text(report, encoding='utf-8')
print('\nReport saved to', report_file)
