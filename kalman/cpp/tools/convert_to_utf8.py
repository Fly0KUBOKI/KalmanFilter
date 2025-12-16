#!/usr/bin/env python3
"""
convert_to_utf8.py
簡易エンコーディング変換ツール: 指定ファイルを CP932/UTF-8 として読み込み、UTF-8(BOM付き)で上書きします。
使い方:
  python tools/convert_to_utf8.py <file1> <file2> ...
(引数未指定時は mex_sensor_filter.cpp と主要ヘッダを変換します)
"""
import sys
from pathlib import Path

def convert(path: Path):
    print('Processing', path)
    data = path.read_bytes()
    text = None
    # try several encodings
    for enc in ('utf-8', 'utf-8-sig', 'cp932', 'shift_jis', 'latin-1'):
        try:
            text = data.decode(enc)
            print('  decoded as', enc)
            break
        except Exception:
            continue
    if text is None:
        print('  Failed to decode', path)
        return False
    # normalize line endings to CRLF and write as UTF-8 with BOM
    text = text.replace('\r\n', '\n').replace('\r', '\n')
    with path.open('w', encoding='utf-8-sig', newline='\r\n') as fh:
        fh.write(text)
    print('  wrote UTF-8 (BOM) with CRLF')
    return True

if __name__ == '__main__':
    args = sys.argv[1:]
    if not args:
        # discover files under kalman/cpp
        base = Path('kalman/cpp')
        args = [str(p) for p in base.rglob('*.cpp')] + [str(p) for p in base.rglob('*.hpp')] + [str(p) for p in base.rglob('*.c')] + [str(p) for p in base.rglob('*.h')]
    ok = True
    for p in args:
        path = Path(p)
        if not path.exists():
            print('Not found:', path)
            ok = False
            continue
        if not convert(path):
            ok = False
    if not ok:
        sys.exit(2)
    print('Done')
