#!/usr/bin/env python3
import sys
from pathlib import Path

FILES = [
    "../Inc/ESKF/eskf_initializer.hpp",
    "../MEX/Inc/mex_eskf_common.hpp",
    "../MEX/Inc/mex_helpers.hpp",
    "../MEX/Inc/mex_type_conversion.hpp",
    "../MEX/mex_eskf_initializer.cpp",
    "../../src/ESKF/eskf_initializer.cpp",
    "../Lib/MEUKF/inc/meukf_core.hpp",
    "../Lib/MEUKF/inc/meukf_types.hpp",
    "../Lib/MEUKF/inc/unified_filter.hpp",
    "../Lib/MEUKF/inc/unified_types.hpp",
    "../Lib/MEUKF/src/meukf_core.cpp",
    "../Lib/KF/inc/kf_core.hpp",
    "../Lib/Common/inc/standalone.hpp",
    "../Lib/Common/src/standalone.cpp",
    "../Lib/ESKF/inc/filter.hpp",
    "../Lib/MEUKF/src/unified_filter.cpp",
    "../examples/main_eskf.cpp",
]

# Additional files reported by pre-commit checks
FILES += [
    "../Lib/Common/inc/utils.hpp",
    "../Lib/Common/src/interface_stub.cpp",
    "../Lib/ESKF/src/filter.cpp",
    "../examples/test_interface.cpp",
]
FILES += [
    "../examples/main_standalone.cpp",
]

ROOT = Path(__file__).resolve().parent

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
    for f in FILES:
        ok = add_bom(ROOT.joinpath(f)) and ok
    if not ok:
        sys.exit(2)

if __name__ == '__main__':
    main()
