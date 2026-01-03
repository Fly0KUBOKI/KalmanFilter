CMake build instructions (kalman/cpp)

# Quick build (Windows, assuming MSVC + CMake installed)

1. Create build directory:

```bash
mkdir build && cd build
cmake .. -S .. -B . -G "Visual Studio 17 2022"
cmake --build . --config Release --target kalman_lib
```

2. To build examples:

```bash
cmake --build . --config Release --target main_eskf
```

# Notes
- The CMake skeleton collects sources under `Lib/*/src/*.cpp` and builds `kalman_lib` static library.
- Include dirs include `Lib/*/inc` and top-level `Inc` (for backward compatibility).
- This is a minimal starting point; refine compile options and per-module targets as needed.
