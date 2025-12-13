# Kalman Filter Implementation - AI Coding Agent Guide

## Project Overview

This is a hybrid MATLAB/C++ implementation of multiple Kalman filter variants (KF, EKF, ESKF, UKF, MEUKF) for sensor fusion and state estimation, with critical computational cores migrated to C++ MEX for performance.

**Architecture Philosophy**: MATLAB orchestrates the simulation workflow, sensor filtering, and adaptive noise estimation, while C++ MEX handles performance-critical filter computations (predict/update steps).

## Critical Architecture Knowledge

### Hybrid MATLAB/C++ Design

**C++ Core (31% of codebase, 100% of compute-intensive operations)**:
- All filter prediction/update steps executed in [`cpp/MEUKF/meukf_core.cpp`](kalman/cpp/MEUKF/meukf_core.cpp) (~1100 lines)
- Main MEX interface: `mex_meukf_step_v2.mexw64` handles predict + 4 sensor updates (accel/mag/GPS/baro)
- Supporting libraries: quaternion math, fixed-size matrix operations, sensor filtering (C++ versions in [`cpp/include/Common/`](kalman/cpp/include/Common/))

**MATLAB Orchestration Layer (69% of codebase)**:
- [`ESKF/@ESKF/ESKF.m`](kalman/ESKF/@ESKF/ESKF.m): Main filter class (217 lines) - handles initialization, state management
- [`@ESKF/sensor_updates.m`](kalman/ESKF/@ESKF/sensor_updates.m): Unified sensor dispatch (4 types)
- [`@ESKF/call_cpp_update_impl.m`](kalman/ESKF/@ESKF/call_cpp_update_impl.m): **Key integration point** - marshals MATLAB structs to MEX interface
- Adaptive components: [`Common/Estimation/NoiseEstimatorLib.m`](kalman/Common/Estimation/NoiseEstimatorLib.m), [`Common/Sensor/SensorFilterLib.m`](kalman/Common/Sensor/SensorFilterLib.m)

### Filter Execution Flow

```
run_simulation.m (entry point)
  └─> ESKF.m constructor (static period noise calibration)
      └─> Main loop (run_filter)
          ├─> predict() → call_cpp_update_impl → mex_meukf_step_v2 (C++)
          ├─> sensor_updates('accel') → call_cpp_update_impl (freq: 5 samples)
          ├─> sensor_updates('mag') → call_cpp_update_impl (freq: 25 samples)
          ├─> sensor_updates('gps') → call_cpp_update_impl (freq: 40 samples)
          ├─> sensor_updates('baro') → call_cpp_update_impl (freq: 50 samples)
          └─> zupt/reset checks (MATLAB logic)
```

**Critical Pattern**: Every sensor update follows this sequence:
1. MATLAB pre-filtering (SensorFilterLib) + outlier detection
2. Pack state into struct: `{p, v, q, ba, bg, P}` + sensor data + params
3. Call `mex_meukf_step_v2(state, sensor, params)` - **this is where the math happens**
4. Unpack C++ output back into MATLAB state variables

### MEX Build System

Build MEX files from MATLAB:
```matlab
cd kalman/cpp/build
build_mex()  % Compiles 7 MEX files to cpp/bin/
```

Output location: [`kalman/cpp/bin/`](kalman/cpp/bin/) (auto-added to path in ESKF constructor)

**Build requirements**: 
- Windows: Visual Studio or MinGW-w64
- Configure once: `mex -setup C++` in MATLAB

**Active MEX files** (see [`cpp/markdown/README.md`](kalman/cpp/markdown/README.md)):
- `mex_meukf_step_v2.mexw64` - Main filter engine
- `mex_kalman_filter_core.mexw64` - Legacy support
- `mex_eskf_math.mexw64` - Math utilities
- `mex_quaternion_lib.mexw64` - Quaternion operations (legacy)

## Key Conventions & Patterns

### State Representation

15-state ESKF vector:
- **Position** (p): [px, py, pz] (3D NED frame)
- **Velocity** (v): [vx, vy, vz]
- **Quaternion** (q): [qw, qx, qy, qz] (normalized, body-to-NED)
- **Accel bias** (ba): [bax, bay, baz]
- **Gyro bias** (bg): [bgx, bgy, bgz]
- **Covariance** (P): 15×15 matrix

**Quaternion convention**: [qw, qx, qy, qz] format (scalar-first). All quaternion operations use [`Common/Math/QuaternionLib.m`](kalman/Common/Math/QuaternionLib.m) or C++ equivalent.

### Coordinate Frames

- **Body frame**: X-forward, Y-right, Z-down (typical aircraft convention)
- **NED (World frame)**: North-East-Down geographic coordinates
- **Rotation**: Body-to-NED via quaternion q

### Sensor Update Frequencies

Defined in ESKF constructor (critical for performance):
- Accel: Every 5 samples (Roll/Pitch correction via MEUKF)
- Mag: Every 25 samples (Yaw correction via MEUKF)
- GPS: Every 40 samples (Position/Velocity via UKF)
- Baro: Every 50 samples (Altitude via EKF)

**Why different frequencies?**: Mimics real sensor rates and reduces computational load.

### MEUKF vs Standard UKF

**MEUKF (Manifold Error-state UKF)**: Custom implementation for quaternion updates
- Used for: Accel/Mag updates (attitude estimation)
- Lives in: [`cpp/MEUKF/meukf_core.cpp`](kalman/cpp/MEUKF/meukf_core.cpp) lines 200-600
- Key difference: Operates on 3D attitude error δθ, not full quaternion (avoids singularities)

**Standard UKF**: For Euclidean state spaces
- Used for: GPS updates (position/velocity)
- See: `update_gps()` in meukf_core.cpp

### Adaptive Noise Estimation

[`NoiseEstimatorLib.m`](kalman/Common/Estimation/NoiseEstimatorLib.m) adjusts measurement covariance R dynamically:
- Tracks innovation statistics (mean, std)
- Applies EMA smoothing (α=0.01)
- Clamps between R_MIN (eps) and R_MAX (1e6)
- **Purpose**: Handle time-varying sensor noise without manual tuning

### Divergence Detection & Recovery

[`DivergenceGuard`](kalman/Common/Core/DivergenceGuard.m) monitors:
- Innovation norm thresholds (sensor-specific)
- Covariance trace growth (P matrix health)
- NaN/Inf checks

Recovery strategy ([`@ESKF/reset.m`](kalman/ESKF/@ESKF/reset.m)):
1. Check trigger conditions (large innovation, |accel| ≈ |g|)
2. Re-initialize attitude from raw accel/mag
3. Reset biases, increase Q temporarily
4. Log reset event (for analysis)

## Development Workflows

### Running Simulations

**Single run** (with data generation):
```matlab
run_simulation()  % Uses default random seed
```

**Batch testing** (10 runs with different seeds):
```matlab
run_batch_10sets()  % Outputs: Results/batch_10sets_summary.csv
```

**Quick test** (skip data generation):
```matlab
run_simulation(42, true)  % seed=42, skip_data_gen=true
```

### Data Generation

[`GenerateData/sim_generate.m`](kalman/GenerateData/sim_generate.m) creates synthetic IMU/GPS/Baro data:
- Motion types: `'circular'`, `'random_walk'`, `'stationary'`
- Configured via [`config_params.m`](kalman/GenerateData/config_params.m)
- Outputs: `sensor_data.csv`, `truth_data.csv`

**Key pattern**: Always regenerate data after changing motion parameters, otherwise filters use stale observations.

### Testing & Verification

Post-simulation analysis:
```matlab
% Results automatically saved to Results/estimation.csv
plot_csv_file('Results/estimation.csv', 'Results/truth_data.csv')
```

**Accuracy metrics** (see [`run_batch_10sets.m`](kalman/run_batch_10sets.m)):
- Position RMSE: Target < 5.0m
- Attitude RMSE: Roll/Pitch < 1.0°, Yaw < 2.0°
- No NaN/Inf in output

### Modifying C++ Code

1. Edit source in [`cpp/MEUKF/meukf_core.cpp`](kalman/cpp/MEUKF/meukf_core.cpp) or [`cpp/mex/mex_meukf_step.cpp`](kalman/cpp/mex/mex_meukf_step.cpp)
2. Rebuild: `cd cpp/build; build_mex()`
3. **Critical**: Restart MATLAB or `clear mex` to reload MEX file
4. Test with `run_simulation()`

**Common mistake**: Forgetting to clear MEX cache → old code still executes.

## Project-Specific Patterns

### MATLAB Class Method Organization

ESKF uses MATLAB's `@ClassName` folder pattern for method files:
- `@ESKF/predict.m` → `obj.predict(a, w)`
- `@ESKF/sensor_updates.m` → `obj.sensor_updates('accel', data)`

**Why**: Keeps methods modular, but requires understanding this is **one class split across files**, not separate classes.

### MEX Interface Pattern

All C++ calls go through struct packing (see `call_cpp_update_impl.m`):
```matlab
state_in = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
                  'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P);
sensor = struct('dt', dt, 'accel', a(:), 'update_accel', 1, ...);
params = struct('g', obj.g(:), 'noise_accel', noise_accel, ...);

[state_out, debug] = mex_meukf_step_v2(state_in, sensor, params);
```

**Rationale**: Simple, explicit interface beats MATLAB's object marshalling (which doesn't work with MEX).

### Documentation Structure

- **Architecture docs**: [`md/ESKF_Complete_Documentation.md`](kalman/md/ESKF_Complete_Documentation.md) (745 lines, most comprehensive)
- **Migration history**: [`md/cpp_migration_complete.md`](kalman/md/cpp_migration_complete.md)
- **Current status**: [`md/MATLAB_Implementation_Status.md`](kalman/md/MATLAB_Implementation_Status.md)

**Pattern**: Markdown docs are treated as first-class design artifacts, updated with code changes.

### File Naming Conventions

- `run_*.m` - Entry point scripts
- `*Lib.m` - Reusable utility libraries (QuaternionLib, SensorFilterLib, NoiseEstimatorLib)
- `mex_*.cpp` - MEX interface wrappers
- `*_core.cpp` - C++ implementation (not directly callable from MATLAB)

## Common Pitfalls

1. **Quaternion normalization**: Always normalize after mathematical operations. C++ code does this automatically; MATLAB code must call `QuaternionLib.normalize(q)`.

2. **Covariance symmetry**: P matrix can lose symmetry due to floating-point errors. ESKF updates enforce `P = (P + P')/2` in C++ code.

3. **Sensor data missing values**: GPS lat/lon may be NaN (signal loss). Always check `~isnan(obs.lat(k))` before calling GPS update.

4. **Static initialization period**: First `static_time` seconds (default 3s) are used for noise calibration. Filter does not run during this period. See ESKF constructor logic.

5. **Path dependencies**: ESKF constructor calls `addpath(genpath('cpp'))` - if MEX files aren't found, check this executed correctly.

## Integration Points

### External Dependencies

- **MATLAB Toolboxes**: None required (all custom implementations)
- **C++ Libraries**: Header-only (no external linking)

### Data I/O

- Input: CSV files (sensor_data.csv) via [`read_csv.m`](kalman/GenerateData/read_csv.m)
- Output: CSV files (estimation.csv) + MAT files (batch_10sets_results.mat)
- **Format**: Time-series tables with columns: time, px, py, pz, vx, vy, vz, roll, pitch, yaw, ...

### Visualization

[`Graph/plot_csv_file.m`](kalman/Graph/plot_csv_file.m) - 2-file comparison (estimation vs truth)
- Subplots: Position, Velocity, Attitude, Biases, Errors
- **Usage**: Called automatically after `run_simulation()`

## When Making Changes

**To add a new sensor type**:
1. Add update method in `meukf_core.cpp` (e.g., `update_lidar()`)
2. Add case in `call_cpp_update_impl.m` to pack sensor data
3. Update MEX interface in `mex_meukf_step.cpp` to parse new sensor struct
4. Rebuild MEX and test

**To tune filter parameters**:
- Edit [`GenerateData/config_params.m`](kalman/GenerateData/config_params.m) for process noise Q
- For measurement noise R, adjust in ESKF constructor or use adaptive estimation

**To debug C++ crashes**:
- Add `mexPrintf()` statements in MEX wrapper
- Check matrix dimensions (fixed-size templates fail silently on mismatches)
- Use `try/catch` around MEX calls in MATLAB to see error messages

## Key Files to Understand First

1. [`run_simulation.m`](kalman/run_simulation.m) - Execution flow
2. [`ESKF/@ESKF/ESKF.m`](kalman/ESKF/@ESKF/ESKF.m) - State initialization
3. [`@ESKF/call_cpp_update_impl.m`](kalman/ESKF/@ESKF/call_cpp_update_impl.m) - MATLAB/C++ boundary
4. [`cpp/MEUKF/meukf_core.cpp`](kalman/cpp/MEUKF/meukf_core.cpp) - Core algorithms
5. [`md/ESKF_Complete_Documentation.md`](kalman/md/ESKF_Complete_Documentation.md) - Full architecture reference
