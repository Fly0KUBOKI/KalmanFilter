# Kalman Filter Implementation - AI Coding Agent Guide

## Project Overview

This is a **full C++ implementation** of multiple Kalman filter variants (KF, EKF, ESKF, UKF, MEUKF) for sensor fusion and state estimation, with MATLAB serving as a thin orchestration layer.

**Architecture Philosophy (Full C++ Migration)**: 
- **C++ Core**: All filter logic (predict, update, sensor filtering, noise estimation, divergence detection) executed in C++ MEX
- **MATLAB Layer**: Minimal - data I/O, simulation setup, visualization only
- **Unified Interface**: Single `mex_unified_filter()` function with input/output structs
- **Automatic Update Control**: C++ detects sensor data changes internally - MATLAB sends duplicate data at different frequencies

## Critical Architecture Knowledge

### Full C++ Architecture (Target Design)

**C++ Core (Target: 95% of logic)**:
- **Unified filter interface**: [`cpp/MEUKF/unified_filter.cpp`](kalman/cpp/MEUKF/unified_filter.cpp) - single entry point
- **Main MEX wrapper**: [`cpp/MEX/mex_unified_filter.cpp`](kalman/cpp/MEX/mex_unified_filter.cpp) - handles all sensor updates
- **Core algorithms**: [`cpp/MEUKF/meukf_core.cpp`](kalman/cpp/MEUKF/meukf_core.cpp) (~1100 lines) - predict/update math
- **Sensor filtering**: [`cpp/include/Common/Sensor/sensor_filter.hpp`](kalman/cpp/include/Common/Sensor/sensor_filter.hpp) - EMA, Biquad, outlier detection
- **Noise estimation**: `cpp/include/Common/Estimation/noise_estimator.hpp` - adaptive R matrix
- **Divergence detection**: `cpp/include/Common/Validation/divergence_guard.hpp` - innovation monitoring
- **Change detection**: Internal buffers track previous sensor values - only updates when data changes

**MATLAB Thin Layer (Target: 5% orchestration only)**:
- [`run_simulation.m`](kalman/run_simulation.m): Entry point - loads data, calls C++, saves results
- [`ESKF/@ESKF/ESKF.m`](kalman/ESKF/@ESKF/ESKF.m): Minimal wrapper - one-time initialization, state storage
- [`@ESKF/call_unified_filter.m`](kalman/ESKF/@ESKF/call_unified_filter.m): **Single C++ call per timestep** - packs input struct, unpacks output struct
- [`GenerateData/sim_gene (Full C++ Target)

```
run_simulation.m (entry point)
  └─> ESKF.m constructor (reads config, initializes state)
      └─> Main loop (run_filter)
          └─> ONE CALL PER TIMESTEP: mex_unified_filter(input_struct) → output_struct
              ├─ C++ internals (all in one call):
              │   ├─ Change detection (compare with prev sensor values)
              │   ├─ Predict step (always runs)
              │   ├─ Sensor filtering (EMA/Biquad/outlier detection)
              │   ├─ Update accel (if data changed)
              │   ├─ Update mag (if data changed)
              │   ├─ Update GPS (if data changed)
              │   ├─ Update baro (if data changed)
              │   ├─ ZUPT check & update
              │   ├─ Divergence detection & reset
              │   └─ Adaptive noise estimation
              └─ MATLAB: Unpack output_struct → save results
```

**Critical Pattern - Unified Interface**:
```matlab
% MATLAB sends ALL sensor data every timestep
input.accel = [ax; ay; az];        % Accel sensor (100 Hz in data)
input.gyro = [wx; wy; wz];         % Gyro sensor (100 Hz)
input.mag = [mx; my; mz];          % Mag sensor (25 Hz - same value 4x)
input.gps_pos = [px; py; pz];      % GPS (4 Hz - same value 25x)
input.baro_alt = alt;              % Baro (2 Hz - same value 50x)
input.dt = 0.01;                   % Timestep

% C++ detects which sensors changed internally
output = mex_unified_filter(prev_state, input, params);

% MATLAB just unpacks and stores
eskf.p = output.position;
eskf.v = output.velocity;
eskf.q = output.quaternion;
```

**Key Innovation**: Sensor update frequency controlled by **data duplication in MATLAB**, not by skip logic. C++ auto-detects changes via tolerance comparison (`norm(new - prev) > 1e-9`).etection
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
- `mex_meukf_step_v2.mexw64` - Main filter engine (built from [`cpp/MEX/mex_meukf_step.cpp`](kalman/cpp/MEX/mex_meukf_step.cpp))
- `mex_kalman_filter_core.mexw64` - Legacy support
- `mex_eskf_math.mexw64` - Math utilities
- `mex_quaternion_lib.mexw64` - Quaternion operations
- 13 additional MEX files for (Data Duplication Strategy)

**MATLAB data generation** ([`sim_generate.m`](kalman/GenerateData/sim_generate.m)):
- Base rate: 100 Hz (IMU accel/gyro)
- Mag: 25 Hz → duplicate each value 4 times (indices: 1,1,1,1, 2,2,2,2, ...)
- GPS: 4 Hz → duplicate each value 25 times 
- Baro: 2 Hz → duplicate each value 50 times

**C++ change detection** (automatic):
```cpp
// In unified_filter.cpp
if (norm(input.mag - prev_mag) > tolerance) {
    update_mag(input.mag);  // Only runs when data changes
    prev_mag = input.mag;
}
```

**Why this approach?**:
- Simpler MATLAB code (no skip logic, just send all data)
- C++ controls all logic (sensor filtering, outlier detection, update decisions)
- Matches real sensor behavior (high-rate sensors always stream, low-rate sensors hold values)
- Eliminates MATLAB/C++ synchronization issues
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
.m`](kalman/KF/Utils/NoiseEstimator.m) adjusts measurement covariance R dynamically:
- Tracks innovation statistics (mean, std)
- Applies EMA smoothing (α=0.01)
- Clamps between R_MIN (eps) and R_MAX (1e6)
- **Purpose**: Handle time-varying sensor noise without manual tuning

### Divergence Detection & Recovery

[`DivergenceGuard.m`](kalman/KF/Utils
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

### Building the Unified C++ Filter

**Build command** (from MATLAB):
```matlab
cd kalman/cpp/build
build_mex()  % Compiles mex_unified_filter.mexw64 + utilities
```

**Active MEX files** (post full C++ migration):
- `mex_unified_filter.mexw64` - **Primary interface** (all filter operations)
- `mex_quaternion_lib.mexw64` - Utility for MATLAB-side quaternion conversions
- Legacy MEX files retained for backward compatibility

### Running Simulations

**Single run**:
```matlab
run_simulation()  % Generates data with frequency duplication, calls C++ once per timestep
```

**Batch testing** (10 runs with different seeds):
```matlab
run_batch_10sets()  % Outputs: Results/batch_10sets_summary.csv
```

**Quick test** (skip data generation):
```matlab
run_simulation(42, true)  % seed=42, skip_data_gen=true
```

### Data Generation with Frequency Control

[`GenerateData/sim_generate.m`](kalman/GenerateData/sim_generate.m):
- Generates base IMU data at 100 Hz
- **Duplicates low-rate sensor data**: Mag (4x), GPS (25x), Baro (50x)
- Outputs: `sensor_data.csv` with all columns at 100 Hz (duplicated values for low-rate sensors)
- Motion types: `'circular'`, `'random_walk'`, `'stationary'` (set in [`config_params.m`](kalman/GenerateData/config_params.m))

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

- `run_*.m` - Entry point scripts (e.g., [`run_simulation.m`](kalman/run_simulation.m), [`run_batch_10sets.m`](kalman/run_batch_10sets.m))
- `*Lib.m` - Reusable utility libraries (e.g., [`QuaternionLib.m`](kalman/Common/Math/QuaternionLib.m))
- `@ClassName/` - MATLAB class methods organized in folders (e.g., [`@ESKF/`](kalman/ESKF/@ESKF/))
- `mex_*.cpp` - MEX interface wrappers in [`cpp/MEX/`](kalman/cpp/MEX/)
- `*_core.cpp` - C++ implementation (not directly callable from MATLAB)

## Common Pitfalls

1. **Quaternion normalization**: Always normalize after operations. C++ unified filter does this automatically in every update cycle.

2. **Covariance symmetry**: P matrix symmetry enforced in C++ via `P = (P + P')/2` after each update.

3. **Sensor data duplication**: **Critical for full C++ design** - Low-rate sensors must duplicate values in MATLAB data generation. If GPS is 4 Hz, each GPS measurement appears 25 times consecutively in the 100 Hz data stream. C++ relies on change detection to determine updates.

4. **Change detection tolerance**: Set in C++ (`tolerance = 1e-9`). If sensors have natural noise smaller than this, C++ may skip legitimate updates. Adjust tolerance in `unified_filter.cpp` if needed.

5. **MEX cache**: After rebuilding, **always run `clear mex`** in MATLAB before `run_simulation()`. Old binaries stay in memory otherwise.

6. **Input struct completeness**: C++ expects all sensor fields in input struct every timestep. Missing fields cause MEX errors. Even if sensor is not active, send zero/NaN with proper dimensions.

7. **Stateless C++ design**: Unlike old MATLAB ESKF class, C++ unified filter is stateless - must pass full state on every call. MATLAB maintains state between calls in `ESKF.m` properties.

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

### Adding a New Sensor Type

**Full C++ approach** (all logic in C++):

1. **Add C++ update function** in [`cpp/MEUKF/meukf_core.cpp`](kalman/cpp/MEUKF/meukf_core.cpp):
```cpp
void update_lidar(const Vec3& z_lidar, const MEUKFState& state, ...) {
    // Measurement model, Kalman gain, state update
}
```

2. **Add to unified filter** in [`cpp/MEUKF/unified_filter.cpp`](kalman/cpp/MEUKF/unified_filter.cpp):
```cpp
// Change detection
if (norm(input.lidar_range - prev_lidar) > tolerance) {
    update_lidar(input.lidar_range, state, params);
    prev_lidar = input.lidar_range;
}
```

3. **Extend input struct** in [`cpp/MEX/mex_unified_filter.cpp`](kalman/cpp/MEX/mex_unified_filter.cpp):
```cpp
// Parse input struct
mxArray* lidar_field = mxGetField(input_struct, 0, "lidar_range");
Vec3 lidar_range = get_vec3_from (Full C++ Architecture)

1. [`cpp/MEX/mex_unified_filter.cpp`](kalman/cpp/MEX/mex_unified_filter.cpp) - **Primary entry point** - MEX interface for unified filter
2. [`cpp/MEUKF/unified_filter.cpp`](kalman/cpp/MEUKF/unified_filter.cpp) - Orchestrates predict + all sensor updates in one call
3. [`cpp/MEUKF/meukf_core.cpp`](kalman/cpp/MEUKF/meukf_core.cpp) - Core MEUKF algorithms (predict, update math)
4. [`GenerateData/sim_generate.m`](kalman/GenerateData/sim_generate.m) - Data generation with frequency duplication
5. [`ESKF/@ESKF/call_unified_filter.m`](kalman/ESKF/@ESKF/call_unified_filter.m) - MATLAB wrapper (minimal logic)
6. [`run_simulation.m`](kalman/run_simulation.m) - Main execution loop
7. [`cpp/include/Common/`](kalman/cpp/include/Common/) - Sensor filtering, noise estimation, divergence detection (all C++)

## Migration Status

**Phase 1: Infrastructure (IN PROGRESS)**

✅ Completed:
- Unified filter types定義 (`cpp/include/MEUKF/unified_types.hpp`)
- 基本的な統一フィルタクラス骨格 (`cpp/MEUKF/unified_filter.cpp`)
- 予測ステップの実装
- 変更検知ロジック
- ZUPT実装

🚧 In Progress:
- MEX wrapper for unified filter (`cpp/MEX/mex_unified_filter.cpp`)
- センサー更新関数の完全実装
- Build script updates

**Phase 2-5: Pending**
- Sensor filtering migration
- Noise estimation migration  
- Divergence detection migration
- MATLAB wrapper simplification
- Full integration testing

**Current Active Files**:
- ✅ `cpp/include/MEUKF/unified_types.hpp` - FilterInput/Output/State structures
- 🚧 `cpp/MEUKF/unified_filter.cpp` - Main filter logic (partial)
- ❌ `cpp/MEX/mex_unified_filter.cpp` - Not yet created
- ❌ MATLAB wrappers - Not yet simplified

**MATLAB Files to be Deprecated** (after full migration):
- `ESKF/@ESKF/predict.m` → Moved to `unified_filter.cpp::predict_step()`
- `ESKF/@ESKF/sensor_updates.m` → Moved to `unified_filter.cpp::update_*()`
- `ESKF/@ESKF/zupt.m` → Moved to `unified_filter.cpp::check_zupt()/update_zupt()`
- `ESKF/@ESKF/call_cpp_update_impl.m` → Replaced by `call_unified_filter.m`
- `KF/Utils/*.m` → Moved to `cpp/include/Common/`

**Retained Files** (thin wrappers):
- ✅ `Common/Math/QuaternionLib.m` - C++ MEX wrapper (kept)
- ✅ `run_simulation.m` - Entry point (minimal changes)
- ✅ `GenerateData/sim_generate.m` - Data generation with frequency control
obs.lidar_range = repelem(lidar_10hz, 10);  % Duplicate to 100 Hz
```

5. **Rebuild and test**:
```matlab
cd kalman/cpp/build; build_mex()
clear mex  % Force reload
run_simulation()
```

### Tuning Filter Parameters

**All tuning now in C++ headers**:
- Process noise Q: Edit `cpp/include/MEUKF/meukf_types.hpp`
- Measurement noise R: Edit `cpp/include/Common/Estimation/noise_estimator.hpp`
- Adaptive parameters: Edit `cpp/MEUKF/meukf_core.cpp` (adaptive Q logic)

**No MATLAB config files** (except motion parameters in [`config_params.m`](kalman/GenerateData/config_params.m) for data generation)

**To debug C++ crashes**:
- Add `mexPrintf()` statements in MEX wrapper (e.g., in [`cpp/MEX/mex_meukf_step.cpp`](kalman/cpp/MEX/mex_meukf_step.cpp))
- Check matrix dimensions (fixed-size templates fail silently on mismatches)
- Use `try/catch` around MEX calls in MATLAB to see error messages
- Check MEX build output: warnings often indicate issues
- Verify `mex -setup C++` is configured correctly for your compiler

**To analyze filter performance**:
- Use [`run_batch_10sets.m`](kalman/run_batch_10sets.m) for statistical analysis across multiple random seeds
- Results saved to `Results/batch_10sets_summary.csv` with RMSE metrics
- Target accuracy: Position RMSE < 5.0m, Attitude < 1-2°
- Check detailed logs in [`md/`](kalman/md/) directory (design artifacts)

## Key Files to Understand First

1. [`run_simulation.m`](kalman/run_simulation.m) - Execution flow
2. [`ESKF/@ESKF/ESKF.m`](kalman/ESKF/@ESKF/ESKF.m) - State initialization
3. [`@ESKF/call_cpp_update_impl.m`](kalman/ESKF/@ESKF/call_cpp_update_impl.m) - MATLAB/C++ boundary
4. [`cpp/MEUKF/meukf_core.cpp`](kalman/cpp/MEUKF/meukf_core.cpp) - Core algorithms
5. [`md/ESKF_Complete_Documentation.md`](kalman/md/ESKF_Complete_Documentation.md) - Full architecture reference
