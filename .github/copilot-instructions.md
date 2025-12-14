# Kalman Filter Implementation - AI Coding Agent Guide

## Architecture Overview

Hybrid MATLAB + C++ MEX system for ESKF-based IMU/GPS/Mag/Baro sensor fusion.

- **MATLAB**: Orchestration, state management, data I/O (`run_simulation.m`, `@ESKF/`)
- **C++ MEX**: Core math - MEUKF updates, quaternion ops, UKF sigma points (`cpp/MEUKF/`, `cpp/MEX/`)
- **State**: 15-dim `[p(3), v(3), q(4), ba(3), bg(3)]` with 15×15 covariance P
- **Quaternion**: `[qw, qx, qy, qz]` scalar-first, body-to-NED frame

## Execution Flow

```
run_simulation.m → sim_generate.m → ESKF.m(init) → Main loop:
  predict(a,w) → sensor_updates('accel'|'mag'|'gps'|'baro') → reset('check')
```

All sensor updates go through `mex_meukf_step_v2` (C++ core).

## Key Workflows

### Build MEX
```matlab
cd kalman/cpp/build
build_mex()  % → cpp/bin/*.mexw64
```
**Critical**: Run `clear mex` after rebuild to reload binaries.

### Run Simulation
```matlab
run_simulation()           % Full run with data generation
run_simulation(42, true)   % Seed=42, skip data generation
run_batch_10sets()         % Batch testing (10 seeds)
```

### Sensor Update Frequencies
Controlled via modulo in main loop (not data rate):
- Accel: every 5 samples | Mag: every 25 | GPS: every 40 | Baro: every 50

## Project Patterns

### MATLAB `@ClassName` Folder
ESKF is one class split across files: `@ESKF/ESKF.m`, `@ESKF/predict.m`, `@ESKF/sensor_updates.m`, etc.

### MEX Interface (struct-based)
```matlab
state_in = struct('p', obj.p, 'v', obj.v, 'q', obj.q, 'ba', obj.ba, 'bg', obj.bg, 'P', obj.P);
sensor = struct('dt', dt, 'accel', a, ...);
[state_out, debug] = mex_meukf_step_v2(state_in, sensor, params);
```

### Change Detection
MATLAB buffers (`prev_accel`, `prev_mag`, etc.) skip updates when data unchanged.

## Key Files

| File | Purpose |
|------|---------|
| `kalman/run_simulation.m` | Entry point |
| `kalman/ESKF/@ESKF/ESKF.m` | State initialization from static period |
| `kalman/ESKF/@ESKF/sensor_updates.m` | Sensor dispatch + change detection |
| `kalman/cpp/MEUKF/meukf_core.cpp` | Core MEUKF algorithms (~1100 lines) |
| `kalman/cpp/MEX/mex_meukf_step.cpp` | MEX wrapper |
| `kalman/GenerateData/config_params.m` | Simulation parameters |

## Common Pitfalls

1. **MEX cache**: Always `clear mex` after C++ rebuild
2. **Quaternion normalization**: Enforce after operations (C++ does this automatically)
3. **Covariance symmetry**: Use `P = (P + P')/2` after updates
4. **Config regeneration**: Re-run `sim_generate()` after changing `config_params.m`

## Testing & Accuracy

- Output: `Results/estimation.csv` (compare with `GenerateData/truth_data.csv`)
- Targets: Position RMSE < 5m, Attitude < 1-2°
- Visualization: `plot_csv_file('Results/estimation.csv', 'GenerateData/truth_data.csv')`
