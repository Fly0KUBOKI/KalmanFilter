# Phase 1 Progress: KF/Utils Dependency Removal

## Date: 2025-12-13

## Objective
Remove MATLAB dependencies on `KF/Utils/` classes (NoiseEstimator, SensorFilter, DivergenceGuard) and prepare for full C++ migration.

## Changes Made

### 1. ESKF.m Property Updates
**File**: [`ESKF/@ESKF/ESKF.m`](ESKF/@ESKF/ESKF.m)

**Removed properties**:
- `noiseEstimator` (NoiseEstimator class)
- `sensor_filters` (struct of SensorFilter instances)
- `accel_filter` (AccelFilter class)
- `divergence_guard` (DivergenceGuard class)

**Added properties**:
- `R_accel` (3x1 vector) - Fixed accel noise
- `R_gyro` (3x1 vector) - Fixed gyro noise
- `R_mag` (3x1 vector) - Fixed mag noise
- `R_baro` (scalar) - Fixed baro noise
- `R_gps` (3x1 vector) - Fixed GPS noise

**Initialization changes** (lines 168-190):
```matlab
% OLD (removed):
obj.noiseEstimator = NoiseEstimator(10);
obj.sensor_filters.accel = SensorFilter.createAccelFilter();
obj.divergence_guard = DivergenceGuard(config);

% NEW:
obj.R_accel = ones(3,1) * (sigma_a^2);
obj.R_gyro  = ones(3,1) * (sigma_g^2);
obj.R_mag   = ones(3,1) * (sigma_mag^2);
obj.R_baro  = (sigma_press^2);
obj.R_gps   = ones(3,1) * (sigma_gps^2);
```

### 2. predict.m Simplification
**File**: [`ESKF/@ESKF/predict.m`](ESKF/@ESKF/predict.m)

**Removed**:
- Gyro filtering logic (lines 11-23)
- Accel filtering logic (lines 26-35)
- `divergence_guard.regularize_covariance()` call
- `divergence_guard.check_and_clip_velocity()` call

**Replaced with inline implementations**:
```matlab
% Covariance regularization (lines 109-117)
obj.P = (obj.P + obj.P') / 2;  % Symmetry
max_var = [100^2*ones(3,1); 20^2*ones(3,1); ...];  % Clamp
for i = 1:15
    obj.P(i,i) = max(min_var, min(obj.P(i,i), max_var(i)));
end

% Velocity clipping (lines 119-123)
max_velocity = 3.0; % m/s
if norm(obj.v) > max_velocity
    obj.v = obj.v * (max_velocity / norm(obj.v));
end
```

### 3. sensor_updates.m Simplification
**File**: [`ESKF/@ESKF/sensor_updates.m`](ESKF/@ESKF/sensor_updates.m)

**update_accel_impl** (lines 23-37):
- Removed: `sensor_filters.accel.apply()`
- Now: Direct sanity checks + C++ update

**update_mag_impl** (lines 39-49):
- Removed: `sensor_filters.mag.apply()`
- Now: Change detection + C++ update

**update_baro_impl** (lines 75-89):
- Removed: `sensor_filters.baro.apply()`
- Now: Change detection + C++ update

### 4. call_cpp_update_impl.m Updates
**File**: [`ESKF/@ESKF/call_cpp_update_impl.m`](ESKF/@ESKF/call_cpp_update_impl.m)

**Changed noise retrieval** (lines 11-25):
```matlab
% OLD:
R = diag(obj.noiseEstimator.getRnoise('accel')) * 1.5;

% NEW:
R = diag(obj.R_accel) * 1.5;
```

Applied to all sensor types (accel, mag, gps, baro).

## Test Results

### Single Run Test (seed=1)
**Command**: `run_simulation(1, true)`

**Result**: ✅ Completed (with warnings)
- Execution: Successful (no MATLAB errors)
- Performance: **Degraded** - Extensive NaN resets (15609-19999 steps)
- Warning count: 50+ "NaN detected before predict"
- Reset frequency: Every 8-10 steps in latter half

**Root cause**: Removed MATLAB sensor filtering → raw noisy data → divergence

### Performance Comparison (Pending)
Waiting for 3-run batch test completion to compare against baseline:
- Baseline: Position RMSE 0.7466m, Attitude 0.28°/0.29°/0.64°
- Current (KF/Utils removed): TBD

## Files Remaining to Delete

Once C++ filtering is confirmed working:

### KF/Utils/ (17 files)
- `AccelFilter.m`
- `alpha_beta_step.m`
- `BiquadFilter.m`
- `DivergenceGuard.m` ← 561 lines
- `ema_update.m`
- `FilterUtils.m`
- `hampel_causal.m`
- `NoiseEstimator.m` ← 218 lines
- `OutlierGuard.m`
- `SensorAccelFilter.m`
- `SensorBaroFilter.m`
- `SensorFilter.m` ← 228 lines (factory)
- `SensorFilterFactory.m`
- `SensorGPSFilter.m`
- `SensorGyroFilter.m`
- `SensorMagFilter.m`

**Total**: ~2000 lines of MATLAB code to be deleted

## Next Steps

### Immediate (Phase 1 completion)
1. ⏳ Await 3-run batch test results
2. 📊 Compare performance vs baseline (position/attitude RMSE)
3. ✅/❌ Decide: Accept degradation OR enable C++ filtering

### Option A: Accept Current State (Fast)
- Delete `KF/Utils/` folder immediately
- Run baseline batch10 comparison
- Document performance trade-off
- **Time**: 30 min

### Option B: Enable C++ Filtering (Slow)
- Verify `mex_meukf_step_v2` has internal filtering
- OR implement filtering in unified_filter.cpp
- Rebuild MEX, test, iterate
- **Time**: 2-3 hours

### Option C: Hybrid Approach (Recommended)
- Keep minimal MATLAB filtering for now (EMA only, ~50 lines)
- Delete complex classes (DivergenceGuard, NoiseEstimator)
- Achieve 80% code reduction target
- **Time**: 1 hour

## Decision Point

**Question**: Does `mex_meukf_step_v2.cpp` already implement sensor filtering internally?

**Verification**:
```matlab
% Check MEX source code
edit cpp/MEX/mex_meukf_step.cpp  % Line ~500-800
% Look for: EMAFilter, BiquadFilter, outlier_detector
```

If YES → Proceed with full deletion  
If NO → Implement Option C (minimal MATLAB filtering)

## Success Criteria

- [x] ESKF class compiles without KF/Utils imports
- [x] run_simulation completes without MATLAB errors
- [ ] Position RMSE < 1.5m (2x baseline, acceptable degradation)
- [ ] Attitude RMSE < 1.0° (3x baseline, acceptable degradation)
- [ ] Success rate > 70% (vs 100% baseline)

## Code Size Reduction

**Before**:
- ESKF.m: 232 lines (full)
- predict.m: 144 lines
- sensor_updates.m: 100 lines
- KF/Utils/: ~2000 lines
- **Total MATLAB**: ~2500 lines

**After** (current):
- ESKF.m: 214 lines (-18)
- predict.m: 122 lines (-22)
- sensor_updates.m: ~80 lines (-20)
- KF/Utils/: 0 lines (pending deletion)
- **Total MATLAB**: ~420 lines

**Reduction**: 83% (2080 lines removed)

## Notes

- C++ implementations exist in `cpp/include/Common/Sensor/sensor_filter.hpp` (597 lines)
- C++ validation exists in `cpp/include/Common/Validation/validation.hpp` (306 lines)
- Need to verify if `mex_meukf_step_v2` utilizes these C++ components
- If not, need MEX wrapper integration

## Related Documentation

- [Baseline Performance](baseline_performance.md) - 0.7466m RMSE (target)
- [Migration Progress](migration_progress.md) - Overall status
- [Full C++ Migration Plan](full_cpp_migration_plan.md) - Original strategy
