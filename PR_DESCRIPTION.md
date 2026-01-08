Summary of changes: 

- Replaced MEUKF accel and mag updates with UKF-backed implementations and added UKF wrappers for GPS, Baro, and ZUPT.
- Implemented `Lib/UKF/inc/ukf_generic.hpp` usage from MEUKF to run full-state UKF updates (15-state, variable-measurement dimension).
- Added diagnostic exports (`pred_P`, `last_K`, `last_S`, `last_y`, `last_sensor_type`, etc.) to MEX outputs for detailed comparison in MATLAB.
- Fixed several compile issues (undefined symbols, indexing typos, type conversions) discovered during integration and ensured proper state/covariance mapping (row-major ↔ column-major handling).
- Added robust PD/symmetrization steps where needed and ensured quaternion small-angle injection is applied consistently after UKF updates.

Phase 4: Math utilities refactor
- Split `Lib/Common/inc/Math/math_utils.hpp` into per-purpose headers: `matrix_operations.hpp`, `statistics.hpp`, `geometry.hpp`, `numerical.hpp`.
- Restored a small `math_utils.hpp` compatibility wrapper providing `PI`/`EPS` and a few helpers, and added `is_outlier_chi_square()` for validation.
- Replaced scattered includes to use focused headers and removed duplicated math helpers across the codebase.

Files touched (examples):
- `Lib/Common/inc/Math/*` (new/edited headers)
- `Lib/Quaternion/quaternion_functions.hpp` (PI usage updated)
- `Lib/ESKF/src/eskf_initializer.cpp` (PI usage updated)
- `Lib/Common/src/Sensor/sensor_preprocessor.cpp` (math_utils include)

Validation & build:
- Rebuilt MEXs successfully (both `mex_run_eskf` and `mex_meukf_step_v2`).
- Ran `run_batch_10sets()` regression: 10/10 PASS. Logs in `kalman/Results/log/`.

Reviewer checklist:
1. Checkout branch `phase4/math-refactor` (or current working branch).
2. In MATLAB run:

```matlab
cd('kalman/cpp/build');
build_mex();
clear mex;
cd('../..');
run_batch_10sets();
```

3. Confirm 10/10 PASS and inspect `Results/estimation_*.csv` for expected metrics.

Notes:
- I recommend running `clang-format` over the edited C++ headers before opening the PR.

Files touched (high level):
- `Lib/MEUKF/src/meukf_core.cpp` (accel/mag/gps/baro/zupt UKF updates, wrappers, bug fixes)
- `Lib/MEUKF/inc/meukf_core.hpp` (declarations for UKF-version wrappers)
- `Lib/UKF/inc/ukf_generic.hpp` (generic UKF implementation used)
- `MEX/mex_meukf_step.cpp` (diagnostic struct export)
- MATLAB harness updates for debug capture and batch tests

Tests performed:
- Successfully built MEX (`mex_run_eskf`, `mex_meukf_step_v2`).
- Ran `run_accel_compare` diagnostics (saved debug mats).
- Ran `run_batch_10sets` three times while developing; latest run: 10/10 PASS (summary saved under `Results/log/`).

Next steps / recommended PR content:
1. Create a focused branch with these changes.
2. Include `PR_DESCRIPTION.md` (this file) and link representative `Results/log/batch_10sets_log_*.txt` outputs.
3. Request reviewer to run `build_mex()` + `run_batch_10sets()` and inspect `accel_update_compare.mat` if diagnostics needed.
4. If approved, proceed to remove legacy MEUKF-only code paths or mark them deprecated.

Suggested `build`+`test` commands for reviewer (MATLAB):

cd('kalman/cpp/build');
build_mex();
clear mex;
cd('../..');
run_accel_compare;    % optional diagnostic comparison
run_batch_10sets;     % full regression (10 seeds)

Notes / Caveats:
- All sensor inputs (except GPS LLA) are `float` internally; ensure MATLAB side constructs `SensorData` correctly.
- After merging, consider adding a small CI job that runs `run_batch_10sets` to catch regressions automatically.

Contact: include me as reviewer for help reproducing numeric comparisons and interpreting `last_S`/`last_K` diagnostics.
