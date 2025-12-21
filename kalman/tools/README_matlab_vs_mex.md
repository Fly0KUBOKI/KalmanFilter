Run MATLAB vs MEX Comparison

This helper shows how to compare filter behavior when using the C++ MEX core vs forcing MATLAB fallbacks for sensor filters/divergence handling.

Usage (MATLAB):

cd kalman
% Run both modes with seed 42
run_compare_matlab_vs_mex(42)

What it does:
- Runs `run_simulation(seed,false)` twice.
  - First run: default (MEX) execution.
  - Second run: sets `FORCE_MATLAB_FILTERS=1` so `SensorFilters`, `DivergenceGuard`, etc. use MATLAB fallbacks.
- Saves outputs into `kalman/Results/` as:
  - `estimation_mex_seed_<seed>.csv`
  - `estimation_matlab_filters_seed_<seed>.csv`

Notes:
- This assumes `run_simulation` writes its estimation CSV to `kalman/Results/estimation_mex.csv` (the repo's convention). If your run_simulation uses different filenames, update the movefile calls in `run_compare_matlab_vs_mex.m` accordingly.
- You need MATLAB to execute these scripts; the script only toggles behaviour via environment variable — no source edits required.
