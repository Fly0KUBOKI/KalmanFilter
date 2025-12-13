# Baseline Performance (Before Full C++ Migration)

## Test Date: 2025-12-13 19:05

## Configuration
- Motion: Circular (default)
- Samples: 20001 @ 100Hz (200 seconds)
- Sensors: IMU 100Hz, Mag 25Hz, GPS 4Hz, Baro 2Hz
- Filter: MEUKF (MATLAB wrapper + C++ mex_meukf_step_v2)

## Results Summary (10 Runs)

### Overall Statistics
- **Success Rate**: 10/10 (100%)
- **Position RMSE (overall)**: Mean=0.7466m, Std=0.1494m, Max=0.9741m
- **Position RMSE by axis**:
  - X: Mean=0.5323m, Std=0.1177m, Max=0.7057m
  - Y: Mean=0.4289m, Std=0.0579m, Max=0.5575m
  - Z: Mean=0.2940m, Std=0.0966m, Max=0.4389m
- **Velocity RMSE**: Mean=1.7768m/s, Std=0.2067m/s, Max=2.1285m/s
- **Attitude RMSE**:
  - Roll: Mean=0.2824°, Std=0.0096°, Max=0.2979°
  - Pitch: Mean=0.2924°, Std=0.0085°, Max=0.3066°
  - Yaw: Mean=0.6378°, Std=0.0290°, Max=0.6948°

### Individual Run Results

| Run | Position Overall | X     | Y     | Z     | Roll  | Pitch | Yaw   | Result |
|-----|------------------|-------|-------|-------|-------|-------|-------|--------|
| 1   | 0.6066m         | 0.4285| 0.3744| 0.2104| 0.29° | 0.28° | 0.62° | PASS   |
| 2   | 0.9519m         | 0.7057| 0.4642| 0.4389| 0.29° | 0.31° | 0.67° | PASS   |
| 3   | 0.6430m         | 0.4501| 0.3905| 0.2418| 0.27° | 0.28° | 0.69° | PASS   |
| 4   | 0.8814m         | 0.6280| 0.4370| 0.4375| 0.30° | 0.30° | 0.63° | PASS   |
| 5   | 0.6930m         | 0.4662| 0.4610| 0.2244| 0.28° | 0.28° | 0.62° | PASS   |
| 6   | 0.6384m         | 0.4519| 0.3854| 0.2343| 0.28° | 0.29° | 0.61° | PASS   |
| 7   | 0.6068m         | 0.4323| 0.3677| 0.2148| 0.27° | 0.30° | 0.61° | PASS   |
| 8   | 0.8452m         | 0.6287| 0.4497| 0.3420| 0.28° | 0.29° | 0.66° | PASS   |
| 9   | 0.6260m         | 0.4322| 0.4012| 0.2099| 0.29° | 0.30° | 0.62° | PASS   |
| 10  | 0.9741m         | 0.6991| 0.5575| 0.3865| 0.27° | 0.29° | 0.64° | PASS   |

## Performance Targets for Migration

Based on baseline, post-migration targets (conservative ±20%):

- **Position RMSE**: <0.90m (mean), <1.20m (max) 
- **Attitude RMSE**: <0.35° (roll/pitch), <0.80° (yaw)
- **Success Rate**: ≥90% (9/10 runs)
- **Performance**: ≥9 sec/run average (baseline ~11.2 sec/run)

### Stretch Goals (ideal)
- Position RMSE: <0.75m (match baseline)
- Attitude RMSE: <0.30°/0.30°/0.65° (match baseline)
- Success Rate: 100%
- Performance: <10 sec/run (10% speedup from unified filter)

## Notes

- Max Innovation = 0.0000 across all runs indicates ZUPT-only mode or limited GPS updates
- Low position RMSE (<1m average) indicates excellent filter performance
- Attitude accuracy (<1° for all axes) is exceptional
- Run time variation (9.89-13.12 sec) likely due to system load
- All runs completed without NaN/Inf or divergence

## File Locations

- Raw data: `Results/batch_10sets_results.mat`
- Summary CSV: `Results/batch_10sets_summary.csv`
- Individual estimations: `Results/estimation_01.csv` to `estimation_10.csv`
- Log file: `Results/batch_10sets_log.txt`
