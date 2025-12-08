#!/usr/bin/env python3
import pandas as pd
import numpy as np

print("Loading results...")
est = pd.read_csv('Results/estimation.csv')
truth = pd.read_csv('GenerateData/truth_data.csv')

print(f"\n=== Basic Info ===")
print(f"Total samples: {len(est)}")
print(f"Time range: {est['time'].iloc[0]:.2f} - {est['time'].iloc[-1]:.2f} seconds")

# 初期化期間後のデータ
init_samples = 2000
idx = slice(init_samples, None)

# 位置誤差
pos_err = np.sqrt(
    (est['px'].iloc[idx] - truth['x'].iloc[idx])**2 +
    (est['py'].iloc[idx] - truth['y'].iloc[idx])**2 +
    (est['pz'].iloc[idx] - truth['z'].iloc[idx])**2
)

# 速度誤差
vel_err = np.sqrt(
    (est['vx'].iloc[idx] - truth['vx'].iloc[idx])**2 +
    (est['vy'].iloc[idx] - truth['vy'].iloc[idx])**2 +
    (est['vz'].iloc[idx] - truth['vz'].iloc[idx])**2
)

# 姿勢誤差
def wrap_to_pi(angles):
    return (angles + np.pi) % (2*np.pi) - np.pi

roll_err = np.rad2deg(wrap_to_pi(np.deg2rad(est['roll'].iloc[idx] - truth['roll'].iloc[idx])))
pitch_err = np.rad2deg(wrap_to_pi(np.deg2rad(est['pitch'].iloc[idx] - truth['pitch'].iloc[idx])))
yaw_err = np.rad2deg(wrap_to_pi(np.deg2rad(est['yaw'].iloc[idx] - truth['yaw'].iloc[idx])))

print(f"\n=== Estimation Errors (after init) ===")
print(f"Position RMSE: {np.sqrt(np.mean(pos_err**2)):.4f} m")
print(f"Velocity RMSE: {np.sqrt(np.mean(vel_err**2)):.4f} m/s")
print(f"Roll RMSE: {np.sqrt(np.mean(roll_err**2)):.4f} deg")
print(f"Pitch RMSE: {np.sqrt(np.mean(pitch_err**2)):.4f} deg")
print(f"Yaw RMSE: {np.sqrt(np.mean(yaw_err**2)):.4f} deg")

print(f"\nPosition Max Error: {np.max(pos_err):.4f} m")
print(f"Velocity Max Error: {np.max(vel_err):.4f} m/s")
print(f"Roll Max Error: {np.max(np.abs(roll_err)):.4f} deg")
print(f"Pitch Max Error: {np.max(np.abs(pitch_err)):.4f} deg")
print(f"Yaw Max Error: {np.max(np.abs(yaw_err)):.4f} deg")

# NaN/Infチェック
print(f"\n=== Divergence Check ===")
has_nan = (est[['px', 'py', 'pz', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw']].isna().any().any())
has_inf = (np.isinf(est[['px', 'py', 'pz', 'vx', 'vy', 'vz']]).any().any())

if has_nan:
    print("WARNING: NaN detected!")
else:
    print("No NaN detected")

if has_inf:
    print("WARNING: Inf detected!")
else:
    print("No Inf detected")

# バイアス確認
print(f"\n=== Bias Estimation ===")
print(f"Accel bias (final): [{est['ba_x'].iloc[-1]:.6f}, {est['ba_y'].iloc[-1]:.6f}, {est['ba_z'].iloc[-1]:.6f}] m/s^2")
print(f"Gyro bias (final): [{np.rad2deg(est['bg_x'].iloc[-1]):.6f}, {np.rad2deg(est['bg_y'].iloc[-1]):.6f}, {np.rad2deg(est['bg_z'].iloc[-1]):.6f}] deg/s")

# 判定
print(f"\n=== Overall Assessment ===")
pos_rmse = np.sqrt(np.mean(pos_err**2))
roll_rmse = np.sqrt(np.mean(roll_err**2))
pitch_rmse = np.sqrt(np.mean(pitch_err**2))

pass_test = True
if pos_rmse > 5.0:
    print(f"FAIL: Position RMSE too high ({pos_rmse:.4f} > 5.0 m)")
    pass_test = False
if roll_rmse > 5.0 or pitch_rmse > 5.0:
    print(f"FAIL: Attitude RMSE too high (Roll: {roll_rmse:.4f}, Pitch: {pitch_rmse:.4f})")
    pass_test = False
if has_nan or has_inf:
    print("FAIL: NaN or Inf detected")
    pass_test = False

if pass_test:
    print("✓ PASS: All checks passed!")
else:
    print("✗ FAIL: Issues detected")
