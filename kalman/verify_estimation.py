#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ESKF推定結果の検証スクリプト
8d81b13と現在のバージョンで推定が正しく動作していることを確認
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

def angle_diff(a, b):
    """角度差を-180~180度に正規化"""
    d = a - b
    d = np.where(d > 180, d - 360, d)
    d = np.where(d < -180, d + 360, d)
    return d

def analyze_estimation():
    """推定結果の解析"""
    
    # ファイルパス
    truth_file = 'GenerateData/truth_data.csv'
    est_file = 'Results/estimation.csv'
    
    if not os.path.exists(truth_file):
        print(f"Error: {truth_file} not found")
        return False
    if not os.path.exists(est_file):
        print(f"Error: {est_file} not found")
        return False
    
    # データ読み込み
    truth_df = pd.read_csv(truth_file)
    est_df = pd.read_csv(est_file)
    
    print("="*60)
    print("ESKF Estimation Performance Verification")
    print("="*60)
    print(f"Samples: {len(truth_df)}")
    print(f"Duration: {truth_df['time'].iloc[-1]:.2f} seconds")
    print("="*60)
    
    # 位置誤差
    pos_err = np.sqrt((truth_df['x'] - est_df['px'])**2 + 
                      (truth_df['y'] - est_df['py'])**2 + 
                      (truth_df['z'] - est_df['pz'])**2)
    pos_rmse = np.sqrt(np.mean(pos_err**2))
    pos_max = np.max(pos_err)
    
    # 速度誤差
    vel_err = np.sqrt((truth_df['vx'] - est_df['vx'])**2 + 
                      (truth_df['vy'] - est_df['vy'])**2 + 
                      (truth_df['vz'] - est_df['vz'])**2)
    vel_rmse = np.sqrt(np.mean(vel_err**2))
    vel_max = np.max(vel_err)
    
    # 姿勢誤差
    roll_err = angle_diff(truth_df['roll'], est_df['roll'])
    pitch_err = angle_diff(truth_df['pitch'], est_df['pitch'])
    yaw_err = angle_diff(truth_df['yaw'], est_df['yaw'])
    
    roll_rmse = np.sqrt(np.mean(roll_err**2))
    pitch_rmse = np.sqrt(np.mean(pitch_err**2))
    yaw_rmse = np.sqrt(np.mean(yaw_err**2))
    
    roll_max = np.max(np.abs(roll_err))
    pitch_max = np.max(np.abs(pitch_err))
    yaw_max = np.max(np.abs(yaw_err))
    
    # 結果表示
    print("\n[Position Estimation]")
    print(f"  RMSE:     {pos_rmse:8.4f} m")
    print(f"  Max Error: {pos_max:8.4f} m")
    
    print("\n[Velocity Estimation]")
    print(f"  RMSE:      {vel_rmse:8.4f} m/s")
    print(f"  Max Error: {vel_max:8.4f} m/s")
    
    print("\n[Attitude Estimation]")
    print(f"  Roll RMSE:  {roll_rmse:8.4f} deg (Max: {roll_max:.4f} deg)")
    print(f"  Pitch RMSE: {pitch_rmse:8.4f} deg (Max: {pitch_max:.4f} deg)")
    print(f"  Yaw RMSE:   {yaw_rmse:8.4f} deg (Max: {yaw_max:.4f} deg)")
    
    print("\n" + "="*60)
    
    # 判定
    pos_ok = pos_rmse < 10.0 and not np.isnan(pos_rmse)
    vel_ok = vel_rmse < 5.0 and not np.isnan(vel_rmse)
    att_ok = roll_rmse < 5.0 and pitch_rmse < 5.0 and yaw_rmse < 50.0
    
    if pos_ok:
        print("[PASS] Position estimation is STABLE")
    else:
        print("[FAIL] Position estimation DIVERGED or FAILED")
    
    if vel_ok:
        print("[PASS] Velocity estimation is STABLE")
    else:
        print("[FAIL] Velocity estimation DIVERGED or FAILED")
    
    if att_ok:
        print("[PASS] Attitude estimation is STABLE")
    else:
        print("[FAIL] Attitude estimation DIVERGED or FAILED")
    
    print("="*60)
    
    overall_success = pos_ok and vel_ok and att_ok
    
    if overall_success:
        print("\n*** ESTIMATION VERIFICATION: SUCCESS ***\n")
    else:
        print("\n*** ESTIMATION VERIFICATION: FAILED ***\n")
    
    # グラフ作成
    create_plots(truth_df, est_df, pos_err, vel_err, roll_err, pitch_err, yaw_err)
    
    return overall_success

def create_plots(truth_df, est_df, pos_err, vel_err, roll_err, pitch_err, yaw_err):
    """結果プロット作成"""
    
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    
    # 位置
    axes[0, 0].plot(truth_df['time'], truth_df['x'], 'b-', label='Truth', linewidth=2)
    axes[0, 0].plot(est_df['time'], est_df['px'], 'r--', label='Estimated', linewidth=1.5)
    axes[0, 0].set_title('Position X', fontsize=12, fontweight='bold')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('X [m]')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # 位置誤差
    axes[0, 1].plot(truth_df['time'], pos_err, 'r-', linewidth=1.5)
    axes[0, 1].set_title('Position Error', fontsize=12, fontweight='bold')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Error [m]')
    axes[0, 1].grid(True, alpha=0.3)
    
    # 速度
    axes[1, 0].plot(truth_df['time'], truth_df['vx'], 'b-', label='Truth', linewidth=2)
    axes[1, 0].plot(est_df['time'], est_df['vx'], 'r--', label='Estimated', linewidth=1.5)
    axes[1, 0].set_title('Velocity X', fontsize=12, fontweight='bold')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Vx [m/s]')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # 速度誤差
    axes[1, 1].plot(truth_df['time'], vel_err, 'r-', linewidth=1.5)
    axes[1, 1].set_title('Velocity Error', fontsize=12, fontweight='bold')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Error [m/s]')
    axes[1, 1].grid(True, alpha=0.3)
    
    # Yaw
    axes[2, 0].plot(truth_df['time'], truth_df['yaw'], 'b-', label='Truth', linewidth=2)
    axes[2, 0].plot(est_df['time'], est_df['yaw'], 'r--', label='Estimated', linewidth=1.5)
    axes[2, 0].set_title('Yaw Angle', fontsize=12, fontweight='bold')
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 0].set_ylabel('Yaw [deg]')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)
    
    # Yaw誤差
    axes[2, 1].plot(truth_df['time'], yaw_err, 'r-', linewidth=1.5)
    axes[2, 1].set_title('Yaw Error', fontsize=12, fontweight='bold')
    axes[2, 1].set_xlabel('Time [s]')
    axes[2, 1].set_ylabel('Error [deg]')
    axes[2, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = 'Results/verification_plot.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved: {output_path}")

if __name__ == "__main__":
    success = analyze_estimation()
    exit(0 if success else 1)
