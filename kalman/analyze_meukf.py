import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def calculate_rmse(truth, est, columns):
    rmse = {}
    for col in columns:
        diff = truth[col] - est[col]
        rmse[col] = np.sqrt(np.mean(diff**2))
    return rmse

def detect_drift(truth, est, window_size=1000):
    """
    ドリフトを検知する関数
    window_size: 移動平均のウィンドウサイズ（サンプル数）
    """
    drift_info = {}
    
    # 位置のドリフト (各軸)
    for axis, col_truth, col_est in [('x', 'x', 'px'), ('y', 'y', 'py'), ('z', 'z', 'pz')]:
        diff = truth[col_truth] - est[col_est]
        drift_trend = pd.Series(diff).rolling(window=window_size, center=True).mean()
        drift_info[f'pos_{axis}_drift'] = {
            'max_drift': np.nanmax(np.abs(drift_trend)),
            'final_drift': np.abs(drift_trend.iloc[-1]) if not np.isnan(drift_trend.iloc[-1]) else 0,
            'trend': drift_trend
        }
    
    # 速度のドリフト
    for axis, col in [('x', 'vx'), ('y', 'vy'), ('z', 'vz')]:
        diff = truth[col] - est[col]
        drift_trend = pd.Series(diff).rolling(window=window_size, center=True).mean()
        drift_info[f'vel_{axis}_drift'] = {
            'max_drift': np.nanmax(np.abs(drift_trend)),
            'final_drift': np.abs(drift_trend.iloc[-1]) if not np.isnan(drift_trend.iloc[-1]) else 0,
            'trend': drift_trend
        }
    
    # 姿勢のドリフト
    for axis, col in [('roll', 'roll'), ('pitch', 'pitch'), ('yaw', 'yaw')]:
        diff = angle_diff(truth[col], est[col])
        drift_trend = pd.Series(diff).rolling(window=window_size, center=True).mean()
        drift_info[f'att_{axis}_drift'] = {
            'max_drift': np.nanmax(np.abs(drift_trend)),
            'final_drift': np.abs(drift_trend.iloc[-1]) if not np.isnan(drift_trend.iloc[-1]) else 0,
            'trend': drift_trend
        }
    
    return drift_info

def angle_diff(a, b):
    """角度差を計算（-180～180度に正規化）"""
    d = a - b
    d = (d + 180) % 360 - 180
    return d

def analyze_meukf():
    # パス設定
    base_dir = os.path.dirname(os.path.abspath(__file__))
    truth_file = os.path.join(base_dir, 'GenerateData', 'truth_data.csv')
    est_file = os.path.join(base_dir, 'Results', 'estimation_meukf.csv')

    if not os.path.exists(truth_file):
        print(f"Error: {truth_file} not found.")
        return
    if not os.path.exists(est_file):
        print(f"Error: {est_file} not found.")
        return

    # データ読み込み
    truth_df = pd.read_csv(truth_file)
    est_df = pd.read_csv(est_file)
    
    if len(truth_df) == 0:
        print("Error: truth_data.csv is empty")
        return
    if len(est_df) == 0:
        print("Error: estimation_meukf.csv is empty")
        return

    # 位置
    pos_rmse = np.sqrt(np.mean((truth_df['x'] - est_df['px'])**2 + 
                               (truth_df['y'] - est_df['py'])**2 + 
                               (truth_df['z'] - est_df['pz'])**2))
    
    # 速度
    vel_rmse = np.sqrt(np.mean((truth_df['vx'] - est_df['vx'])**2 + 
                               (truth_df['vy'] - est_df['vy'])**2 + 
                               (truth_df['vz'] - est_df['vz'])**2))
    
    # 姿勢
    roll_err = angle_diff(truth_df['roll'], est_df['roll'])
    pitch_err = angle_diff(truth_df['pitch'], est_df['pitch'])
    yaw_err = angle_diff(truth_df['yaw'], est_df['yaw'])

    att_rmse = np.sqrt(np.mean(roll_err**2 + pitch_err**2 + yaw_err**2))

    print(f"\n{'='*60}")
    print(f"MEUKF Performance Analysis (with Truth Comparison)")
    print(f"{'='*60}")
    print(f"Position RMSE: {pos_rmse:.4f} m")
    print(f"Velocity RMSE: {vel_rmse:.4f} m/s")
    print(f"Attitude RMSE: {att_rmse:.4f} deg")

    # 個別のRMSE
    print(f"\n  Roll RMSE: {np.sqrt(np.mean(roll_err**2)):.4f} deg")
    print(f"  Pitch RMSE: {np.sqrt(np.mean(pitch_err**2)):.4f} deg")
    print(f"  Yaw RMSE: {np.sqrt(np.mean(yaw_err**2)):.4f} deg")
    
    # ドリフト検知
    print(f"\n{'='*60}")
    print("Drift Analysis")
    print(f"{'='*60}")
    drift_info = detect_drift(truth_df, est_df)
    
    print("\nPosition Drift (m):")
    for axis in ['x', 'y', 'z']:
        key = f'pos_{axis}_drift'
        print(f"  {axis.upper()}: Max={drift_info[key]['max_drift']:.4f} m, Final={drift_info[key]['final_drift']:.4f} m")
    
    print("\nVelocity Drift (m/s):")
    for axis in ['x', 'y', 'z']:
        key = f'vel_{axis}_drift'
        print(f"  {axis.upper()}: Max={drift_info[key]['max_drift']:.4f} m/s, Final={drift_info[key]['final_drift']:.4f} m/s")
    
    print("\nAttitude Drift (deg):")
    for axis in ['roll', 'pitch', 'yaw']:
        key = f'att_{axis}_drift'
        print(f"  {axis.upper()}: Max={drift_info[key]['max_drift']:.4f} deg, Final={drift_info[key]['final_drift']:.4f} deg")

    # 発散チェック
    print(f"\n{'='*60}")
    print("Divergence Check")
    print(f"{'='*60}")
    if pos_rmse > 1000 or np.isnan(pos_rmse):
        print("WARNING: Position estimation DIVERGED!")
    else:
        print(f"Position OK (RMSE = {pos_rmse:.2f} m)")
    if att_rmse > 1000 or np.isnan(att_rmse):
        print("WARNING: Attitude estimation DIVERGED!")
    else:
        print(f"Attitude OK (RMSE = {att_rmse:.2f} deg)")
    
    # ドリフト警告
    if drift_info['att_roll_drift']['max_drift'] > 5.0:
        print(f"WARNING: Roll drift detected (max = {drift_info['att_roll_drift']['max_drift']:.2f} deg)")
    if drift_info['att_pitch_drift']['max_drift'] > 5.0:
        print(f"WARNING: Pitch drift detected (max = {drift_info['att_pitch_drift']['max_drift']:.2f} deg)")

    # グラフ作成
    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    
    # 位置 X
    axes[0, 0].plot(truth_df['time'], truth_df['x'], label='Truth X', linewidth=1.5)
    axes[0, 0].plot(est_df['time'], est_df['px'], label='MEUKF Est X', linestyle='--', alpha=0.8)
    axes[0, 0].set_title('Position X (MEUKF)')
    axes[0, 0].set_ylabel('X (m)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 位置誤差 X
    axes[0, 1].plot(truth_df['time'], truth_df['x'] - est_df['px'], label='Error X', color='red')
    axes[0, 1].plot(truth_df['time'], drift_info['pos_x_drift']['trend'], label='Drift Trend', color='black', linewidth=2)
    axes[0, 1].set_title('Position X Error & Drift (MEUKF)')
    axes[0, 1].set_ylabel('Error (m)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Roll
    axes[1, 0].plot(truth_df['time'], truth_df['roll'], label='Truth Roll', linewidth=1.5)
    axes[1, 0].plot(est_df['time'], est_df['roll'], label='MEUKF Est Roll', linestyle='--', alpha=0.8)
    axes[1, 0].set_title('Roll (MEUKF)')
    axes[1, 0].set_ylabel('Roll (deg)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Roll誤差とドリフト
    axes[1, 1].plot(truth_df['time'], roll_err, label='Error Roll', color='red')
    axes[1, 1].plot(truth_df['time'], drift_info['att_roll_drift']['trend'], label='Drift Trend', color='black', linewidth=2)
    axes[1, 1].set_title('Roll Error & Drift (MEUKF)')
    axes[1, 1].set_ylabel('Error (deg)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    # Pitch
    axes[2, 0].plot(truth_df['time'], truth_df['pitch'], label='Truth Pitch', linewidth=1.5)
    axes[2, 0].plot(est_df['time'], est_df['pitch'], label='MEUKF Est Pitch', linestyle='--', alpha=0.8)
    axes[2, 0].set_title('Pitch (MEUKF)')
    axes[2, 0].set_ylabel('Pitch (deg)')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)

    # Pitch誤差とドリフト
    axes[2, 1].plot(truth_df['time'], pitch_err, label='Error Pitch', color='red')
    axes[2, 1].plot(truth_df['time'], drift_info['att_pitch_drift']['trend'], label='Drift Trend', color='black', linewidth=2)
    axes[2, 1].set_title('Pitch Error & Drift (MEUKF)')
    axes[2, 1].set_ylabel('Error (deg)')
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)
    
    # Yaw
    axes[3, 0].plot(truth_df['time'], truth_df['yaw'], label='Truth Yaw', linewidth=1.5)
    axes[3, 0].plot(est_df['time'], est_df['yaw'], label='MEUKF Est Yaw', linestyle='--', alpha=0.8)
    axes[3, 0].set_title('Yaw (MEUKF)')
    axes[3, 0].set_ylabel('Yaw (deg)')
    axes[3, 0].set_xlabel('Time (s)')
    axes[3, 0].legend()
    axes[3, 0].grid(True, alpha=0.3)

    # Yaw誤差とドリフト
    axes[3, 1].plot(truth_df['time'], yaw_err, label='Error Yaw', color='red')
    axes[3, 1].plot(truth_df['time'], drift_info['att_yaw_drift']['trend'], label='Drift Trend', color='black', linewidth=2)
    axes[3, 1].set_title('Yaw Error & Drift (MEUKF)')
    axes[3, 1].set_ylabel('Error (deg)')
    axes[3, 1].set_xlabel('Time (s)')
    axes[3, 1].legend()
    axes[3, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(base_dir, 'Results', 'analysis_plot_meukf.png'), dpi=150)
    print(f"\n{'='*60}")
    print("Plot saved to Results/analysis_plot_meukf.png")
    print(f"{'='*60}\n")

if __name__ == "__main__":
    analyze_meukf()
