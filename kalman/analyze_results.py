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

def analyze():
    # パス設定
    base_dir = os.path.dirname(os.path.abspath(__file__))
    truth_file = os.path.join(base_dir, 'GenerateData', 'truth_data.csv')
    est_file = os.path.join(base_dir, 'Results', 'estimation.csv')

    if not os.path.exists(truth_file):
        print(f"Error: {truth_file} not found.")
        return
    if not os.path.exists(est_file):
        print(f"Error: {est_file} not found.")
        return

    # データ読み込み
    truth_df = pd.read_csv(truth_file)
    est_df = pd.read_csv(est_file)

    # 時間合わせ（必要なら）
    # ここではサンプル数が同じで時間が同期していると仮定

    # カラムマッピング
    # truth: time, x, y, z, vx, vy, vz, roll, pitch, yaw
    # est: time, px, py, pz, vx, vy, vz, roll, pitch, yaw, ...
    
    # 位置
    pos_rmse = np.sqrt(np.mean((truth_df['x'] - est_df['px'])**2 + 
                               (truth_df['y'] - est_df['py'])**2 + 
                               (truth_df['z'] - est_df['pz'])**2))
    
    # 速度
    vel_rmse = np.sqrt(np.mean((truth_df['vx'] - est_df['vx'])**2 + 
                               (truth_df['vy'] - est_df['vy'])**2 + 
                               (truth_df['vz'] - est_df['vz'])**2))
    
    # 姿勢 (Yawの不連続性を考慮する必要があるが、ここでは単純差分)
    # wrapToPi 相当の処理が必要
    def angle_diff(a, b):
        d = a - b
        d = (d + 180) % 360 - 180
        return d

    roll_err = angle_diff(truth_df['roll'], est_df['roll'])
    pitch_err = angle_diff(truth_df['pitch'], est_df['pitch'])
    yaw_err = angle_diff(truth_df['yaw'], est_df['yaw'])

    att_rmse = np.sqrt(np.mean(roll_err**2 + pitch_err**2 + yaw_err**2))

    print(f"Position RMSE: {pos_rmse:.4f} m")
    print(f"Velocity RMSE: {vel_rmse:.4f} m/s")
    print(f"Attitude RMSE: {att_rmse:.4f} deg")

    # 個別のRMSE
    print(f"  Roll RMSE: {np.sqrt(np.mean(roll_err**2)):.4f} deg")
    print(f"  Pitch RMSE: {np.sqrt(np.mean(pitch_err**2)):.4f} deg")
    print(f"  Yaw RMSE: {np.sqrt(np.mean(yaw_err**2)):.4f} deg")

    # 発散チェック
    if pos_rmse > 1000 or np.isnan(pos_rmse):
        print("WARNING: Position estimation DIVERGED!")
    if att_rmse > 1000 or np.isnan(att_rmse):
        print("WARNING: Attitude estimation DIVERGED!")

    # グラフ作成
    plt.figure(figsize=(12, 10))
    
    plt.subplot(3, 1, 1)
    plt.plot(truth_df['time'], truth_df['x'], label='Truth X')
    plt.plot(est_df['time'], est_df['px'], label='Est X', linestyle='--')
    plt.title('Position X')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(truth_df['time'], truth_df['roll'], label='Truth Roll')
    plt.plot(est_df['time'], est_df['roll'], label='Est Roll', linestyle='--')
    plt.title('Roll')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(truth_df['time'], truth_df['yaw'], label='Truth Yaw')
    plt.plot(est_df['time'], est_df['yaw'], label='Est Yaw', linestyle='--')
    plt.title('Yaw')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(base_dir, 'Results', 'analysis_plot.png'))
    print("Plot saved to Results/analysis_plot.png")

if __name__ == "__main__":
    analyze()
