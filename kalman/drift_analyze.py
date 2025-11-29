#!/usr/bin/env python3
import csv
import math

def read_csv(filepath):
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        return list(reader)

def angle_diff(a, b):
    """角度差を計算（-180～180度に正規化）"""
    d = a - b
    d = (d + 180) % 360 - 180
    return d

def moving_average(data, window=1000):
    """移動平均を計算"""
    if len(data) < window:
        return [sum(data) / len(data)] * len(data)
    
    result = []
    for i in range(len(data)):
        start = max(0, i - window // 2)
        end = min(len(data), i + window // 2)
        result.append(sum(data[start:end]) / (end - start))
    return result

def detect_drift(truth_data, est_data):
    """ドリフトを検知"""
    n = min(len(truth_data), len(est_data))
    
    # Roll誤差とドリフト
    roll_err = []
    for i in range(n):
        err = angle_diff(float(truth_data[i]['roll']), float(est_data[i]['roll']))
        roll_err.append(err)
    roll_drift = moving_average(roll_err, window=1000)
    
    # Pitch誤差とドリフト
    pitch_err = []
    for i in range(n):
        err = angle_diff(float(truth_data[i]['pitch']), float(est_data[i]['pitch']))
        pitch_err.append(err)
    pitch_drift = moving_average(pitch_err, window=1000)
    
    # Yaw誤差とドリフト
    yaw_err = []
    for i in range(n):
        err = angle_diff(float(truth_data[i]['yaw']), float(est_data[i]['yaw']))
        yaw_err.append(err)
    yaw_drift = moving_average(yaw_err, window=1000)
    
    # 位置誤差
    pos_x_err = []
    pos_y_err = []
    pos_z_err = []
    for i in range(n):
        pos_x_err.append(float(truth_data[i]['x']) - float(est_data[i]['px']))
        pos_y_err.append(float(truth_data[i]['y']) - float(est_data[i]['py']))
        pos_z_err.append(float(truth_data[i]['z']) - float(est_data[i]['pz']))
    
    pos_x_drift = moving_average(pos_x_err, window=1000)
    pos_y_drift = moving_average(pos_y_err, window=1000)
    pos_z_drift = moving_average(pos_z_err, window=1000)
    
    return {
        'roll_err': roll_err,
        'roll_drift': roll_drift,
        'pitch_err': pitch_err,
        'pitch_drift': pitch_drift,
        'yaw_err': yaw_err,
        'yaw_drift': yaw_drift,
        'pos_x_err': pos_x_err,
        'pos_x_drift': pos_x_drift,
        'pos_y_err': pos_y_err,
        'pos_y_drift': pos_y_drift,
        'pos_z_err': pos_z_err,
        'pos_z_drift': pos_z_drift,
    }

def calculate_rmse(truth_data, est_data):
    n = min(len(truth_data), len(est_data))
    
    if n == 0:
        print("Error: No data to compare")
        return None
    
    pos_err = []
    vel_err = []
    roll_err = []
    pitch_err = []
    yaw_err = []
    
    for i in range(n):
        t = truth_data[i]
        e = est_data[i]
        
        # 位置誤差
        px_err = float(t['x']) - float(e['px'])
        py_err = float(t['y']) - float(e['py'])
        pz_err = float(t['z']) - float(e['pz'])
        pos_err.append(math.sqrt(px_err**2 + py_err**2 + pz_err**2))
        
        # 速度誤差
        vx_err = float(t['vx']) - float(e['vx'])
        vy_err = float(t['vy']) - float(e['vy'])
        vz_err = float(t['vz']) - float(e['vz'])
        vel_err.append(math.sqrt(vx_err**2 + vy_err**2 + vz_err**2))
        
        # 姿勢誤差 (deg)
        roll_err.append(angle_diff(float(t['roll']), float(e['roll'])))
        pitch_err.append(angle_diff(float(t['pitch']), float(e['pitch'])))
        yaw_err.append(angle_diff(float(t['yaw']), float(e['yaw'])))
    
    # RMSE計算
    pos_rmse = math.sqrt(sum(e**2 for e in pos_err) / n)
    vel_rmse = math.sqrt(sum(e**2 for e in vel_err) / n)
    roll_rmse = math.sqrt(sum(e**2 for e in roll_err) / n)
    pitch_rmse = math.sqrt(sum(e**2 for e in pitch_err) / n)
    yaw_rmse = math.sqrt(sum(e**2 for e in yaw_err) / n)
    att_rmse = math.sqrt((sum(e**2 for e in roll_err) + sum(e**2 for e in pitch_err) + sum(e**2 for e in yaw_err)) / (3*n))
    
    return {
        'pos_rmse': pos_rmse,
        'vel_rmse': vel_rmse,
        'roll_rmse': roll_rmse,
        'pitch_rmse': pitch_rmse,
        'yaw_rmse': yaw_rmse,
        'att_rmse': att_rmse
    }

if __name__ == '__main__':
    import os
    import sys
    
    base_dir = os.path.dirname(os.path.abspath(__file__))
    truth_file = os.path.join(base_dir, 'GenerateData', 'truth_data.csv')
    est_file = os.path.join(base_dir, 'Results', 'estimation.csv')
    
    if not os.path.exists(truth_file):
        print(f"Error: {truth_file} not found")
        sys.exit(1)
    if not os.path.exists(est_file):
        print(f"Error: {est_file} not found")
        sys.exit(1)
    
    truth_data = read_csv(truth_file)
    est_data = read_csv(est_file)
    
    print(f"Truth data rows: {len(truth_data)}")
    print(f"Estimation data rows: {len(est_data)}")
    
    if len(truth_data) == 0:
        print("Error: truth_data.csv is empty")
        sys.exit(1)
    
    results = calculate_rmse(truth_data, est_data)
    
    if results is None:
        sys.exit(1)
    
    print("=" * 60)
    print("ESKF Performance Analysis (with Truth Comparison)")
    print("=" * 60)
    print(f"Position RMSE:  {results['pos_rmse']:.4f} m")
    print(f"Velocity RMSE:  {results['vel_rmse']:.4f} m/s")
    print(f"Attitude RMSE:  {results['att_rmse']:.4f} deg")
    print(f"  - Roll RMSE:  {results['roll_rmse']:.4f} deg")
    print(f"  - Pitch RMSE: {results['pitch_rmse']:.4f} deg")
    print(f"  - Yaw RMSE:   {results['yaw_rmse']:.4f} deg")
    
    # ドリフト解析
    print("\n" + "=" * 60)
    print("Drift Analysis")
    print("=" * 60)
    
    drift = detect_drift(truth_data, est_data)
    
    # Roll ドリフト
    roll_drift_max = max(abs(d) for d in drift['roll_drift'])
    roll_drift_final = abs(drift['roll_drift'][-1])
    print(f"\nRoll Drift:")
    print(f"  Max drift:   {roll_drift_max:.4f} deg")
    print(f"  Final drift: {roll_drift_final:.4f} deg")
    
    # Pitch ドリフト
    pitch_drift_max = max(abs(d) for d in drift['pitch_drift'])
    pitch_drift_final = abs(drift['pitch_drift'][-1])
    print(f"\nPitch Drift:")
    print(f"  Max drift:   {pitch_drift_max:.4f} deg")
    print(f"  Final drift: {pitch_drift_final:.4f} deg")
    
    # Yaw ドリフト
    yaw_drift_max = max(abs(d) for d in drift['yaw_drift'])
    yaw_drift_final = abs(drift['yaw_drift'][-1])
    print(f"\nYaw Drift:")
    print(f"  Max drift:   {yaw_drift_max:.4f} deg")
    print(f"  Final drift: {yaw_drift_final:.4f} deg")
    
    # 位置ドリフト
    pos_x_drift_max = max(abs(d) for d in drift['pos_x_drift'])
    pos_y_drift_max = max(abs(d) for d in drift['pos_y_drift'])
    pos_z_drift_max = max(abs(d) for d in drift['pos_z_drift'])
    print(f"\nPosition Drift:")
    print(f"  X max drift: {pos_x_drift_max:.4f} m")
    print(f"  Y max drift: {pos_y_drift_max:.4f} m")
    print(f"  Z max drift: {pos_z_drift_max:.4f} m")
    
    print("\n" + "=" * 60)
    print("Divergence & Drift Check")
    print("=" * 60)
    
    # 発散チェック
    if results['pos_rmse'] > 100:
        print("WARNING: Position estimation may have DIVERGED!")
    else:
        print(f"Position OK (RMSE = {results['pos_rmse']:.2f} m)")
    
    if results['att_rmse'] > 50:
        print("WARNING: Attitude estimation may have DIVERGED!")
    else:
        print(f"Attitude OK (RMSE = {results['att_rmse']:.2f} deg)")
    
    # ドリフト警告
    if roll_drift_max > 5.0:
        print(f"WARNING: Roll drift detected (max = {roll_drift_max:.2f} deg)")
    else:
        print(f"Roll drift OK (max = {roll_drift_max:.2f} deg)")
    
    if pitch_drift_max > 5.0:
        print(f"WARNING: Pitch drift detected (max = {pitch_drift_max:.2f} deg)")
    else:
        print(f"Pitch drift OK (max = {pitch_drift_max:.2f} deg)")
    
    print("=" * 60)
