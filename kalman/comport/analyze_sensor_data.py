#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
センサーCSVデータの包括的分析スクリプト
- 全CSVファイルの概要統計
- 運動パターン自動検出
- シミュレーションモデルとの整合性検証
"""

import pandas as pd
import numpy as np
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

COMPORT_DIR = Path(__file__).parent

CSV_FILES = [
    'stillness.csv',
    'Tilt right.csv',
    'Upward.csv',
    'Clockwise.csv',
    'Right rotation.csv',
    'Upward and downward.csv',
    'One rotation left and right.csv',
    'Up and down.csv',
    'Back and forth.csv',
]

SENSOR_COLS = {
    'accel': ['accel_x', 'accel_y', 'accel_z'],
    'gyro': ['gyro_x', 'gyro_y', 'gyro_z'],
    'mag': ['mag_x', 'mag_y', 'mag_z'],
    'baro': ['baro'],
}

# ========================== 1. 基本統計 ==========================
def load_csv(name):
    path = COMPORT_DIR / name
    if not path.exists():
        print(f"  [SKIP] {name} not found")
        return None
    df = pd.read_csv(path)
    return df

def print_overview(name, df):
    duration = df['time'].iloc[-1] - df['time'].iloc[0]
    dt_mean = np.diff(df['time'].values).mean()
    dt_std = np.diff(df['time'].values).std()
    fs = 1.0 / dt_mean if dt_mean > 0 else 0
    print(f"\n{'='*80}")
    print(f"📄 {name}")
    print(f"{'='*80}")
    print(f"  サンプル数: {len(df)},  期間: {duration:.2f}s,  平均dt: {dt_mean*1000:.1f}ms (σ={dt_std*1000:.2f}ms),  推定Fs: {fs:.1f}Hz")

def print_sensor_stats(df):
    """各センサーの平均・標準偏差・最小・最大"""
    print(f"\n  {'センサー':<12} {'平均':>10} {'σ':>10} {'最小':>10} {'最大':>10}")
    print(f"  {'-'*54}")
    for group, cols in SENSOR_COLS.items():
        for col in cols:
            if col in df.columns:
                vals = df[col].values
                print(f"  {col:<12} {np.mean(vals):>10.4f} {np.std(vals):>10.4f} {np.min(vals):>10.4f} {np.max(vals):>10.4f}")

def compute_accel_magnitude(df):
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    return np.sqrt(ax**2 + ay**2 + az**2)

def compute_gyro_magnitude(df):
    gx, gy, gz = df['gyro_x'].values, df['gyro_y'].values, df['gyro_z'].values
    return np.sqrt(gx**2 + gy**2 + gz**2)

def compute_mag_magnitude(df):
    mx, my, mz = df['mag_x'].values, df['mag_y'].values, df['mag_z'].values
    return np.sqrt(mx**2 + my**2 + mz**2)

# ========================== 2. 運動パターン検出 ==========================
def detect_stationary_segments(df, accel_threshold=0.3, gyro_threshold=5.0, window_s=0.5):
    """静止区間を検出する"""
    dt = np.diff(df['time'].values).mean()
    w = max(int(window_s / dt), 1)
    
    accel_mag = compute_accel_magnitude(df)
    gyro_mag = compute_gyro_magnitude(df)
    
    # 加速度のノルムが9.81近辺で安定 + ジャイロが小さい
    accel_dev = pd.Series(np.abs(accel_mag - 9.81)).rolling(w, center=True).mean().values
    gyro_avg = pd.Series(gyro_mag).rolling(w, center=True).mean().values
    
    # 静止判定 (gyroはバイアスがあるので、変動量で判定)
    gyro_std_rolling = pd.Series(gyro_mag).rolling(w, center=True).std().values
    
    is_stationary = (accel_dev < accel_threshold) & (gyro_std_rolling < gyro_threshold)
    return is_stationary

def detect_motion_phases(df):
    """運動フェーズを検出してレポート"""
    accel_mag = compute_accel_magnitude(df)
    gyro_mag = compute_gyro_magnitude(df)
    
    # ジャイロの各成分の変動量（バイアス除去のため、移動標準偏差を使用）
    dt = np.diff(df['time'].values).mean()
    w = max(int(1.0 / dt), 10)  # 1秒ウィンドウ
    
    gx_std = pd.Series(df['gyro_x'].values).rolling(w, center=True).std().fillna(0).values
    gy_std = pd.Series(df['gyro_y'].values).rolling(w, center=True).std().fillna(0).values
    gz_std = pd.Series(df['gyro_z'].values).rolling(w, center=True).std().fillna(0).values
    
    ax_std = pd.Series(df['accel_x'].values).rolling(w, center=True).std().fillna(0).values
    ay_std = pd.Series(df['accel_y'].values).rolling(w, center=True).std().fillna(0).values
    az_std = pd.Series(df['accel_z'].values).rolling(w, center=True).std().fillna(0).values
    
    # 運動強度
    gyro_activity = np.sqrt(gx_std**2 + gy_std**2 + gz_std**2)
    accel_activity = np.sqrt(ax_std**2 + ay_std**2 + az_std**2)
    
    # 区間ごとの支配軸検出
    motion_threshold = 2.0  # deg/s の標準偏差
    
    time = df['time'].values
    
    # 簡易フェーズ分割: 活動量の変化点を検出
    is_moving = gyro_activity > motion_threshold
    
    # 連続する同じ状態の区間を抽出
    phases = []
    if len(is_moving) == 0:
        return phases
    
    current_state = is_moving[0]
    start_idx = 0
    
    for i in range(1, len(is_moving)):
        if is_moving[i] != current_state or i == len(is_moving) - 1:
            end_idx = i
            phase_time = time[end_idx] - time[start_idx]
            if phase_time > 0.3:  # 0.3秒以上の区間のみ
                # この区間の特徴量
                seg_gx = gx_std[start_idx:end_idx]
                seg_gy = gy_std[start_idx:end_idx]
                seg_gz = gz_std[start_idx:end_idx]
                
                dominant_axis = ''
                if current_state:  # 運動中
                    avg_gx = np.mean(seg_gx)
                    avg_gy = np.mean(seg_gy)
                    avg_gz = np.mean(seg_gz)
                    max_g = max(avg_gx, avg_gy, avg_gz)
                    if max_g == avg_gx:
                        dominant_axis = 'Roll(X)'
                    elif max_g == avg_gy:
                        dominant_axis = 'Pitch(Y)'
                    else:
                        dominant_axis = 'Yaw(Z)'
                
                phases.append({
                    'start': time[start_idx],
                    'end': time[end_idx],
                    'duration': phase_time,
                    'is_moving': current_state,
                    'dominant_axis': dominant_axis,
                    'gyro_activity': np.mean(gyro_activity[start_idx:end_idx]),
                    'accel_activity': np.mean(accel_activity[start_idx:end_idx]),
                })
            
            current_state = is_moving[i]
            start_idx = i
    
    return phases

def print_motion_phases(phases):
    print(f"\n  運動フェーズ検出:")
    print(f"  {'#':>3} {'開始':>8} {'終了':>8} {'期間':>6} {'状態':>6} {'支配軸':>10} {'ジャイロ活動':>12} {'加速度活動':>10}")
    print(f"  {'-'*72}")
    for i, p in enumerate(phases):
        state = '運動' if p['is_moving'] else '静止'
        print(f"  {i+1:>3} {p['start']:>8.2f} {p['end']:>8.2f} {p['duration']:>6.2f}s {state:>6} {p['dominant_axis']:>10} {p['gyro_activity']:>12.2f} {p['accel_activity']:>10.4f}")

# ========================== 3. モデル検証 ==========================

def verify_stillness(df):
    """静止データのモデル検証"""
    print(f"\n  🔍 静止モデル検証:")
    
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    gx, gy, gz = df['gyro_x'].values, df['gyro_y'].values, df['gyro_z'].values
    mx, my, mz = df['mag_x'].values, df['mag_y'].values, df['mag_z'].values
    baro = df['baro'].values
    
    # 加速度: 重力ベクトル確認
    accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
    print(f"    加速度ノルム: 平均 {np.mean(accel_mag):.4f} m/s² (期待: 9.81)")
    print(f"      誤差: {abs(np.mean(accel_mag) - 9.81):.4f} m/s² ({abs(np.mean(accel_mag) - 9.81)/9.81*100:.2f}%)")
    print(f"    加速度各軸平均: X={np.mean(ax):.4f}, Y={np.mean(ay):.4f}, Z={np.mean(az):.4f}")
    print(f"    加速度各軸σ:    X={np.std(ax):.4f}, Y={np.std(ay):.4f}, Z={np.std(az):.4f}")
    
    # 重力方向の推定（最大成分から姿勢推定）
    gravity_dir = np.array([np.mean(ax), np.mean(ay), np.mean(az)])
    gravity_dir_norm = gravity_dir / np.linalg.norm(gravity_dir)
    print(f"    重力方向（正規化）: [{gravity_dir_norm[0]:.4f}, {gravity_dir_norm[1]:.4f}, {gravity_dir_norm[2]:.4f}]")
    
    # ジャイロ: バイアス確認
    print(f"\n    ジャイロ平均(=バイアス): X={np.mean(gx):.4f}, Y={np.mean(gy):.4f}, Z={np.mean(gz):.4f} deg/s")
    print(f"    ジャイロσ:              X={np.std(gx):.4f}, Y={np.std(gy):.4f}, Z={np.std(gz):.4f} deg/s")
    gyro_bias = np.sqrt(np.mean(gx)**2 + np.mean(gy)**2 + np.mean(gz)**2)
    print(f"    ジャイロバイアスノルム: {gyro_bias:.4f} deg/s")
    if gyro_bias > 5.0:
        print(f"    ⚠️ ジャイロバイアスが非常に大きい ({gyro_bias:.1f} deg/s)。キャリブレーション未実施の可能性")
    
    # 磁気: ノルム確認
    mag_mag = np.sqrt(mx**2 + my**2 + mz**2)
    print(f"\n    磁気ノルム: 平均 {np.mean(mag_mag):.2f} μT (σ={np.std(mag_mag):.2f})")
    print(f"    磁気各軸平均: X={np.mean(mx):.2f}, Y={np.mean(my):.2f}, Z={np.mean(mz):.2f} μT")
    
    # 気圧: 安定性
    print(f"\n    気圧: 平均 {np.mean(baro):.2f} Pa (σ={np.std(baro):.2f} Pa)")
    baro_alt = (101325 - np.mean(baro)) / 12.0  # 簡易高度換算
    print(f"    推定高度（標準大気モデル）: {baro_alt:.1f} m")
    
    # ノイズモデル比較
    print(f"\n    📊 ノイズレベル比較 (実測σ vs シミュレーション設定):")
    print(f"      加速度: 実 {np.std(accel_mag):.4f} m/s² vs シミュ 0.1 (white) + 0.2 (pink)")
    print(f"      ジャイロ: 実 X={np.std(gx):.4f} Y={np.std(gy):.4f} Z={np.std(gz):.4f} deg/s vs シミュ 0.5 (white) + 0.2 (pink)")
    print(f"      磁気: 実 {np.std(mag_mag):.2f} μT vs シミュ 5.0 μT ← 単位をμTに統一しました")
    print(f"      気圧: 実 {np.std(baro):.2f} Pa vs シミュ 2.0 m (≈24 Pa)")

def verify_tilt(df, expected_axis='y', expected_angle_deg=90):
    """傾斜データのモデル検証"""
    print(f"\n  🔍 傾斜モデル検証:")
    
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    
    # 静止区間を探す（ジャイロ活動量で判定）
    phases = detect_motion_phases(df)
    
    # 最初と最後の静止区間を使って姿勢変化を明確にする
    stationary_phases = [p for p in phases if not p['is_moving'] and p['duration'] > 1.0]
    
    if len(stationary_phases) >= 2:
        first = stationary_phases[0]
        # 最後の長い静止区間を探す
        last = stationary_phases[-1]
        
        # 各静止区間での重力方向
        t = df['time'].values
        for i, phase in enumerate(stationary_phases):
            mask = (t >= phase['start']) & (t <= phase['end'])
            ax_seg = df.loc[mask, 'accel_x'].values
            ay_seg = df.loc[mask, 'accel_y'].values
            az_seg = df.loc[mask, 'accel_z'].values
            
            g_vec = np.array([np.mean(ax_seg), np.mean(ay_seg), np.mean(az_seg)])
            g_mag = np.linalg.norm(g_vec)
            g_norm = g_vec / g_mag
            
            roll = np.degrees(np.arctan2(g_norm[1], g_norm[2]))
            pitch = np.degrees(np.arctan2(-g_norm[0], np.sqrt(g_norm[1]**2 + g_norm[2]**2)))
            
            print(f"    静止区間{i+1} ({phase['start']:.1f}s-{phase['end']:.1f}s, {phase['duration']:.1f}s):")
            print(f"      重力ベクトル: [{g_vec[0]:.4f}, {g_vec[1]:.4f}, {g_vec[2]:.4f}] (|g|={g_mag:.4f})")
            print(f"      推定Roll: {roll:.1f}°, Pitch: {pitch:.1f}°")
    else:
        # 全体の統計
        g_vec = np.array([np.mean(ax), np.mean(ay), np.mean(az)])
        g_mag = np.linalg.norm(g_vec)
        g_norm = g_vec / g_mag
        roll = np.degrees(np.arctan2(g_norm[1], g_norm[2]))
        pitch = np.degrees(np.arctan2(-g_norm[0], np.sqrt(g_norm[1]**2 + g_norm[2]**2)))
        print(f"    全体の重力ベクトル: [{g_vec[0]:.4f}, {g_vec[1]:.4f}, {g_vec[2]:.4f}] (|g|={g_mag:.4f})")
        print(f"    推定Roll: {roll:.1f}°, Pitch: {pitch:.1f}°")

def verify_rotation(df, name):
    """回転データのモデル検証"""
    print(f"\n  🔍 回転モデル検証:")
    
    gx, gy, gz = df['gyro_x'].values, df['gyro_y'].values, df['gyro_z'].values
    time = df['time'].values
    
    # バイアス推定（最初の安定区間から）
    # 最初の0.5秒 or 最初の10%のデータ
    n_bias = min(50, len(gx) // 10)
    if n_bias < 5:
        n_bias = 5
    
    # バイアス推定は静止区間から行う
    phases = detect_motion_phases(df)
    stationary_phases = [p for p in phases if not p['is_moving'] and p['duration'] > 0.5]
    
    if stationary_phases:
        # 最初の静止区間からバイアスを推定
        first_static = stationary_phases[0]
        mask = (time >= first_static['start']) & (time <= first_static['end'])
        bias_gx = np.mean(gx[mask])
        bias_gy = np.mean(gy[mask])
        bias_gz = np.mean(gz[mask])
    else:
        bias_gx = np.mean(gx[:n_bias])
        bias_gy = np.mean(gy[:n_bias])
        bias_gz = np.mean(gz[:n_bias])
    
    print(f"    推定ジャイロバイアス: X={bias_gx:.4f}, Y={bias_gy:.4f}, Z={bias_gz:.4f} deg/s")
    
    # バイアス補正
    gx_corr = gx - bias_gx
    gy_corr = gy - bias_gy
    gz_corr = gz - bias_gz
    
    # 積分（台形公式）で回転角度推定
    dt = np.diff(time)
    angle_x = np.cumsum((gx_corr[:-1] + gx_corr[1:]) / 2 * dt)
    angle_y = np.cumsum((gy_corr[:-1] + gy_corr[1:]) / 2 * dt)
    angle_z = np.cumsum((gz_corr[:-1] + gz_corr[1:]) / 2 * dt)
    
    total_rot_x = angle_x[-1] if len(angle_x) > 0 else 0
    total_rot_y = angle_y[-1] if len(angle_y) > 0 else 0
    total_rot_z = angle_z[-1] if len(angle_z) > 0 else 0
    
    print(f"    バイアス補正後の総回転角度:")
    print(f"      X(Roll):  {total_rot_x:.1f}°")
    print(f"      Y(Pitch): {total_rot_y:.1f}°")
    print(f"      Z(Yaw):   {total_rot_z:.1f}°")
    
    # 最大角速度（バイアス補正後）
    print(f"    バイアス補正後の最大角速度:")
    print(f"      X: {np.max(np.abs(gx_corr)):.1f} deg/s")
    print(f"      Y: {np.max(np.abs(gy_corr)):.1f} deg/s")
    print(f"      Z: {np.max(np.abs(gz_corr)):.1f} deg/s")
    
    # 運動区間の角速度平均
    moving_phases = [p for p in phases if p['is_moving']]
    for i, phase in enumerate(moving_phases):
        mask = (time >= phase['start']) & (time <= phase['end'])
        seg_gx = gx_corr[mask]
        seg_gy = gy_corr[mask]
        seg_gz = gz_corr[mask]
        print(f"    運動区間{i+1} ({phase['start']:.1f}s-{phase['end']:.1f}s, {phase['duration']:.1f}s):")
        print(f"      平均角速度: X={np.mean(seg_gx):.1f}, Y={np.mean(seg_gy):.1f}, Z={np.mean(seg_gz):.1f} deg/s")
        print(f"      支配軸: {phase['dominant_axis']}")

def verify_translation(df, name):
    """並進運動データのモデル検証"""
    print(f"\n  🔍 並進運動モデル検証:")
    
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    baro = df['baro'].values
    time = df['time'].values
    
    # 加速度ノルムの変動
    accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
    print(f"    加速度ノルム: 平均 {np.mean(accel_mag):.4f}, σ={np.std(accel_mag):.4f}, 最大偏差: {np.max(np.abs(accel_mag - 9.81)):.4f}")
    
    # 気圧変化
    baro_range = np.max(baro) - np.min(baro)
    alt_range = baro_range / 12.0  # 簡易換算
    print(f"    気圧変化: {baro_range:.2f} Pa (≈{alt_range:.2f}m 高度差)")
    print(f"    気圧: 最小 {np.min(baro):.2f} Pa, 最大 {np.max(baro):.2f} Pa")
    
    # 各軸の加速度変動
    print(f"    加速度変動:")
    print(f"      X(前後): 平均 {np.mean(ax):.4f}, σ={np.std(ax):.4f}")
    print(f"      Y(左右): 平均 {np.mean(ay):.4f}, σ={np.std(ay):.4f}")
    print(f"      Z(上下): 平均 {np.mean(az):.4f}, σ={np.std(az):.4f}")

def check_unit_consistency(df, name):
    """単位一貫性チェック"""
    print(f"\n  🔍 単位一貫性チェック:")
    
    # 加速度: m/s²（9.81付近であるべき）
    accel_mag = compute_accel_magnitude(df)
    if 9.0 < np.mean(accel_mag) < 11.0:
        print(f"    ✅ 加速度: m/s² で正しい (|a|={np.mean(accel_mag):.4f})")
    else:
        print(f"    ❌ 加速度: 期待値9.81から大きく外れている (|a|={np.mean(accel_mag):.4f})")
    
    # 磁気: μT（日本では25-60μT程度）
    mag_mag = compute_mag_magnitude(df)
    mean_mag = np.mean(mag_mag)
    if 20 < mean_mag < 70:
        print(f"    ✅ 磁気: μT で正しい (|B|={mean_mag:.2f} μT)")
    elif 20000 < mean_mag < 70000:
        print(f"    ⚠️ 磁気: 単位表示更新: μT を想定 (|B|={mean_mag:.2f} μT)")
    else:
        print(f"    ❓ 磁気: 単位不明 (|B|={mean_mag:.2f})")
    
    # 気圧: Pa（101325付近）
    baro = df['baro'].values
    if 80000 < np.mean(baro) < 120000:
        print(f"    ✅ 気圧: Pa で正しい ({np.mean(baro):.0f} Pa)")
    elif 800 < np.mean(baro) < 1200:
        print(f"    ⚠️ 気圧: hPa の可能性 ({np.mean(baro):.1f})")
    else:
        print(f"    ❓ 気圧: 単位不明 ({np.mean(baro):.1f})")
    
    # シミュレーションとの単位差異
    print(f"\n    ⚠️ シミュレーションとの単位差異:")
    print(f"      - 磁気: 実センサー = μT, シミュレーション(config_params.m) = μT (統一済み)")

# ========================== 4. メイン処理 ==========================

def analyze_all():
    print("=" * 80)
    print("  センサーデータ総合分析レポート")
    print("=" * 80)
    
    for csv_name in CSV_FILES:
        df = load_csv(csv_name)
        if df is None:
            continue
        
        print_overview(csv_name, df)
        print_sensor_stats(df)
        
        # 運動フェーズ検出
        phases = detect_motion_phases(df)
        print_motion_phases(phases)
        
        # 単位チェック
        check_unit_consistency(df, csv_name)
        
        # ファイル毎の検証
        base_name = csv_name.lower().replace('.csv', '')
        
        if 'stillness' in base_name:
            verify_stillness(df)
        elif 'tilt' in base_name or 'upward' == base_name:
            verify_tilt(df)
        elif 'rotation' in base_name or 'clockwise' in base_name or 'one rotation' in base_name:
            verify_rotation(df, csv_name)
        elif 'upward and downward' in base_name:
            verify_rotation(df, csv_name)
        elif 'up and down' in base_name or 'back and forth' in base_name:
            verify_translation(df, csv_name)
            verify_rotation(df, csv_name)  # 並進中も回転する場合あり
        
        print()

if __name__ == '__main__':
    analyze_all()
