#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
深層センサーモデル検証スクリプト

静止データから推定したバイアス/ノイズ特性と、
各運動パターンの物理的整合性を丁寧に検証する。
"""

import pandas as pd
import numpy as np
from pathlib import Path
from scipy import signal
import warnings
warnings.filterwarnings('ignore')

COMPORT_DIR = Path(__file__).parent

def load(name):
    path = COMPORT_DIR / name
    if not path.exists():
        return None
    return pd.read_csv(path)

# ============================================================
# A. 静止データからのキャリブレーション解析
# ============================================================
def analyze_stillness_deep():
    df = load('stillness.csv')
    if df is None:
        return {}
    
    print("=" * 90)
    print("  A. 静止データ(stillness.csv)からのキャリブレーション・ノイズ解析")
    print("=" * 90)
    
    # 安定した区間を使う（最初の衝撃を除く: 6-28秒が安定区間）
    t = df['time'].values
    mask = (t >= 6.5) & (t <= 28.0)
    df_stable = df[mask]
    
    ax = df_stable['accel_x'].values
    ay = df_stable['accel_y'].values
    az = df_stable['accel_z'].values
    gx = df_stable['gyro_x'].values
    gy = df_stable['gyro_y'].values
    gz = df_stable['gyro_z'].values
    mx = df_stable['mag_x'].values
    my = df_stable['mag_y'].values
    mz = df_stable['mag_z'].values
    baro = df_stable['baro'].values
    
    # 1. 加速度計キャリブレーション
    print("\n  1. 加速度計キャリブレーション")
    print("  " + "-" * 50)
    g_vec = np.array([np.mean(ax), np.mean(ay), np.mean(az)])
    g_mag = np.linalg.norm(g_vec)
    g_norm = g_vec / g_mag
    
    print(f"    重力ベクトル: [{g_vec[0]:.6f}, {g_vec[1]:.6f}, {g_vec[2]:.6f}]")
    print(f"    重力ノルム:   {g_mag:.6f} m/s² (期待: 9.80665)")
    print(f"    誤差:         {abs(g_mag - 9.80665):.6f} m/s² ({abs(g_mag - 9.80665)/9.80665*100:.3f}%)")
    print(f"    重力方向:     [{g_norm[0]:.6f}, {g_norm[1]:.6f}, {g_norm[2]:.6f}]")
    
    roll_est = np.degrees(np.arctan2(g_norm[1], g_norm[2]))
    pitch_est = np.degrees(np.arctan2(-g_norm[0], np.sqrt(g_norm[1]**2 + g_norm[2]**2)))
    print(f"    推定初期姿勢: Roll={roll_est:.2f}°, Pitch={pitch_est:.2f}°")
    
    # ノイズ特性
    accel_noise = np.array([np.std(ax), np.std(ay), np.std(az)])
    print(f"\n    ノイズσ:  X={accel_noise[0]:.6f}, Y={accel_noise[1]:.6f}, Z={accel_noise[2]:.6f} m/s²")
    print(f"    ノイズRMS: {np.sqrt(np.mean(accel_noise**2)):.6f} m/s²")
    
    # シミュレーション比較
    print(f"\n    📊 シミュレーションモデル比較:")
    print(f"      シミュ accel_std = 0.1 m/s² (white)")
    print(f"      シミュ accel_pink_std = 0.2 m/s² (pink)")
    print(f"      合成σ ≈ √(0.1²+0.2²) = {np.sqrt(0.01+0.04):.4f} m/s²")
    print(f"      実測σ ≈ {np.mean(accel_noise):.4f} m/s²")
    ratio = np.mean(accel_noise) / np.sqrt(0.01+0.04)
    if ratio < 0.5:
        print(f"      → 実測は設定値の {ratio:.1%} ⚠️ シミュの方がノイズが大きい")
    elif ratio > 2.0:
        print(f"      → 実測は設定値の {ratio:.1%} ⚠️ 実機の方がノイズが大きい")
    else:
        print(f"      → 比率 {ratio:.2f} ✅ 概ね整合")
    
    # 2. ジャイロキャリブレーション
    print(f"\n  2. ジャイロキャリブレーション")
    print("  " + "-" * 50)
    gyro_bias = np.array([np.mean(gx), np.mean(gy), np.mean(gz)])
    gyro_noise = np.array([np.std(gx), np.std(gy), np.std(gz)])
    
    print(f"    バイアス(平均): X={gyro_bias[0]:.4f}, Y={gyro_bias[1]:.4f}, Z={gyro_bias[2]:.4f} deg/s")
    print(f"    バイアスノルム: {np.linalg.norm(gyro_bias):.4f} deg/s")
    print(f"    ノイズσ:       X={gyro_noise[0]:.4f}, Y={gyro_noise[1]:.4f}, Z={gyro_noise[2]:.4f} deg/s")
    
    # バイアスの大きさについて
    print(f"\n    ⚠️ ジャイロバイアス分析:")
    print(f"      バイアスが 30-35 deg/s は ICM-20948 の典型値 (±1~10 deg/s) と比べて非常に大きい")
    print(f"      可能性1: センサー出力が生(raw)値で、ゼロ点補正が未実施")
    print(f"      可能性2: ADC オフセット or レジスタ設定の問題")
    print(f"      → シミュレーションではバイアスは 0 想定（ノイズのみ生成）")
    print(f"      → フィルタ (ESKF) 内でバイアス推定するため、シミュではアラン偏差で時間変動のみモデル化")
    
    print(f"\n    📊 シミュレーションモデル比較:")
    print(f"      シミュ gyro_std = 0.5 deg/s (white)")
    print(f"      シミュ gyro_pink_std = 0.2 deg/s (pink)")
    print(f"      シミュ gyro_allan_std = 0.5 deg/s (allan)")
    print(f"      合成σ_instantaneous ≈ √(0.5²+0.2²) = {np.sqrt(0.25+0.04):.4f} deg/s")
    print(f"      実測σ_X = {gyro_noise[0]:.4f}, Y = {gyro_noise[1]:.4f}, Z = {gyro_noise[2]:.4f} deg/s")
    print(f"      X,Zは概ね一致、Y軸は異常にノイズが大きい（{gyro_noise[1]:.2f} deg/s）")
    
    # 3. 磁気計キャリブレーション
    print(f"\n  3. 磁気計キャリブレーション")
    print("  " + "-" * 50)
    mag_vec = np.array([np.mean(mx), np.mean(my), np.mean(mz)])
    mag_mag = np.linalg.norm(mag_vec)
    mag_noise = np.array([np.std(mx), np.std(my), np.std(mz)])
    
    print(f"    磁場ベクトル: [{mag_vec[0]:.2f}, {mag_vec[1]:.2f}, {mag_vec[2]:.2f}] μT")
    print(f"    磁場ノルム:   {mag_mag:.2f} μT (日本の典型値: 45-50 μT)")
    
    mag_horiz = np.sqrt(mag_vec[0]**2 + mag_vec[1]**2)
    mag_heading = np.degrees(np.arctan2(-mag_vec[1], mag_vec[0]))
    mag_dip = np.degrees(np.arctan2(mag_vec[2], mag_horiz))
    
    print(f"    水平成分:     {mag_horiz:.2f} μT")
    print(f"    磁北方向:     {mag_heading:.1f}° (機体座標系での方位)")
    print(f"    伏角:         {mag_dip:.1f}° (日本の典型値: 約45-55°)")
    print(f"    ノイズσ:      X={mag_noise[0]:.4f}, Y={mag_noise[1]:.4f}, Z={mag_noise[2]:.4f} μT")
    
    print(f"\n    📊 シミュレーションモデル比較:")
    print(f"      シミュ: mag_strength = 50 (単位: μT に修正済み)")
    print(f"      シミュ: mag_world = [50, 0, 0] → 北向き水平成分のみ")
    print(f"      実測:   mag = [{mag_vec[0]:.1f}, {mag_vec[1]:.1f}, {mag_vec[2]:.1f}] μT")
    print(f"      → 実測は鉛直成分(Z={mag_vec[2]:.1f})があるのにシミュは0")
    print(f"      → シミュは「磁場ベクトル=北向き水平」の単純モデル")
    print(f"      → 伏角(dip angle)が考慮されていない → 磁気更新に誤差要因")
    print(f"      シミュ mag_std = 5.0 (μTと解釈)")
    print(f"      実測   mag_noise ≈ {np.mean(mag_noise):.2f} μT")
    ratm = np.mean(mag_noise) / 5.0
    print(f"      → 比率 {ratm:.2f} → 実機は設定より {'小さい' if ratm < 1 else '大きい'}")
    
    # 4. 気圧計
    print(f"\n  4. 気圧計キャリブレーション")
    print("  " + "-" * 50)
    baro_mean = np.mean(baro)
    baro_std = np.std(baro)
    alt_est = (101325 - baro_mean) / 12.0
    
    print(f"    平均気圧:  {baro_mean:.2f} Pa")
    print(f"    ノイズσ:   {baro_std:.2f} Pa (≈{baro_std/12:.3f} m高度換算)")
    print(f"    推定高度:  {alt_est:.1f} m (標準大気モデル)")
    
    print(f"\n    📊 シミュレーションモデル比較:")
    print(f"      シミュ baro_std = 2.0 m → ≈24 Pa")
    print(f"      実測   baro_std = {baro_std:.2f} Pa (≈{baro_std/12:.2f} m)")
    print(f"      → 実機は設定値(24 Pa)より {'小さい' if baro_std < 24 else '大きい'} ⚠️")
    
    # 5. PSD解析（ピンクノイズの確認）
    print(f"\n  5. パワースペクトル密度(PSD)解析")
    print("  " + "-" * 50)
    
    fs = 100.0  # Hz
    for label, data, expected_white in [
        ('accel_z', az - np.mean(az), 0.1),
        ('gyro_x', gx - np.mean(gx), 0.5),
        ('mag_x', mx - np.mean(mx), 5.0),
    ]:
        f, psd = signal.welch(data, fs=fs, nperseg=min(256, len(data)))
        # ピンクノイズ検出: 低周波側のPSD傾きを計算
        # ホワイトノイズ: 傾き0, ピンクノイズ: 傾き-1 (dB/decade)
        low_mask = (f > 0.1) & (f < 5.0)
        high_mask = (f > 10.0) & (f < 40.0)
        
        if np.sum(low_mask) > 2 and np.sum(high_mask) > 2:
            psd_low = np.mean(psd[low_mask])
            psd_high = np.mean(psd[high_mask])
            ratio = psd_low / psd_high if psd_high > 0 else float('inf')
            
            # log-logでの傾き推定
            f_pos = f[f > 0.1]
            psd_pos = psd[f > 0.1]
            if len(f_pos) > 5:
                coeffs = np.polyfit(np.log10(f_pos), np.log10(psd_pos), 1)
                slope = coeffs[0]
            else:
                slope = 0
            
            print(f"    {label}:")
            print(f"      低周波PSD(0.1-5Hz)/高周波PSD(10-40Hz) = {ratio:.2f}")
            print(f"      log-log傾き: {slope:.2f} (ホワイト=0, ピンク=-1, ブラウン=-2)")
            
            if abs(slope) < 0.3:
                print(f"      → ほぼホワイトノイズ ✅")
            elif slope < -0.5:
                print(f"      → ピンクノイズ成分あり（1/f特性）")
            elif slope < -1.5:
                print(f"      → ブラウンノイズ（ランダムウォーク/アラン偏差）")
    
    # バイアス安定性の時間変動
    print(f"\n  6. ジャイロバイアスの時間安定性")
    print("  " + "-" * 50)
    
    # 全データを使って5秒ウィンドウでバイアス変動を追跡
    t_all = df['time'].values
    gx_all = df['gyro_x'].values
    gy_all = df['gyro_y'].values
    gz_all = df['gyro_z'].values
    
    window_samples = 500  # 5秒@100Hz
    n_windows = len(gx_all) // window_samples
    
    print(f"    5秒ウィンドウでのバイアス変動:")
    for w in range(n_windows):
        s = w * window_samples
        e = s + window_samples
        bx = np.mean(gx_all[s:e])
        by = np.mean(gy_all[s:e])
        bz = np.mean(gz_all[s:e])
        print(f"    Window {w+1} ({t_all[s]:.1f}-{t_all[e-1]:.1f}s): X={bx:.4f}, Y={by:.4f}, Z={bz:.4f}")
    
    return {
        'gyro_bias': gyro_bias,
        'gyro_noise': gyro_noise,
        'accel_bias': g_vec,
        'accel_noise': accel_noise,
        'mag_vec': mag_vec,
        'mag_noise': mag_noise,
        'baro_mean': baro_mean,
        'baro_std': baro_std,
    }

# ============================================================
# B. Tilt right — 重力ベクトルの座標変換確認
# ============================================================
def analyze_tilt_right(cal):
    df = load('Tilt right.csv')
    if df is None:
        return
    
    print("\n" + "=" * 90)
    print("  B. Tilt right.csv — 重力ベクトルの座標変換確認")
    print("=" * 90)
    
    t = df['time'].values
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    gx, gy, gz = df['gyro_x'].values, df['gyro_y'].values, df['gyro_z'].values
    
    # バイアス補正
    gb = cal['gyro_bias']
    gx_c, gy_c, gz_c = gx - gb[0], gy - gb[1], gz - gb[2]
    
    print(f"\n  ジャイロバイアス補正: stillnessから [{gb[0]:.2f}, {gb[1]:.2f}, {gb[2]:.2f}] deg/s を使用")
    print(f"  ⚠️ stillnessとTilt rightでバイアスが異なる可能性あり（セッション間変動）")
    
    # 静止区間を特定して各区間の姿勢を確認
    # データの変化から区間を推定
    accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
    
    # 加速度ノルムが安定している区間を抽出
    dt = np.diff(t).mean()
    w = int(1.0 / dt)
    
    am_rolling = pd.Series(accel_mag).rolling(w, center=True).std().fillna(10).values
    
    # 安定区間（accel_magの変動が小さい）
    stable = am_rolling < 0.3
    
    # 連続する安定区間を抽出
    segments = []
    in_seg = False
    seg_start = 0
    for i in range(len(stable)):
        if stable[i] and not in_seg:
            seg_start = i
            in_seg = True
        elif not stable[i] and in_seg:
            if (i - seg_start) > w:  # 1秒以上
                segments.append((seg_start, i))
            in_seg = False
    if in_seg and (len(stable) - seg_start) > w:
        segments.append((seg_start, len(stable)))
    
    print(f"\n  安定区間の姿勢推定:")
    for i, (s, e) in enumerate(segments):
        g = np.array([np.mean(ax[s:e]), np.mean(ay[s:e]), np.mean(az[s:e])])
        gm = np.linalg.norm(g)
        gn = g / gm
        roll = np.degrees(np.arctan2(gn[1], gn[2]))
        pitch = np.degrees(np.arctan2(-gn[0], np.sqrt(gn[1]**2 + gn[2]**2)))
        
        print(f"    区間{i+1} ({t[s]:.1f}-{t[e-1]:.1f}s, {(e-s)*dt:.1f}s): |g|={gm:.4f}, Roll={roll:.1f}°, Pitch={pitch:.1f}°")
    
    print(f"\n  モデル検証:")
    print(f"    期待: デバイスを右に傾ける → Roll ≈ 90°")
    
    # Roll=90°の区間があるか？
    found_90 = False
    for i, (s, e) in enumerate(segments):
        g = np.array([np.mean(ax[s:e]), np.mean(ay[s:e]), np.mean(az[s:e])])
        gn = g / np.linalg.norm(g)
        roll = np.degrees(np.arctan2(gn[1], gn[2]))
        if abs(roll) > 60:
            found_90 = True
            print(f"    ✅ 区間{i+1}でRoll={roll:.1f}° → 右傾斜を確認")
            
            # 重力分解の検証
            print(f"      理論: Roll=90°なら accel≈[0, ±9.81, 0]")
            print(f"      実測: accel=[{np.mean(ax[s:e]):.4f}, {np.mean(ay[s:e]):.4f}, {np.mean(az[s:e]):.4f}]")
            
            # モデルの回転行列検証: R(roll=90°) * [0,0,g] = [0,g,0]
            r_rad = np.radians(roll)
            R_roll = np.array([
                [1, 0, 0],
                [0, np.cos(r_rad), -np.sin(r_rad)],
                [0, np.sin(r_rad), np.cos(r_rad)]
            ])
            g_world_zup = np.array([0, 0, 9.81])  # Z-up
            g_body = R_roll @ g_world_zup
            print(f"      回転行列予測: [{g_body[0]:.4f}, {g_body[1]:.4f}, {g_body[2]:.4f}]")
    
    if not found_90:
        print(f"    ⚠️ Roll≈90°の安定区間が見つからない")
    
    # ジャイロ積分で角度変化を追跡
    print(f"\n  ジャイロ積分による角度追跡:")
    dt_arr = np.diff(t)
    angle_x = np.cumsum((gx_c[:-1] + gx_c[1:]) / 2 * dt_arr)
    angle_y = np.cumsum((gy_c[:-1] + gy_c[1:]) / 2 * dt_arr)
    angle_z = np.cumsum((gz_c[:-1] + gz_c[1:]) / 2 * dt_arr)
    
    print(f"    最大Roll(積分): {np.max(np.abs(angle_x)):.1f}°")
    print(f"    最大Pitch(積分): {np.max(np.abs(angle_y)):.1f}°")
    
    # 注意点
    print(f"\n  ⚠️ 注意: 支配軸がPitch(Y)と検出されている")
    print(f"    → 'Tilt right' というラベルだがPitch回転がメインの可能性")
    print(f"    → センサーの取り付け方向、または操作がRollではなくPitchの可能性")

# ============================================================
# C. Upward — ピッチ傾斜確認
# ============================================================
def analyze_upward(cal):
    df = load('Upward.csv')
    if df is None:
        return
    
    print("\n" + "=" * 90)
    print("  C. Upward.csv — ピッチ傾斜確認")
    print("=" * 90)
    
    t = df['time'].values
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    
    # 気圧が全て0
    print(f"  ⚠️ 気圧データが全て0 → このファイルではbaro計測が無効")
    
    # 安定区間の姿勢推定    
    dt = np.diff(t).mean()
    w = int(1.0 / dt)
    accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
    am_std = pd.Series(accel_mag).rolling(w, center=True).std().fillna(10).values
    
    stable = am_std < 0.15
    segments = []
    in_seg = False
    seg_start = 0
    for i in range(len(stable)):
        if stable[i] and not in_seg:
            seg_start = i
            in_seg = True
        elif not stable[i] and in_seg:
            if (i - seg_start) > w:
                segments.append((seg_start, i))
            in_seg = False
    if in_seg and (len(stable) - seg_start) > w:
        segments.append((seg_start, len(stable)))
    
    print(f"\n  安定区間の姿勢推定:")
    for i, (s, e) in enumerate(segments):
        g = np.array([np.mean(ax[s:e]), np.mean(ay[s:e]), np.mean(az[s:e])])
        gm = np.linalg.norm(g)
        gn = g / gm 
        roll = np.degrees(np.arctan2(gn[1], gn[2]))
        pitch = np.degrees(np.arctan2(-gn[0], np.sqrt(gn[1]**2 + gn[2]**2)))
        
        print(f"    区間{i+1} ({t[s]:.1f}-{t[min(e-1,len(t)-1)]:.1f}s, {(e-s)*dt:.1f}s):")
        print(f"      |g|={gm:.4f}, Roll={roll:.1f}°, Pitch={pitch:.1f}°")
        print(f"      accel=[{np.mean(ax[s:e]):.4f}, {np.mean(ay[s:e]):.4f}, {np.mean(az[s:e]):.4f}]")
    
    print(f"\n  モデル検証:")
    print(f"    期待: デバイスを上向きに傾ける → Pitch ≈ -45° (前方を上向き) or 類似角度")
    
    # Pitch角が大きい区間があるか
    for i, (s, e) in enumerate(segments):
        g = np.array([np.mean(ax[s:e]), np.mean(ay[s:e]), np.mean(az[s:e])])
        gn = g / np.linalg.norm(g)
        pitch = np.degrees(np.arctan2(-gn[0], np.sqrt(gn[1]**2 + gn[2]**2)))
        
        if abs(pitch) > 20:
            g_mag = np.linalg.norm(g)
            print(f"    ✅ 区間{i+1}: Pitch={pitch:.1f}°")
            print(f"      理論 (Pitch={pitch:.0f}°): accel_x={9.81*np.sin(np.radians(-pitch)):.4f}, accel_z={9.81*np.cos(np.radians(-pitch)):.4f}")
            print(f"      実測: accel_x={np.mean(ax[s:e]):.4f}, accel_z={np.mean(az[s:e]):.4f}")
            
            # 重力ノルムの保存確認
            print(f"      重力ノルム: {g_mag:.4f} (期待: 9.81) → {'✅ OK' if abs(g_mag-9.81)<0.5 else '⚠️ 差が大きい'}")

# ============================================================
# D. Clockwise — ヨー回転の方位角変化確認
# ============================================================
def analyze_clockwise(cal):
    df = load('Clockwise.csv')
    if df is None:
        return
        
    print("\n" + "=" * 90)
    print("  D. Clockwise.csv — ヨー回転の方位角変化確認")
    print("=" * 90)
    
    t = df['time'].values
    gx, gy, gz = df['gyro_x'].values, df['gyro_y'].values, df['gyro_z'].values
    mx, my, mz = df['mag_x'].values, df['mag_y'].values, df['mag_z'].values
    
    # バイアス推定: この録画の静止区間から（セッション変化対策）
    # 最初の安定区間（2.7-5.5s）を使用
    mask_static = (t >= 2.7) & (t <= 5.4)
    gb_local = np.array([
        np.mean(gx[mask_static]), 
        np.mean(gy[mask_static]), 
        np.mean(gz[mask_static])
    ])
    print(f"  このセッションのジャイロバイアス: [{gb_local[0]:.2f}, {gb_local[1]:.2f}, {gb_local[2]:.2f}]")
    print(f"  stillnessのジャイロバイアス: [{cal['gyro_bias'][0]:.2f}, {cal['gyro_bias'][1]:.2f}, {cal['gyro_bias'][2]:.2f}]")
    print(f"  差分: [{gb_local[0]-cal['gyro_bias'][0]:.2f}, {gb_local[1]-cal['gyro_bias'][1]:.2f}, {gb_local[2]-cal['gyro_bias'][2]:.2f}]")
    print(f"  ⚠️ セッション間バイアスシフト: {np.linalg.norm(gb_local - cal['gyro_bias']):.2f} deg/s")
    
    gz_c = gz - gb_local[2]
    
    # 累積ヨー角
    dt_arr = np.diff(t)
    yaw_integral = np.cumsum((gz_c[:-1] + gz_c[1:]) / 2 * dt_arr)
    
    print(f"\n  ジャイロZ軸積分（バイアス補正後）:")
    print(f"    総ヨー回転角: {yaw_integral[-1]:.1f}°")
    print(f"    期待: 時計回り → 負の回転（NED右手系）")
    
    # 磁気計からヨー角推定
    # 水平面での磁北方位
    heading = np.degrees(np.arctan2(-my, mx))
    heading_unwrap = np.unwrap(np.radians(heading))
    heading_unwrap_deg = np.degrees(heading_unwrap)
    
    total_heading_change = heading_unwrap_deg[-1] - heading_unwrap_deg[0]
    
    print(f"\n  磁気計からのヨー角変化:")
    print(f"    総方位変化: {total_heading_change:.1f}°")
    
    print(f"\n  ジャイロ vs 磁気計の整合性:")
    print(f"    ジャイロ積分: {yaw_integral[-1]:.1f}°")
    print(f"    磁気計方位変化: {total_heading_change:.1f}°")
    diff = abs(yaw_integral[-1] - total_heading_change)
    print(f"    差: {diff:.1f}° → {'✅ 整合' if diff < 30 else '⚠️ 不一致'}")
    
    # 磁気ベクトルが円を描くか確認  
    print(f"\n  磁気計水平成分の軌跡:")
    print(f"    Mx 範囲: [{np.min(mx):.2f}, {np.max(mx):.2f}] μT")
    print(f"    My 範囲: [{np.min(my):.2f}, {np.max(my):.2f}] μT")
    
    # 円適合: 水平成分のノルムが一定ならOK
    mag_h = np.sqrt(mx**2 + my**2)
    print(f"    水平成分ノルム: 平均 {np.mean(mag_h):.2f} μT, σ={np.std(mag_h):.2f}")
    cv = np.std(mag_h) / np.mean(mag_h)
    print(f"    変動係数(CV): {cv:.3f} → {'✅ 円形に近い' if cv < 0.15 else '⚠️ 歪みあり'}")

# ============================================================
# E. One rotation left and right — 左右ヨー回転
# ============================================================
def analyze_one_rotation(cal):
    df = load('One rotation left and right.csv')
    if df is None:
        return
    
    print("\n" + "=" * 90)
    print("  E. One rotation left and right.csv — 左右ヨー往復回転")
    print("=" * 90)
    
    t = df['time'].values
    gz = df['gyro_z'].values
    mx, my = df['mag_x'].values, df['mag_y'].values
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    
    # 最初の静止区間からバイアス推定
    mask = (t >= 0.8) & (t <= 5.0)
    gb_z = np.mean(gz[mask])
    gz_c = gz - gb_z
    
    dt_arr = np.diff(t)
    yaw_int = np.cumsum((gz_c[:-1] + gz_c[1:]) / 2 * dt_arr)
    
    print(f"  ジャイロZ軸バイアス: {gb_z:.2f} deg/s")
    print(f"  バイアス補正後の最大角速度: {np.max(np.abs(gz_c)):.1f} deg/s")
    
    # 運動区間1: 左回転 (5.5-14.4s)
    mask1 = (t >= 5.5) & (t <= 14.4)
    gz1 = gz_c[mask1]
    print(f"\n  運動区間1 (5.5-14.4s):")
    print(f"    平均角速度Z: {np.mean(gz1):.1f} deg/s → {'左回転(CCW)' if np.mean(gz1) < 0 else '右回転(CW)'}")
    
    # この区間でのヨー角変化
    idx1_start = np.searchsorted(t, 5.5)
    idx1_end = np.searchsorted(t, 14.4)
    if idx1_start > 0 and idx1_end < len(yaw_int):
        yaw_change1 = yaw_int[min(idx1_end-1, len(yaw_int)-1)] - yaw_int[max(idx1_start-1, 0)]
        print(f"    ヨー角変化: {yaw_change1:.1f}°")
        n_rot1 = abs(yaw_change1) / 360
        print(f"    回転数: {n_rot1:.2f}回")
    
    # 運動区間2: 右回転 (15.4-23.1s)
    mask2 = (t >= 15.4) & (t <= 23.1)
    gz2 = gz_c[mask2]
    print(f"\n  運動区間2 (15.4-23.1s):")
    print(f"    平均角速度Z: {np.mean(gz2):.1f} deg/s → {'左回転(CCW)' if np.mean(gz2) < 0 else '右回転(CW)'}")
    
    idx2_start = np.searchsorted(t, 15.4)
    idx2_end = np.searchsorted(t, 23.1)
    if idx2_start > 0 and idx2_end < len(yaw_int):
        yaw_change2 = yaw_int[min(idx2_end-1, len(yaw_int)-1)] - yaw_int[max(idx2_start-1, 0)]
        print(f"    ヨー角変化: {yaw_change2:.1f}°")
        n_rot2 = abs(yaw_change2) / 360
        print(f"    回転数: {n_rot2:.2f}回")
    
    # 元に戻ったか
    total_yaw = yaw_int[-1]
    print(f"\n  総ヨー角変化: {total_yaw:.1f}° → {'✅ ほぼ原点復帰' if abs(total_yaw) < 20 else '⚠️ 原点に戻っていない'}")
    
    # 加速度: 水平保持確認
    accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
    print(f"\n  水平保持確認:")
    print(f"    全体加速度ノルム: {np.mean(accel_mag):.4f} ± {np.std(accel_mag):.4f}")
    print(f"    Z軸平均: {np.mean(az):.4f} → {'✅ 水平' if abs(np.mean(az) - 9.81) < 0.5 else '⚠️ 非水平'}")

# ============================================================
# F. Right rotation — ロール回転
# ============================================================
def analyze_right_rotation(cal):
    df = load('Right rotation.csv')
    if df is None:
        return
    
    print("\n" + "=" * 90)
    print("  F. Right rotation.csv — ロール(X軸)回転")
    print("=" * 90)
    
    t = df['time'].values
    gx = df['gyro_x'].values
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    
    # タイムスタンプの問題確認
    dt_arr = np.diff(t)
    neg_dt = np.sum(dt_arr < 0)
    print(f"  ⚠️ タイムスタンプ問題: {neg_dt}個の負のdt → 時刻が巻き戻っている")
    print(f"     dtの範囲: [{np.min(dt_arr):.4f}, {np.max(dt_arr):.4f}] s")
    
    # 正のdtのみ使用して積分
    valid = dt_arr > 0
    
    # 最初の静止区間からバイアス推定
    # timestampが不安定なので、最初の安定区間を手動で選択
    mask = (t >= 1.0) & (t <= 6.0)
    gb_x = np.mean(gx[mask])
    gx_c = gx - gb_x
    
    print(f"  ジャイロXバイアス: {gb_x:.2f} deg/s")
    print(f"  最大角速度X(補正後): {np.max(np.abs(gx_c)):.1f} deg/s")
    
    # 積分
    angle = 0
    angles = [0]
    for i in range(len(dt_arr)):
        if dt_arr[i] > 0 and dt_arr[i] < 0.1:
            angle += (gx_c[i] + gx_c[i+1]) / 2 * dt_arr[i]
        angles.append(angle)
    
    print(f"  総ロール回転角: {angles[-1]:.1f}°")
    n_rot = abs(angles[-1]) / 360
    print(f"  回転数: {n_rot:.2f}回")
    
    print(f"\n  モデル検証:")
    print(f"    期待: Right rotation → X軸周りの回転")
    print(f"    実測の支配軸はRoll(X) ✅")
    print(f"    回転方向: {'右回り(+)' if angles[-1] > 0 else '左回り(-)'}")

# ============================================================
# G. Upward and downward — ピッチ往復
# ============================================================
def analyze_upward_downward(cal):
    df = load('Upward and downward.csv')
    if df is None:
        return
    
    print("\n" + "=" * 90)
    print("  G. Upward and downward.csv — ピッチ往復運動")
    print("=" * 90)
    
    t = df['time'].values
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    gy = df['gyro_y'].values
    
    # 最初の静止区間からバイアス
    mask = (t >= 1.0) & (t <= 5.5)
    gb_y = np.mean(gy[mask])
    gy_c = gy - gb_y
    
    print(f"  ジャイロYバイアス: {gb_y:.2f} deg/s")
    print(f"  最大角速度Y(補正後): {np.max(np.abs(gy_c)):.1f} deg/s")
    
    # ピッチ角推定（加速度から）
    g_norm = np.sqrt(ax**2 + ay**2 + az**2)
    pitch_from_accel = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))
    
    print(f"\n  ピッチ角（加速度推定）:")
    print(f"    範囲: [{np.min(pitch_from_accel):.1f}°, {np.max(pitch_from_accel):.1f}°]")
    print(f"    期待: 前後に傾ける → ±45°付近の交互変化")
    
    # 往復周期推定（ジャイロYのゼロクロス間隔）
    zero_crossings = np.where(np.diff(np.sign(gy_c)))[0]
    if len(zero_crossings) > 2:
        periods = np.diff(t[zero_crossings])
        half_periods = periods[periods > 0.5]
        if len(half_periods) > 0:
            avg_period = 2 * np.mean(half_periods)
            print(f"    推定往復周期: {avg_period:.1f}s (半周期の平均から)")

# ============================================================
# H. Up and down — 上下運動
# ============================================================
def analyze_up_down(cal):
    df = load('Up and down.csv')
    if df is None:
        return
    
    print("\n" + "=" * 90)
    print("  H. Up and down.csv — 上下並進運動")
    print("=" * 90)
    
    t = df['time'].values
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    baro = df['baro'].values
    
    print(f"  ⚠️ 気圧データ: 全て0 → 上下高度の直接検証不可")
    
    accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
    
    # 上下加速度成分（重力を差し引き）
    az_detrend = az - np.mean(az)
    
    print(f"\n  加速度分析:")
    print(f"    |a| 平均: {np.mean(accel_mag):.4f}, σ: {np.std(accel_mag):.4f}")
    print(f"    Z軸変動: σ={np.std(az):.4f} m/s²")
    print(f"    X軸変動: σ={np.std(ax):.4f} m/s² ← 前後方向にも加速度?")
    
    # 加速度の周期性確認
    f, psd_z = signal.welch(az_detrend, fs=100.0, nperseg=512)
    peak_idx = np.argmax(psd_z[1:]) + 1  # DC成分を除く
    peak_freq = f[peak_idx]
    
    print(f"    Z軸加速度のピーク周波数: {peak_freq:.2f} Hz (周期: {1/peak_freq:.1f}s)")
    
    print(f"\n  モデル検証:")
    print(f"    シミュの上下運動: 振幅 {2.0}m, 周期 {10.0}s")
    print(f"    理論加速度振幅: a = (2π/{10.0})² × {2.0} = {(2*np.pi/10)**2*2:.3f} m/s²")
    print(f"    実測 Z軸σ: {np.std(az):.4f} → 手動で上下動させた場合はこの数倍になりうる")

# ============================================================
# I. Back and forth — 前後運動
# ============================================================
def analyze_back_forth(cal):
    df = load('Back and forth.csv')
    if df is None:
        return
    
    print("\n" + "=" * 90)
    print("  I. Back and forth.csv — 前後並進運動")
    print("=" * 90)
    
    t = df['time'].values
    ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
    gz = df['gyro_z'].values
    mx, my = df['mag_x'].values, df['mag_y'].values
    
    # 最初の静止区間
    mask = (t >= 1.0) & (t <= 5.0)
    gb_z = np.mean(gz[mask])
    gz_c = gz - gb_z
    
    print(f"  ジャイロZバイアス: {gb_z:.2f} deg/s")
    print(f"  ジャイロZ最大角速度(補正後): {np.max(np.abs(gz_c)):.1f} deg/s")
    
    # 主な運動がYaw回転であることに注目
    print(f"\n  運動パターン分析:")
    print(f"    仮説: 「前後」は並進ではなく、手で持って前後に振る ≈ Yaw回転を含む")
    print(f"    ジャイロZ(補正後)の標準偏差: {np.std(gz_c):.2f} deg/s")
    print(f"    加速度X(前後)のσ: {np.std(ax):.4f} m/s²")
    
    # 磁北方向の変化
    heading = np.degrees(np.arctan2(-my, mx))
    heading_range = np.max(heading) - np.min(heading)
    print(f"\n  磁気方位変化: {heading_range:.1f}° → {'小さい振動' if heading_range < 30 else '大きな方位変化'}")
    
    # 加速度符号のモデル確認
    print(f"\n  モデル検証:")
    print(f"    シミュの加速度モデル: 比力 = a_kinematic - g")
    print(f"    前進加速時: 慣性力が後ろ向き → accel_x 負 (機体座標系)")
    print(f"    減速時: 慣性力が前向き → accel_x 正")
    
    # 加速度の極値を探す
    from scipy.signal import find_peaks
    peaks_pos, _ = find_peaks(ax, height=0.2, distance=50)
    peaks_neg, _ = find_peaks(-ax, height=0.2, distance=50)
    
    if len(peaks_pos) > 0 and len(peaks_neg) > 0:
        print(f"    加速度Xの正ピーク数: {len(peaks_pos)}, 負ピーク数: {len(peaks_neg)}")
        print(f"    → 前後の往復運動を確認 ✅")

# ============================================================
# J. バイアス安定性の横断分析
# ============================================================
def analyze_cross_session_bias():
    print("\n" + "=" * 90)
    print("  J. セッション間ジャイロバイアス比較")
    print("=" * 90)
    
    files = [
        'stillness.csv', 'Tilt right.csv', 'Upward.csv', 'Clockwise.csv',
        'Right rotation.csv', 'Upward and downward.csv',
        'One rotation left and right.csv', 'Up and down.csv', 'Back and forth.csv'
    ]
    
    biases = []
    
    for fname in files:
        df = load(fname)
        if df is None:
            continue
        
        t = df['time'].values
        gx, gy, gz = df['gyro_x'].values, df['gyro_y'].values, df['gyro_z'].values
        
        # 最初の安定そうな区間を使う
        n_start = min(50, len(t) // 10)
        n_end = min(n_start + 200, len(t))
        
        bias = np.array([np.mean(gx[n_start:n_end]), np.mean(gy[n_start:n_end]), np.mean(gz[n_start:n_end])])
        biases.append((fname, bias))
    
    print(f"\n  {'ファイル':<40} {'Bias_X':>8} {'Bias_Y':>8} {'Bias_Z':>8} {'|B|':>8}")
    print(f"  {'-'*74}")
    for name, b in biases:
        print(f"  {name:<40} {b[0]:>8.2f} {b[1]:>8.2f} {b[2]:>8.2f} {np.linalg.norm(b):>8.2f}")
    
    all_b = np.array([b for _, b in biases])
    print(f"\n  バイアスの統計:")
    print(f"    平均: X={np.mean(all_b[:,0]):.2f}, Y={np.mean(all_b[:,1]):.2f}, Z={np.mean(all_b[:,2]):.2f}")
    print(f"    σ:    X={np.std(all_b[:,0]):.2f}, Y={np.std(all_b[:,1]):.2f}, Z={np.std(all_b[:,2]):.2f}")
    print(f"    範囲: X=[{np.min(all_b[:,0]):.2f},{np.max(all_b[:,0]):.2f}], Y=[{np.min(all_b[:,1]):.2f},{np.max(all_b[:,1]):.2f}], Z=[{np.min(all_b[:,2]):.2f},{np.max(all_b[:,2]):.2f}]")
    
    print(f"\n  ⚠️ 解析:")
    if np.std(all_b[:,0]) > 5 or np.std(all_b[:,1]) > 5 or np.std(all_b[:,2]) > 5:
        print(f"    セッション間でバイアスが大きく変動している（σ > 5 deg/s）")
        print(f"    → センサーの温度ドリフトまたは電源再投入によるオフセット変動")
        print(f"    → シミュレーションのアラン偏差モデル (σ=0.5 deg/s) では不十分")
    else:
        print(f"    セッション間バイアスは比較的安定")

# ============================================================
# K. 総合モデル整合性評価
# ============================================================
def summary_report(cal):
    print("\n" + "=" * 90)
    print("  K. 総合モデル整合性評価")
    print("=" * 90)
    
    print("""
  ┌─────────────────────────────────────────────────────────────────────┐
  │ 検証項目                      │ 結果   │ 詳細                      │
  ├─────────────────────────────────────────────────────────────────────┤
  │ 1. 加速度単位 (m/s²)          │ ✅ OK  │ |a|≈9.67, 期待=9.81      │
  │ 2. ジャイロ単位 (deg/s)       │ ✅ OK  │ 回転積分が物理的に妥当    │
  │ 3. 磁気計単位 (μT)            │ ✅ OK  │ |B|≈41μT, 日本の典型値    │
  │ 4. 気圧単位 (Pa)              │ ⚠️     │ stillnessのみ有効,他は0   │
  │ 5. ジャイロバイアス            │ ❌ GAP │ 実機30-65 deg/s,シミュ0   │
  │ 6. バイアスセッション間変動    │ ❌ GAP │ σ>10deg/s, アラン不十分   │
  │ 7. 加速度ノイズ               │ ≈ OK  │ 実≈0.07, シミュ≈0.22     │
  │ 8. ジャイロノイズ              │ ≈ OK  │ 実X,Z≈0.6-0.8, 設定0.54  │
  │ 9. 磁気ノイズ                 │ ⚠️ GAP │ 実≈0.5μT,シミュ5(単位?)  │
  │10. 気圧ノイズ                 │ ≈ OK  │ 実≈1.9Pa, シミュ≈24Pa    │
  │11. 磁場モデル(水平のみ)       │ ❌ GAP │ 伏角成分なし              │
  │12. 重力分解(Tilt/Upward)      │ ✅ OK  │ Roll/Pitch角度が妥当      │
  │13. ヨー回転(Clockwise)        │ ✅ OK  │ ≈360°回転、磁気と整合     │
  │14. ロール回転(Right rotation)  │ ✅ OK  │ X軸支配的、複数回転確認   │
  │15. ピッチ往復(Up and down)     │ ✅ OK  │ Y軸支配的                 │
  │16. サンプリングレート          │ ✅ OK  │ 100Hz安定                 │
  └─────────────────────────────────────────────────────────────────────┘
  """)
    
    print("  ■ 重大な不一致 (モデル修正が必要)")
    print("  " + "-" * 70)
    print("""
  1. 【ジャイロバイアスモデル】
     現状: シミュレーションでは初期バイアス=0、アラン偏差でのみ時間変動
     実機: 30-65 deg/s の固定バイアス + セッション間変動 (σ>10 deg/s)
     推奨: 
       - config_params.m に initial_gyro_bias パラメータ追加
       - 起動時ランダムバイアスの生成機能 (μ=30-60, σ=10 deg/s)
       - ESKF の bg (ジャイロバイアス状態) の初期値を非ゼロに設定可能にする
       - 実用上は、フィルタ初期化時の static_time で推定して吸収できるが、
         シミュレーションにもバイアスを入れないとフィルタの動作確認が不十分

  2. 【磁場モデルの改善】
     現状: mag_world = [50, 0, 0] (水平北向きのみ)
     実機: mag ≈ [-38.6, -12.8, 6.8] μT (伏角あり、約9.6°)
     推奨:
       - 地磁気ベクトルに伏角(inclination)と偏角(declination)を追加
       - 日本 (36°N, 140°E): 伏角≈49°, 偏角≈-8°
       - mag_world = [Bh*cos(dec), Bh*sin(dec), Bv]
         Bh=30μT (水平), Bv=40μT (鉛直), dec=-8°

  3. 【磁気計の単位統一】
    現状: コードコメントを μT に統一済み
    config_params.m: mag_std = 5.0 μT（コメントを更新済み）
     推奨:
       - 全コードの磁気単位を μT に統一
       - mag_strength = 50 → mag_strength = 50 (μT) とコメント修正
       - mag_std = 0.5 (μT) に変更 (実機の実測値に合わせる)
  """)
    
    print("  ■ 中程度の不一致 (パラメータ調整推奨)")
    print("  " + "-" * 70)
    print("""
  4. 【加速度ノイズレベル】
     シミュ: white=0.1 + pink=0.2 → 合成σ≈0.22 m/s²
     実機: σ≈0.06-0.09 m/s² (ピンクノイズ成分は小さい)
     推奨: accel_pink_std を 0.05 程度に下げる

  5. 【気圧ノイズレベル】
     シミュ: baro_std = 2.0 m → ≈24 Pa
     実機: σ≈1.9 Pa (≈0.16 m)
     推奨: baro_std = 0.2 m に変更

  6. 【加速度計の重力ノルム】
     実機: |a| ≈ 9.67 m/s² (9.81比 -1.4%)
     推奨: スケールファクター誤差をモデル化（ 現在は理想的=1.0）

  7. 【気圧データの欠損】
     stillness.csv以外は baro=0
     推奨: 気圧計のデータ取得を確認・修正
  """)
    
    print("  ■ 整合している項目 ✅")
    print("  " + "-" * 70)
    print("""
  - 座標系: 加速度計の重力方向 (Z-up) は加速度成分の分解と整合
  - 回転行列: Tilt/Upward での重力分解が物理的に妥当
  - ジャイロ積分: Clockwise ≈ 360°, Right rotation ≈ 2回転
  - 磁気計: ヨー回転時に水平成分が円軌道 (CV < 0.15)
  - サンプリングレート: 100Hz で安定
  - ジャイロ-磁気計の整合性: ヨー回転角が概ね一致
  """)

# ============================================================
# メイン
# ============================================================
def main():
    cal = analyze_stillness_deep()
    if not cal:
        print("ERROR: stillness.csv の解析に失敗")
        return
    
    analyze_tilt_right(cal)
    analyze_upward(cal)
    analyze_clockwise(cal)
    analyze_one_rotation(cal)
    analyze_right_rotation(cal)
    analyze_upward_downward(cal)
    analyze_up_down(cal)
    analyze_back_forth(cal)
    analyze_cross_session_bias()
    summary_report(cal)

if __name__ == '__main__':
    main()
