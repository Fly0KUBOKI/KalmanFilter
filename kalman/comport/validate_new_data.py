#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
新しいセンサーデータの検証スクリプト
重力加速度の正確性とジャイロバイアスの軽減を確認
"""

import pandas as pd
import numpy as np
from pathlib import Path

CSV_PATH = Path(r"C:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\comport\sensor_data_20260209_054222.csv")

def analyze_new_data():
    print("=" * 90)
    print("  新しいセンサーデータ検証")
    print("=" * 90)
    
    df = pd.read_csv(CSV_PATH)
    # カラム名のストリップ
    df.columns = df.columns.str.strip()
    t = df['time'].values
    ax = df['accel_x'].values
    ay = df['accel_y'].values
    az = df['accel_z'].values
    gx = df['gyro_x'].values
    gy = df['gyro_y'].values
    gz = df['gyro_z'].values
    mx = df['mag_x'].values
    my = df['mag_y'].values
    mz = df['mag_z'].values
    baro = df['baro'].values
    
    # 概要
    print(f"\n  ファイル: {CSV_PATH.name}")
    print(f"  データ点数: {len(df)}")
    print(f"  期間: {t[0]:.3f} - {t[-1]:.3f} s (Duration: {t[-1]-t[0]:.3f} s)")
    
    # ============================================================
    # 1. 重力加速度の検証
    # ============================================================
    print(f"\n  1️⃣  重力加速度検証")
    print("  " + "-" * 70)
    
    accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
    
    print(f"    加速度Z軸: 平均={np.mean(az):.6f}, σ={np.std(az):.6f} m/s²")
    print(f"    加速度ノルム: 平均={np.mean(accel_mag):.6f}, σ={np.std(accel_mag):.6f} m/s²")
    
    # 重力ノルムの期待値との比較
    g_expected = 9.80665
    g_measured = np.mean(accel_mag)
    error_g = g_measured - g_expected
    error_pct = error_g / g_expected * 100
    
    print(f"\n    期待値 (9.80665 m/s²) との比較:")
    print(f"      測定値: {g_measured:.6f} m/s²")
    print(f"      誤差: {error_g:.6f} m/s² ({error_pct:+.3f}%)")
    
    if abs(error_pct) < 2.0:
        print(f"      ✅ 優秀 (誤差 < 2%)")
    elif abs(error_pct) < 5.0:
        print(f"      👍 良好 (誤差 < 5%)")
    else:
        print(f"      ⚠️要注意 (誤差 >= 5%)")
    
    # ノイズレベル
    print(f"\n    ノイズレベル (1σ):")
    accel_noise = np.array([np.std(ax), np.std(ay), np.std(az)])
    print(f"      X軸: {accel_noise[0]:.6f} m/s²")
    print(f"      Y軸: {accel_noise[1]:.6f} m/s²")
    print(f"      Z軸: {accel_noise[2]:.6f} m/s²")
    print(f"      平均: {np.mean(accel_noise):.6f} m/s²")
    
    # 重力ベクトル方向
    g_vec = np.array([np.mean(ax), np.mean(ay), np.mean(az)])
    g_norm = g_vec / np.linalg.norm(g_vec)
    roll = np.degrees(np.arctan2(g_norm[1], g_norm[2]))
    pitch = np.degrees(np.arctan2(-g_norm[0], np.sqrt(g_norm[1]**2 + g_norm[2]**2)))
    
    print(f"\n    推定初期姿勢:")
    print(f"      Roll: {roll:.2f}° (期待: ≈0°)")
    print(f"      Pitch: {pitch:.2f}° (期待: ≈0°)")
    
    if abs(roll) < 2 and abs(pitch) < 2:
        print(f"      ✅ デバイスが水平に保たれている")
    else:
        print(f"      ⚠️ デバイスが傾いている")
    
    # ============================================================
    # 2. ジャイロバイアスの検証
    # ============================================================
    print(f"\n  2️⃣  ジャイロバイアス検証")
    print("  " + "-" * 70)
    
    gyro_bias = np.array([np.mean(gx), np.mean(gy), np.mean(gz)])
    gyro_noise = np.array([np.std(gx), np.std(gy), np.std(gz)])
    gyro_bias_mag = np.linalg.norm(gyro_bias)
    
    print(f"    バイアス (平均値):")
    print(f"      X軸: {gyro_bias[0]:.6f} deg/s")
    print(f"      Y軸: {gyro_bias[1]:.6f} deg/s")
    print(f"      Z軸: {gyro_bias[2]:.6f} deg/s")
    print(f"      ノルム: {gyro_bias_mag:.6f} deg/s")
    
    print(f"\n    ノイズレベル (1σ):")
    print(f"      X軸: {gyro_noise[0]:.6f} deg/s")
    print(f"      Y軸: {gyro_noise[1]:.6f} deg/s")
    print(f"      Z軸: {gyro_noise[2]:.6f} deg/s")
    print(f"      平均: {np.mean(gyro_noise):.6f} deg/s")
    
    # 以前のデータとの比較
    print(f"\n    以前のデータ (stillness.csv) との比較:")
    old_bias = 56.60  # deg/s
    old_noise = 0.47  # deg/s
    
    print(f"      以前のバイアスノルム: {old_bias:.2f} deg/s")
    print(f"      新しいバイアスノルム: {gyro_bias_mag:.6f} deg/s")
    reduction = (old_bias - gyro_bias_mag) / old_bias * 100
    print(f"      削減率: {reduction:.1f}% ✅")
    
    print(f"\n      以前のノイズ平均: {old_noise:.6f} deg/s")
    print(f"      新しいノイズ平均: {np.mean(gyro_noise):.6f} deg/s")
    
    if gyro_bias_mag < 0.1:
        print(f"      ✅ バイアスが極めて小さい (< 0.1 deg/s) — 優秀")
    elif gyro_bias_mag < 1.0:
        print(f"      👍 バイアスが許容範囲内 (< 1.0 deg/s)")
    elif gyro_bias_mag < 10.0:
        print(f"      ⚠️ バイアスが中程度 (1-10 deg/s)")
    else:
        print(f"      ❌ バイアスが大きい (> 10 deg/s)")
    
    # ============================================================
    # 3. 磁気計の検証
    # ============================================================
    print(f"\n  3️⃣  磁気計検証")
    print("  " + "-" * 70)
    
    mag_vec = np.array([np.mean(mx), np.mean(my), np.mean(mz)])
    mag_mag = np.linalg.norm(mag_vec)
    mag_noise = np.array([np.std(mx), np.std(my), np.std(mz)])
    
    print(f"    磁場ベクトル:")
    print(f"      X軸: {mag_vec[0]:.2f} μT")
    print(f"      Y軸: {mag_vec[1]:.2f} μT")
    print(f"      Z軸: {mag_vec[2]:.2f} μT")
    print(f"      ノルム: {mag_mag:.2f} μT (期待: 45-50 μT)")
    
    print(f"\n    ノイズレベル (1σ):")
    print(f"      X軸: {mag_noise[0]:.4f} μT")
    print(f"      Y軸: {mag_noise[1]:.4f} μT")
    print(f"      Z軸: {mag_noise[2]:.4f} μT")
    
    # ============================================================
    # 4. 気圧計の検証
    # ============================================================
    print(f"\n  4️⃣  気圧計検証")
    print("  " + "-" * 70)
    
    print(f"    気圧:")
    print(f"      平均: {np.mean(baro):.2f} Pa")
    print(f"      ノイズ: {np.std(baro):.2f} Pa")
    alt = (101325 - np.mean(baro)) / 12.0
    print(f"      推定高度: {alt:.1f} m")
    
    if np.std(baro) > 0:
        print(f"      ✅ 気圧データが記録されている")
    else:
        print(f"      ⚠️ 気圧データが全て同じ値 (センサーが動作していない可能性)")
    
    # ============================================================
    # 5. 総合評価
    # ============================================================
    print(f"\n  5️⃣  総合評価")
    print("  " + "-" * 70)
    
    scores = []
    
    # 重力加速度スコア
    if abs(error_pct) < 1.0:
        score_g = "🟢 優秀"
        scores.append(1)
    elif abs(error_pct) < 2.0:
        score_g = "🟢 良好"
        scores.append(1)
    elif abs(error_pct) < 5.0:
        score_g = "🟡 許容範囲"
        scores.append(2)
    else:
        score_g = "🔴 要改善"
        scores.append(3)
    
    # ジャイロバイアススコア
    if gyro_bias_mag < 0.1:
        score_gyro = "🟢 優秀"
        scores.append(1)
    elif gyro_bias_mag < 1.0:
        score_gyro = "🟢 良好"
        scores.append(1)
    elif gyro_bias_mag < 5.0:
        score_gyro = "🟡 許容範囲"
        scores.append(2)
    else:
        score_gyro = "🔴 要改善"
        scores.append(3)
    
    # 加速度ノイズスコア
    if np.mean(accel_noise) < 0.1:
        score_a_noise = "🟢 優秀"
        scores.append(1)
    elif np.mean(accel_noise) < 0.2:
        score_a_noise = "🟢 良好"
        scores.append(1)
    elif np.mean(accel_noise) < 0.5:
        score_a_noise = "🟡 許容範囲"
        scores.append(2)
    else:
        score_a_noise = "🔴 要改善"
        scores.append(3)
    
    # ジャイロノイズスコア
    if np.mean(gyro_noise) < 0.3:
        score_g_noise = "🟢 優秀"
        scores.append(1)
    elif np.mean(gyro_noise) < 1.0:
        score_g_noise = "🟢 良好"
        scores.append(1)
    else:
        score_g_noise = "🟡 許容範囲"
        scores.append(2)
    
    print(f"    重力加速度精度: {score_g}")
    print(f"    ジャイロバイアス: {score_gyro}")
    print(f"    加速度ノイズ: {score_a_noise}")
    print(f"    ジャイロノイズ: {score_g_noise}")
    
    avg_score = np.mean(scores)
    if avg_score < 1.5:
        overall = "🟢 PASS — シミュレーション検証に十分"
    elif avg_score < 2.0:
        overall = "🟡 CAUTION — 許容範囲だがモデル改善の余地あり"
    else:
        overall = "🔴 FAIL — 要改善"
    
    print(f"\n    総合評価: {overall}")
    
    # ============================================================
    print(f"\n  6️⃣  詳細メトリクス")
    print("  " + "-" * 70)
    
    print(f"    加速度   [m/s²]: [{np.mean(ax):.4f}, {np.mean(ay):.4f}, {np.mean(az):.4f}]")
    print(f"    ジャイロ [deg/s]: [{gyro_bias[0]:.4f}, {gyro_bias[1]:.4f}, {gyro_bias[2]:.4f}]")
    print(f"    磁気計   [μT]:    [{mag_vec[0]:.2f}, {mag_vec[1]:.2f}, {mag_vec[2]:.2f}]")
    print(f"    気圧     [Pa]:    {np.mean(baro):.2f}")
    
    # 時系列の安定性確認
    n_chunks = 5
    chunk_size = len(ax) // n_chunks
    
    print(f"\n  7️⃣  時間方向の安定性 ({n_chunks}分割)")
    print("  " + "-" * 70)
    
    for chunk in range(n_chunks):
        s = chunk * chunk_size
        e = (chunk + 1) * chunk_size if chunk < n_chunks - 1 else len(ax)
        
        chunk_bias_norm = np.linalg.norm([np.mean(gx[s:e]), np.mean(gy[s:e]), np.mean(gz[s:e])])
        chunk_g = np.mean(np.sqrt(ax[s:e]**2 + ay[s:e]**2 + az[s:e]**2))
        
        print(f"    区間{chunk+1} (t={t[s]:.2f}-{t[e-1]:.2f}s): |g|={chunk_g:.4f}, gyro_bias_mag={chunk_bias_norm:.4f}")

if __name__ == '__main__':
    analyze_new_data()
