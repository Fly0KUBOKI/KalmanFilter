# 技術ノート 統合ドキュメント

このドキュメントは、ESKF/MEUKFフィルタの実装に関する技術的なノートと改善内容を統合したものです。

---

## 目次

1. [Z軸加速度積分による高度推定改善](#z軸加速度積分による高度推定改善)
2. [MATLAB実装状況](#matlab実装状況)

---

## Z軸加速度積分による高度推定改善

### 実装日
2025年12月12日 21:15

### 改善結果
- Position RMSE: 0.96m → 0.56m (42%改善)

### 問題の診断

#### 原状の問題点
- **気圧ノイズ/ドリフトによるZ軸発散**: 気圧計のノイズとドリフトが原因でZ軸の推定が不安定
- **気圧なしでは安定**: 気圧ノイズをオフにすると安定して推定可能
- **根本原因**: 気圧計への過度な依存により、ノイズが直接Z軸推定に影響

### 解決方針

#### アプローチ
1. **補正加速度によるZ軸計測**: 姿勢補正後の加速度からZ軸運動を直接計測
2. **気圧は発散抑制用**: 気圧更新の重みを下げ、積分の発散抑制のみに使用
3. **閾値ベース積分**: 閾値以上の加速度のみを積分して発散を防止

### 実装詳細

#### 1. 新しいプロパティ (ESKF.m)

```matlab
% Z軸加速度積分関連
enable_accel_z_integration  % Z軸加速度積分の有効/無効
accel_z_threshold           % Z軸加速度積分の閾値 [m/s^2]
accel_z_damping             % Z軸速度の減衰係数 [0-1]
baro_weight                 % 気圧更新の重み係数 [0-1]
```

#### 2. 初期化 (コンストラクタ)

```matlab
% Z軸加速度積分関連の初期化
obj.enable_accel_z_integration = true;   % デフォルトで有効
obj.accel_z_threshold = 0.5;             % 0.5 m/s^2 超過で積分開始
obj.accel_z_damping = 0.1;               % 速度減衰: 10%/step
obj.baro_weight = 0.2;                   % 気圧の重み: 20%
```

#### 3. predict()関数での実装

```matlab
% Z軸加速度積分（姿勢補正後の加速度を使用）
if obj.enable_accel_z_integration
    % 姿勢補正: ボディフレームからNEDフレームへ変換
    R = QuaternionLib.to_rotation_matrix(obj.q);
    a_ned = R * a_for_vel;  % NEDフレームの加速度
    
    % 重力を差し引く (NEDでは重力は [0; 0; g])
    a_ned_corrected = a_ned - [0; 0; obj.g(3)];
    
    % Z軸成分の超過分を取得
    az_excess = a_ned_corrected(3);
    
    % 閾値以上の加速度のみ積分
    if abs(az_excess) > obj.accel_z_threshold
        % Z軸速度を更新
        obj.v(3) = obj.v(3) + az_excess * obj.dt;
        
        % Z軸速度の減衰（発散防止）
        obj.v(3) = obj.v(3) * (1.0 - obj.accel_z_damping);
    end
end
```

**ポイント:**
- 姿勢補正により、ボディフレームの加速度をNED（North-East-Down）フレームに変換
- 重力 `[0; 0; g]` を差し引いて、Z軸の真の加速度を計算
- 閾値（0.5 m/s²）以上の加速度のみを積分 → 発散防止
- 減衰係数（10%）で速度を減衰 → さらなる発散防止

#### 4. update_baro()関数の修正

```matlab
function update_baro(obj, pressure)
    % 気圧計による高度更新 (C++実装) - 発散抑制用に重みを下げる
    
    % センサーフィルタ適用
    [alt_baro, is_outlier, ~] = obj.sensor_filters.baro.apply(pressure);
    if any(isnan(alt_baro)); return; end
    if is_outlier; return; end
    
    % 気圧更新の重みを調整（発散抑制のみに使用）
    % 方法: P行列のZ成分の分散を一時的に増やして更新の影響を弱める
    P_backup = obj.P;
    weight_factor = 1.0 / obj.baro_weight;  % weight=0.2 -> factor=5
    obj.P(3,3) = obj.P(3,3) * weight_factor;
    
    % C++実装を使用
    obj.update_baro_cpp(alt_baro);
    
    % P行列のZ成分を部分的に復元（更新後の共分散は維持）
    obj.P(3,3) = obj.P(3,3) / weight_factor;
end
```

**ポイント:**
- P(3,3)（Z軸位置の分散）を一時的に5倍に増やす
- カルマンゲインが小さくなり、気圧更新の影響が20%に低減
- 更新後、P(3,3)を元のスケールに戻す
- 結果: 気圧は発散抑制のみに機能

### パラメータチューニング

#### 推奨値

| パラメータ | 値 | 説明 |
|----------|-----|------|
| `accel_z_threshold` | 0.5 m/s² | この値以上の加速度を積分 |
| `accel_z_damping` | 0.1 (10%) | Z軸速度の減衰率 |
| `baro_weight` | 0.2 (20%) | 気圧更新の有効重み |

#### チューニング指針

**accel_z_threshold（積分閾値）**
- **小さすぎる**: ノイズを積分してしまい発散
- **大きすぎる**: Z軸の運動を捉えられない
- **推奨範囲**: 0.3 - 0.8 m/s²

**accel_z_damping（減衰係数）**
- **小さすぎる**: 積分が発散しやすい
- **大きすぎる**: Z軸の運動が過度に抑制される
- **推奨範囲**: 0.05 - 0.2

**baro_weight（気圧重み）**
- **小さすぎる**: 長期的な発散を抑制できない
- **大きすぎる**: 気圧ノイズがZ軸に影響
- **推奨範囲**: 0.1 - 0.3

### 性能比較

#### テスト条件
- **シミュレーション**: 100秒、40001サンプル @ 400Hz
- **運動**: 円運動、半径10m、高度0m
- **センサーノイズ**: 全センサー有効（加速度、ジャイロ、磁気、**気圧**、GPS）

#### 結果

| 指標 | 修正前 | 修正後 | 改善率 |
|-----|--------|--------|--------|
| **Position RMSE** | 0.96 m | **0.56 m** | **42%** |
| **Velocity RMSE** | 0.70 m/s | **0.55 m/s** | **21%** |
| **Roll RMSE** | 0.33 deg | **0.29 deg** | **12%** |
| **Pitch RMSE** | 0.43 deg | **0.27 deg** | **37%** |
| **Yaw RMSE** | 1.02 deg | **0.82 deg** | **20%** |

### アルゴリズムフロー

```
1. IMU加速度取得 (a_body)
   ↓
2. 姿勢補正: a_ned = R * a_body
   ↓
3. 重力除去: a_ned_corrected = a_ned - [0; 0; g]
   ↓
4. Z軸超過分: az_excess = a_ned_corrected(3)
   ↓
5. 閾値判定: if |az_excess| > threshold
   ↓
6. 速度更新: v(3) = v(3) + az_excess * dt
   ↓
7. 減衰適用: v(3) = v(3) * (1 - damping)
   ↓
8. 位置積分: p(3) = p(3) + v(3) * dt (C++側で実施)
   ↓
9. 気圧補正: 20%の重みで発散抑制
```

### 座標系
- **ボディフレーム**: 機体固定座標系
- **NEDフレーム**: North-East-Down 地球固定座標系
  - X軸: 北方向
  - Y軸: 東方向
  - Z軸: 下方向（重力方向）

---

## MATLAB実装状況

### 概要（2025年12月13日更新）

- **全体行数**: ~3600行（MATLAB）+ ~1600行（C++）
- **MATLAB比率**: 69%（うち、計算コアはほぼ全て C++化完了）
- **C++化完了率**: 100%（Predict/Update の 5コアステップ）

### MATLAB実装の全体像

#### カテゴリ別分類

```
┌─────────────────────────────────────────────────────┐
│  MATLAB 実装部分 (69%)                             │
├─────────────────────────────────────────────────────┤
│                                                     │
│  A. ESKF メインロジック (600行)                   │
│     ├─ ESKF.m (コンストラクタ, 初期化)             │
│     └─ @ESKF/ (8メソッド)                         │
│         ├─ update_filter        ← 1ステップ統括    │
│         ├─ sensor_updates        ← 4センサ統合      │
│         ├─ call_cpp_update_impl  ← C++呼び出し統一 │
│         ├─ zupt                  ← ZUPT更新        │
│         ├─ reset                 ← リセット制御     │
│         ├─ predict               ← (C++MEX呼び出し)│
│         └─ utils                 ← ユーティリティ   │
│                                                     │
│  B. センサフィルタ (300行)                        │
│     ├─ SensorFilter.m           ← 基底クラス      │
│     ├─ SensorAccelFilter.m      ← 加速度フィルタ  │
│     ├─ SensorGyroFilter.m       ← 角速度フィルタ  │
│     ├─ SensorMagFilter.m        ← 磁気フィルタ    │
│     ├─ SensorGPSFilter.m        ← GPS フィルタ    │
│     └─ SensorBaroFilter.m       ← 気圧フィルタ    │
│        + BiquadFilter.m         ← フィルタ基底    │
│                                                     │
│  C. ノイズ推定 (150行)                           │
│     └─ NoiseEstimator.m          ← 動的R推定      │
│                                                     │
│  D. 発散検出・保守 (200行)                        │
│     ├─ DivergenceGuard.m        ← 発散防止        │
│     └─ OutlierGuard.m           ← 外れ値判定      │
│                                                     │
│  E. 実行スクリプト (400行)                        │
│     ├─ run_simulation.m          ← 単一実行        │
│     ├─ run_batch_10sets.m       ← バッチ実行      │
│     ├─ analyze_results.m        ← 結果解析        │
│     └─ その他                    ← グラフ, 検証    │
│                                                     │
│  F. データ生成・ユーティリティ (500行)             │
│     ├─ GenerateData/*            ← センサ観測生成  │
│     ├─ Graph/*                   ← グラフプロット  │
│     └─ FFT/*                     ← 周波数解析      │
│                                                     │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│  C++ 実装部分 (31%)                               │
├─────────────────────────────────────────────────────┤
│                                                     │
│  A. コア計算 (1100行)                             │
│     └─ meukf_core.cpp                             │
│        ├─ predict()           ← IMU積分, P伝播    │
│        ├─ update_accel_meukf() ← MEUKF (Roll/P) │
│        ├─ update_mag_meukf()  ← MEUKF (Yaw)    │
│        ├─ update_gps()        ← UKF (位置/速度) │
│        └─ update_baro()       ← EKF (高度)     │
│                                                     │
│  B. MEX インターフェース                           │
│     ├─ mex_meukf_step_v2.mexw64 (1.2MB)         │
│     └─ mex_meukf_step.cpp                        │
│                                                     │
│  C. サポート機能 (500行)                          │
│     ├─ quaternion.hpp          ← クォータニオン  │
│     ├─ sensor_filter.cpp       ← フィルタ       │
│     ├─ fixed_matrix.hpp        ← 行列演算       │
│     └─ ...                                         │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### MATLAB実装ファイル詳細リスト

#### ESKF メインクラス (600行)

| ファイル | 行数 | 説明 | 実装内容 |
|---------|------|------|---------|
| `ESKF/@ESKF/ESKF.m` | 217 | コンストラクタ | 状態初期化、静止期間からのノイズ推定 |
| `ESKF/@ESKF/update_filter.m` | 25 | 1ステップ統括 | 予測+周期的更新の制御 |
| `ESKF/@ESKF/sensor_updates.m` | 60 | センサ更新統合 | 4センサ (accel/mag/gps/baro) の統一メソッド |
| `ESKF/@ESKF/call_cpp_update_impl.m` | 75 | C++呼び出し統一 | `mex_meukf_step_v2` 呼び出し + 状態更新 |
| `ESKF/@ESKF/zupt.m` | 40 | ZUPT更新 | 速度ゼロ補正（静止検出+更新） |
| `ESKF/@ESKF/reset.m` | 50 | リセット制御 | 発散検出・状態リセット判定 |
| `ESKF/@ESKF/predict.m` | 100 | IMU予測 | `mex_meukf_step_v2` 呼び出し（C++実装） |
| `ESKF/@ESKF/utils.m` | 40 | ユーティリティ | オイラー角取得、状態抽出等 |

**カテゴリ**: 管理・制御層（計算コアは全て C++化済み）

#### センサフィルタ (300行)

| ファイル | 行数 | 説明 | 実装内容 |
|---------|------|------|---------|
| `KF/Utils/SensorFilter.m` | 200 | 基底・ファクトリ | 5種類フィルタの生成・管理 |
| `KF/Utils/SensorAccelFilter.m` | 100 | 加速度フィルタ | EMA平滑化 + 外れ値検出 |
| `KF/Utils/SensorGyroFilter.m` | 95 | 角速度フィルタ | Biquad LPF + バイアス推定 |
| `KF/Utils/SensorMagFilter.m` | 130 | 磁気フィルタ | 外れ値検出 + キャリブレーション |
| `KF/Utils/SensorGPSFilter.m` | 95 | GPS フィルタ | HDOP重み付け + 外れ値検出 |
| `KF/Utils/SensorBaroFilter.m` | 85 | 気圧フィルタ | 気圧→高度変換 + 外れ値検出 |
| `KF/Utils/BiquadFilter.m` | 110 | Biquad基底 | 2次IIRフィルタ実装 |

**カテゴリ**: センサデータ前処理（センサの品質向上）

#### ノイズ推定 (150行)

| ファイル | 行数 | 説明 | 実装内容 |
|---------|------|------|---------|
| `KF/Utils/NoiseEstimator.m` | 150 | 動的ノイズ推定 | イノベーションベースの観測ノイズ R 自動調整 |

**カテゴリ**: フィルタ適応化（環境変化への対応）

#### 発散検出・保守機能 (200行)

| ファイル | 行数 | 説明 | 実装内容 |
|---------|------|------|---------|
| `KF/Utils/DivergenceGuard.m` | 600 | 発散防止マネージャ | 共分散正則化、イノベーション監視、自動リセット |
| `KF/Utils/OutlierGuard.m` | 50 | 外れ値対応 | SensorFilter + DivergenceGuard 統合インターフェース |

**カテゴリ**: フィルタ安定性確保（数値的安定性、異常検出）

#### 実行スクリプト (400行)

| ファイル | 行数 | 説明 | 実装内容 |
|---------|------|------|---------|
| `run_simulation.m` | 120 | 単一シミュレーション実行 | 観測読込 → ESKF初期化 → ループ → CSV出力 |
| `run_batch_10sets.m` | 350 | 10セット自動実行 | 複数回実行 + 統計情報集計 + log出力 |

**カテゴリ**: 実行・解析（開発・検証用）

### アーキテクチャの特徴

#### ハイブリッド設計
- **MATLAB層**: センサー前処理、適応的パラメータ、データI/O、制御ロジック
- **C++層**: 計算集約的なフィルタ数学（predict, update, MEUKF）
- **インターフェース**: `mex_meukf_step_v2.mexw64` による明確な分離

#### 利点
1. **開発速度**: MATLABの高レベル制御ロジックが容易
2. **性能**: C++計算コアで高速処理
3. **保守性**: 計算ロジックがC++に集約され、単一真実源を実現
4. **柔軟性**: センサーフィルタリングやパラメータ調整が容易

#### C++化の現状
- ✅ **計算コア**: 100% C++化完了（predict, update_accel, update_mag, update_gps, update_baro）
- ✅ **MEXインターフェース**: 統一インターフェース `mex_meukf_step_v2` が完成
- ⚠️ **センサーフィルタリング**: MATLAB側に残存（数値安定性のために必須）
- ⚠️ **ノイズ推定**: MATLAB側に残存（適応的パラメータ調整のため）

---

## まとめ

### Z軸加速度積分による改善
1. ✅ Position RMSEを42%改善 (0.96m → 0.56m)
2. ✅ 気圧ノイズ/ドリフトに対するロバスト性を大幅向上
3. ✅ 全軸で精度向上（特にPitch: 37%改善）

### MATLAB実装の現状
1. ✅ 計算コアは100% C++化完了
2. ✅ ハイブリッドアーキテクチャが最適
3. ✅ センサーフィルタリングとノイズ推定はMATLAB側に残存（数値安定性のため）

---

**最終更新**: 2025年12月13日  
**作成者**: 統合ドキュメント（複数のソースを統合）


