# Lib フォルダ構造と関数リファレンス

**更新日**: 2026年1月9日  
**対象**: `kalman/cpp/Lib/` 配下の全 37 ファイル、150+ 関数  
**目的**: Lib フォルダの構造、モジュール設計、全関数を一元管理

---

## 目次

1. [アーキテクチャ概観](#アーキテクチャ概観)
2. [7層レイヤー設計](#7層レイヤー設計)
3. [モジュール別関数リスト](#モジュール別関数リスト)
4. [重複・未使用検出](#重複未使用検出)
5. [依存関係グラフ](#依存関係グラフ)
6. [バイナリ最適化](#バイナリ最適化)

---

## アーキテクチャ概観

### 全体構成（最適化版）
```
kalman/cpp/Lib/
├── Common/             (11 files)
│   ├── inc/
│   │   ├── Math/              (math_utils, statistics, geometry)
│   │   ├── Sensor/            (sensor_filter, preprocessor, outlier)
│   │   └── *.hpp              (filter_mgmt, types)
│   └── src/
│
├── ESKF/              (11 files) ← 主フィルタ実装
│   ├── inc/           (core, state, runner, updates)
│   └── src/
│
├── MEUKF/             (5 files)  ← 副フィルタ実装
│   ├── inc/
│   └── src/
│
├── EKF/               (3 files)  ← テンプレート（未使用、削除予定）
├── UKF/               (3 files)  ← テンプレート（未使用、削除予定）
├── KF/                (2 files)  ← 基本実装（要検証）
├── Matrix/            (1 file)   ← 固定行列テンプレート
│   └── fixed_matrix.hpp
│
├── Quaternion/        (1 file)   ← 四元数演算
│   └── quaternion_functions.hpp
│
└── Sensor/            (7 files)  ← 重複フォルダ（Lib/Sensor と同じ）

❌ 削除済み:
├── Core/                        ← 古い wrapper（機能しない）
```

---

## バイナリ最適化

### 2026-01-13 最適化実施

**実施内容**:
1. ✅ `Lib/Core/` 削除（古い wrapper フォルダ）
2. ⏳ `Lib/Sensor/` 統一（Lib/Sensor を正とする方針）

**効果測定**:
| 項目 | 前 | 後 | 削減 | 率 |
|------|-----|-----|---------|-------|
| バイナリサイズ | 347 KB | 339 KB | 8 KB | 2.3% |
| ビルド時間 | N/A | N/A | 推定微小 | - |

**継続課題**:
- Lib/Sensor の削除または統合（include パス複雑化により保留）
- EKF/UKF テンプレート削除（未使用確認後）
- meukf_core.cpp 分割（1346行）

---

## 7層レイヤー設計

```
LAYER 4: ESKF/MEUKF実装（ホットパス）
  ├─ eskf_runner.hpp — double↔float変換、state積分、センサー更新委譲
  ├─ eskf_core.hpp — 予測/更新のアルゴリズム実装
  └─ meukf_core.cpp — 代替フィルタ（1346行、要分割候補）

LAYER 3: フィルタ更新・統計（センサー処理）
  ├─ sensor_filter.hpp — 外れ値検出、Mahalanobis距離（831行）
  ├─ sensor_preprocessor.hpp — IMUノイズ，GPS→ENU変換
  └─ Common/filter_mgmt.hpp — ZUPT，発散検出，共分散保証

LAYER 2: 基本フィルタテンプレート
  ├─ KF/kalman_filter_core.hpp — 汎用KFコア
  ├─ EKF/ekf_core.hpp — 非線形観測モデル
  └─ UKF/ukf_core.hpp — Unscented変換（実装途上）

LAYER 1: ユーティリティ（型変換、数学）
  ├─ Matrix/fixed_matrix.hpp — 固定サイズ行列テンプレート
  ├─ Quaternion/quaternion_functions.hpp — 四元数演算
  ├─ Math/math_utils.hpp — ベクトル・スキュー対称化
  └─ interface.hpp — MATLAB⟷C++のstruct定義（Version 2準拠）
```

---

## モジュール別関数リスト

### LAYER 1: 基盤（数学・行列・四元数）

#### Matrix/fixed_matrix.hpp
**役割**: 固定サイズ行列テンプレート実装（R×C、型パラメータ）  
**実装度**: ★★★ 完全実装（350行）  
**依存**: なし（完全独立）

| 関数名 | 説明 |
|--------|------|
| `operator+` | 行列加算 |
| `operator-` | 行列減算 |
| `operator*` | 行列乗算・スカラー乗算 |
| `transpose()` | 転置 |
| `inverse()` | 逆行列（ガウス・ジョルダン） |
| `cholesky(L)` | Cholesky分解（正定値性確認） |
| `Zero()` | ゼロ行列 |
| `Identity()` | 単位行列 |

---

#### Quaternion/quaternion_functions.hpp
**役割**: 四元数演算（q = [w, x, y, z]）  
**実装度**: ★★★ 完全実装（172行）  
**依存**: Matrix/fixed_matrix.hpp

| 関数名 | 説明 | 重要度 |
|--------|------|--------|
| `cquat::normalize_quat()` | 正規化（標準実装） | ★★★ |
| `multiply_quat()` | 四元数乗算 | ★★★ |
| `quat_to_rotm()` | 四元数→回転行列 | ★★★ |
| `from_euler_deg()` | オイラー角→四元数 | ★★★ |
| `to_euler_deg()` | 四元数→オイラー角 | ★★★ |

**統一状況** ✅
- 正規化は `cquat::normalize_quat<float>()` に統一（Phase 3で完成）
- 二重正規化チェック完了

---

#### Common/inc/Math/math_utils.hpp
**役割**: 数学ユーティリティ（角度、ベクトル、統計、Mahalanobis）  
**実装度**: ★★★ 包括的実装（400+行）  
**依存**: Matrix/fixed_matrix.hpp, statistics.hpp

| グループ | 関数名 | 説明 |
|---------|--------|------|
| **角度処理** | `wrap_to_pi(T)` | 角度を [-π, π] に正規化 |
| | `angle_difference()` | 角度差計算 |
| **ベクトル** | `normalize_vector()` | ベクトル正規化 |
| | `clip_vector()` | ノルム制限 |
| | `skew_symmetric()` | スキュ対称行列 |
| **統計** | `mean()`, `variance()`, `median()` | 統計関数 |
| **Mahalanobis** | `mahalanobis_distance_squared()` | 距離（2乗） |
| | `compute_innovation_and_S()` | Innovation & 共分散計算 |

---

### LAYER 2: センサー処理

#### Common/inc/filter_mgmt.hpp
**役割**: 共分散管理、ZUPT、発散検出  
**実装度**: ★★★ 完全実装（150行）  
**依存**: Matrix/fixed_matrix.hpp

| 機能 | 関数 |
|------|------|
| **共分散チェック** | `hasNaNOrInf()` |
| **初期化** | `setIdentityScaled()` |
| **発散検出** | `check_divergence()`, `check_state_divergence()` |
| **ZUPT** | `check_zupt_condition()`, `apply_zupt()` |
| **正規化** | `normalize_covariance()`, `symmetrize_covariance()` |

**統一状況** ✅
- 共分散正規化は `filter_mgmt.cpp` の統一実装を使用
- 対称化は出力時に1度だけ実行

---

#### Lib/Sensor/sensor_filter.hpp
**役割**: 外れ値検出、フィルタリング、ロバスト統計  
**実装度**: ★★★ 完全実装（現在分割作業中）  
**依存**: Matrix/fixed_matrix.hpp, Math/math_utils.hpp

**分割計画**（Phase 3 進行中）:

| モジュール | 行数 | 機能 |
|-----------|------|------|
| `ema_filter.hpp` | ~90 | 指数移動平均フィルタ |
| `biquad_filter.hpp` | ~130 | Biquad ローパスフィルタ |
| `alpha_beta_filter.hpp` | ~100 | Alpha-Betaトラッキングフィルタ |
| `outlier_detector.hpp` | ~120 | Mahalanobis + 履歴ベース外れ値検出 ✅ |
| `robust_statistics.hpp` | ~150 | ノイズ推定・発散検出 |
| `sensor_filter.hpp` | ~150 | 統合ディスパッチャ |

**現在の進捗**:
- ✅ `ema_filter.hpp` 抽出完了
- ✅ `biquad_filter.hpp` 抽出完了
- ✅ `alpha_beta_filter.hpp` 抽出完了
- ✅ `robust_statistics.hpp` 抽出完了
- ✅ `outlier_detector.hpp` 抽出完了（2026-01-09）
- 🔄 `sensor_filter.hpp` 再ディスパッチ化完了

**再発防止チェックリスト** ✅
- [ ] 分割前に元実装の依存関係を grep で全検索
- [ ] 各モジュールヘッダーで単独コンパイルテスト
- [ ] stub 実装を完全に移植（コピペ→論理検証）
- [ ] 分割後、必ず単体テスト実行
- [x] ビルド成功 (2026-01-09 11:02)
- [x] 回帰テスト 10/10 PASS (2026-01-09 11:03)

---

#### Lib/Sensor/sensor_preprocessor.hpp
**役割**: センサー入力の前処理（キャリブレーション、座標変換）  
**実装度**: ★★★ 完全実装（140行）  
**依存**: Matrix/fixed_matrix.hpp, Math/math_utils.hpp

| 関数 | 説明 |
|------|------|
| `preprocess_accel()` | 加速度前処理 |
| `preprocess_mag()` | 磁気前処理 |
| `preprocess_baro()` | 気圧→高度変換 |
| `preprocess_gps()` | GPS (WGS84) → ENU変換 |

---

### LAYER 3: フィルタ実装

#### ESKF (11 files)
**役割**: Error-State Kalman Filter（主フィルタ）  
**実装度**: ★★★ Production

| ファイル | 役割 |
|---------|------|
| `eskf_core.hpp` | 予測・更新アルゴリズム |
| `eskf_runner.hpp` | MEX→float変換、ランナー |
| `eskf_state.hpp` | 状態ベクトル定義 |
| `eskf_sensor_updates.hpp` | センサー更新関数 |
| `eskf_postprocess.hpp` | 出力後処理 |

**特性**:
- 状態ベクトル: [p(3), v(3), q(4), ba(3), bg(3)] = 15次元
- 共分散: P[15x15] float32 column-major
- センサー: IMU (accel, gyro), GPS, Magnetometer, Barometer

---

#### MEUKF (5 files)
**役割**: Modified Extended Unscented Kalman Filter（代替フィルタ）  
**実装度**: ★★ Development

| ファイル | 行数 | 状態 |
|---------|------|------|
| `meukf_core.cpp` | 1346 | ⚠️ 要分割（Phase 3） |
| `meukf_core.hpp` | 50 | ✅ |
| `meukf_observation_models.hpp` | 200 | 🔄 開発中 |

**分割計画**（Phase 3 予定）:
```
meukf_core.cpp (1346行)
  ├─ meukf_predict.cpp (400行)
  ├─ meukf_sigma_points.cpp (350行)
  └─ meukf_update.cpp (400行)
```

---

## 重複・未使用検出

### ✅ 統一済み（Phase 2-3）

#### 1. 四元数正規化 ✅
```cpp
// 標準実装: cquat::normalize_quat<float/double>()
cquat::normalize_quat<float>(q);  // ✅ 推奨
```

**完了**: 全呼び出しを統一（2026-01-09）

#### 2. 共分散対称化 ✅
```cpp
// 標準実装: common::filter::symmetrize_covariance()
common::filter::symmetrize_covariance(P);  // ✅ 推奨
```

**完了**: 出力時に一度だけ実行（2026-01-09）

---

### ⚠️ 未解決（Phase 3 優先度 MEDIUM）

#### 1. Mahalanobis距離計算（重複）
```cpp
// 現況: 3箇所に実装
1. Common/Math/math_utils.hpp (正規実装)
2. sensor_filter.hpp (OutlierDetector内)
3. ESKF/sensor_updates.cpp (ローカル)

推奨: Math/math_utils 版に統一、他は委譲
```

#### 2. Innovation計算（重複）
```cpp
// 現況: 2分散
1. KF/kalman_filter_core.hpp (テンプレート版)
2. Common/Math/math_utils.hpp (統一版)

推奨: KF → MathUtils に委譲
```

---

## 依存関係グラフ

```
MATLAB Frontend
    ↓
MEX/mex_run_eskf
    ↓ init/step/get_state
ESKF/eskf_runner (double ↔ float変換)
    ↓
ESKF/eskf_core (予測・更新)
    ├─→ Sensor/sensor_filter (フィルタリング・外れ値)
    ├─→ Sensor/sensor_preprocessor (GPS→ENU)
    ├─→ Common/filter_mgmt (ZUPT・発散検出)
    └─→ Quaternion/quaternion_functions (四元数演算)
        ↓
    Matrix/fixed_matrix (行列演算)
    Math/math_utils (統計・Mahalanobis)
```

---

## 次のアクション（Phase 3）

### 短期（1週間）
1. ✅ sensor_filter.hpp 分割完了 (2026-01-09)
2. 🔄 meukf_core.cpp 分割開始予定
3. 🔄 Mahalanobis/Innovation 統一計画策定

### 中期（2週間）
1. MEUKF分割完了
2. 回帰テスト拡充（外れ値ケース追加）
3. ドキュメント最終更新

### 長期（1ヶ月）
1. UKF ライブラリ抽出（計画中）
2. 外れ値処理ロバスト化
3. パフォーマンス最適化

