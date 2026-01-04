# KalmanFilter C++ Lib 構造分析レポート

**作成日**: 2026年1月4日  
**分析対象**: kalman/cpp/Lib/ 配下の全ファイル（40ファイル）  
**目的**: 機能概要、依存関係、重複・未使用機能の検出

---

## 📋 目次

1. [モジュール概要](#モジュール概要)
2. [各モジュール詳細分析](#各モジュール詳細分析)
3. [依存関係グラフ](#依存関係グラフ)
4. [重複・削除候補リスト](#重複削除候補リスト)
5. [統合推奨事項](#統合推奨事項)
6. [実装状況サマリー](#実装状況サマリー)

---

## モジュール概要

### レイヤー構成

```
┌─────────────────────────────────────────────┐
│ MATLAB MEX インタフェース層                 │
│ (MEX/mex_run_eskf.cpp など)                │
└────────────────────┬────────────────────────┘
                     │
┌────────────────────┴────────────────────────┐
│ アプリケーション層                          │
├─────────────┬──────────────┬────────────────┤
│ ESKF        │ MEUKF        │ Standalone API │
│ (主実装)    │ (副実装)     │ (filter_type)  │
└─────────────┴──────────────┴────────────────┘
                     │
┌────────────────────┴────────────────────────┐
│ フィルタ・アルゴリズム層                    │
├─────────┬──────┬──────┬───────┬────────────┤
│ ESKF    │ EKF  │ UKF  │ MEUKF │ KF (基本) │
│ Core    │ Core │ Core │ Core  │ Core      │
└─────────┴──────┴──────┴───────┴────────────┘
                     │
┌────────────────────┴────────────────────────┐
│ ユーティリティ層                            │
├──────────┬──────────┬─────────┬────────────┤
│ Matrix   │Quaternion│ Common  │ Validation │
│(固定サイ │(四元数)  │(管理機能)            │
│ズ行列)   │          │         │            │
└──────────┴──────────┴─────────┴────────────┘
```

### モジュール一覧（機能別）

| モジュール | 説明 | ファイル数 | 実装状況 |
|-----------|------|----------|--------|
| **Common** | 共通機能（型定義、フィルタ管理、センサー処理） | 11 | ★★★ (完全) |
| **ESKF** | Error-State Kalman Filter（主フィルタ） | 11 | ★★★ (完全) |
| **MEUKF** | Multiple-Estimation UKF（副フィルタ） | 5 | ★★ (部分実装) |
| **EKF** | Extended Kalman Filter（テンプレート） | 3 | ★★ (テンプレートのみ) |
| **UKF** | Unscented Kalman Filter（テンプレート） | 3 | ★★ (テンプレートのみ) |
| **KF** | 基本カルマンフィルタ（テンプレート） | 2 | ★ (基本のみ) |
| **Matrix** | 固定サイズ行列演算 | 1 | ★★★ (完全) |
| **Quaternion** | 四元数演算 | 1 | ★★★ (完全) |
| **合計** | | 37 | |

---

## 各モジュール詳細分析

### 1. Common モジュール（★★★ 優先度: 必須）

#### 1.1 types.hpp
- **機能**: 基本型定義、定数定義
- **公開インターフェース**:
  - `using Scalar = float` - 浮動小数点型
  - `using Index = uint8_t` - インデックス型
  - `enum Status` - ステータスコード (STATUS_OK, STATUS_ERROR, etc.)
  - `const int MAX_STATE_DIM = 20`, `MAX_MEAS_DIM = 10`
- **用途**: 全モジュールで使用される基本定義
- **実装**: ヘッダーのみ（定義）

#### 1.2 interface.hpp
- **機能**: MATLAB⟷C++間のデータ構造体定義
- **公開インターフェース**:
  - `struct SensorData` - センサー入力（加速度、ジャイロ、磁気、GPS、気圧）
  - `struct State` - 状態ベクトル（p, v, q, ba, bg, P[15x15], euler）
  - `struct Params` - フィルタパラメータ（ノイズ、磁気参照など）
  - `class Filter` - 抽象フィルタクラス（init, update, getState, reset）
  - `struct SensorInput` / `struct FilterOutput` - 統一インターフェース
- **詳細**: 
  - 型混在に注意：GPS(`double`)、その他(`float`)
  - q = [w, x, y, z] で統一
  - P[15x15] は column-major、出力時対称化必須
- **実装**: ヘッダーのみ

#### 1.3 filter_mgmt.hpp / filter_mgmt.cpp
- **機能**: 共分散行列の健全性チェック、リセット、ZUPT処理
- **公開インターフェース**:
  - `bool hasNaNOrInf(P)` - NaN/Inf検出
  - `void setIdentityScaled(P, scale)` - 初期化
  - `bool check_divergence(P)` - 発散検出
  - `void apply_zupt(v_in, P_in, v_out, P_out)` - Zero Velocity Update
  - `void normalize_covariance(P)` - 共分散正規化
  - `bool check_state_divergence(p, v, q, ba, bg, P)` - 拡張発散チェック
  - `bool check_zupt_condition(a, w, thresholds)` - 静止状態判定
  - `void reset_state_on_divergence(v, ba, bg, q, P)` - 発散時リセット
- **詳細**: 
  - 速度分散を 0.01 倍にスケーリング（ZUPT）
  - 各状態変数の最大分散をハード制限
- **依存**: Matrix/fixed_matrix.hpp
- **実装**: ヘッダー + cpp（174行）

#### 1.4 utils.hpp
- **機能**: �易ユーティリティ（共分散対称化、四元数正規化）
- **公開インターフェース**:
  - `inline void symmetrizeCov(State &s)` - 共分散行列の強制対称化
  - `inline void normalizeQuat(float q[4])` - 四元数正規化
- **詳細**: 
  - MATLAB出力用の対称化処理
  - q = [w, x, y, z] 対応
- **実装**: インライン関数のみ

#### 1.5 Sensor/ サブモジュール

##### sensor_filter.hpp（重要！）
- **機能**: センサー外れ値検出、フィルタリング、ロバスト統計
- **公開インターフェース**:
  - `class EMAFilter` - 指数移動平均フィルタ
  - `class BiquadLowpassFilter` - Biquad ローパスフィルタ
  - `class OutlierDetector` - 外れ値検出（Mahalanobis距離、IQR）
  - `class RobustStatistics` - ロバスト統計（中央値、MAD）
  - `struct SensorFilterLib` - フィルタライブラリ統合
  - `enum SensorOutlierReason` - 外れ値の理由分類
- **詳細**:
  - ログ出力のグローバル制御 (`sensor_log_enable`)
  - g_log_counter - ログ呼び出し追跡
  - 長大なファイル（831行）
- **状態**: ★★★ 完全実装、 MEX ビルド対応
- **依存**: Matrix/fixed_matrix.hpp
- **実装**: ヘッダーのみ

##### sensor_preprocessor.hpp / sensor_preprocessor.cpp
- **機能**: センサーデータの前処理（変更検知、座標変換、外れ値判定）
- **公開インターフェース**:
  - `struct PreprocessResult` - 処理結果（output, is_outlier, no_change）
  - `PreprocessResult preprocess_accel(a_meas, prev_a)` - 加速度前処理
  - `PreprocessResult preprocess_mag(m_meas, prev_m)` - 磁気前処理
  - `double preprocess_baro(pressure)` - 気圧→高度変換
  - `PreprocessResult preprocess_gps(lat, lon, alt, origin)` - GPS→ENU変換
- **詳細**:
  - バッファ許容誤差チェック（デフォルト 1e-9）
  - 加速度外れ値: 0.1 m/s² 未満、または 9.81±3.0 m/s² 外
  - GPS のみ ENU 座標変換処理
- **実装**: ヘッダー + cpp（140行）

##### Math/ サブモジュール

###### math_utils.hpp
- **機能**: 数学ユーティリティ（角度処理、ベクトル操作、行列操作）
- **公開インターフェース**:
  - `float wrap_to_pi(angle)` - 角度正規化 [-π, π]
  - `float wrap_to_180(angle)` - 角度正規化 [-180°, 180°]
  - `float angle_difference(a1, a2)` - 角度差計算
  - `cm normalize_vector(v)` - ベクトル正規化
  - `cm clip_vector(v, max_norm)` - ベクトルノルム制限
  - `cm enforce_symmetry(M)` - 行列対称化
  - `cm skew_symmetric(v)` - スキュ対称行列 (v×)
- **実装**: インライン関数のみ（335行）

###### statistics.hpp（読み込み未実施）
- **推定機能**: 平均、標準偏差計算（初期化時に使用）
- **用途**: ESKF初期化時のセンサーノイズ推定

###### vector_utils.hpp（読み込み未実施）
- **推定機能**: ベクトル操作補助

##### Validation/ サブモジュール

###### validation.hpp
- **機能**: 共分散行列の正則化、Joseph形式更新
- **公開インターフェース**:
  - `class CovarianceRegularizer` - 共分散正規化
    - `static cm regularize(P)` - 対称化・スケーリング・正定値化
    - `static cm joseph_form_update(P_pred, K, H, R)` - Joseph形式更新
  - 定数: `MIN_VARIANCE = 1e-12`, `MAX_VARIANCE = 1e6`
- **詳細**: 対角優位性チェック、固有値下限設定
- **実装**: ヘッダーのみ（306行）

#### 1.6 interface_stub.cpp
- **機能**: �易スタブ実装（テスト用）
- **公開インターフェース**:
  - `void initStateZero(State &s)` - 状態ゼロ初期化
  - `void finalizeCov(State &s)` - 共分散最終化（対称化・クリップ）
- **実装**: cpp（29行）

#### 1.7 standalone.hpp / standalone.cpp
- **機能**: スタンドアロン API（フィルタ型選択、global state）
- **公開インターフェース**:
  - `enum FilterType` - FILTER_KF, FILTER_EKF, FILTER_UKF, FILTER_ESKF, FILTER_MEUKF
  - `uint8_t filter_init/update/getState/reset/setType()`
- **詳細**: グローバルフィルタポインタで管理
- **実装**: ヘッダー + cpp（37行）

---

### 2. ESKF モジュール（★★★ 優先度: 最高、メイン実装）

#### 2.1 eskf_core.hpp / eskf_core.cpp
- **機能**: ESKF コア処理（積分、更新、共分散予測）
- **公開インターフェース**:
  - `static void integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, ...)` - ノミナル状態積分（RK2）
  - `static void update_accel(q, a_meas)` - 加速度更新 (Roll/Pitch)
  - `static void update_mag(q, P, m_meas, m_world, R_mag, K_out, dx_out)` - 磁気計更新
  - `static void update_gps(p, v, P, gps_pos, R_gps, ...)` - GPS更新
  - `static void update_baro(p, P, altitude, R_baro, ...)` - 気圧計更新
  - `static void predict_covariance(P, q, a_meas, ba, w_meas, bg, Q, dt, P_new)` - 共分散予測
  - `static void compute_adaptive_Q(Q_nominal, a_meas, w_meas, Q_adapted)` - Adaptive Q
  - `static void compute_F_matrix(...)` - F行列計算
  - `static void inject_error_state(p, v, q, ba, bg, dx)` - 誤差状態注入
  - `static void update_zupt(v_in, P_in, v_out, P_out)` - ZUPT更新
- **詳細**:
  - RK2 積分（四元数）
  - バイアス補正：`a_corrected = a_meas - ba`
  - 静的メンバー：`prev_a_world`, `prev_v`, `prev_initialized`
- **状態**: ★★★ 完全実装
- **依存**: Quaternion, KF/kalman_filter_core.hpp, Math/math_utils.hpp
- **実装**: ヘッダー + cpp（248行）

#### 2.2 eskf_filter.hpp / filter.cpp
- **機能**: ESKF フィルタクラス（Filter 仮想クラス継承）
- **公開インターフェース**:
  - `class ESKFFilter : public Filter`
    - `uint8_t init(obs, static_time)`
    - `uint8_t update(obs)`
    - `uint8_t getState(state)`
    - `uint8_t setParams(params)`
    - `uint8_t reset()`
- **詳細**: 単純な実装、主に ESKFCore に委譲
- **実装**: ヘッダー + cpp（160行）

#### 2.3 eskf_state.hpp
- **機能**: ESKF 状態構造体（ダブル精度ストレージ）
- **構成**: 
  - `p[3], v[3], q[4], ba[3], bg[3]` (double)
  - `P[15*15]` (double, Q_nominal[15*15])
  - `g[3], dt, gps_origin[3]`
  - `prev_accel[3], prev_gyro[3], prev_mag[3]` - 前回値
  - `prev_gps_lat/lon/alt, prev_baro` - 前回センサー値
  - `buffer_tolerance, w_body[3]`
  - ZUPT関連: `zupt_threshold_*`, `zupt_min_duration`, `zupt_counter`, `is_stationary`
  - フラグ: `adaptive_q_enabled`, `enable_accel_z_integration`, `valid`
  - パラメータ: `velocity_damping, baro_weight, accel_z_threshold, accel_z_damping`
- **用途**: MEX や runner.cpp で使用される低レベル状態コンテナ
- **実装**: ヘッダーのみ

#### 2.4 eskf_helper.hpp
- **機能**: テンプレートヘルパー（誤差状態注入、制約付き更新）
- **公開インターフェース**:
  - `struct NominalState` - p, v, q, ba, bg
  - `static void inject_error_state(nominal, dx)` - 誤差状態注入
  - `static void inject_with_constraints(nominal, dx, ...)` - 制約付き注入
  - `static void joseph_form_covariance_update(P, K, H, R)` - Joseph形式更新
  - `static void regularize_covariance(P, eps)` - 共分散正規化
- **詳細**: テンプレート実装、T = float
- **実装**: ヘッダーのみ

#### 2.5 eskf_initializer.hpp / eskf_initializer.cpp
- **機能**: 初期化処理（静止センサーデータから状態推定）
- **公開インターフェース**:
  - `struct ESKFInitializationData` - 初期化入力（センサー配列ポインタ、n_samples, n_static, dt）
  - `ESKFState* initialize_eskf_state(data)` - 状態初期化
- **詳細**:
  - 加速度から Roll/Pitch 推定
  - ジャイロから Roll/Pitch の不確実性推定
  - 磁気から Yaw 推定
  - GPS から位置原点設定
  - 気圧から高度推定
  - 共分散行列 P, Q_nominal を自動計算
  - センサーノイズを統計から推定
- **状態**: ★★★ 完全実装（超複雑、1行化）
- **依存**: Quaternion, statistics.hpp
- **実装**: ヘッダー + cpp（単一行）

#### 2.6 eskf_math.hpp / eskf_math.cpp
- **機能**: 数学関数群（積分、行列計算、座標変換）
- **公開インターフェース**:
  - `class ESKFMath` (static only)
  - `void quaternion_integration(q_in, w, dt, q_out)`
  - `void accel_to_quaternion(a_meas, scale_factor, q_out)`
  - `struct PVIntegrationInput / Output` - p-v 積分用
  - `void pv_integration(input, output)` - p-v 積分
  - `void compute_F_matrix(...)` - F行列計算
  - `void covariance_prediction(...)` - 共分散予測
  - `void inject_error_state(...)` - 誤差注入
  - `void mag_observation_prediction(...)` - 磁気観測予測
  - `void gps_to_local(gps_pos, origin, local_pos)` - GPS→ENU変換
  - `Scalar pressure_to_altitude(pressure)` - 気圧→高度
  - テンプレート: `kalman_update<N, M>(...)`
- **詳細**: AB2積分対応、速度・位置の最大値制限
- **実装**: ヘッダー + cpp（テンプレート含）

#### 2.7 eskf_postprocess.hpp / eskf_postprocess.cpp
- **機能**: ポストプロセス（状態調整、共分散更新）
- **公開インターフェース**:
  - `struct PredictPostprocessParams` - パラメータ（enable_accel_z_integration, damping）
  - `struct PredictPostprocessResult` - v, P の結果
  - `void predict_postprocess(v, q, P, a_for_vel, dt, g, params)`
  - `struct UpdatePostprocessResult` - p, v, q, ba, bg, P, skip_flag
  - `UpdatePostprocessResult update_state_from_dx(dx, state_*, P_new)`
  - `void symmetrize_covariance(P)` - 強制対称化
- **詳細**:
  - `accel_z_integration` - Z加速度を速度に統合（閾値付き）
  - `velocity_damping` - 速度減衰（モデルシンク）
- **実装**: ヘッダー + cpp（最小化）

#### 2.8 eskf_runner.hpp / eskf_runner.cpp
- **機能**: ESKF ランナー（状態積分と更新の統合）
- **公開インターフェース**:
  - `class ESKFRunner` (static)
  - `static void predict(ESKFState* s, a_meas, w_meas)` - 予測ステップ
- **詳細**:
  - double ⟷ float 変換を処理
  - column-major ⟷ row-major 変換（P[i + j*15]）
  - Adaptive Q, accel_z_integration, velocity_damping を適用
  - `apply_accel_z_integration(...)` - Z軸統合
  - `apply_velocity_clipping(v, P, max_vel)` - 速度制限
  - `regularize_covariance(P)` - 対角スケーリング（速度分散用）
- **依存**: sensor_filter.hpp（DivergenceGuard）
- **実装**: ヘッダー + cpp（超複雑）

#### 2.9 eskf_sensor_updates.hpp
- **機能**: センサー更新関数シグネチャ（実装は別）
- **公開インターフェース**:
  - `struct SensorUpdateResult` - should_skip, updated
  - `SensorUpdateResult update_accel_sensor(...)`
  - `SensorUpdateResult update_mag_sensor(...)`
  - `SensorUpdateResult update_baro_sensor(...)`
  - `SensorUpdateResult update_gps_sensor(...)`
- **実装**: 宣言のみ

---

### 3. MEUKF モジュール（★★ 副実装）

#### 3.1 meukf_core.hpp / meukf_core.cpp
- **機能**: Multiple-Estimation UKF コア処理
- **状態**: ★★ 部分実装（1346行、複雑）
- **公開インターフェース**:
  - `class MEUKFCore` (static)
  - `static void step(input, output)` - メインステップ
  - 内部関数: `predict(...)`, `update_accel_meukf(...)`, `update_mag_meukf(...)` など
- **詳細**: 
  - 複数のシグマポイント集合を使用
  - Cholesky分解（3x3用専用実装）
  - EVM（Evidential Vector Measurement）統合予定
- **依存**: unified_types.hpp, meukf_types.hpp
- **実装**: cpp（1346行、最大級）

#### 3.2 meukf_types.hpp / meukf_core.hpp
- **機能**: MEUKF用型定義（State, SensorData, Params, Input/Output）
- **用途**: meukf_core との I/O インターフェース
- **詳細**: 
  - `State`: p[3], v[3], q[4], ba[3], bg[3], P[15*15]
  - `SensorData`: accel, gyro, mag, gps_pos, alt_baro, update flags, dt
  - `Params`: g, mag_ref, noise_*, alpha, beta, kappa (UKF parameters)
  - `MEUKFInput`: prev_state + sensor + params
  - `MEUKFOutput`: new_state + debug_info + status + last_K, last_S, last_S_inv
- **実装**: ヘッダーのみ

#### 3.3 unified_types.hpp / unified_filter.hpp / unified_filter.cpp
- **機能**: 統一フィルタインターフェース
- **公開インターフェース**:
  - `struct FilterInput` - dt, accel, gyro, mag, gps_pos, baro_alt, flags, g, mag_ref, noise_*, alpha/beta/kappa
  - `struct FilterOutput` - position, velocity, quaternion, euler, accel_bias, gyro_bias, P, status
  - `struct FilterState` - 内部状態保持
  - `class UnifiedFilter` - update(state, input) → output
- **詳細**: 
  - 複数のセンサー更新関数を統合
  - ZUPT チェック、発散チェック
  - 前回値との変更検知
- **実装**: ヘッダー + cpp（210行）

---

### 4. EKF モジュール（★★ テンプレート実装）

#### 4.1 ekf_core.hpp
- **機能**: Extended Kalman Filter テンプレート
- **公開インターフェース**:
  - `template<int N, int M, typename T> class EKFCore`
  - `static void predict(x, P, f_func, Q)` - 予測（ヤコビアンなし）
  - `static void predict_with_jacobian(x, P, f_func, F, Q)` - 予測（ヤコビアン付き）
  - `static void update(x, P, z, h_func, H, R, K_out=nullptr, S_out=nullptr, y_out=nullptr)` - 更新
- **詳細**: テンプレート実装、汎用性重視
- **実装**: ヘッダーのみ（175行）

#### 4.2 ekf_linear_update.hpp / ekf_linear_update.cpp
- **機能**: 線形観測モデル用 EKF 更新
- **公開インターフェース**:
  - `void ekf_linear_update(x, P, z, H, R, x_upd, P_upd)`
- **詳細**: H が定数行列の場合の最適化版
- **実装**: ヘッダー + cpp（127行）

---

### 5. UKF モジュール（★★ テンプレート実装）

#### 5.1 ukf_core.hpp
- **機能**: Unscented Kalman Filter テンプレート
- **公開インターフェース**:
  - `template<int N, int M, typename T> class UKFCore`
  - `static void update(x, P, z, h_func, R, alpha, beta, kappa, K_out, S_out, y_out)` - 観測更新
- **詳細**: 
  - シグマポイント生成、重み計算
  - 汎用観測関数（ラムダ式対応）
- **実装**: ヘッダーのみ（319行）

#### 5.2 ukf_sigma_points.hpp / ukf_sigma_points.cpp（未実装）
- **機能**: シグマポイント生成（動的サイズ版 MEX用）
- **宣言**: 
  - `void generate_sigma_points_dynamic(x, P, n, alpha, beta, kappa, sig, wm, wc)`
- **状態**: ★ 宣言のみ、実装未完成

#### 5.3 ukf_update.hpp（未実装）
- **機能**: UKF 更新ステップ（フルスタック）
- **宣言**: 
  - `class UKFUpdate`
  - `static void update(...)` - テンプレートなし
- **状態**: ★ 宣言のみ

---

### 6. KF モジュール（★ 基本実装）

#### 6.1 kalman_filter_core.hpp
- **機能**: 基本カルマンフィルタ（テンプレート）
- **公開インターフェース**:
  - `class KalmanFilterCore` (static)
  - テンプレート関数群:
    - `compute_kalman_gain<N, M, T>(P, H, S) → K`
    - `compute_innovation_and_S<N, M, T>(z, h, H, P, R, y, S, R_out)`
    - `update_state_covariance<N, M, T>(x, P, K, H, y, R, x_upd, P_upd)` - Joseph形式
- **詳細**: 汎用カルマンフィルタの基本操作
- **実装**: ヘッダーのみ（92行）

#### 6.2 kf_core.hpp
- **機能**: 固定サイズ行列用カルマンフィルタ
- **公開インターフェース**:
  - `class KFCore` - 非テンプレート
  - `void initialize(x0, P0, Q)`
  - `void predict(F, u)` - x = F*x + u, P = F*P*F' + Q
  - `void update(z, H, R)` - Kalman gain と状態更新
  - `bool getData(K, S, y)` - 最新結果の取得
- **詳細**: 内部でセンサーフィルタライブラリを使用
- **実装**: ヘッダーのみ（253行）

---

### 7. Matrix モジュール（★★★ 基盤）

#### 7.1 fixed_matrix.hpp
- **機能**: 固定サイズ行列テンプレート（すべて実装含む）
- **公開インターフェース**:
  - `template<int R, int C, typename T> struct Matrix`
  - 演算: `+`, `-`, `*` (スカラー・行列)
  - `transpose()`, `inverse()`, `cholesky(L)`, `operator()(r, c)`
  - 静的メソッド: `Zero()`, `Identity()`
- **詳細**:
  - Row-major メモリレイアウト
  - Gauss-Jordan 逆行列計算（ピボット選択付き）
  - Cholesky 分解（正定値性チェック）
- **状態**: ★★★ 完全実装
- **実装**: ヘッダーのみ（350行）

---

### 8. Quaternion モジュール（★★★ 基盤）

#### 8.1 quaternion_functions.hpp
- **機能**: 四元数演算（q = [w, x, y, z]）
- **公開インターフェース**:
  - `namespace cquat` テンプレート関数群:
  - `void normalize_quat(q)` - 正規化
  - `void multiply_quat(a, b, out)` - 四元数乗算
  - `void quat_to_rotm(q, R)` - クォータニオン → 回転行列
  - `void from_euler_deg(roll, pitch, yaw, q_out)` - オイラー角 → Q
  - `void to_euler_deg(q, euler_deg)` - Q → オイラー角 (3要素)
  - オーバーロード版: `void to_euler_deg(q, roll, pitch, yaw)` (参照返却)
- **詳細**:
  - テンプレート（T = float or double）
  - Gimbal lock 対応 (pitch = ±π/2)
  - 単位四元数前提
- **状態**: ★★★ 完全実装
- **実装**: ヘッダーのみ（165行）

---

## 依存関係グラフ

### 依存マトリクス

```
┌─────────────────────────────────────────────────────────────────────┐
│ レイヤー別依存関係                                                   │
├─────────────────────────────────────────────────────────────────────┤
│ LAYER 0: 基盤（他に依存しない）                                     │
│  ✓ Matrix/fixed_matrix.hpp                                           │
│  ✓ Quaternion/quaternion_functions.hpp                               │
│  ✓ Common/types.hpp                                                  │
├─────────────────────────────────────────────────────────────────────┤
│ LAYER 1: 基盤を使用                                                 │
│  ✓ Common/utils.hpp → types, Matrix, Quaternion                    │
│  ✓ Common/Math/math_utils.hpp → Matrix                             │
│  ✓ Common/Math/statistics.hpp → (推定: 基本のみ)                   │
│  ✓ Common/Math/vector_utils.hpp → (推定: Matrix)                   │
│  ✓ Common/Validation/validation.hpp → Matrix                       │
│  ✓ KF/kalman_filter_core.hpp → Matrix                              │
├─────────────────────────────────────────────────────────────────────┤
│ LAYER 2: 基本フィルタ                                              │
│  ✓ EKF/ekf_core.hpp → KF, Matrix                                    │
│  ✓ UKF/ukf_core.hpp → Matrix                                        │
│  ✓ KF/kf_core.hpp → Matrix, sensor_filter                          │
├─────────────────────────────────────────────────────────────────────┤
│ LAYER 3: センサー処理                                              │
│  ✓ Common/Sensor/sensor_filter.hpp → Matrix                         │
│  ✓ Common/Sensor/sensor_preprocessor.hpp → Matrix, sensor_filter   │
├─────────────────────────────────────────────────────────────────────┤
│ LAYER 4: フィルタ実装（メイン）                                     │
│  ✓ ESKF/eskf_core.hpp → Matrix, Quaternion, KF, Math               │
│  ✓ ESKF/eskf_helper.hpp → Matrix, Quaternion                       │
│  ✓ ESKF/eskf_math.hpp → Matrix, Quaternion                         │
│  ✓ ESKF/eskf_initializer.hpp → Quaternion, statistics              │
│  ✓ ESKF/eskf_postprocess.hpp → Matrix, Quaternion, filter_mgmt    │
│  ✓ ESKF/eskf_runner.hpp → sensor_filter, eskf_*                    │
│  ✓ ESKF/eskf_state.hpp → (型定義のみ)                              │
│  ✓ MEUKF/meukf_core.hpp → Matrix, Quaternion, unified_types        │
│  ✓ MEUKF/unified_types.hpp → Matrix, Quaternion                    │
│  ✓ MEUKF/unified_filter.hpp → unified_types, meukf_core            │
├─────────────────────────────────────────────────────────────────────┤
│ LAYER 5: インターフェース                                           │
│  ✓ Common/interface.hpp → types, Matrix                             │
│  ✓ ESKF/filter.hpp → interface, eskf_core                           │
│  ✓ Common/filter_mgmt.hpp → Matrix                                  │
├─────────────────────────────────────────────────────────────────────┤
│ LAYER 6: アプリケーション                                           │
│  ✓ Common/standalone.hpp → interface                                │
│  ✓ (MEX層) → 全フィルタインターフェース                             │
└─────────────────────────────────────────────────────────────────────┘
```

### 詳細なインクルード情報

```
Matrix/fixed_matrix.hpp
├── (なし - 独立)

Quaternion/quaternion_functions.hpp
├── Matrix/fixed_matrix.hpp

Common/types.hpp
├── (なし - 型定義のみ)

Common/utils.hpp
├── interface.hpp
├── cmath (std)

Common/interface.hpp
├── types.hpp
├── (複数定義あり - interface_v1 と interface_v2 の混在)

Common/filter_mgmt.hpp
├── Matrix/fixed_matrix.hpp

Common/Math/math_utils.hpp
├── Matrix/fixed_matrix.hpp

Common/Sensor/sensor_filter.hpp
├── Matrix/fixed_matrix.hpp
├── mex.h (MEX時のみ)

Common/Sensor/sensor_preprocessor.hpp
├── Matrix/fixed_matrix.hpp

Common/Validation/validation.hpp
├── Matrix/fixed_matrix.hpp

KF/kalman_filter_core.hpp
├── Matrix/fixed_matrix.hpp

KF/kf_core.hpp
├── Matrix/fixed_matrix.hpp
├── sensor_filter.hpp

EKF/ekf_core.hpp
├── Matrix/fixed_matrix.hpp
├── KF/kalman_filter_core.hpp

EKF/ekf_linear_update.cpp
├── ekf_linear_update.hpp
├── Matrix/fixed_matrix.hpp
├── KF/kalman_filter_core.hpp

UKF/ukf_core.hpp
├── Matrix/fixed_matrix.hpp

ESKF/eskf_core.hpp
├── Matrix/fixed_matrix.hpp
├── Quaternion/quaternion_functions.hpp
├── (⚠️ 相対パス: ../Lib/Quaternion) - 冗長？

ESKF/eskf_core.cpp
├── eskf_core.hpp
├── eskf_math.hpp
├── KF/kalman_filter_core.hpp
├── Quaternion/quaternion_functions.hpp
├── Math/math_utils.hpp

ESKF/eskf_helper.hpp
├── Matrix/fixed_matrix.hpp
├── Quaternion/quaternion_functions.hpp

ESKF/eskf_math.hpp
├── Matrix/fixed_matrix.hpp
├── Quaternion/quaternion_functions.hpp (⚠️ ../Lib/)

ESKF/eskf_initializer.cpp
├── Statistics関数群（未定義？）
├── Quaternion/quaternion_functions.hpp

ESKF/eskf_postprocess.hpp
├── Matrix/fixed_matrix.hpp

ESKF/filter.hpp
├── Common/interface.hpp

ESKF/filter.cpp
├── filter.hpp
├── eskf_core.hpp

MEUKF/meukf_core.cpp
├── meukf_core.hpp
├── Math/math_utils.hpp
├── Quaternion/quaternion_functions.hpp

MEUKF/meukf_types.hpp
├── Matrix/fixed_matrix.hpp

MEUKF/unified_types.hpp
├── Matrix/fixed_matrix.hpp
├── Quaternion/quaternion_functions.hpp

MEUKF/unified_filter.hpp
├── unified_types.hpp

MEUKF/unified_filter.cpp
├── unified_filter.hpp
├── sensor_filter.hpp
├── Math/math_utils.hpp
├── meukf_core.hpp
```

---

## 重複・削除候補リスト

### ⚠️ 高優先度：修正が必要

#### 1. **interface.hpp の重複定義**
- **問題**: 2つの異なる定義が混在
  - Version 1: SensorData, State, Params, Filter (abstract class)
  - Version 2: SensorInput, FilterOutput (統一インターフェース)
- **原因**: リファクタリング途上で不完全に統合
- **影響**: MEX層での型混同、コンパイルエラーリスク
- **推奨処理**: 
  - ✓ **Version 2 を基準に統一**
  - ✓ Version 1 は非推奨マーク、廃止予定日を設定
  - ✓ 関連コード一括置換（interface_v1 → interface (deprecated)）

#### 2. **インクルードパスの混在**
- **問題**:
  ```cpp
  // OK
  #include "../../Matrix/fixed_matrix.hpp"
  #include "../../Quaternion/quaternion_functions.hpp"
  
  // NG (eskf_core.cpp)
  #include "../Lib/Quaternion/quaternion_functions.hpp"  // ❌ Lib/ は重複
  #include "../Lib/Common/inc/Math/math_utils.hpp"        // ❌ 
  ```
- **推奨処理**: 
  - ✓ すべてを `../../` 相対パスに統一
  - ✓ `#include "../Lib/...` を削除、代わりに `#include "../../..."`

#### 3. **Type Conversion 重複 (float/double)**
- **問題**:
  - ESKF state は `double` で保存（eskf_state.hpp）
  - ESKFCore は `float` で計算
  - eskf_runner.cpp で毎回変換 → パフォーマンス低下
- **推奨処理**: 
  - ✓ ESKF state を `float` に統一するか、または
  - ✓ 変換は初期化時のみに限定（ホットパスから除外）

#### 4. **Four Quaternion Normalize Implementations**
- **問題**: 正規化が複数箇所に存在
  - `quaternion_functions.hpp`: `normalize_quat()`
  - `utils.hpp`: `normalizeQuat()` (異なる名前！)
  - `eskf_core.cpp`: `cquat::normalize_quat()`
  - `meukf_core.cpp`: `normalize_quaternion()`
- **推奨処理**: 
  - ✓ `cquat::normalize_quat()` に統一（テンプレート、汎用）
  - ✓ 他の実装を削除または廃止マーク

---

### 🟡 中優先度：整理が推奨

#### 5. **sensor_filter.hpp が巨大（831行）**
- **問題**: 単一ファイルに複数クラス混在
  - EMAFilter, BiquadLowpassFilter, OutlierDetector, RobustStatistics, SensorFilterLib
- **推奨処理**: 
  - Option A: クラスごとに .hpp に分割
    ```
    sensor_filter/
    ├── ema_filter.hpp
    ├── biquad_filter.hpp
    ├── outlier_detector.hpp
    ├── robust_statistics.hpp
    └── sensor_filter_lib.hpp (統合)
    ```
  - Option B: 現状のままロック（実装完了）

#### 6. **meukf_core.cpp が超長（1346行）**
- **問題**: 単一ファイルに全実装
- **推奨処理**: 
  - 関数ごとに分割 (.cpp ファイル追加)
    ```
    MEUKF/src/
    ├── meukf_core.cpp (↔→predict/update 関数)
    ├── meukf_predict.cpp
    ├── meukf_update_accel.cpp
    ├── meukf_update_mag.cpp
    ├── meukf_update_gps.cpp
    └── meukf_update_baro.cpp
    ```

#### 7. **eskf_initializer.cpp が1行化**
- **問題**: 1340文字超の単一行コード（保守困難）
- **原因**: ビルドキャッシュ回避のため意図的？
- **推奨処理**: 
  - ✓ 複数行に整形（可読性向上）
  - ✓ 関数を分割 (init_attitude, init_gps, init_covariance など)

---

### 🟢 低優先度：削除可能（未使用機能）

#### 8. **EKF/UKF テンプレート（実装されていない）**
- **状態**: 
  - `EKF/ukf_core.hpp` - テンプレートのみ
  - `UKF/ukf_sigma_points.hpp` - 宣言のみ
  - `UKF/ukf_update.hpp` - 宣言のみ
- **用途**: 将来の拡張予定？
- **推奨処理**: 
  - 近期使用予定がなければ削除 or 別フォルダへ移動 (Experimental/)

#### 9. **kf_core.hpp（基本KF未使用）**
- **状態**: テンプレート実装、253行
- **用途**: KF/ekf の基盤だが、実際には ESKF が主流
- **推奨処理**: 
  - 保留（参考実装として保持、削除不要）

#### 10. **Validation/validation.hpp（重複機能）**
- **問題**:
  - `CovarianceRegularizer::regularize()` 
  - vs `Common/filter_mgmt.hpp::normalize_covariance()`
  - vs `ESKF/eskf_runner.cpp::regularize_covariance()`
  - **3つの異なる正規化実装が存在！**
- **推奨処理**: 
  - ✓ 統一的な正規化関数を定義（Common/filter_mgmt.hpp）
  - ✓ その他の実装を削除

---

## 統合推奨事項

### フェーズ1: 即座（1-2週間）

#### 1.1 インクルードパス統一
```bash
# 実行内容
find Lib -name "*.hpp" -o -name "*.cpp" | xargs sed -i 's|#include "../Lib/|#include "../../|g'
```

#### 1.2 interface.hpp 統一
```cpp
// Lib/Common/inc/interface.hpp
// ✓ SensorData, State, Params, Filter を削除（後方互換性注意）
// ✓ SensorInput, FilterOutput, Filter (新) を推奨
// ⚠️ 依存コード: ESKF/filter.cpp, standalone.cpp を更新
```

#### 1.3 Quaternion 正規化統一
```cpp
// 修正対象:
// 1. utils.hpp: normalizeQuat() → cquat::normalize_quat() に置換
// 2. meukf_core.cpp: normalize_quaternion() → cquat::normalize_quat() に統一
```

### フェーズ2: 短期（2-4週間）

#### 2.1 共分散正規化関数統一
```cpp
// 公式版: Common/filter_mgmt.hpp::normalize_covariance()
// 削除予定:
//   - Validation/validation.hpp::CovarianceRegularizer::regularize()
//   - ESKF/eskf_runner.cpp::regularize_covariance()
// 移行:
//   - filter_mgmt.cpp に実装を一本化
```

#### 2.2 型変換の最小化
```cpp
// ESKF state を float に変更 or 変換時機を限定
// before: ESKFState[double] → runner[double→float] → core[float]
// after:  ESKFState[float] → core[float]（初期化時のみ double変換）
```

### フェーズ3: 中期（1ヶ月）

#### 3.1 meukf_core.cpp 分割
```
MEUKF/src/
├── meukf_core.cpp (↔→predict, helpers)
├── meukf_predict.cpp
├── meukf_update.cpp
└── meukf_util.cpp
```

#### 3.2 sensor_filter.hpp クラス分割
```
Common/Sensor/
├── ema_filter.hpp
├── biquad_filter.hpp
├── outlier_detector.hpp
├── robust_statistics.hpp
└── sensor_filter_lib.hpp
```

#### 3.3 eskf_initializer.cpp 整形
```cpp
// 複数行に展開、関数分割:
//   - init_attitude_from_accel()
//   - init_gyro_bias()
//   - init_yaw_from_mag()
//   - init_gps_origin()
//   - init_covariance()
```

### フェーズ4: 長期（最適化フェーズ）

#### 4.1 未使用テンプレートの整理
```bash
# EKF/UKF が実際に使用されているか確認
grep -r "EKFCore\|UKFCore" MEX/ --include="*.cpp"
# → 使用なければ削除 or Experimental/ へ移動
```

#### 4.2 ホットパス最適化
```cpp
// Profiling 後、以下の関数に focus:
// - ESKF/eskf_runner.cpp::predict() - 毎フレーム呼び出し
// - ESKF/eskf_core.cpp::integrate_nominal() - 状態積分
// - Matrix::operator() - インデックスアクセス
// → SIMD化、キャッシュ最適化候補
```

---

## 実装状況サマリー

### 機能別実装完成度

| 機能 | 実装度 | 用途 | 状態 |
|-----|------|------|------|
| **基本数学** | ★★★ | 全レイヤーで使用 | ✓ Production |
| **ESKF（メイン）** | ★★★ | 主フィルタ | ✓ Production |
| **センサー処理** | ★★★ | 前処理、外れ値検出 | ✓ Production |
| **共分散管理** | ★★★ | 数値安定性保証 | ⚠️ 要統一 |
| **MEUKF（副）** | ★★ | 代替フィルタ | ⚠️ 保守中 |
| **EKF** | ★★ | テンプレート参考実装 | ⚠️ 部分実装 |
| **UKF** | ★ | テンプレート予定 | ❌ 未完成 |
| **基本KF** | ★ | テンプレート参考実装 | ⚠️ 保守対象外 |

### 既知の問題（必修修正）

| ID | 問題 | 影響度 | 対応期限 |
|----|-----|--------|--------|
| #001 | interface.hpp 重複定義 | **HIGH** | Phase 1 |
| #002 | インクルードパス混在 | **HIGH** | Phase 1 |
| #003 | Quaternion normalize 重複 | **MEDIUM** | Phase 1 |
| #004 | 共分散正規化の分散実装 | **MEDIUM** | Phase 2 |
| #005 | float/double 変換のホットパス | **MEDIUM** | Phase 2 |
| #006 | meukf_core 超長ファイル | **LOW** | Phase 3 |
| #007 | eskf_initializer 1行化 | **LOW** | Phase 3 |
| #008 | UKF 実装不完全 | **LOW** | 将来 |

### メンテナンス必要ファイル（Top 10）

1. **ESKF/eskf_runner.cpp** - 複雑な状態管理、double↔float変換多数
2. **MEUKF/meukf_core.cpp** - 超長ファイル、複数アルゴリズム混在
3. **Common/Sensor/sensor_filter.hpp** - 重大なバグ修正が頻繁、8つのクラス
4. **ESKF/eskf_initializer.cpp** - 統計計算、仕様変更リスク高い
5. **Common/interface.hpp** - 型定義の変更は波及効果大
6. **ESKF/eskf_core.cpp** - 核心アルゴリズム、検証負荷高い
7. **ESKF/eskf_math.hpp** - テンプレート実装、インスタンス化エラーリスク
8. **Matrix/fixed_matrix.hpp** - 全モジュール依存、変更禁止級
9. **Quaternion/quaternion_functions.hpp** - 全フィルタ依存、正確性必須
10. **KF/kalman_filter_core.hpp** - 理論的基盤、参考実装レベル

---

## 推奨チェックリスト（今後のレビュー時）

- [ ] 新ファイル追加時は Lib/ フォルダ構造に従う
- [ ] `#include "../Lib/..."` は禁止、`../../...` に統一
- [ ] float/double 混在の確認
- [ ] 四元数正規化は `cquat::normalize_quat()` のみ
- [ ] 共分散正規化は `common::filter::normalize_covariance()` のみ
- [ ] MEX出力時は `P = (P+P')/2` で強制対称化
- [ ] 新しい"正規化"関数は `filter_mgmt.hpp` に追加（重複防止）
- [ ] テンプレートの明示的インスタンス化を記述
- [ ] ファイルサイズ > 500行の場合は分割検討

---

## まとめ

### 強み
✅ **完全実装**: ESKF, 基本数学, Matrix, Quaternion  
✅ **ロバスト**: センサー外れ値検出, 共分散保証, ZUPT  
✅ **モジュール設計**: 依存関係が明確で拡張性良好  

### 弱み
❌ **インターフェース混在**: interface.hpp v1/v2 分裂  
❌ **型変換負荷**: ESKF state double, core float の変換ロス  
❌ **ファイルサイズ**: meukf_core 1346行, sensor_filter 831行  
❌ **コード重複**: normalize, regularize, inject 複数実装  

### 次のステップ
1. **即座**: インクルード統一、interface統一
2. **短期**: 型統一、関数統一
3. **中期**: ファイル分割、コード整形
4. **長期**: パフォーマンス最適化、テスト充実

---

**文書版**: 1.0  
**最終更新**: 2026年1月4日 20:30 JST

