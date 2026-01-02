# C++ファイル・関数インデックス

## ファイル一覧

### MEXファイル

| ファイル | 行数 | 主要関数 | 状態 |
|---------|------|---------|------|
| `MEX/mex_run_eskf.cpp` | 91 | `mexFunction` | ✅ 使用中 |
| `MEX/mex_meukf_step.cpp` | 359 | `mexFunction` | ⚠️ 非推奨（統合済み） |

### ESKF実装

| ファイル | 行数 | 主要関数 | 状態 |
|---------|------|---------|------|
| `src/ESKF/eskf_core.cpp` | 597 | `integrate_nominal`, `predict_covariance`, `update_accel`, `update_mag`, `update_gps`, `update_baro`, `update_zupt`, `compute_adaptive_Q` | ✅ 使用中 |
| `src/ESKF/eskf_runner.cpp` | 282 | `predict`, `apply_accel_z_integration`, `apply_velocity_clipping`, `regularize_covariance` | ✅ 使用中 |
| `src/ESKF/eskf_initializer.cpp` | 365 | `initialize_eskf_from_matlab` | ✅ 使用中 |
| `src/ESKF/eskf_sensor_updates.cpp` | 342 | `update_accel_sensor`, `update_mag_sensor`, `update_baro_sensor`, `update_gps_sensor` | ✅ 使用中 |
| `src/ESKF/eskf_postprocess.cpp` | 94 | `predict_postprocess`, `update_state_from_dx`, `symmetrize_covariance` | ✅ 使用中 |
| `src/ESKF/eskf_math.cpp` | 243 | `quaternion_integration`, `accel_to_quaternion`, `pv_integration`, `compute_F_matrix`, `covariance_prediction`, `inject_error_state`, `mag_observation_prediction`, `gps_to_local`, `pressure_to_altitude` | ✅ 使用中 |

### MEUKF実装

| ファイル | 行数 | 主要関数 | 状態 |
|---------|------|---------|------|
| `src/MEUKF/meukf_core.cpp` | 1346 | `step`, `predict`, `update_accel_meukf`, `update_mag_meukf`, `update_gps`, `update_baro`, `update_zupt`, `state_to_vars`, `vars_to_state` | ✅ 使用中 |
| `src/MEUKF/unified_filter.cpp` | 210 | `update`, `reset` | ✅ 使用中 |
| `MEUKF/meukf_core.cpp` | 1348 | (重複) | ❌ 削除推奨 |
| `MEUKF/unified_filter.cpp` | 164 | (重複、未完成) | ❌ 削除推奨 |

### 共通ライブラリ

| ファイル | 行数 | 主要関数 | 状態 |
|---------|------|---------|------|
| `src/Common/filter_management.cpp` | 173 | `hasNaNOrInf`, `setIdentityScaled`, `check_divergence`, `apply_zupt`, `normalize_covariance`, `check_state_divergence`, `check_zupt_condition`, `reset_state_on_divergence` | ✅ 使用中 |
| `src/Common/Sensor/sensor_preprocessor.cpp` | 140 | `preprocess_accel`, `preprocess_mag`, `preprocess_baro`, `preprocess_gps` | ✅ 使用中 |

### その他のフィルター

| ファイル | 行数 | 主要関数 | 状態 |
|---------|------|---------|------|
| `src/EKF/ekf_linear_update.cpp` | 127 | `ekf_linear_update` | ✅ 使用中 |
| `src/UKF/ukf_sigma_points.cpp` | 115 | `generate_sigma_points_dynamic` | ✅ 使用中 |

### テストファイル

| ファイル | 行数 | 主要関数 | 状態 |
|---------|------|---------|------|
| `tests/compare_cholesky.cpp` | - | - | ✅ 保持 |

## 関数一覧（カテゴリ別）

### MEX関数

#### mex_run_eskf
- `mexFunction` - メインエントリーポイント
- `do_init` - ESKF初期化
- `do_step` - ESKFステップ実行
- `do_get_state` - 状態取得
- `do_free` - メモリ解放
- `do_meukf_step` - MEUKFステップ実行
- `do_sensor_filter_reset_zero` - センサーフィルターリセット（ゼロ）
- `do_sensor_filter_reset` - センサーフィルターリセット
- `do_sensor_filter_update` - センサーフィルター更新

#### mex_meukf_step (非推奨)
- `mexFunction` - メインエントリーポイント（エラーを返す）
- `matlab_to_state` - MATLAB→C++状態変換
- `state_to_matlab` - C++→MATLAB状態変換

### ESKF Core関数

#### ESKFCore (静的メソッド)
- `integrate_nominal` - ノミナル状態積分（RK2）
- `predict_covariance` - 共分散予測
- `compute_F_matrix` - 状態遷移行列計算
- `update_accel` - 加速度更新（Roll/Pitch）
- `update_mag` - 磁気計更新
- `update_gps` - GPS更新
- `update_baro` - 気圧計更新
- `update_zupt` - ZUPT更新
- `compute_adaptive_Q` - 適応プロセスノイズ計算
- `inject_error_state` - 誤差状態注入
- `pressure_to_altitude` - 気圧→高度変換
- `gps_to_local` - GPS座標→ローカル座標変換

#### ESKFRunner
- `predict` - 予測ステップ
- `apply_accel_z_integration` - 加速度Z軸積分
- `apply_velocity_clipping` - 速度クリッピング
- `regularize_covariance` - 共分散正則化

#### ESKFInitializer
- `initialize_eskf_from_matlab` - MATLABデータからESKF初期化

#### ESKFSensorUpdates
- `update_accel_sensor` - 加速度センサー更新
- `update_mag_sensor` - 磁気センサー更新
- `update_baro_sensor` - 気圧センサー更新
- `update_gps_sensor` - GPSセンサー更新

#### ESKFPostprocess
- `predict_postprocess` - 予測後処理
- `update_state_from_dx` - 誤差状態から状態更新
- `symmetrize_covariance` - 共分散対称化

#### ESKFMath
- `quaternion_integration` - クォータニオン積分
- `accel_to_quaternion` - 加速度→クォータニオン
- `pv_integration` - 位置・速度積分
- `compute_F_matrix` - F行列計算
- `covariance_prediction` - 共分散予測
- `inject_error_state` - 誤差状態注入
- `mag_observation_prediction` - 磁気観測予測
- `gps_to_local` - GPS→ローカル座標変換
- `pressure_to_altitude` - 気圧→高度変換

### MEUKF Core関数

#### MEUKFCore (静的メソッド)
- `step` - MEUKFメインループ
- `predict` - 状態予測
- `update_accel_meukf` - UKF加速度更新（2D観測）
- `update_mag_meukf` - UKF磁気更新（3D観測）
- `update_gps` - 線形KF GPS更新
- `update_baro` - 線形KF気圧更新
- `update_zupt` - 線形KF ZUPT更新
- `state_to_vars` - 状態→変数変換
- `vars_to_state` - 変数→状態変換

#### UnifiedFilter
- `update` - 統一フィルター更新
- `reset` - リセット

### 共通関数

#### FilterManagement
- `hasNaNOrInf` - NaN/Infチェック
- `setIdentityScaled` - スケール付き単位行列設定
- `check_divergence` - 発散チェック
- `apply_zupt` - ZUPT適用
- `normalize_covariance` - 共分散正規化
- `check_state_divergence` - 状態発散チェック
- `check_zupt_condition` - ZUPT条件チェック
- `reset_state_on_divergence` - 発散時の状態リセット

#### SensorPreprocessor
- `preprocess_accel` - 加速度前処理
- `preprocess_mag` - 磁気前処理
- `preprocess_baro` - 気圧前処理
- `preprocess_gps` - GPS前処理

### 型変換関数

#### mex_conv
- `mxArrayToFloatArray` - MATLAB配列→float配列
- `floatArrayToMxArray` - float配列→MATLAB配列（double）
- `floatArrayToMxArrayFloat` - float配列→MATLAB配列（single）
- `mxGetScalarAsFloat` - MATLABスカラー→float

#### mex_type_conversion (テンプレート)
- `matToVector` - MATLAB配列→Vector
- `matToMatrix` - MATLAB配列→Matrix
- `vectorToMat` - Vector→MATLAB配列
- `matrixToMat` - Matrix→MATLAB配列

### その他のフィルター関数

#### EKF
- `ekf_linear_update` - 線形EKF更新

#### UKF
- `generate_sigma_points_dynamic` - シグマポイント生成

## 関数統計

### 総関数数
- MEX関数: 9
- ESKF関数: 30+
- MEUKF関数: 9
- 共通関数: 12
- 型変換関数: 8
- その他: 2

### 総行数（概算）
- MEX: ~450行
- ESKF: ~2000行
- MEUKF: ~1500行
- 共通: ~300行
- その他: ~250行
- **合計: ~4500行**

## ファイル依存関係マトリックス

| ファイル | 依存する主要ファイル |
|---------|---------------------|
| `mex_run_eskf.cpp` | `mex_run_eskf_impl.hpp`, `mex_eskf_common.hpp` |
| `eskf_runner.cpp` | `eskf_core.hpp`, `eskf_postprocess.hpp`, `sensor_filter.hpp` |
| `eskf_core.cpp` | `eskf_math.hpp`, `kalman_filter_core.hpp`, `quaternion.hpp` |
| `meukf_core.cpp` | `meukf_types.hpp`, `fixed_matrix.hpp`, `quaternion.hpp` |
| `unified_filter.cpp` | `unified_filter.hpp`, `meukf_core.hpp`, `sensor_filter.hpp` |

## 名前空間マッピング

| 名前空間 | ファイル | 主要クラス/関数 |
|---------|---------|----------------|
| `eskf` | `src/ESKF/*.cpp` | `ESKFCore`, `ESKFRunner`, `ESKFState` |
| `meukf` | `src/MEUKF/*.cpp` | `MEUKFCore`, `UnifiedFilter`, `State` |
| `common::filter` | `src/Common/filter_management.cpp` | フィルター管理関数 |
| `common::sensor` | `src/Common/Sensor/sensor_preprocessor.cpp` | センサー前処理関数 |
| `mex_conv` | `MEX/mex_type_conv.hpp` | 型変換関数 |
| `mex_run_eskf_impl` | `Inc/MEX/mex_run_eskf_impl.hpp` | MEX実装関数 |

## ビルド対象ファイル

### 必須ファイル（MEXビルド）
1. `MEX/mex_run_eskf.cpp`
2. `src/ESKF/eskf_core.cpp`
3. `src/ESKF/eskf_runner.cpp`
4. `src/ESKF/eskf_initializer.cpp`
5. `src/ESKF/eskf_sensor_updates.cpp`
6. `src/ESKF/eskf_postprocess.cpp`
7. `src/ESKF/eskf_math.cpp`
8. `src/MEUKF/meukf_core.cpp`
9. `src/MEUKF/unified_filter.cpp`
10. `src/Common/filter_management.cpp`
11. `src/Common/Sensor/sensor_preprocessor.cpp`
12. `src/EKF/ekf_linear_update.cpp`
13. `src/UKF/ukf_sigma_points.cpp`

### オプションファイル
- `MEX/mex_meukf_step.cpp` (非推奨)

### 削除推奨ファイル
- `MEUKF/meukf_core.cpp`
- `MEUKF/unified_filter.cpp`
- `MEUKF/meukf_types.hpp`

