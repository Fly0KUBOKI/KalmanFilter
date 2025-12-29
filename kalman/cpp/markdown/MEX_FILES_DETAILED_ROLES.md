# MEXファイル詳細役割一覧

## ESKF関連ファイル

### mex_eskf_constructor.cpp
**役割:** ESKF状態の初期化（静止データから姿勢推定）
**入力:** `('init', obs, static_time, dt)`
**出力:** ESKF状態構造体
**依存:** なし（MATLABロジックをC++で実装）
**使用箇所:** `mex_run_eskf.cpp`

### mex_eskf_init.cpp
**役割:** 状態ハンドルの初期化（メモリ管理）
**入力:** `(state_struct)`
**出力:** `state_handle` (uint64)
**依存:** なし
**使用箇所:** 不明（古いAPIの可能性）

### mex_eskf_get_state.cpp
**役割:** 状態ハンドルから状態を取得
**入力:** `(state_handle)`
**出力:** 状態構造体 `{p, v, q, ba, bg, P}`
**依存:** なし
**使用箇所:** 不明（古いAPIの可能性）

### mex_eskf_set_state.cpp
**役割:** 状態ハンドルに状態を設定
**入力:** `(state_handle, state_struct)`
**出力:** なし
**依存:** なし
**使用箇所:** 不明（古いAPIの可能性）

### mex_eskf_free.cpp
**役割:** 状態ハンドルの解放
**入力:** `(state_handle)`
**出力:** なし
**依存:** なし
**使用箇所:** 不明（古いAPIの可能性）

### mex_adaptive_predict.cpp
**役割:** 予測ステップ（C++実装のラッパー）
**入力:** `(p, v, q, ba, bg, P, a_meas, w_meas, dt, Q_nominal, adaptive_q_enabled, ...)`
**出力:** `(p_new, v_new, q_new, ba_new, bg_new, P_new)`
**依存:** `Src/ESKF/eskf_core.cpp`
**使用箇所:** `mex_run_eskf.cpp`, `mex_eskf_full.cpp`

### mex_eskf_predict_postprocess.cpp
**役割:** 予測後の後処理（速度ダンピング、加速度Z統合制御）
**入力:** `('postprocess', v, q, P, a, dt, g, enable_accel_z, accel_z_threshold, accel_z_damping, velocity_damping)`
**出力:** `(v_out, P_out)`
**依存:** なし
**使用箇所:** `mex_run_eskf.cpp`, `mex_eskf_full.cpp`

### mex_eskf_do_update.cpp
**役割:** センサー更新（do_cpp_updateのMEX化）
**入力:** `(sensor_type, meas, p, v, q, ba, bg, P, g, dt, [sample])`
**出力:** `(p, v, q, ba, bg, P, should_skip)`
**依存:** `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess`
**使用箇所:** `mex_eskf_sensor_updates_full.cpp`

### mex_eskf_do_cpp_update.cpp
**役割:** do_cpp_updateの別実装（？）
**入力:** 不明
**出力:** 不明
**依存:** `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess`
**使用箇所:** 不明
**状態:** ビルド対象外、使用状況不明

### mex_eskf_sensor_update.cpp
**役割:** 単一センサー更新
**入力:** `(sensor_type, meas, state_struct, [sample])`
**出力:** 更新された状態構造体
**依存:** `mex_sensor_preprocessor`, `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess`
**使用箇所:** 不明

### mex_eskf_sensor_update_full.cpp
**役割:** センサー更新（完全版）
**入力:** 不明
**出力:** 不明
**依存:** `mex_sensor_preprocessor`
**使用箇所:** 不明
**状態:** ビルド対象外、使用状況不明

### mex_eskf_sensor_updates.cpp
**役割:** 複数センサー更新
**入力:** 不明
**出力:** 不明
**依存:** 不明
**使用箇所:** 不明

### mex_eskf_sensor_updates_full.cpp
**役割:** 複数センサー更新（完全版）
**入力:** `(sensor_type, meas, state_struct, [sample])`
**出力:** 更新された状態構造体
**依存:** `mex_sensor_preprocessor`, `mex_eskf_do_update`
**使用箇所:** `mex_run_eskf.cpp`

### mex_eskf_update_postprocess.cpp
**役割:** 更新後の後処理（誤差状態注入、外れ値検出）
**入力:** `('postprocess', sensor_type, dx, innov, p, v, q, ba, bg, P_old, P_new, sample)`
**出力:** `(p, v, q, ba, bg, P, should_skip)`
**依存:** なし
**使用箇所:** `mex_eskf_do_update.cpp`, `mex_eskf_sensor_update.cpp`, `mex_eskf_do_cpp_update.cpp`

### mex_eskf_step.cpp
**役割:** 統合ESKFステップ（予測+更新を1回の呼び出しで実行）
**入力:** `(state_in, sensor_data, params)`
**出力:** `(state_out)`
**依存:** `Inc/MEUKF/unified_filter.hpp`, `Src/MEUKF/unified_filter.cpp`, `Src/MEUKF/meukf_core.cpp`
**使用箇所:** 不明（新しいAPI）

### mex_eskf_step_handle.cpp
**役割:** 状態ハンドル版ステップ
**入力:** `(state_handle, input_struct)`
**出力:** 更新された状態構造体
**依存:** `mex_unified_filter`
**使用箇所:** 不明（古いAPIの可能性）

### mex_run_eskf.cpp
**役割:** ESKF.m完全置き換え（ハンドルベースAPI）
**入力:** 
- `('init', obs, static_time, dt)` → `handle`
- `('step', handle, obs, k)` → なし
- `('get_state', handle)` → `state`
- `('free', handle)` → なし
**出力:** ハンドルまたは状態
**依存:** 多数のMEX関数（`mex_adaptive_predict`, `mex_eskf_predict_postprocess`, `mex_eskf_sensor_updates_full`, `mex_eskf_zupt`, `mex_filter_management`, `mex_eskf_constructor`）
**使用箇所:** ESKF.mの代替

### mex_eskf_full.cpp
**役割:** 完全MEX化ESKF（クラスベースAPI）
**入力:** 不明
**出力:** 不明
**依存:** `mex_adaptive_predict`, `mex_eskf_predict_postprocess`
**使用箇所:** 不明

### mex_eskf_math.cpp
**役割:** ESKF数学関数群（状態非依存の計算関数）
**入力:** `('function_name', args...)`
**関数一覧:**
- `quaternion_integration`
- `accel_to_quaternion`
- `pv_integration`
- `compute_F_matrix`
- `covariance_prediction`
- `inject_error_state`
- `kalman_update`
- `mag_observation_prediction`
- `gps_to_local`
- `pressure_to_altitude`
**依存:** `Inc/ESKF/eskf_math.hpp`, `Src/ESKF/eskf_math.cpp`
**使用箇所:** 不明（直接使用されていない可能性）

### mex_eskf_helper.cpp
**役割:** ESKFヘルパー関数
**入力:** 
- `('inject_error_state', nominal, dx)`
- `('regularize', P, [eps])`
**依存:** `Inc/ESKF/eskf_helper.hpp`
**使用箇所:** 不明
**状態:** ビルド対象外

### mex_eskf_core.cpp
**役割:** ESKFコア関数（legacy）
**入力:** `('function_name', ...args)`
**関数一覧:**
- `integrate_nominal`
- `predict_covariance`
**依存:** 不明
**使用箇所:** 不明
**状態:** ビルドスキップ（locked legacy）

### mex_eskf_core_v2.cpp
**役割:** ESKFコア関数（v2）
**入力:** `('function_name', ...args)`
**関数一覧:**
- `integrate_nominal`
- `update_accel`
- `update_mag`
- `update_gps`
- `update_baro`
**依存:** 不明
**使用箇所:** 不明
**状態:** ビルド対象外

### mex_eskf_zupt.cpp
**役割:** Zero Velocity Update（ZUPT）
**入力:** `('update', v, P)`
**出力:** `(v_out, P_out)`
**依存:** なし
**使用箇所:** `mex_run_eskf.cpp`

## フィルタコア

### mex_kalman_filter_core.cpp
**役割:** 基本カルマンフィルタ
**入力:** 不明
**出力:** 不明
**依存:** なし
**使用箇所:** 不明

### mex_kf_core.cpp
**役割:** KFコア
**入力:** 不明
**出力:** 不明
**依存:** 不明
**使用箇所:** 不明
**状態:** ビルド対象外

### mex_ekf.cpp
**役割:** 拡張カルマンフィルタ
**入力:** 不明
**出力:** 不明
**依存:** `Src/EKF/ekf_linear_update.cpp`
**使用箇所:** 不明

### mex_ukf.cpp
**役割:** Unscented Kalman Filter
**入力:** 不明
**出力:** 不明
**依存:** `Src/EKF/ekf_linear_update.cpp`
**使用箇所:** 不明

### mex_ukf_sigma_points.cpp
**役割:** UKFシグマポイント生成
**入力:** 不明
**出力:** 不明
**依存:** `Src/UKF/ukf_sigma_points.cpp`
**使用箇所:** 不明

### mex_ukf_update.cpp
**役割:** UKF更新ステップ
**入力:** `(x, P, z, h_func, R, alpha, beta, kappa)`
**出力:** `(x_upd, P_upd, K, S, y)`
**依存:** なし（MATLAB関数`h_func`を呼び出し）
**使用箇所:** 不明

### mex_ukf_update_minimal.cpp
**役割:** UKF更新（最小版）
**入力:** 不明
**出力:** 不明
**依存:** なし
**使用箇所:** 不明
**状態:** ビルド対象外、`mex_ukf_update`との違い不明

### mex_meukf_step.cpp
**役割:** MEUKFステップ
**入力:** 不明
**出力:** 不明
**依存:** `Src/MEUKF/meukf_core.cpp`
**使用箇所:** `mex_eskf_do_update.cpp`（`mex_meukf_step_v2`として）
**状態:** `mex_meukf_step_v2`としてビルド

### mex_unified_filter.cpp
**役割:** 統一フィルタインターフェース
**入力:** `(prev_state, input)`
**出力:** `(new_state)`
**依存:** `Inc/MEUKF/unified_types.hpp`, `Inc/MEUKF/unified_filter.hpp`, `Src/MEUKF/unified_filter.cpp`, `Src/MEUKF/meukf_core.cpp`
**使用箇所:** `mex_eskf_step.cpp`, `mex_eskf_step_handle.cpp`

## ユーティリティ

### mex_matlab_helpers.cpp
**役割:** MATLABヘルパー関数
**入力:** 不明
**出力:** 不明
**依存:** なし
**使用箇所:** 不明

### mex_sensor_preprocessor.cpp
**役割:** センサーデータ前処理
**入力:** 不明
**出力:** 不明
**依存:** なし
**使用箇所:** `mex_eskf_sensor_updates_full.cpp`, `mex_eskf_sensor_update.cpp`, `mex_eskf_sensor_update_full.cpp`

### mex_sensor_filter.cpp
**役割:** センサーフィルタ（ノイズ推定、R行列取得）
**入力:** 
- `('get_R', sensor_type)`
- `('noise_estimate', sensor_type, innov, H, P)`
- `('reset')`
- `('reset_zero')`
- `('log', 'on'/'off')`
**依存:** `Inc/Common/Sensor/sensor_filter.hpp`, `Inc/Common/Math/fixed_matrix.hpp`
**使用箇所:** `mex_eskf_do_update.cpp`, `mex_eskf_sensor_update.cpp`, `mex_eskf_do_cpp_update.cpp`

### mex_filter_management.cpp
**役割:** フィルタ管理（発散検出、リセット）
**入力:**
- `('check_divergence', P)`
- `('reset_state', p, v, q, ba, bg, P, reset_scale)`
**依存:** `Inc/Common/Math/fixed_matrix.hpp`
**使用箇所:** `mex_run_eskf.cpp`

### mex_quaternion_lib.cpp
**役割:** クォータニオンライブラリ
**入力:** 
- `('to_rotation_matrix', q)`
- `('to_euler', q)`
- `('from_euler', roll, pitch, yaw)`
- `('normalize', q)`
- `('multiply', q1, q2)`
- `('small_angle_quat', theta)`
- `('integrate', q, omega, dt)`
- その他多数
**依存:** `Inc/Common/Math/quaternion_lib.hpp`
**使用箇所:** `mex_matlab_helpers.m`
**状態:** ビルドスキップ（locked）、使用されているためビルド対象に追加を検討

### mex_kalman_compute.cpp
**役割:** 統一計算関数群（状態非依存）
**入力:** `('function_name', input)`
**関数一覧:**
- Quaternion関数: `quat_multiply`, `quat_normalize`, `quat_conjugate`, `quat_inverse`
- Rotation関数: `rotm_multiply`, `rotm_transpose`, `rotm_inverse`
- その他多数
**依存:** `Inc/Common/Math/quaternion_compute.hpp`, `Inc/Common/Math/rotation_compute.hpp`
**使用箇所:** 不明
**状態:** ビルド対象外

### mex_common_lib.cpp
**役割:** 共通ライブラリ
**入力:** `('function_name', ...args)`
**関数一覧:**
- MathUtils: `wrap_to_pi`, `normalize_vector`, `skew_symmetric`, `enforce_symmetry`
- QuaternionLib: `quat_multiply`, `quat_normalize`, 等
**依存:** 不明
**使用箇所:** 不明
**状態:** ビルド対象外

### mex_filter_utils.cpp
**役割:** フィルタユーティリティ
**入力:** 不明
**出力:** 不明
**依存:** 不明
**使用箇所:** 不明
**状態:** ビルド対象外

### mex_type_conv.hpp
**役割:** 型変換ヘルパー（ヘッダーファイル）
**機能:** MATLAB配列とC++型の相互変換
**依存:** なし
**使用箇所:** 多くのMEXファイルでインクルード

## まとめ

### 主要なAPIパターン

1. **ハンドルベースAPI**（古い）
   - `mex_eskf_init` → `mex_eskf_get_state` → `mex_eskf_set_state` → `mex_eskf_free`
   - `mex_eskf_step_handle`

2. **構造体ベースAPI**（中程度）
   - `mex_eskf_step`
   - `mex_eskf_sensor_updates_full`
   - `mex_adaptive_predict`

3. **完全統合API**（新しい）
   - `mex_run_eskf`（ESKF.m完全置き換え）
   - `mex_eskf_full`

### 推奨される使用パターン

- **新規開発**: `mex_run_eskf`を使用
- **既存コード**: 段階的に`mex_run_eskf`へ移行
- **低レベルアクセス**: `mex_eskf_step`または個別のMEX関数を使用




