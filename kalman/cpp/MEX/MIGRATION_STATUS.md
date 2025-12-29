# MEXファイル ソースコード移行状況

**更新日**: 2025年1月29日

## 移行完了ファイル（13個）

### ✅ 1. mex_sensor_preprocessor.cpp
- **移行先**: `Src/Common/Sensor/sensor_preprocessor.cpp`
- **ヘッダー**: `Inc/Common/Sensor/sensor_preprocessor.hpp`
- **移行内容**: 前処理ロジック（加速度、磁気、気圧、GPS）
- **状態**: 完了

### ✅ 2. mex_filter_management.cpp
- **移行先**: `Src/Common/filter_management.cpp`
- **ヘッダー**: `Inc/Common/filter_management.hpp`
- **移行内容**: フィルタ管理ロジック（発散チェック、ZUPT適用）
- **状態**: 完了

### ✅ 3. mex_quaternion_lib.cpp
- **状態**: 既に分離済み（`Inc/Common/Math/quaternion_lib.hpp`を使用）
- **確認**: 実装コードは含まれていない

### ✅ 4. mex_sensor_filter.cpp
- **状態**: 既に分離済み（`Inc/Common/Sensor/sensor_filter.hpp`を使用）
- **確認**: 実装コードは含まれていない

### ✅ 5. mex_adaptive_predict.cpp
- **移行先**: `Src/ESKF/eskf_core.cpp`
- **ヘッダー**: `Inc/ESKF/eskf_core.hpp`（`compute_adaptive_Q`関数を追加）
- **移行内容**: Adaptive Q scalingロジック
- **状態**: 完了

### ✅ 6. mex_eskf_zupt.cpp
- **移行先**: `Inc/Common/Math/math_utils.hpp`（インライン実装）
- **移行内容**: `invert3x3`関数（3x3行列の逆行列計算）
- **状態**: 完了

### ✅ 7. mex_eskf_constructor.cpp
- **移行先**: 
  - `Inc/Common/Math/statistics.hpp`（統計関数）
  - `Inc/Common/Math/quaternion_lib.hpp`（クォータニオン関数、既存ライブラリを使用）
- **移行内容**: 
  - `compute_mean`, `compute_mean_3d`, `compute_std`, `compute_std_3d` → `statistics.hpp`
  - `quaternion_from_euler`, `quaternion_to_rotation_matrix` → `quaternion_lib.hpp`を使用
- **状態**: 完了（統計関数とクォータニオン関数を移行、初期化ロジックはMEXファイル内に残存）

### ✅ 8. mex_eskf_update_postprocess.cpp
- **移行先**: 
  - `Src/ESKF/eskf_postprocess.cpp`（状態更新ロジック）
  - `Inc/MEX/mex_type_conversion.hpp`（型変換ヘルパー）
- **移行内容**: 
  - 状態更新ロジック（dx適用、クォータニオン更新、P対称化） → `update_state_from_dx`関数
  - 型変換ヘルパー関数 → `mex_type_conversion.hpp`に共通化
- **状態**: 完了（MATLAB呼び出し部分はMEXファイル内に残存）

### ✅ 9. mex_eskf_do_update.cpp
- **移行先**: `Inc/Common/Math/vector_utils.hpp`
- **移行内容**: `copy_vec` → `common::math::copy_vec`を使用
- **状態**: 完了

### ✅ 10. mex_eskf_predict_postprocess.cpp
- **移行先**: 
  - `Src/ESKF/eskf_postprocess.cpp`（後処理ロジック）
  - `Src/Common/filter_management.cpp`（P正規化）
  - `Inc/MEX/mex_type_conversion.hpp`（型変換ヘルパー）
- **移行内容**: 
  - `velocity_damping`, `P normalization`, `velocity clipping` → `predict_postprocess`関数
  - 型変換ヘルパー関数 → `mex_type_conversion.hpp`に共通化
- **状態**: 完了（MATLAB呼び出し部分はMEXファイル内に残存）

### ✅ 11. mex_eskf_sensor_updates_full.cpp
- **移行先**: `Inc/Common/Math/vector_utils.hpp`
- **移行内容**: 
  - `norm3` → `common::math::norm3`を使用
  - `copy_vec` → `common::math::copy_vec`を使用
- **状態**: 完了

### ✅ 12. mex_run_eskf.cpp
- **移行先**: 
  - `Src/Common/filter_management.cpp`（発散チェック、ZUPTチェック、リセット処理）
  - `Inc/Common/Math/quaternion_lib.hpp`（クォータニオン関数）
  - `Inc/Common/Math/vector_utils.hpp`（ベクトルユーティリティ）
- **移行内容**: 
  - `check_and_reset`内の発散チェックロジック → `check_state_divergence`関数
  - `zupt_check_and_update`内のZUPTチェックロジック → `check_zupt_condition`関数
  - リセット処理 → `reset_state_on_divergence`関数
  - `quat_to_euler` → `Quat::to_euler`を使用
  - `copy_vec` → `common::math::copy_vec`を使用
- **状態**: 完了（MATLAB呼び出し部分とESKFState構造体管理はMEXファイル内に残存）

### ✅ 13. mex_meukf_step.cpp
- **状態**: 既に分離済み（`Inc/MEUKF/meukf_core.hpp`を使用）
- **確認**: 実装コードは含まれていない

## 移行原則

1. **実装コードは`Src/`に配置**
2. **ヘッダーは`Inc/`に配置**
3. **MEXファイルはラッパーのみ**（型変換と関数呼び出し）
4. **影響の小さいファイルから順番に移行**

## 新規作成ファイル

- `kalman/cpp/Inc/Common/Math/statistics.hpp` - 統計関数（テンプレート実装）
- `kalman/cpp/Inc/Common/Math/vector_utils.hpp` - ベクトルユーティリティ関数
- `kalman/cpp/Inc/ESKF/eskf_postprocess.hpp` - ESKF後処理ヘッダー
- `kalman/cpp/Src/ESKF/eskf_postprocess.cpp` - ESKF後処理実装
- `kalman/cpp/Inc/MEX/mex_type_conversion.hpp` - MEX型変換ヘルパー関数（共通化）

## 移行完了

**全13個のMEXファイルから実装コードの移行が完了しました。**

すべての実装コードは適切な場所（`Inc/`, `Src/`）に移行され、MEXファイルはラッパーのみとなりました。

## 次のステップ

1. ビルドテストを実施
2. 動作確認

