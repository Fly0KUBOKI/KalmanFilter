# MEXファイル ビルド状況と推奨事項

## ビルドスクリプトの分類

### ✅ ビルド対象（約30ファイル）

#### 基本ユーティリティ
- `mex_matlab_helpers.cpp`
- `mex_sensor_preprocessor.cpp`

#### ESKF関連
- `mex_adaptive_predict.cpp` (Src/ESKF/eskf_core.cppとリンク)
- `mex_eskf_constructor.cpp`
- `mex_eskf_init.cpp`
- `mex_eskf_get_state.cpp`
- `mex_eskf_set_state.cpp`
- `mex_eskf_free.cpp`
- `mex_eskf_step_handle.cpp`
- `mex_eskf_math.cpp` (Src/ESKF/eskf_math.cppとリンク)
- `mex_eskf_do_update.cpp`
- `mex_eskf_sensor_update.cpp`
- `mex_eskf_sensor_updates.cpp`
- `mex_eskf_sensor_updates_full.cpp`
- `mex_eskf_step.cpp` (Src/MEUKF/unified_filter.cpp, meukf_core.cppとリンク)
- `mex_eskf_full.cpp`
- `mex_run_eskf.cpp`
- `mex_eskf_predict_postprocess.cpp` (ソース削除済み→再作成必要)
- `mex_eskf_update_postprocess.cpp` (ソース削除済み→再作成必要)
- `mex_eskf_zupt.cpp` (ソース削除済み→再作成必要)

#### フィルタコア
- `mex_kalman_filter_core.cpp`
- `mex_ekf.cpp` (Src/EKF/ekf_linear_update.cppとリンク)
- `mex_ukf.cpp` (Src/EKF/ekf_linear_update.cppとリンク)
- `mex_ukf_sigma_points.cpp` (Src/UKF/ukf_sigma_points.cppとリンク)
- `mex_ukf_update.cpp`
- `mex_meukf_step.cpp` (Src/MEUKF/meukf_core.cppとリンク、出力名: `mex_meukf_step_v2`)
- `mex_unified_filter.cpp` (Src/MEUKF/unified_filter.cpp, meukf_core.cppとリンク)

#### センサー・フィルタ管理
- `mex_sensor_filter.cpp`
- `mex_filter_management.cpp`

### ❌ ビルドスキップ（明示的）

| ファイル | 理由 | 推奨アクション |
|---------|------|--------------|
| `mex_eskf_core.cpp` | "Skip locked legacy target" | 使用されていない場合は削除 |
| `mex_quaternion_lib.cpp` | "locked/skipped" | 使用されている場合はビルド対象に追加 |

### ❓ ビルド対象外（build_mex.mに記載なし）

以下のファイルはソースが存在するが、`build_mex.m`に記載がありません：

| ファイル | バイナリ存在 | 使用状況 | 推奨アクション |
|---------|------------|---------|--------------|
| `mex_eskf_core_v2.cpp` | ❌ | 不明 | 使用状況を確認 |
| `mex_eskf_do_cpp_update.cpp` | ❌ | 不明 | `mex_eskf_do_update`との違いを確認 |
| `mex_eskf_sensor_update_full.cpp` | ❌ | 不明 | `mex_eskf_sensor_update`との違いを確認 |
| `mex_eskf_helper.cpp` | ❌ | 不明 | 使用状況を確認 |
| `mex_kf_core.cpp` | ❌ | 不明 | 使用状況を確認 |
| `mex_kalman_compute.cpp` | ✅ | 不明 | 使用状況を確認、必要ならビルド対象に追加 |
| `mex_common_lib.cpp` | ❌ | 不明 | 使用状況を確認 |
| `mex_filter_utils.cpp` | ❌ | 不明 | 使用状況を確認 |
| `mex_quaternion_lib.cpp` | ✅ | ✅ (`mex_matlab_helpers.m`) | ビルド対象に追加を検討 |
| `mex_ukf_update_minimal.cpp` | ✅ | 不明 | `mex_ukf_update`との違いを確認 |

## ビルドエラー

### ソースが削除されたファイル

以下のファイルは以前存在していたが、現在削除されています：

1. `mex_eskf_predict_postprocess.cpp` - 再作成済み（2025-12-29）
2. `mex_eskf_update_postprocess.cpp` - 再作成済み（2025-12-29）
3. `mex_eskf_zupt.cpp` - 再作成済み（2025-12-29）

**注意:** これらのファイルは再作成されましたが、実装が完全でない可能性があります。

## 推奨アクション

### 即座に実行すべき

1. **MEXフォルダ内のバイナリを削除**
   - `mex_eskf_predict_postprocess.mexw64`
   - `mex_eskf_update_postprocess.mexw64`
   - `mex_eskf_zupt.mexw64`
   - `mex_eskf_reset.mexw64`

2. **binフォルダの古いバイナリを削除**
   - `mex_eskf_run.mexw64` (正しくは`mex_run_eskf.mexw64`)
   - `mex_meukf_step.mexw64` (正しくは`mex_meukf_step_v2.mexw64`)

### 調査が必要

1. **使用状況の確認**
   - `mex_quaternion_lib.cpp` - `mex_matlab_helpers.m`で使用されている
   - `mex_kalman_compute.cpp` - 使用箇所を検索
   - `mex_common_lib.cpp` - 使用箇所を検索
   - `mex_filter_utils.cpp` - 使用箇所を検索
   - `mex_eskf_helper.cpp` - 使用箇所を検索

2. **重複ファイルの整理**
   - `mex_eskf_do_update.cpp` vs `mex_eskf_do_cpp_update.cpp`
   - `mex_eskf_sensor_update.cpp` vs `mex_eskf_sensor_update_full.cpp`
   - `mex_ukf_update.cpp` vs `mex_ukf_update_minimal.cpp`

### ビルドスクリプトの改善

1. **ビルド対象に追加**
   ```matlab
   % mex_quaternion_lib (mex_matlab_helpers.mで使用)
   if exist('mex_quaternion_lib.cpp', 'file')
       if wants('mex_quaternion_lib') && build_single_mex('mex_quaternion_lib.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
           built_count = built_count + 1;
       end
   end
   ```

2. **使用されていないファイルの削除**
   - 使用箇所が見つからないファイルは削除を検討

## ビルド順序の推奨

依存関係を考慮したビルド順序：

1. **基本ユーティリティ**（依存なし）
   - `mex_matlab_helpers`
   - `mex_sensor_preprocessor`
   - `mex_type_conv.hpp` (ヘッダーのみ)

2. **数学・計算ライブラリ**（基本ユーティリティに依存）
   - `mex_quaternion_lib`
   - `mex_kalman_compute`
   - `mex_common_lib`

3. **フィルタコア**（数学ライブラリに依存）
   - `mex_kalman_filter_core`
   - `mex_ekf`
   - `mex_ukf_sigma_points`
   - `mex_ukf`
   - `mex_meukf_step`
   - `mex_unified_filter`

4. **ESKF基本機能**（フィルタコアに依存）
   - `mex_eskf_math`
   - `mex_adaptive_predict`
   - `mex_eskf_constructor`

5. **ESKF更新機能**（ESKF基本機能に依存）
   - `mex_sensor_filter`
   - `mex_eskf_do_update`
   - `mex_eskf_sensor_updates_full`
   - `mex_eskf_predict_postprocess`
   - `mex_eskf_update_postprocess`
   - `mex_eskf_zupt`

6. **ESKF統合**（すべての機能に依存）
   - `mex_eskf_step`
   - `mex_run_eskf`
   - `mex_eskf_full`


