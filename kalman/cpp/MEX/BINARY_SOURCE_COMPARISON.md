# バイナリとソースファイルの比較

## binディレクトリのバイナリ（13個）

1. mex_adaptive_predict.mexw64
2. mex_eskf_constructor.mexw64
3. mex_eskf_do_update.mexw64
4. mex_eskf_predict_postprocess.mexw64
5. mex_eskf_sensor_updates_full.mexw64
6. mex_eskf_update_postprocess.mexw64
7. mex_eskf_zupt.mexw64
8. mex_filter_management.mexw64
9. mex_meukf_step_v2.mexw64
10. mex_quaternion_lib.mexw64
11. mex_run_eskf.mexw64
12. mex_sensor_filter.mexw64
13. mex_sensor_preprocessor.mexw64

## MEXディレクトリのソースファイル（16個）

1. mex_adaptive_predict.cpp → ✅ バイナリあり
2. mex_common_lib.cpp → ❌ バイナリなし
3. mex_eskf_constructor.cpp → ✅ バイナリあり
4. mex_eskf_core_v2.cpp → ❌ バイナリなし
5. mex_eskf_do_cpp_update.cpp → ❌ バイナリなし
6. mex_eskf_do_update.cpp → ✅ バイナリあり
7. mex_eskf_helper.cpp → ❌ バイナリなし
8. mex_eskf_sensor_update_full.cpp → ❌ バイナリなし（mex_eskf_sensor_updates_full.mexw64は別ファイルから）
9. mex_eskf_sensor_updates_full.cpp → ✅ バイナリあり
10. mex_filter_management.cpp → ✅ バイナリあり
11. mex_filter_utils.cpp → ❌ バイナリなし
12. mex_kf_core.cpp → ❌ バイナリなし
13. mex_quaternion_lib.cpp → ✅ バイナリあり
14. mex_run_eskf.cpp → ✅ バイナリあり
15. mex_sensor_filter.cpp → ✅ バイナリあり
16. mex_sensor_preprocessor.cpp → ✅ バイナリあり

## 重要な発見

### バイナリはあるがソースがない（4個）

1. **mex_meukf_step_v2.mexw64** ❌
   - ソース: `mex_meukf_step.cpp` がMEXディレクトリに存在しない
   - build_mex.mでは `mex_meukf_step.cpp` を `mex_meukf_step_v2` としてコンパイルしている
   - ソースファイルが削除されたか、別の場所にある可能性

2. **mex_eskf_predict_postprocess.mexw64** ❌
   - ソース: `mex_eskf_predict_postprocess.cpp` がMEXディレクトリに存在しない
   - build_mex.mでは存在チェックをしているが、ファイルが見つからない

3. **mex_eskf_update_postprocess.mexw64** ❌
   - ソース: `mex_eskf_update_postprocess.cpp` がMEXディレクトリに存在しない
   - build_mex.mでは存在チェックをしているが、ファイルが見つからない

4. **mex_eskf_zupt.mexw64** ❌
   - ソース: `mex_eskf_zupt.cpp` がMEXディレクトリに存在しない
   - build_mex.mでは存在チェックをしているが、ファイルが見つからない

### ソースはあるがバイナリがない（7個）

1. **mex_common_lib.cpp** ❌
   - バイナリなし
   - build_mex.mにコンパイル設定なし
   - MATLABコードで使用されていない

2. **mex_eskf_core_v2.cpp** ❌
   - バイナリなし
   - build_mex.mにコンパイル設定なし
   - MATLABコードで使用されていない

3. **mex_eskf_do_cpp_update.cpp** ❌
   - バイナリなし
   - build_mex.mにコンパイル設定なし
   - mex_eskf_do_update.cppと重複機能の可能性

4. **mex_eskf_helper.cpp** ❌
   - バイナリなし
   - build_mex.mにコンパイル設定なし
   - MATLABコードで使用されていない

5. **mex_eskf_sensor_update_full.cpp** ❌
   - バイナリなし
   - build_mex.mにコンパイル設定なし
   - mex_eskf_sensor_updates_full.cppと重複機能の可能性

6. **mex_filter_utils.cpp** ❌
   - バイナリなし
   - build_mex.mにコンパイル設定なし
   - MATLABコードで使用されていない

7. **mex_kf_core.cpp** ❌
   - バイナリなし
   - build_mex.mにコンパイル設定なし
   - MATLABコードで使用されていない

## まとめ

### バイナリはあるがソースがない（4個）⚠️
これらのバイナリは存在しますが、対応するソースファイルがMEXディレクトリにありません：
1. `mex_meukf_step_v2.mexw64` → `mex_meukf_step.cpp` が存在しない
2. `mex_eskf_predict_postprocess.mexw64` → `mex_eskf_predict_postprocess.cpp` が存在しない
3. `mex_eskf_update_postprocess.mexw64` → `mex_eskf_update_postprocess.cpp` が存在しない
4. `mex_eskf_zupt.mexw64` → `mex_eskf_zupt.cpp` が存在しない

**状況**: build_mex.mでは`exist()`チェックでこれらのファイルを探していますが、見つからないためコンパイルされていません。バイナリは以前のビルドで作成されたものです。

### ソースはあるがバイナリがない（7個）✅
これらのソースファイルは存在しますが、バイナリがありません：
1. `mex_common_lib.cpp` - build_mex.mにコンパイル設定なし、MATLABコードで未使用
2. `mex_eskf_core_v2.cpp` - build_mex.mにコンパイル設定なし、MATLABコードで未使用
3. `mex_eskf_do_cpp_update.cpp` - build_mex.mにコンパイル設定なし、mex_eskf_do_update.cppと重複
4. `mex_eskf_helper.cpp` - build_mex.mにコンパイル設定なし、MATLABコードで未使用
5. `mex_eskf_sensor_update_full.cpp` - build_mex.mにコンパイル設定なし、mex_eskf_sensor_updates_full.cppと重複
6. `mex_filter_utils.cpp` - build_mex.mにコンパイル設定なし、MATLABコードで未使用
7. `mex_kf_core.cpp` - build_mex.mにコンパイル設定なし、MATLABコードで未使用

## 推奨アクション

### 緊急対応が必要
1. **ソースファイルが欠落している4つのバイナリ**
   - これらのソースファイルを探すか、git履歴から復元する必要がある
   - または、これらのバイナリが不要なら削除を検討
   - 現在のbuild_mex.mでは`exist()`チェックでスキップされているため、再コンパイルされません

### 整理推奨
2. **未使用のソースファイル（7個）**
   - MATLABコードで使用されていない
   - 削除またはarchiveディレクトリに移動を検討

