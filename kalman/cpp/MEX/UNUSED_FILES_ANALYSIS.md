# 未使用MEXファイル分析

## binディレクトリにあるバイナリ（13個）

1. mex_adaptive_predict.mexw64 ✓
2. mex_eskf_constructor.mexw64 ✓
3. mex_eskf_do_update.mexw64 ✓
4. mex_eskf_predict_postprocess.mexw64 ✓
5. mex_eskf_sensor_updates_full.mexw64 ✓
6. mex_eskf_update_postprocess.mexw64 ✓
7. mex_eskf_zupt.mexw64 ✓
8. mex_filter_management.mexw64 ✓
9. mex_meukf_step_v2.mexw64 ✓
10. mex_quaternion_lib.mexw64 ✓
11. mex_run_eskf.mexw64 ✓
12. mex_sensor_filter.mexw64 ✓
13. mex_sensor_preprocessor.mexw64 ✓

## MEXディレクトリにあるソースファイル（17個）

### コンパイルされているファイル（13個）
1. mex_adaptive_predict.cpp → mex_adaptive_predict.mexw64 ✓
2. mex_eskf_constructor.cpp → mex_eskf_constructor.mexw64 ✓
3. mex_eskf_do_update.cpp → mex_eskf_do_update.mexw64 ✓
4. mex_eskf_predict_postprocess.cpp → mex_eskf_predict_postprocess.mexw64 ✓
5. mex_eskf_sensor_updates_full.cpp → mex_eskf_sensor_updates_full.mexw64 ✓
6. mex_eskf_update_postprocess.cpp → mex_eskf_update_postprocess.mexw64 ✓
7. mex_eskf_zupt.cpp → mex_eskf_zupt.mexw64 ✓
8. mex_filter_management.cpp → mex_filter_management.mexw64 ✓
9. mex_meukf_step.cpp → mex_meukf_step_v2.mexw64 ✓ (別名でビルド)
10. mex_quaternion_lib.cpp → mex_quaternion_lib.mexw64 ✓
11. mex_run_eskf.cpp → mex_run_eskf.mexw64 ✓
12. mex_sensor_filter.cpp → mex_sensor_filter.mexw64 ✓
13. mex_sensor_preprocessor.cpp → mex_sensor_preprocessor.mexw64 ✓

### コンパイルされていないファイル（7個）

1. **mex_common_lib.cpp** ❌
   - build_mex.mにない
   - binにバイナリなし
   - 使用状況: 不明（MATLABコードで検索が必要）

2. **mex_eskf_core_v2.cpp** ❌
   - build_mex.mにない
   - binにバイナリなし
   - 使用状況: 不明（MATLABコードで検索が必要）

3. **mex_eskf_do_cpp_update.cpp** ❌
   - build_mex.mにない
   - binにバイナリなし
   - 理由: mex_eskf_do_update.cppと重複機能の可能性
   - 使用状況: 不明

4. **mex_eskf_helper.cpp** ❌
   - build_mex.mにない
   - binにバイナリなし
   - 使用状況: 不明（MATLABコードで検索が必要）

5. **mex_eskf_sensor_update_full.cpp** ❌
   - build_mex.mにない
   - binにバイナリなし
   - 理由: mex_eskf_sensor_updates_full.cppと重複機能の可能性
   - 使用状況: 不明

6. **mex_filter_utils.cpp** ❌
   - build_mex.mにない
   - binにバイナリなし
   - 使用状況: 不明（MATLABコードで検索が必要）

7. **mex_kf_core.cpp** ❌
   - build_mex.mにない
   - binにバイナリなし
   - 使用状況: 不明（MATLABコードで検索が必要）

## 推奨アクション

1. **MATLABコードで使用状況を確認**
   - 上記7ファイルについて、MATLABコード（.mファイル）内で参照されているか検索
   - 参照がなければ削除候補

2. **重複ファイルの整理**
   - mex_eskf_do_cpp_update.cpp vs mex_eskf_do_update.cpp
   - mex_eskf_sensor_update_full.cpp vs mex_eskf_sensor_updates_full.cpp
   - 違いを確認し、不要なら削除

3. **archiveディレクトリへの移動**
   - 削除前にarchiveディレクトリに移動してバックアップ

