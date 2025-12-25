# ESKF MEX化 現状サマリー

**更新日**: 2025-01-XX

## 現状

### MEXファイルビルド状況
- ✅ Phase 1: `mex_matlab_helpers` - ビルド済み
- ✅ Phase 2: `mex_sensor_filter` (拡張) - ビルド済み・部分的に使用中
- ✅ Phase 3: `mex_sensor_preprocessor` - ビルド済み
- ✅ Phase 4: `mex_adaptive_predict` - ビルド済み
- ✅ Phase 5: `mex_filter_management` - ビルド済み

### ESKF.m統合状況
- ❌ Phase 1, 3, 4, 5: **未統合**（MEXファイルは存在するがESKF.mで使用されていない）
- ⚠️ **問題**: MEXを使用すると推定が失敗する

## 作業方針

段階的にMEXを統合し、各段階でテストを実行：
1. Phase 1統合 → `build_mex` → `run_batch_10sets` → 問題があれば修正
2. Phase 3統合 → `build_mex` → `run_batch_10sets` → 問題があれば修正
3. Phase 4統合 → `build_mex` → `run_batch_10sets` → 問題があれば修正
4. Phase 5統合 → `build_mex` → `run_batch_10sets` → 問題があれば修正

各段階で問題が見つかったら即座に中断してC++ファイルを修正。

