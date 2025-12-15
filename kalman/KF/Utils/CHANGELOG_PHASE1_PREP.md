# CHANGELOG: Phase1 準備 (自動作成)

## 2025-12-15
- 不要ファイルを判定・アーカイブ:
  - 移動先: `kalman/KF/Utils/archive/`
  - 対象: `ema_update_cpp.m`, `hampel_causal_cpp.m`, `PHASE0_COMPLETE.md`, `PHASE0_TEST_GUIDE.md`, `cpp_migration_plan.md`, `PHASE1_COMPLETE.md`
- 元ファイルをトップレベルから削除
- `PHASE1_READY.md` を作成（移行手順・実行コマンドを記載）

次のアクション候補:
- `run_batch_10sets` を実行して現状のテストを確認
- Phase 1 の優先ファイル（`alpha_beta_step.m`, `ema_update.m`, `hampel_causal.m`）の C++ 実装状況を確認
