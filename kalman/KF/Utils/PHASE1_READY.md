# Phase 1 準備完了

このフォルダ (`kalman/KF/Utils`) は Phase 0 関連の不要ファイルをアーカイブし、Phase 1 へ移行する準備を行いました。

実施内容:
- `ema_update_cpp.m`, `hampel_causal_cpp.m`, `PHASE0_COMPLETE.md`, `PHASE0_TEST_GUIDE.md`, `cpp_migration_plan.md`, `PHASE1_COMPLETE.md` を `kalman/KF/Utils/archive/` に移動（アーカイブ）
- 元ファイルをトップレベルから削除

次の推奨手順（Phase 1 実行前）:

1. MEX ビルド（必要な場合）

```bash
cd kalman/cpp/MEX
# MATLAB で実行するか、適切なビルドスクリプトを使用
# 例: build_filter_utils など
```

2. バッチテスト実行

```matlab
cd kalman
run_batch_10sets
```

3. Phase 1 の実施項目（優先）
- `alpha_beta_step.m` の C++ 実装確認/実装
- `ema_update.m`, `hampel_causal.m` の最終確認（MEX フォールバックの確認）
- `BiquadFilter.m` と `AccelFilter.m` を Phase 1 の対象として整理

4. 変更ログを作成し、コードレビューを依頼

このファイルを更新して追加の実行手順やチェックリストを追記してください。