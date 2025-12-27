# 再発防止ガイドライン

## Phase 1完了後の重要な学び

### 1. MEXファイルの再ビルド時の注意事項

**問題**: MEXファイルがロックされていて再ビルドできない

**解決方法**:
- MATLABを**完全に終了**してから再ビルド
- `clear_and_rebuild.m`を使用してクリーンビルド
- ビルドエラーが発生する場合は、すべてのMATLABウィンドウを閉じて再試行

**チェックリスト**:
- [ ] すべてのMATLABウィンドウを閉じた
- [ ] `clear_and_rebuild.m`を実行
- [ ] ビルドエラーがないことを確認
- [ ] `run_batch_10sets(true)`で動作確認

### 2. 環境変数の設定

**問題**: MEX実装が使用されない

**原因**: `USE_MEX_PREDICT_POSTPROCESS`環境変数が設定されていない

**解決方法**:
- `run_batch_10sets(true)`を使用すると自動設定される
- 手動で設定する場合: `setenv('USE_MEX_PREDICT_POSTPROCESS', '1')`

**チェックリスト**:
- [ ] `run_batch_10sets.m`で環境変数が設定されているか確認（lines 55-61）
- [ ] MEX実装を使用する場合は`use_mex=true`で実行
- [ ] MATLAB実装を使用する場合は`use_mex=false`で実行

### 3. MATLAB実装との整合性

**問題**: MEX実装とMATLAB実装で結果が異なる

**解決方法**:
- `mexCallMATLAB`を使用してMATLAB関数を直接呼び出し
- 数値計算はMATLABの精度を保証
- 定数の精度に注意（例: `deg2rad(45)`の高精度値を使用）

**チェックリスト**:
- [ ] `mex_eskf_predict_postprocess.cpp`で`mexCallMATLAB`を使用
- [ ] 定数の精度がMATLAB実装と一致しているか確認
- [ ] `run_batch_10sets(true)`と`run_batch_10sets(false)`で結果を比較

### 4. デバッグ用ファイルの管理

**問題**: デバッグ用ファイルが残っている

**解決方法**:
- Phase完了後はデバッグ用ファイルを削除
- 比較用のmarkdownファイルも削除
- 完了報告のみを残す

**削除すべきファイル**:
- `debug_*.m`
- `test_*.m`（MEUKFのテストファイルは除く）
- `*comparison*.md`
- `*DIFF*.md`

### 5. テスト方法

**標準テスト手順**:
1. MEX実装をテスト: `run_batch_10sets(true)`
2. MATLAB実装をテスト: `run_batch_10sets(false)`
3. 結果を比較: Position RMSE、Velocity RMSE、Attitude RMSE
4. 10/10成功を確認

**成功基準**:
- 成功率: 10/10 (100%)
- Position RMSE: MATLAB実装と差 < 0.01m
- Velocity RMSE: MATLAB実装と差 < 0.01 m/s
- Attitude RMSE: MATLAB実装と差 < 0.01°

## 次のPhaseへの推奨事項

1. **段階的な移行**: 一度にすべてを移行せず、小さな単位で移行
2. **検証の徹底**: 各Phase完了後は必ずバッチテストを実行
3. **ドキュメント更新**: 完了報告を作成し、学びを記録
4. **コード整理**: Phase完了後はデバッグ用ファイルを削除

