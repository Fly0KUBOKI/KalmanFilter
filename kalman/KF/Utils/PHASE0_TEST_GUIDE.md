# Phase 0 テスト実行ガイド

## テスト手順

### 1. MEX関数のビルド（必要に応じて）

```matlab
cd kalman/cpp/MEX
build_filter_utils
```

既に`mex_filter_utils.mexw64`が存在する場合は、このステップはスキップ可能です。
最新のコードで再ビルドする場合は実行してください。

### 2. バッチテストの実行

```matlab
cd kalman
run_batch_10sets
```

### 3. 結果の確認

テスト結果は以下に保存されます：
- `kalman/Results/batch_10sets_log.txt` - ログファイル
- `kalman/Results/estimation_01.csv` ～ `estimation_10.csv` - 各実行の結果

### 4. 確認項目

- 全てのRunが`PASS`であること
- エラーメッセージが無いこと
- 位置RMSEが各軸1.00m以内であること
- 姿勢RMSE（Roll/Pitch/Yaw）が5.0deg以内であること

### 5. 問題が発生した場合

1. MEX関数がビルドされていない場合：
   - `build_filter_utils`を実行
   - エラーメッセージを確認

2. MEX関数が見つからない場合：
   - `kalman/cpp/MEX`ディレクトリをMATLABパスに追加
   - または、`mex_filter_utils.mexw64`を`kalman`ディレクトリにコピー

3. 関数が正しく動作しない場合：
   - `alpha_beta_step_cpp.m`, `ema_update_cpp.m`, `hampel_causal_cpp.m`のfallback実装が使用される
   - MEX関数のビルドエラーを確認

## Phase 1への移行条件

Phase 0が完了し、テストが成功したらPhase 1へ進みます：
- ✅ 全てのRunがPASS
- ✅ エラーなし
- ✅ 結果が期待値内
