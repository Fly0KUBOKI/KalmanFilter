# ベースラインテスト報告

## 日時
2025-12-26 17:40頃

## 目的
センサー周期更新とmex_eskf_runの変更を元に戻し、元のMATLAB実装でベースライン動作を確認する

## 実施した変更の無効化

### 1. センサー周期更新機能の無効化
**ファイル**: `kalman/GenerateData/config_params.m`
- 変更前: `params.sensor_freq`構造体で各センサーの周波数を定義
- 変更後: `sensor_freq`設定を削除（コメントに記録）

**ファイル**: `kalman/GenerateData/generate_sensor_observations.m`
- 変更前: `mod(i-1, interval)`でセンサー周期をチェックし、更新間隔でない場合は前回の値を保持
- 変更後: 毎サンプルで新しい値を計算（元の動作）

### 2. mex_eskf_runの無効化
**ファイル**: `kalman/run_simulation.m`
- 変更前: `USE_MEX_ESKF_RUN = true`（デバッグログ付き）
- 変更後: `USE_MEX_ESKF_RUN = false`（デバッグログ削除）

## テスト実行
- コマンド: `run_batch_10sets(false)`（MATLAB実装）
- 実行状況: バックグラウンド実行中
- ログファイル: ターミナル出力は`c:\Users\takut\.cursor\projects\c-Users-takut-OneDrive-MATLAB-KalmanFilter\terminals\6.txt`に保存

## 期待される結果
- Position RMSE < 5m (X, Y)
- Roll/Pitch/Yaw RMSE < 5 deg
- 10回中5回以上成功（50%以上）

## 次のステップ
1. テスト結果を確認
2. ベースラインと比較
3. 問題が解決していれば、段階的にMEX化を再実装
4. 問題が残っていれば、さらなる調査を実施




