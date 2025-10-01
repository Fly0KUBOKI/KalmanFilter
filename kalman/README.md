# MATLAB Kalman Simulation

このワークスペースは、2D平面上で運動する物体の模擬データを生成し、カルマンフィルタで位置を推定、結果を可視化するサンプルです。

含まれるファイル:

- `run_simulation.m` - ラッパー: リアルタイム逐次EKFランナーを起動します
- `run_simulation_realtime.m` - リアルタイムで逐次データ生成・EKF更新・可視化を行う実行ファイル
- `sim_generate.m` - バッチ生成用の模擬データ生成関数（全データを生成）
- `sim_step.m` - 1ステップ分の真値と観測を生成する関数（realtime 用）
- `ekf_filter_step.m` - 1ステップのEKF予測・更新（逐次処理用）
- `kalman_filter.m` - (既存) バッチ処理用のカルマンフィルタ実装
- `visualize_sim.m` - (既存) 結果のプロットとアニメーション（バッチ表示用）

使い方:

設定と実行が分離されました。主なファイル:

- `config_params.m` - 設定ファイル。シミュレーション・センサノイズ・フィルタ選択などのパラメータをここで編集します。
- `generate_and_save_data.m` - 設定を読み `sim_generate` を呼び、CSV に模擬センサデータを保存します。
- `run_simulation_realtime.m` - 実行ファイル。`config_params` を読み込み、`params.data.source` が 'sim' の場合は逐次生成、'csv' の場合は CSV から読み出して逐次実行します。

使い方:

1. 設定を編集する (任意): `kalman/config_params.m` を開き、`params.motion` や `params.noise`、`params.filter`、`params.data` を変更します。

2. 疑似データを生成して保存する（必要な場合）:
	fname = generate_and_save_data();
	これにより `config_params` の設定に基づく `sim_data.csv` が作成されます。

3. CSV を読み込んで逐次実行する場合は、`config_params` の `params.data.source = 'csv'` とし、`params.data.file` に CSV のパスを設定します。

4. 実行:
	run_simulation


将来の改良案:

- 観測を選択的に使う（速度や方位を観測モデルに加える）
- 非線形観測（方位角）を扱うために拡張カルマンフィルタ（EKF）へ拡張
- 可視化を改善して不確かさを楕円で表示

