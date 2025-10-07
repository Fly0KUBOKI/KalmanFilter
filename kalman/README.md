MEX を使って C++ 実装を MATLAB から呼ぶ（高速化）
-------------------------------------------------

このリポジトリには C++ 実装（`cpp/kalman_filter.cpp` / `cpp/kalman_filter.hpp`）と、最小限の MEX ラッパー `cpp/kalman_mex.cpp`、ビルド補助スクリプト `cpp/kalman_mex_build.m` を追加しました。手順は次の通りです。

1. MATLAB を開き、`kalman/cpp` ディレクトリをカレントにします。

```matlab
cd('path/to/kalman/cpp');  % 例: cd('c:/.../KalmanFilter/kalman/cpp')
```

2. MEX をビルドします（Windows/MSVC の MATLAB 環境を想定）。

```matlab
% ビルド補助スクリプトを実行
kalman_mex_build
% または手動で
mex('kalman_mex.cpp','kalman_filter.cpp')
```

3. ビルドに成功したら、MATLAB から C++ 実装を呼び出せます。簡単な使用例:

```matlab
% ハンドル生成
h = kalman_mex('new');
% 初期化: state_size=2, obs_size=1 など
kalman_mex('init', h, 2, 1);
% システム行列 F (2x2) と観測行列 H (1x2)
F = eye(2);
H = [1 0];
kalman_mex('setSystem', h, F);
kalman_mex('setObservationMatrix', h, H);
% 予測ベクトルと観測値を設定
kalman_mex('setPrediction', h, [0;0]);
kalman_mex('setObservation', h, 1.23);
% 更新
kalman_mex('update', h);
% 結果取得
out = kalman_mex('get', h);
% ハンドル破棄
kalman_mex('delete', h);
```

注意事項:
- この MEX ラッパーは最小限の機能を提供します。必要に応じて `kalman_mex.cpp` を拡張してください。
- MATLAB (column-major) と C++ 内部 (本実装では row-major の取り扱いを想定) の行列格納順に注意して変換しています。大きな差が出る場合はラッパ内の変換を確認してください。

# MATLAB Kalman Simulation

このワークスペースは、2D平面上で運動する物体の模擬データを生成し、カルマンフィルタで位置を推定、結果を可視化するサンプルです。

含まれるファイル:

- `run_simulation.m` - ラッパー: リアルタイム逐次EKFランナーを起動します
- `run_simulation_realtime.m` - リアルタイムで逐次データ生成・EKF更新・可視化を行う実行ファイル
- `sim_generate.m` - バッチ生成用の模擬データ生成関数（全データを生成）
- `sim_step.m` - 1ステップ分の真値と観測を生成する関数（realtime 用）
- `ekf_filter_step.m` - 1ステップのEKF予測・更新（逐次処理用）
- `kalman_filter.m` - (既存) バッチ処理用のカルマンフィルタ実装
- `rts_smoother.m` - (追加) Rauch-Tung-Striebel 後方スムーザー: 逐次フィルタの出力を用いて時系列全体を平滑化します
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

# MATLAB Kalman Simulation

このリポジトリは、2D 平面で移動する物体の模擬データ生成、カルマンフィルタ（逐次・バッチ）の実装、および可視化を含むサンプル集です。MATLAB 上で動かすことを想定しています。

主要フォルダ構成（相対パス）:

- `GenerateData/` - シミュレーション設定とデータ生成スクリプト
	- `config_params.m` - シミュレーション／センサ／フィルタのパラメータ設定
	- `generate_and_save_data.m` - 全データを生成し CSV に保存
	- `sim_generate.m` / `sim_step.m` - バッチ／逐次用のデータ生成関数

- `KalmanFilter/` - フィルタ本体（逐次ステップ実装）
	- `kf_filter_step.m` - 1 ステップの標準カルマンフィルタ処理（線形）
	- `ekf_filter_step.m` - 1 ステップの拡張カルマンフィルタ（非線形観測向け）
	- `ukf_filter_step.m` - 1 ステップの UKF（Unscented KF）実装
	- `test_filter_steps.m` - フィルタステップの単体テスト／使用例
	- `Common Calculations/` - フィルタで使う共通計算関数群

- `Graph/` - 可視化スクリプト
	- `plot_csv_variables.m` - CSV から変数を抽出してプロット
	- `visualize_sim.m` - シミュレーション結果の描画／アニメーション

- `FFT/` - FFT 解析用スクリプト群（補助ツール）

- `cpp/` - C++ 実装（参考）

ルートにある主なファイル:

- `run_simulation_realtime.m` - 逐次データ生成 → フィルタ更新 → 可視化 をリアルタイムに実行するランナー
- `sim_data.csv` - 生成済みの模擬データ（例）

使い方（簡単なクイックスタート）:

1. MATLAB を起動し、ワークスペースの `kalman` フォルダにカレントディレクトリを移動します。

2. 必要に応じてパラメータを編集します（デフォルトは `GenerateData/config_params.m`）。

3. データを生成して CSV に保存する（任意）:

```matlab
cd('GenerateData');
fname = generate_and_save_data();  % 生成されたファイル名が返る
cd('..');
```

4. リアルタイム逐次実行（シミュレーション生成 or CSV 読み込みのいずれか）:

```matlab
% config_params.m 内の params.data.source を 'sim' または 'csv' に設定
run('run_simulation_realtime.m')
```

run_simulation_realtime は `GenerateData/config_params.m` を読み、
params.data.source が 'sim' の場合は逐次に `sim_step` を呼んでデータを生成しながらフィルタ更新を行います。'csv' の場合は `params.data.file` で指定した CSV から逐次読み出します。

5. 個別のフィルタステップを試す / テストする:

```matlab
cd('KalmanFilter');
test_filter_steps;  % kf/ekf/ukf の簡単な例・テスト
cd('..');
```

備考・補足:

- ここにある実装はステップ単位の関数（kf_filter_step, ekf_filter_step 等）を中心に構成されています。バッチ処理用の汎用 `kalman_filter.m` や `rts_smoother.m` はこのリポジトリの現行版では存在しません。バッチ推定やスムージングが必要な場合は `KalmanFilter` 内のステップ関数を組み合わせて利用してください。

- `cpp/` フォルダには C++ 実装の参考コードが含まれています。MATLAB と比較してパフォーマンスや組み込み用途の検討に使えます。
