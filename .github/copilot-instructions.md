# GitHub Copilot 指示 — KalmanFilter

このファイルは、このリポジトリでAIコーディングエージェントがすぐに生産的に作業できるよう、必須の知識とワークフローを簡潔にまとめたものです。

目的のサマリ
- MATLAB: オーケストレーション、I/O、可視化（`kalman/run_simulation.m` がメイン）
- C++ MEX: 数値コア（`kalman/cpp/` 以下、ビルド成果物は `kalman/cpp/bin/*.mexw64`）

重要なワークフロー（必読）
- C++ を修正したら必ずソースから MEX を再ビルド:

```matlab
cd kalman/cpp/build
build_mex()              % または: build_mex({'mex_meukf_step'})
clear mex               % 重要: MATLAB の MEX キャッシュをクリア
addpath(fullfile(pwd,'..','bin'))
```

- シミュレーション実行例（データ再生成オプション）:

```matlab
cd ../../..  % kalman のルートへ戻る
run_simulation(42, true)  % seed=42, skip_data_gen=true
```

プロジェクト固有ルール（必ず守る）
- 状態ベクトルは15次元: [p(3), v(3), q(4), ba(3), bg(3)]。クォータニオンはスカラー先頭: `qw,qx,qy,qz`。
- MEX バイナリを直接編集しないこと（`kalman/cpp/bin/` は成果物フォルダ）。必ず `kalman/cpp/` のソースを編集してビルド。
- MEX 更新後は `clear mex` を必ず実行する。
- 数値安定化の慣例: 共分散は対称化して扱う（例: `P = (P + P')/2`）。
- クォータニオン正規化は C++ 側で行われる場合がある — MATLAB 側で二重に正規化しない。

重要なファイル・検索ワード（即参照）
- MATLAB エントリ: [run_simulation](kalman/run_simulation.m)
- データ生成: [GenerateData/sim_generate.m](kalman/GenerateData/sim_generate.m), [GenerateData/config_params.m](kalman/GenerateData/config_params.m)
- C++ ビルド: [cpp/build/build_mex.m](kalman/cpp/build/build_mex.m), ログ: [cpp/build/build_log.txt](kalman/cpp/build/build_log.txt)
- MEX 名称: `mex_meukf_step_v2`, `mex_meukf_step`, `mex_eskf_step`（検索で優先）
- ESKF MATLAB クラス: [ESKF/@ESKF/ESKF.m](kalman/ESKF/@ESKF/ESKF.m)
- 比較結果: [Results/estimation_matlab.csv](Results/estimation_matlab.csv) と [Results/estimation_mex.csv](Results/estimation_mex.csv)

典型的な修正・検証フロー（チェックリスト）
1. C++ 修正（例: `kalman/cpp/MEUKF/*`）
2. `build_mex()` で再ビルド（必要なら特定ターゲットのみ）
3. MATLAB で `clear mex` 実行
4. `run_simulation(..., true)` で動作比較（`true` はデータ再生成をスキップ）
5. `Results/` の CSV を比較して差分を確認

デバッグのヒント（よくある失敗と確認箇所）
- ビルドに失敗したら: `mex -setup C++` を確認、`cpp/build/build_log.txt` と `build_result.txt` を参照
- 推定値欠落や数値差が出たら: `Results/estimation_diff.csv` と `tools/compare_estimations.m` を利用
- センサー更新コードの参照例: `kalman/ESKF/@ESKF/sensor_updates.m`

検索用キーワード（コード内でのパターン探索に便利）
- `mex_meukf_step_v2|mex_meukf_step|mex_eskf_step|build_mex|clear mex|estimation_matlab.csv|estimation_mex.csv`

変更ルールと安全策
- 小さな C++ 変更は対象の MEX ターゲットのみビルドして確認する（`build_mex({'mex_meukf_step'})`）。
- ビルド成果物を直接編集しない。変更はソースへ戻し、再ビルドして確認する。
- テスト手順を必ず `Results/` の CSV で保存して差分を残す。

問い合わせ・フィードバック
- 不明点や追記して欲しい箇所があれば、具体的なファイル名や失敗ケースを教えてください。

---
小さく濃い目のガイドを目指しました。追加で「よく壊れるテスト」「頻出のパラメータ」といった情報があれば追記します。
# GitHub Copilot 指示 — KalmanFilter リポジトリ

このリポジトリは MATLAB（オーケストレーション／可視化／I/O）と C++ MEX（数値コア）を組み合わせたハイブリッド実装です。
目的は「C++ コアを安全に修正して MEX をビルドし、MATLAB 側の動作と差分を迅速に検証する」ことです。

必読の場所（短く）
- MATLAB 実行: kalman/run_simulation.m — メインの実験ランナー（引数: seed, skip_data_gen）。
- データ生成: kalman/GenerateData/sim_generate.m, config_params.m, sensor_data.csv。
- C++ ソースとビルド: kalman/cpp/ (src, mex, include, MEUKF, ESKF), ビルドスクリプト: kalman/cpp/build/build_mex.m。
- ビルド成果物: kalman/cpp/bin/*.mexw64（MATLAB で addpath して利用）。
- 結果比較: Results/estimation_matlab.csv, Results/estimation_mex.csv, batch_10sets_matlab_log.txt。

必須ワークフロー（MATLAB での実行例）
1) C++ を修正したら（例: kalman/cpp/MEUKF/*）:

```matlab
cd kalman/cpp/build
build_mex()             % 全体ビルド、または: build_mex({'mex_meukf_step'})
clear mex              % *必ず*キャッシュをクリア
addpath(fullfile(pwd,'..','bin'))
```

2) シミュレーション（データ再生成可）:

```matlab
cd ../../..  % kalman のルートに戻す
run_simulation(42, true)  % seed=42, skip_data_gen=true
```

3) 差分確認:
- 比較: Results/estimation_matlab.csv vs Results/estimation_mex.csv
- バッチ: Results/batch_10sets_matlab_log.txt を参照

重要なプロジェクト固有ルール（必ず守る）
- 状態ベクトルは 15 次元: [p(3), v(3), q(4), ba(3), bg(3)]。クォータニオン順序はスカラー先頭: qw, qx, qy, qz。
- MEX はソースから再ビルドすること。bin 内ファイルを直接編集しない。
- MEX 更新後は `clear mex` を実行して MATLAB のキャッシュを解除する。
- 数値安定化: 共分散は対称化する（例: P = (P + P')/2）とコード内で扱われる箇所あり。
- クォータニオンの正規化は C++ 側で行われる場合があるので、MATLAB 側で二重正規化しない。

よく使う検索キーワード（コード内のパターン）
- `ESKF` クラス（kalman/ESKF/@ESKF） — `predict`, `reset`, `zupt`, `sensor_updates` が主要メソッド。
- `mex_meukf_step_v2`, `mex_meukf_step`, `mex_eskf_step` — MEX のエントリ名。
- `build_mex`, `build_meukf_only`, `run_selective_build` — ビルド補助スクリプト。

デバッグと検証のヒント
- ビルドログ: kalman/cpp/build/build_log.txt と build_result.txt を確認。
- ビルド失敗時: MATLAB で `mex -setup C++` を実行してコンパイラ設定を確認。
- 推定差や欠落が出る場合は CSV（Results/）を比較し、`kalman/Graph/plot_csv_file.m` で可視化する。
- センサー更新の順序や頻度は `ESKF` オブジェクトの `freq_*` プロパティで制御される（run_simulation.m のループ参照）。

変更を加えるときの安全ガイド
- 小さな C++ 変更は単体の mex ターゲットだけビルドして確認（build_mex({'mex_meukf_step'})）。
- 複数ターゲットを更新する場合は、まず `run_simulation(..., true)` でデータ生成をスキップして差分を切り分ける。
- 出力は必ず `Results/` の CSV に保存されるため、自動比較スクリプトを作ると回帰検出が容易。

追加情報が必要な箇所（欲しいフィードバック）
- よく壊れるテストや特定のパラメータセットがあれば教えてください（自動化・サンプルで再現します）。
# GitHub Copilot 指示 — KalmanFilter リポジトリ

このファイルは、このリポジトリで効率よく作業するためのAIエージェント向け具体的指示をまとめます。目的は、MATLAB + C++ MEX ハイブリッドの ESKF/UKF/MEUKF 実装を素早く理解し、安全に変更できるようにすることです。

- **ゴール**: ビルド（MEX）、シミュレーションの実行、C++コアの修正、MATLABラッパーの保守が主な作業領域。

## 主要コンポーネント（ビッグピクチャ）
- MATLAB orchestration: `kalman/run_simulation.m`, `kalman/GenerateData/sim_generate.m`, `kalman/ESKF/@ESKF/ESKF.m` がエントリーポイント。
- C++ MEX core: `kalman/cpp/` 以下に C++ 実装と `build/` スクリプトがある。ビルド成果物は `kalman/cpp/bin/*.mexw64`。
- データ: `kalman/GenerateData/` はシミュレーション入力と生成スクリプトを保持（`config_params.m`, `truth_data.csv`, `sensor_data.csv`）。

## 重要なワークフロー（必須コマンド例）
# GitHub Copilot 指示 — KalmanFilter リポジトリ

このリポジトリは MATLAB（フロー管理、データ入出力）と C++ MEX（高負荷数値演算）を組み合わせたハイブリッド実装です。
AI エージェントが素早く安全に作業できるよう、重点的に必要な知識・手順を簡潔にまとめます。

## 要点（サマリ）
- MATLAB がオーケストレーション（`kalman/run_simulation.m`）、C++ がコア処理（`kalman/cpp/*`）を担当。
- MEX バイナリは `kalman/cpp/bin/*.mexw64` に置かれる。バイナリはソースから再ビルドすること。

## 主要コンポーネント
- MATLAB orchestration: `kalman/run_simulation.m`, `kalman/GenerateData/sim_generate.m`, `kalman/ESKF/@ESKF/ESKF.m`。
- C++ MEX core: `kalman/cpp/MEUKF/`, `kalman/cpp/MEX/`（ビルドスクリプト: `kalman/cpp/build/build_mex.m`）。
- データ: `kalman/GenerateData/`（シミュレーション生成）と `Results/`（出力/ログ）。

## 必須ワークフロー（手順）
1. C++ を編集したら（例: `kalman/cpp/MEUKF/*`）MATLAB で:

```matlab
cd kalman/cpp/build
build_mex()
clear mex   % 重要: MATLAB セッションの古い MEX を切る
```

2. 検証:

```matlab
run_simulation()
run_simulation(42, true)
test_phase1.m
```

3. 出力比較: `Results/estimation_matlab.csv` と `Results/estimation_mex.csv` を比較し差異を精査。

## プロジェクト固有ルール / 注意点
- 状態ベクトルは 15 次元: [p(3), v(3), q(4), ba(3), bg(3)]（クォータニオンはスカラー先頭: qw,qx,qy,qz）。
- MEX バイナリは必ずソースからビルドする。直接編集禁止。
- MEX 更新後は `clear mex` を必ず実行する（キャッシュが原因で古い挙動になる）。
- 共分散は対称性を保つ（更新後 `P = (P + P')/2` が用いられることがある）。
- C++ 側でクォータニオン正規化が行われる場合があるため、MATLAB 側で二重に正規化しない。

## インターフェース / 検索ヒント
- センサー更新は多くの場合 `mex_meukf_step_v2` を経由（参照: `kalman/ESKF/@ESKF/sensor_updates.m`）。
- MEX 呼び出しはstructベースで状態・センサーを渡すパターンが一般的。

## デバッグチェックリスト
- ビルドログ: `kalman/cpp/build/build_log.txt` を確認。
- MEX 実行で推定欠落が起きたら: `Results/run_mex_missing_estimation.txt` を見る。
- 出力差分: `Results/estimation_matlab.csv` vs `Results/estimation_mex.csv`。
- プロット: `kalman/Graph/plot_csv_file.m` を利用。

## 典型的な修正フロー（短く）
1. C++ コア修正 → 2. `build_mex()` → 3. `clear mex` → 4. `run_simulation()` → 5. CSV 比較

---
フィードバック: 追記してほしい詳細（特定ファイル、よく壊れるテスト、頻出パラメータなど）があれば教えてください。
