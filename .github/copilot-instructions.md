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
