# GitHub Copilot 指示 — KalmanFilter (要点)

目的: このリポジトリはMATLAB（実験・可視化）とC++ MEX（高性能フィルタコア）を組み合わせたハイブリッド実装です。AIエージェントは下記ルールとワークフローに従って作業してください。

- **MEX成果物**: [kalman/cpp/bin](kalman/cpp/bin) に配置されるバイナリは生成物です。直接編集しないでください。
- **必須操作**: MEX を差し替えたら必ず MATLAB で `clear mex` を実行してからテストを行う。
- **状態ベクトル順序**: `p, v, q, ba, bg, P`（合計15要素）。`q` はクォータニオンで `[w,x,y,z]`（スカラー先頭）。
- **共分散対称化**: 出力前に `P = (P + P')/2` を適用して数値対称性を担保すること。

すぐ使えるワークフロー（MATLAB）:
```matlab
cd kalman/cpp/build
build_mex();                % 全体 or build_mex({'ターゲット'})
clear mex                  % 必須（MEX入れ替え後）
cd ../..
run_simulation(42, true)   % 単体検証（seed, verbose）
run_batch_10sets()         % 10セットの回帰テスト
compare_mex_matlab_detailed()
```

主要ファイル／参照先（最初に見る場所）:
- ビルド: [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m)
- 実験エントリ: [kalman/run_simulation.m](kalman/run_simulation.m)
- ESKF MATLABラッパ: [kalman/ESKF/@ESKF/ESKF.m](kalman/ESKF/@ESKF/ESKF.m)
- センサーフィルタ実装（C++）: [kalman/cpp/include/Common/Sensor/sensor_filter.hpp](kalman/cpp/include/Common/Sensor/sensor_filter.hpp)
- 型混在チェック: [kalman/cpp/TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md)

プロジェクト固有の慣習と注意点:
- MEX 関数は `mex_` 接頭辞（例: `mex_meukf_step_v2`, `mex_sensor_filter`）が多い。
- MATLAB 側で「フォールバック実装」が残る場合があるが、現在はMEXが主流。MEXが存在すればそちらを使う想定。
- 数値差の主因は「C++ 側の型（float32 vs float64）」「二重ノーマライズ」「MATLAB/C++の検証ロジック不一致」。`TYPE_MIX_REPORT.md` を参照して型の扱いを確認する。

変更時の最小チェックリスト（C++ を編集したら必ず実行）:
1. 小さな単位でコミットする（レビューしやすくする）。
2. `build_mex({'ターゲット'})` で該当ターゲットをビルド。
3. MATLAB で `clear mex` を実行。
4. `run_simulation(seed,true)` で単体確認。
5. `run_batch_10sets()` と `compare_mex_matlab_detailed()` で回帰確認（期待: Roll/Pitch RMSE < 0.30° などプロジェクト基準を参照）。

デバッグで使うキーワード／ファイル:
- `mex_meukf_step_v2`, `sensor_updates`, `filter_accel`, `OutlierDetector`, `normalize`, `float32`
- 結果比較は `Results/estimation_*.csv` を `compare_mex_matlab_detailed.m` で行単位確認。

編集時の具体例（早見）:
- センサーフィルタ閾値を調整 → 対応する C++ ファイル編集 → `build_mex({'mex_sensor_filter'})` → `clear mex` → `run_simulation` → `compare_mex_matlab_detailed()`

小さな運用ルール:
- `kalman/cpp/bin` 内の MEX は生成ファイル。ソースを編集してビルドする。バイナリ直編集禁止。
- ESKF の状態並びやクォータニオン順序を壊すと互換性が崩れるため注意。

追加情報や要望:
もし自動化したい手順（例: CI での `build_mex` 実行やパリティ比較）や、テンプレート化したテストケースがあれば指示してください。短時間で追記します。
