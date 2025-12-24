# GitHub Copilot 指示 — KalmanFilter

このリポジトリは MATLAB をフロントエンド、C++ MEX を計算ホットパスに用いるハイブリッド実装です。
AI エージェントが素早く作業に入れるよう、必須ルールと重要なワークフローを簡潔にまとめます。

必須ルール (最重要)
- MEX の成果物は `kalman/cpp/bin` に置かれます。バイナリは直接編集しないこと（ビルドして置換）。
- MEX を入れ替えたら MATLAB で必ず `clear mex` を実行してからテストする。
- 状態ベクトルの順序は厳格：`[p(3), v(3), q(4), ba(3), bg(3)]`（合計 15）。
  - クォータニオン `q` は常に `[w,x,y,z]`（スカラー先頭）であること。
- 出力前に共分散を常に対称化すること：`P = (P + P')/2`。

主要ワークフロー（素早く再現するためのコマンド）
- MATLAB ビルド → テスト
  - `cd kalman/cpp/build`
  - `build_mex()` または `build_mex({'ターゲット'})`
  - MATLAB で `clear mex`
  - `cd ../..`
  - `run_simulation(<seed>, true)`（単体確認）
  - `run_batch_10sets()`（回帰バッチ）
  - `compare_mex_matlab_detailed()`（差分解析）

重要ファイルと参照例
- ビルドスクリプト: [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m)
- MEX ライブラリ置き場: [kalman/cpp/bin](kalman/cpp/bin)
- MATLAB ラッパ/ESKF クラス: [kalman/ESKF/@ESKF/ESKF.m](kalman/ESKF/@ESKF/ESKF.m)
- MEX 型混在レポート: [kalman/cpp/TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md)
- センサーフィルタ実装例: [kalman/cpp/include/Common/Sensor/sensor_filter.hpp](kalman/cpp/include/Common/Sensor/sensor_filter.hpp)
- CI / バッチ結果: `Results/estimation_*.csv`（パリティ確認に利用）

プロジェクト特有の注意点（必ずチェックすること）
- 浮動小数点の混在: `float32` / `float64` の混在が主要な差分原因。MEX 側と MATLAB 入出力の型整合を常に確認する。
- クォータニオン・順序: `[w,x,y,z]` がコード全体で統一されているか横断検索で確認する（致命的な不一致原因）。
- MEX 名の慣習: 多くは `mex_<component>_<action>.mexw64` の形で、`mex_eskf_*`, `mex_meukf_*` などが存在する。

パリティ検証の手順（簡潔）
- 単体: `run_simulation(seed,true)` を使い、MEX と MATLAB フォールバックの挙動を比較。
- 回帰: `run_batch_10sets()` を実行し、`Results/estimation_*.csv` を差分比較。
- 数値差は `kalman/cpp/TYPE_MIX_REPORT.md` と `kalman/cpp/Common/README.md` を参照して原因を切り分ける。

検索キーワード（迅速診断）
- `mex_meukf_step_v2`, `mex_eskf_step`, `mex_eskf_init`, `normalize`, `OutlierDetector`, `float32`, `sensor_filter`

変更→検証の推奨フロー（C++ を編集した場合）
1. 小さな単位で変更・1 機能 = 1 コミット。
2. `build_mex({'ターゲット'})` でビルド。
3. MATLAB で `clear mex` → `run_simulation(seed,true)` で差分確認。
4. `run_batch_10sets()` → `compare_mex_matlab_detailed()` で回帰確認。
5. 差分は `Results/estimation_*.csv` を行単位で確認。

追加のリソース
- プロジェクト全体のフェーズ計画: [kalman/mardown/ROADMAP_TO_PHASE_13.md](kalman/mardown/ROADMAP_TO_PHASE_13.md)

フィードバック
- このファイルは簡潔に保っています。追加してほしい「現地ルール」や自動化チェック（例：型チェックスクリプト、MEX バージョン検証）があれば教えてください。
