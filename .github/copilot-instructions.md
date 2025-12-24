# GitHub Copilot 指示 — KalmanFilter (要点)

このリポジトリは「MATLABで実験／可視化を行い、計算ホットパスをC++ MEXで高速化する」ハイブリッド実装です。
AIエージェントが最初に押さえるべき事項を短くまとめます。

必須ルール（優先）
- MEXバイナリは成果物: [kalman/cpp/bin](kalman/cpp/bin)。直接編集しない。
- MEXを差し替えたら必ず MATLAB で `clear mex` を実行してからテストする。
- 状態ベクトルは厳格：`[p(3), v(3), q(4), ba(3), bg(3)]`（合計15）。`q` は必ず `[w,x,y,z]`（スカラー先頭）。
- 出力前に共分散を対称化すること：`P = (P + P')/2`。

主要ワークフロー（すぐ使えるコマンド）
```matlab
cd kalman/cpp/build
build_mex();                % or build_mex({'ターゲット'})
clear mex                  % 必須（MEX入れ替え後）
cd ../..
run_simulation(42, true)   % 単体検証（seed, 詳細出力）
run_batch_10sets()         % 10セット回帰
compare_mex_matlab_detailed()
```

重要なファイル（参照先）
- ビルドスクリプト: [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m)
- 試験エントリ: [kalman/run_simulation.m](kalman/run_simulation.m)
- MATLABラッパ: [kalman/ESKF/@ESKF/ESKF.m](kalman/ESKF/@ESKF/ESKF.m)
- MEX 成果物: [kalman/cpp/bin](kalman/cpp/bin)
- 型・精度差のドキュメント: [kalman/cpp/TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md)

プロジェクト固有の注意点
- 数値差の主因は型混在（`float32` vs `float64`）、二重ノーマライズ、検証ロジック不一致。
- センサーフィルタの外れ値ロジックは [kalman/cpp/include/Common/Sensor/sensor_filter.hpp](kalman/cpp/include/Common/Sensor/sensor_filter.hpp) にあり、MATLABと一致させる必要がある。
- 状態の順序やクォータニオンの向きは極めて重要。順序ミスは致命的な差を生む。

テスト／検証の流れ（推奨）
1. C++ を小さな単位で変更する（1機能＝1コミット）。
2. ターゲットをビルド: `build_mex({'ターゲット'})`。
3. MATLAB で `clear mex` を実行し、`run_simulation(seed,true)` を実行して差分を確認。
4. バッチ検証: `run_batch_10sets()`。差分比較: `compare_mex_matlab_detailed()`。
5. 差分は `Results/estimation_*.csv` を行単位で確認する。

よく使う検索キーワード／エントリポイント
- `mex_meukf_step_v2`, `mex_eskf_step`, `sensor_updates`, `filter_accel`, `OutlierDetector`, `normalize`, `float32`

開発者向けヒント（具体例）
- C++側で検証ロジックを追加する場合、MATLAB側（`kalman/` 下）にも同等のチェックがあるか探す。
- 共分散や状態出力に差が出たらまず `TYPE_MIX_REPORT.md` を確認し、float/double の整合をチェックする。
- MEX を入れ替えたら必ず `clear mex` を入れる手順を PR のチェックリストに加える。

CI / 自動化に関して（発見可能なもののみ）
- 現状は手動ビルド＋MATLAB実行フローが主体。自動 CI が必要なら `build_mex` と `compare_mex_matlab_detailed` をラップするスクリプトを追加すること。

フィードバック要求
- この内容で不足している「現地ルール」や、自動化したい具体的なチェック（例：型チェック、MEXバージョン検証）があれば教えてください。追記してマージします。
