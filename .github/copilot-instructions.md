# GitHub Copilot 指示 — KalmanFilter (要点)

このリポジトリは「MATLABで実験／可視化を行い、計算ホットパスをC++ MEXで高速化する」ハイブリッド実装です。
AIエージェントが最初に押さえるべき事項を短くまとめます。

必須ルール（優先）
- MEXバイナリは成果物: [kalman/cpp/bin](kalman/cpp/bin)。直接編集しない。
- MEXを差し替えたら必ず MATLAB で `clear mex` を実行してからテストする。
- 状態ベクトルは厳格：`[p(3), v(3), q(4), ba(3), bg(3)]`（合計15）。`q` は必ず `[w,x,y,z]`（スカラー先頭）。
- 出力前に共分散を対称化すること：`P = (P + P')/2`。

## GitHub Copilot 指示 — KalmanFilter（要点）

このリポジトリはMATLABでのアルゴリズム実験と、計算ホットパスをC++ MEXで高速化するハイブリッド実装です。
以下はAIエージェントが実務で即使える最小限のルールと参照先です。

### 必須ルール
- MEX成果物は `kalman/cpp/bin` にあり、直接編集しない（ビルドして置き換える）。
- MEXを入れ替えたら必ず MATLAB で `clear mex` を行う。
- 状態ベクトル順序は固定：`[p(3), v(3), q(4), ba(3), bg(3)]`（計15）。`q` は `[w,x,y,z]`（スカラー先頭）。
- 共分散は出力前に必ず対称化：`P = (P + P')/2`。

### 主要ワークフロー（よく使うコマンド）
```matlab
cd kalman/cpp/build
build_mex();                % または build_mex({'ターゲット'})
clear mex
cd ../..
run_simulation(42, true)   % 単体検証
run_batch_10sets()         % 回帰バッチ
compare_mex_matlab_detailed()
```

### 重要ファイル（必ず目を通す）
- ビルドスクリプト: [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m)
- テスト入口: [kalman/run_simulation.m](kalman/run_simulation.m)
- MATLABラッパ: [kalman/ESKF/@ESKF/ESKF.m](kalman/ESKF/@ESKF/ESKF.m)
- MEXバイナリ: [kalman/cpp/bin](kalman/cpp/bin)
- 型混在の分析: [kalman/cpp/TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md)
- センサー外れ値ロジック: [kalman/cpp/include/Common/Sensor/sensor_filter.hpp](kalman/cpp/include/Common/Sensor/sensor_filter.hpp)

### プロジェクト特有の注意点
- 数値差の主因は`float32`/`float64`の混在、二重ノーマライズ、検証ロジックの不一致。
- クォータニオン順序・状態のインデックスは致命的なので、変更前に対応箇所を横断検索すること。

### 変更→検証の推奨フロー
1. 小さな単位でC++を変更（1機能＝1コミット）。
2. `build_mex({'ターゲット'})` でビルド。
3. MATLABで `clear mex` → `run_simulation(seed,true)` で差分を確認。
4. `run_batch_10sets()` → `compare_mex_matlab_detailed()` で回帰確認。
5. 差分は `Results/estimation_*.csv` を行単位で確認。

### 探索用キーワード（コード検索）
- `mex_meukf_step_v2`, `mex_eskf_step`, `mex_eskf_init`, `normalize`, `OutlierDetector`, `float32`, `sensor_filter`

### 速攻デバッグチェックリスト
- MEXの入出力型を確認（float/double整合）。
- クォータニオンは `[w,x,y,z]` で一致しているか。
- 共分散が対称化されているか。
- 変更後は必ず `clear mex` → `run_simulation` で再検証。

---
もし追記してほしい「現地ルール」や自動化したいチェック（例：型チェック、MEXバージョン検証）があれば教えてください。追記して反映します。
