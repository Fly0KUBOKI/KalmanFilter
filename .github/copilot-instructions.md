# GitHub Copilot 指示 — KalmanFilter (要点まとめ)

このプロジェクトは「MATLABで実験・可視化を行い、計算ホットパスをC++ MEXで実装する」ハイブリッド構成です。AIエージェントは下記に従って作業してください。

必須ルール（優先度高）
- MEXバイナリは成果物: [kalman/cpp/bin](kalman/cpp/bin)。直接編集しない。
- MEX差し替え後は必ず MATLAB で `clear mex` を実行してからテストを行う。
- 状態ベクトルの順序は厳格: p, v, q, ba, bg, P（計15要素）。`q` は `[w,x,y,z]`（スカラー先頭）。
- 共分散 `P` は出力前に対称化: `P = (P + P')/2`。

主要ワークフロー（再現コマンド）
```matlab
cd kalman/cpp/build
build_mex();                % 全体ビルド、または build_mex({'ターゲット'})
clear mex                  % 必須（MEX入れ替え後）
cd ../..
run_simulation(42, true)   % 単体検証（seed, 詳細出力）
run_batch_10sets()         % 10セット検証
compare_mex_matlab_detailed()
```

重要ファイルと確認ポイント
- ビルド: [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m)
- MATLABエントリ: [kalman/run_simulation.m](kalman/run_simulation.m)
- MATLABラッパ: [kalman/ESKF/@ESKF/ESKF.m](kalman/ESKF/@ESKF/ESKF.m)
- MEX成果物: [kalman/cpp/bin](kalman/cpp/bin)（`*.mexw64` 等）
- 型混在のリファレンス: [kalman/cpp/TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md)

チェックリスト（編集時に自動で/手作業で確認すべき点）
- 型整合: C++ の型（double/float32）と MATLAB 入出力の型の違いは結果差に直結する。`TYPE_MIX_REPORT.md` を参照。
- クォータニオン: 正規化の重複や順序ミスを避ける（`q = [w,x,y,z]` を厳守）。
- センサーフィルタ: 外れ値検出ロジックは [kalman/cpp/include/Common/Sensor/sensor_filter.hpp](kalman/cpp/include/Common/Sensor/sensor_filter.hpp) に実装。MATLAB側の意図と一致するか確認。
- 出力比較: `Results/estimation_*.csv` を `compare_mex_matlab_detailed.m` で行単位比較する。

よく使う検索キーワード（デバッグ用）
- `mex_meukf_step_v2`, `sensor_updates`, `filter_accel`, `OutlierDetector`, `normalize`, `float32`

推奨変更フロー（C++修正時）
1. 小さな単位で変更（理解しやすい差分）
2. `build_mex({'ターゲット'})` でターゲットビルド
3. MATLABで `clear mex` → `run_simulation(seed,true)` 実行
4. `run_batch_10sets()` と `compare_mex_matlab_detailed()` で回帰確認

短い例: センサーフィルタの閾値を変更する場合は、C++の該当ファイルを変更 → ターゲットビルド → `clear mex` → `run_simulation` → `compare_mex_matlab_detailed`。

フィードバック
- この要約に不足する箇所（たとえば追加すべき自動チェックやCI手順）があれば指示してください。迅速に反映します。
# GitHub Copilot 指示 — KalmanFilter

## 概要
MATLAB（制御・I/O・可視化）+ C++ MEX（高速数値処理）のハイブリッド実装によるカルマンフィルタプロジェクト。  
**主な目的**: C++ コアを修正 → MEX ビルド → MATLAB 側と数値パリティ検証 の流れを安全・迅速に実行する。

## アーキテクチャの「なぜ」
- **MATLAB層** (`kalman/run_simulation.m`): 実験設計・結果収集・比較用（1回のシミュレーション: ~50秒）
- **C++ MEX層** (`kalman/cpp/` 以下): 計算ボトルネック（予測・更新×11,600ステップ）をC++で実装
- **状態ベクトル** = 15次元 `[p(3), v(3), q(4), ba(3), bg(3)]`
  - `q = [w, x, y, z]`（スカラー先頭）のクォータニオン形式固定
  - C++ では `float64` で保持（PHASE 4 で精度損失問題特定済み）

## 必須ビルド・テストワークフロー
# GitHub Copilot 指示 — KalmanFilter (要約)

短く直接的に：このリポジトリはMATLABの実験/可視化レイヤと、C++ MEXによる高速フィルタコアを組み合わせたカルマンフィルタ実装です。

短期ゴール（よくある作業）:
- C++修正 → `kalman/cpp/build/build_mex.m` を実行
- `clear mex` を必ず実行してから MATLAB で動作確認
- 単体検証は `run_simulation(seed, true)`、バッチは `run_batch_10sets()`

必読ルール（絶対守る）:
- MEX バイナリは `kalman/cpp/bin/` に置かれる成果物。直接編集禁止。
- MEX更新後は必ず `clear mex` を実行して MATLAB キャッシュをクリア。
- 状態 `state` のフィールド順序は厳格：`p, v, q, ba, bg, P`。
- 共分散 `P` は数値丸めで非対称化するため、出力前に `P = (P + P')/2` を適用。

よく使うコマンド（コピーして使う）:
```matlab
cd kalman/cpp/build
build_mex();     % or build_mex({'mex_meukf_step_v2'})
clear mex
cd ../..
run_simulation(42, true)
% or
run_batch_10sets()
compare_mex_matlab_detailed()
```

設計上の重要事項（AIが理解すべき点）:
- 状態ベクトルは15次元：`[p(3), v(3), q(4), ba(3), bg(3)]`。`q` は `[w,x,y,z]`。
- 多くの数値差は「C++での型（float32 vs float64）」や「二重ノーマライズ」「検証ロジック不一致」が原因。
- センサーフィルタのバリデーション（例：重力ノルム `[8.5,10.5]`）はMATLABとC++で一致させる必要がある。

主要ファイル（参照先）:
- `run_simulation.m` — 実験スクリプト（エントリ）
- `run_batch_10sets.m` — 10セット検証スクリプト
- `kalman/cpp/build/build_mex.m` — MEXビルドスクリプト
- `kalman/ESKF/@ESKF/ESKF.m` — MATLAB側フィルタラッパ（MEX呼出しハブ）
- `kalman/GenerateData/` — 入力データ生成・ノイズモデル

バグパターンの例（検索するとき）:
- `mex_meukf_step_v2`、`sensor_updates`、`filter_accel`、`OutlierDetector`、`float32`、`normalize`

短い作業ガイドライン（AI向け具体例）:
- 変更を入れる前に `build_mex()` をターゲットで実行し、`clear mex` → `run_simulation(seed,true)` で差分を比較。
- C++で検証ロジックを追加・変更する場合、同じチェックがMATLAB側にもあるか `kalman/` 内を確認。
- 結果差分は `Results/estimation_*.csv` で行単位比較する。`compare_mex_matlab_detailed.m` を使う。

最後に：このファイルは「作業の手順」と「破壊的変更を避けるための最小ルール」をまとめたものです。追加で欲しい具体例や、AIが自動で行うべきチェックがあれば教えてください。
- [sensor_filter.hpp#L248-L270](kalman/cpp/include/Common/Sensor/sensor_filter.hpp#L248) の外れ値検出初期化ロジックをMATLAB側と一致させる
