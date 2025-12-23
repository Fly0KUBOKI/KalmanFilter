# NaN/Inf 発生の原因解析と対策（要約）

## 概要
この文書は、utils 再編（コミット 0fbe5f5c…）以降に観測されたフィルタ出力の NaN/Inf 問題について、発見した根本原因、適用した修正、再発防止策をまとめたものです。

## 根本原因（要点）
- C++ 側の型・ヘッダ不整合
  - `unified_filter.hpp` がリポジトリ内の共通型定義（`unified_types.hpp` / `meukf` 型）と整合しておらず、構造体レイアウトや型（float/double 等）が不一致になっていた。
  - 結果として MEX 呼び出し時にメモリ破壊や未定義動作が発生し、数値発散→NaN を引き起こした。
- MEX と MATLAB フォールバックの混在
  - 環境変数 `FORCE_MATLAB_FILTERS` の設定により、実行時に MATLAB 実装と MEX 実装が切り替わる。実装差（数値表現や正規化タイミング等）によって振る舞いが異なり、不整合が露呈した。
- ビルド/ロード手順の不備
  - 古い MEX が MATLAB にロードされたままビルドを上書きできず、修正が反映されないまま実行されるケースがあった。

## 証拠（ログ・トレース）
- 発散が起きた結果ファイル: `kalman/Results/estimation_01.csv`（時刻 ~0.2225s に巨大値、0.225s に NaN）
- 解析に使用したトレース: `predict_trace_90_pre.mat`, `predict_trace_60_pre.mat`, `trace_sample_2001_pre.mat` など
- 修正対象ファイル: `kalman/cpp/include/MEUKF/unified_filter.hpp`（ヘッダを共通型に合わせて置換）、`kalman/KF/Utils/AccelFilter.m`（未初期化変数の修正）、`kalman/cpp/MEX/mex_sensor_filter.cpp`（出力数ミスマッチの修正）

## 適用した修正
1. `unified_filter.hpp` を `unified_types.hpp` / `meukf` 型に合わせて書き換え、名前空間・型定義（`Vec3`, `FilterState`, `FilterInput`, `FilterOutput` 等）を統一。これにより MEX 呼び出し時のレイアウト不整合を解消。
2. MATLAB 側のバグ（`AccelFilter.m` の `residual_norm` が未定義で使われていた）を修正。
3. `mex_sensor_filter.cpp` の `baro` ブランチで期待される出力数が満たされない場合があったため、条件付きで 2 出力を返すよう修正。
4. ビルド手順の周知: MEX を差し替えたら必ず `clear mex` を実行して古いバイナリを解放する運用を徹底。

## なぜ今回は成功したのか
- ヘッダ／型不整合を解消したことで、MEX 実行時のメモリ破壊が起きなくなり、数値の急激な発散が抑えられた。
- `clear mex` を実行して新しいバイナリが確実にロードされるようにしたため、修正が実行環境に反映された。
- `FORCE_MATLAB_FILTERS` を `'0'` にして MEX 実装を強制したため、検証対象が一貫化した。

## 成功時と失敗時の差（まとめ）
- 実行される実装: `FORCE_MATLAB_FILTERS` の値が違うと、全く別のコード経路（MATLAB 実装 vs MEX）が走る。
- ロード済みバイナリ: `clear mex` をしていないと古い MEX が使われ続ける。
- 型整合性: ヘッダ／構造体定義が一致していないと intermittant な未定義動作が発生する。
- 数値型の扱い（float/double）や実装差が累積して発散に至る場合がある。

## 再発防止・運用対策（推奨）
- ビルド→検証のハードルを下げる自動化スクリプト（`build_mex({'mex_unified_filter'}) && clear mex && run_batch_10sets()`）を作成し CI で回す。例:

```matlab
cd(fullfile(pwd,'kalman','cpp','build'));
build_mex({'mex_unified_filter'});
clear mex;
setenv('FORCE_MATLAB_FILTERS','0');
cd(fullfile('..','..'));
run_batch_10sets();
```

- 型定義の一元化: ヘッダ／共通型は 1 か所から include する運用を徹底し、複数箇所での手作業コピーを避ける。
- ビルド時の警告をエラー化（-Werror 相当）や静的解析を導入して型不整合を早期に検出する。
- `TRACE_SAMPLE` を使った部分再現手順をドキュメント化し、失敗時に自動で pre/post トレースを取る仕組みを追加する。
- MEX／MATLAB 実装の乖離を減らす（可能なら共通のテストベクタで整合性チェックを自動化）

## 直近の残作業
- `run_batch_10sets()` を再実行して 10/10 パスを確認する（現在 TODO にて進行中）。
- もし失敗が残る場合は、失敗サンプルの pre/post トレースを解析してどのセンサー更新（accel/mag/gps/baro/zupt）がトリガかを特定する。

---

作成者: 自動解析エージェント
作成日: 2025-12-23
