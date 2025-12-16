# AccelFilter MEX 移行方針

## 目的
- MATLAB 実装(`SensorAccelFilter.m` / `AccelFilter.m`)と MEX 実装(`mex_sensor_filter('accel',...)` / `SensorFilterLib::filter_accel`)の動作を一致させ、MEX 化による性能向上を得つつ挙動差による影響を排除する。

## 現状の観察
- MATLAB 側: `SensorAccelFilter` は `config` (ema_alpha, history_size, gravity_range, large_change_threshold, scale_factor) を使用して EMA と外れ値判定を行う。
- C++ 側: `SensorFilterLib::filter_accel` は EMA + 外れ値検出を実装しているが、現在は MATLAB から動的なパラメータ同期を受け取らない（デフォルト値で動作）。
- そのためパラメータ差や履歴長の違いで出力差が発生する可能性がある。

## ゴール
1. MATLAB と MEX の出力が一致する（許容差内）。
2. MEX 側は MATLAB の `config` を反映できるようにする。
3. 回帰テストを整備して差分を自動検出する。

## 実施ステップ（高レベル）
1. 使用箇所の網羅的調査
   - `AccelFilter.m`, `SensorAccelFilter.m`, `SensorFilter.createAccelFilter`, テストスクリプト(`compare_phase1.m`, `dump_sensor_filter_outputs.m`) を確認。
2. MEX API 設計
   - `mex_sensor_filter('accel_config', configStruct)` などのコマンドを設計し、以下パラメータを受け取れるようにする。
     - `ema_alpha`, `history_size`, `gravity_range`, `threshold_sigma` (外れ値), `min_std` 等。
3. C++ 側改修
   - `SensorFilterLib` に accel 設定の setter を追加。
   - `OutlierDetector` の履歴長や閾値を動的に変更可能にする（現在の MAX_HISTORY を可変化）。
   - 数値丸めや初期化の振る舞いを MATLAB 実装に合わせて調整。
4. MEX ラッパー改修
   - `mex_sensor_filter.cpp` に `'accel_config'` の処理を追加し、`filter_accel` 呼び出し時に内部状態が設定通り動作するようにする。
5. MATLAB 側同期
   - `SensorFilter.createAccelFilter` 実装で、MEX が存在する場合は生成時に `mex_sensor_filter('accel_config', config)` を呼び MEX 状態を同期する（`FORCE_MATLAB_FILTERS` による強制回避は維持）。
6. テストと調整
   - `compare_phase1.m` / `dump_sensor_filter_outputs.m` / `run_batch_10sets.m` を使って MATLAB vs MEX 出力を比較。
   - 差異がある場合、C++ 実装の初期化・丸め・閾値を調整して一致させる。
7. ドキュメント更新
   - `cpp_migration_plan.md` と `MEX_STATUS_REPORT.md` に変更点とテスト結果を記載。

## 具体的な実装メモ
- MEX の `accel_config` は MATLAB 構造体（フィールド名を一致）を受け取り、C++ 側で必要な型に変換する。
- OutlierDetector: 現在は固定配列 `MAX_HISTORY=20` のため、可変化する履歴長を内部動的バッファで置き換える。
- 初期状態: MATLAB 実装は内部フィルタをゼロ初期化する挙動が多い。C++ 側も同様に遅延初期化/ゼロ初期化の振る舞いを一致させる。

## テスト基準
- 同一入力系列に対して、各軸のフィルタ出力差ノルムの平均が小さく、最大差が e.g. 1e-3 程度以下（数値丸めに応じて調整）。
- 外れ値検出の一致率（外れ値と判定したサンプル割合）が高いこと。

## リスクと対応
- リスク: C++ 側での数値丸めやサンプリング周波数の扱いにより微小差が残る。
  - 対応: 比較テストで差の傾向を確認し、C++ 実装のアルゴリズム順序・初期値を MATLAB に合わせる。
- リスク: テストカバレッジ不足。
  - 対応: `dump_sensor_filter_outputs.m` を使い複数シード・シナリオで比較。

## 次のアクション（短期）
- `SensorFilter.createAccelFilter` 周りの呼び出しを一通り列挙・依存調査（実施中）。
- その上で `mex_sensor_filter` に `accel_config` を追加するパッチを作成。

---
作業を進めます: まずは呼び出し箇所の追加確認と既存テストの実行手順を整え、次に C++ 側の小さな改修から着手します。