# Phase3: NoiseEstimator / DivergenceGuard の移行メモ

目的: `Phase3`（`NoiseEstimator.m` と `DivergenceGuard.m`）をC++/MEX化するための現状把握と実行手順。

1) 現状（重要ファイル）
- MATLAB:
  - `kalman/KF/Utils/NoiseEstimator.m` — コンストラクタ `NoiseEstimator(warmup_samples)`、`estimate(sensor_type, innovation, H, P_pred)`、`getRnoise(sensor_type)`、`getThreshold(...)` を提供。
  - `kalman/KF/Utils/DivergenceGuard.m` — `check_and_attenuate_update(sensor_name, innovation, dx_in, ctx)` など、共分散正則化・状態クリップ・ダンプ等のユーティリティを多数提供。
- C++:
  - `kalman/cpp/include/Common/Sensor/sensor_filter.hpp` に `class NoiseEstimator` と `class DivergenceGuard` の実装（主要メソッド: `estimate(...)`, `get_R_matrix(...)`, `check_and_attenuate(...)`, `regularize_covariance(...)`, `clip_state_change(...)` 等）が存在。
  - `kalman/cpp/MEX/mex_sensor_filter.cpp` はセンサーフィルタ（`accel`, `mag`, `gps`, `baro` 等）を提供するMEXで、`SensorFilterLib filter_lib` を `static` に保持している。現在のMEXはNoiseEstimator/DivergenceGuard用コマンドを公開していない。

2) MATLAB ↔ C++ インターフェース差分（要注意点）
- `NoiseEstimator`:
  - MATLAB: `getThreshold` を提供するが、C++ 側には同名のヘルパーが見当たらない（`get_R_matrix` は存在）。
  - 引数/戻り値の形状は C++ の `FixedMatrix` と MATLAB の行列で互換性を持たせる必要がある（列ベクトル／スカラー扱い）。
- `DivergenceGuard`:
  - MATLAB 側は `save_divergence_dump`, `clamp_gain`, `clip_acceleration`, `check_and_clip_velocity` など多くのユーティリティを持つが、C++ 実装は簡易版のメソッド群に留まる。
  - MATLAB 呼び出し側が期待する出力（複数戻り値や`should_skip`フラグ等）をMEX経由で正しく返す必要がある。

3) 推奨作業ステップ（優先順）
1. インターフェース照合
   - `NoiseEstimator.m` の公開メソッドと `sensor_filter.hpp::NoiseEstimator` のシグネチャを行単位で照合。差分（例えば `getThreshold` 相当）があればC++に追加するか、MATLABラッパーで補完する。
2. MEX API設計
   - 既存の `mex_sensor_filter.cpp` に以下コマンドを追加する選択肢:
     - `'noise_estimate'` : `NoiseEstimator::estimate(sensor_type, innovation, H, P_pred)` を呼ぶ（戻り: none または更新済フラグ）
     - `'get_R'` : `NoiseEstimator::get_R_matrix(sensor_type)` を返す（MATLAB行列）
     - `'divergence_check'` : `DivergenceGuard::check_and_attenuate(...)` を呼び、`[dx_out, should_skip, was_attenuated]` を返す
     - `'divergence_regularize'` : `regularize_covariance(P)` を呼ぶ（Pを返す）
   - 代替: 新規MEX（`mex_noise_divergence.cpp`）を作り、NoiseEstimator/DivergenceGuard専用ハンドラを実装する。
3. MEX 実装の注意点
   - `sensor_filter.cpp` と同様に静的/長寿命インスタンスを管理するが、`run_batch_10sets` 実行時に古いインスタンスが残らないよう `reset` / `reset_zero` ハンドラを確実に実装する。
   - 行列のサイズ/型チェックを厳格に行い、エラーを明確に返す（`mexErrMsgIdAndTxt`）。
4. MATLABラッパー更新
   - `kalman/ESKF/@ESKF/sensor_updates.m` や既存の MATLAB 呼び出し箇所が新しいMEXコマンドを利用するようにラッパー関数を追加／置換する。
5. テスト・検証
   - ビルド: `cd kalman/cpp/build; build_mex()`（または `build_sensor_filter.m`）→ `clear mex`
   - 単体: 小さなユニットケースで `get_R` と `divergence_check` の入出力を検証
   - 統合: `run_batch_10sets.m` と `run_simulation()` を比較し `Results/estimation_matlab.csv` と `Results/estimation_mex.csv` の差分を確認

4) 具体的に修正するファイル（提案）
- 追加／編集:
  - `kalman/cpp/MEX/mex_sensor_filter.cpp` — 新コマンド追加（推奨）
  - or `kalman/cpp/MEX/mex_noise_divergence.cpp` — Noise/Divergence専用MEXを追加
- 参照・確認:
  - `kalman/cpp/include/Common/Sensor/sensor_filter.hpp` — C++実装の最終確認
  - `kalman/KF/Utils/NoiseEstimator.m`, `kalman/KF/Utils/DivergenceGuard.m` — MATLAB側期待仕様
  - `kalman/cpp/build/build_mex.m`, `kalman/cpp/build/build_sensor_filter.m` — ビルド手順

5) 追加チェックリスト（レビュー時）
- 戻り値の形（ベクトル vs スカラー）をMATLAB呼び出し元に合わせる
- `ctx` 構造体で期待されるフィールド（`P`, `R`, `H`, `k`, `z`, `h`, `y` 等）をMEX側で受け取り可能か確認
- `save_divergence_dump` のファイル出力パスはC++版で代替実装（またはMATLAB側でラップ）するか決定
- スレッド安全性は不要（MATLABからのシリアル呼び出しが前提）だが、静的インスタンスのライフサイクル管理は慎重に

6) 推定スケジュール（短期）
1. 今日: `sensor_filter.hpp` と MATLABファイルの関数シグネチャ差分を一覧化（完了済）
2. 明日: `mex_sensor_filter.cpp` に `'get_R'` と `'divergence_check'` を試験的に追加、ビルドして単体検証
3. 次週: 統合テスト（`run_batch_10sets.m`）で差分検証、必要ならC++側アルゴリズムの調整

---
補足: 要望があれば、私が `mex_sensor_filter.cpp` に `'get_R'` / `'divergence_check'` の試作パッチを作成してビルド手順まで整えます。どちらの実装（既存MEX拡張 vs 新規MEX）を優先しますか？
