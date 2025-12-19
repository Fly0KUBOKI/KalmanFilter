# Phase3 API 差分レポート

目的: `kalman/cpp/include/Common/Sensor/sensor_filter.hpp`（C++/MEX 側）と
MATLAB の `kalman/KF/Utils/NoiseEstimator.m` / `kalman/KF/Utils/DivergenceGuard.m` の
公開APIを突合し、差分と優先対応を示す。

要点サマリ
- MEX ハンドラ実装 (`kalman/cpp/MEX/mex_sensor_filter.cpp`) は主要コマンドを提供済み:
  - `reset`, `reset_zero`, `get_R`, `noise_estimate`, `accel`, `mag`, `gps`, `baro`, `divergence_check`, `divergence_regularize`
- C++ 実装（`sensor_filter.hpp`）は `NoiseEstimator::estimate` / `get_R_matrix` と
  `DivergenceGuard::check_and_attenuate` / `regularize_covariance` を提供済み。

検出された差分・注意点（優先度付き）
1) 引数の行列メモリレイアウト（優先: 高）
   - `mex_sensor_filter('noise_estimate', sensor, innov, H, P)` で渡す `H` と `P` の列優先/行優先扱いが
     MATLAB 側（列優先 double）と C++ 側の読み込み（ループ順）で一致しているか要確認。
   - 対応策: `mex_sensor_filter.cpp` の読み込みループは現在列-major を想定しているが、重点的に `H`/`P` の
     サイズチェックと単体ケース比較テストを追加して差分を確定する。

2) 型と精度（優先: 中）
   - MATLAB は double、C++ 側は float（FixedMatrix）で内部計算。これは許容だが数値スケール差が出る可能性あり。
   - 対応策: 許容差を明文化し、`tests/test_noise_estimator_mex.m` で許容差チェックを行う。

3) `get_R` の戻り値形状（優先: 低）
   - `get_R` は C++ 側で 1x1 (baro) または NxN を返す。`NoiseEstimator.sync_from_mex` は `if strcmp(s,'baro')` を特別扱いする。
   - 確認点: `mex_sensor_filter('get_R','baro')` が scalar を返すことを保証する（現在実装済み）。

4) ランタイム・ログ制御（優先: 中）
   - C++ 側に `g_enable_sensor_logging` を追加済み（デフォルト false）。しかし MATLAB から動的に切替える MEX コマンド
     (`mex_sensor_filter('log','on'|'off')`) は未実装。
   - 対応策: `mex_sensor_filter.cpp` に `log` コマンドを追加して `sensor_log_enable(true/false)` を呼ぶ。

5) trace / sample 指定パラメータ伝播（優先: 低）
   - `do_cpp_update` から `mex_params.trace_sample` を渡しているが MEX 側が正しく受け取りログに使っているか確認。

推奨短期タスク（実行順序）
1. API 差分検証スクリプトを追加（完了済み: `tests/test_noise_estimator_mex.m`, `tests/test_divergence_guard_mex.m`）。MATLAB 上で再ビルド後に実行して比較結果を取得する。
2. `mex_sensor_filter` の `noise_estimate`/`get_R` 呼び出しで、`H`/`P` の行列要素の読み取り順（列-major vs 行-major）を明示的に注釈・テストする。
3. `mex_sensor_filter` に `log` コマンドを追加して、実行中にログを切替え可能にする。
4. 数値差が大きければ `noise_estimate` のアルゴリズムパラメータ（alpha, warmup_samples）を揃える自動テストを追加。

作業済みファイル（参照）
- `kalman/cpp/MEX/mex_sensor_filter.cpp`  (実装済みハンドラ)
- `kalman/cpp/include/Common/Sensor/sensor_filter.hpp` (C++ 実装)
- `kalman/KF/Utils/NoiseEstimator.m` (MATLAB ラッパ)
- `kalman/KF/Utils/DivergenceGuard.m` (MATLAB ラッパ)
- `kalman/cpp/tests/test_noise_estimator_mex.m` (追加スモークテスト)
- `kalman/cpp/tests/test_divergence_guard_mex.m` (追加スモークテスト)

次の手を私が進める場合（選択）
- A: `mex_sensor_filter.cpp` に `log` コマンドを追加（即実装）
- B: `mex_sensor_filter` の `H`/`P` 読み取りを明示的にコメント化 + 追加ユニットテスト生成（私が差分テストを自動生成）

結論: 基本的な MEX 経路は揃っているため「API 突合と行列レイアウトの自動比較」をまず実行することを推奨します。希望があれば私が `B` を即実行して詳細差分レポート（行列要素ごとの差）を生成します。
