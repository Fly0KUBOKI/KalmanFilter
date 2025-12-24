# Phase 6 — ESKF 機能差分チェックリスト

最終更新: 2025-12-24

目的: `EKF`/`UKF`（MATLAB層）の機能が `ESKF` に完全に置換可能かを検証し、削除準備をする。

重要ファイル:
- ESKF クラス: [kalman/ESKF/@ESKF/ESKF.m](kalman/ESKF/@ESKF/ESKF.m)
- ESKF 予測/更新: [kalman/ESKF/@ESKF/predict.m](kalman/ESKF/@ESKF/predict.m), [kalman/ESKF/@ESKF/sensor_updates.m](kalman/ESKF/@ESKF/sensor_updates.m)
- MEX: `mex_meukf_step_v2.mexw64`, `mex_eskf_init.mexw64` (bin 配下)

チェックリスト (確認/完了フラグをつけること):
- [ ] API parity: `KF.m`, `EKF/@EKF`, `UKF/@UKF` のパブリックメソッドを ESKF が提供しているか
- [ ] 初期化: `EKF` の初期化パラメータを `ESKF`（または `mex_eskf_init`）で再現可能か
- [ ] 予測ステップ: すべての `predict` 呼び出しが `ESKF` の `predict`（MEX 呼出含む）で同等結果を返すか
- [ ] 更新ステップ: 加速度/磁気/GPS/気圧/ZUPT 等、個別センサー更新が同等に動作するか
- [ ] EKF 固有処理: Jacobian を明示的に使っている箇所があれば ESKF 側で代替（MEUKF/UKF 手法）できるか
- [ ] UKF 固有処理: シグマ点生成や重み計算（`mex_ukf_sigma_points`）が ESKF フローで必要か
- [ ] 出力形式: 状態ベクトル順序（`[p(3), v(3), q(4), ba(3), bg(3)]`）とクォータニオン順序 `[w,x,y,z]` が全箇所で一致するか
- [ ] 共分散対称化: ESKF 出力で `P = (P + P')/2` が適用されているか
- [ ] Divergence/Noise: `DivergenceGuard.m` / `NoiseEstimator.m` のロジックが ESKF フローで適用されるか
- [ ] Sensor filters: `SensorBaroFilter`, `SensorGPSFilter`, `SensorMagFilter` 等の前処理が欠落していないか
- [ ] MEX ハンドル API: `mex_eskf_init/get/set/free` が存在し、`ESKF.m` に置換可能か
- [ ] テストカバレッジ: `run_simulation`, `run_batch_10sets` のテストケースで EKF/UKF と ESKF を比較するスクリプトがあるか

推奨アクション（優先度順）:
1. `ESKF` の公開メソッドを `EKF`/`UKF` のメソッド一覧と照合し、差分リストを作る。
2. 小さなユニットテスト（EKF 特有のケース）を `ESKF` に対して書き、数値差を確認する。
3. `ESKF.m` 内の初期化を `mex_eskf_init` に差し替える（テスト用に並列保持）。
4. `EKF`/`UKF` フォルダを無効化（パスから外す）して `run_batch_10sets()` を実行し、10/10 パスが維持されるか確認する。

MATLAB での実行コマンド例:
```matlab
cd kalman/cpp/build
build_mex({'mex_eskf_init','mex_meukf_step_v2'})
clear mex
cd ../..
run_simulation(42, true)
run_batch_10sets()
```

次の担当タスク（提案）:
- `Phase6.1`: 差分リスト作成（このファイル） — 完了にマーク可
- `Phase6.2`: 単体テストケース追加 (`tests/` 下に追加)
- `Phase6.3`: `ESKF.m` の初期化部を MEX 初期化に差し替え（安全に並列で残す）
- `Phase6.4`: EKF/UKF の削除（最終）/パス確認

問題がなければ、次に `ESKF.m` の初期化差し替えを提案して実装します。