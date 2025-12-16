# Phase1 / Phase2 移行検証サマリ

## 結論
- Phase1（`alpha_beta_step`, `ema_update`, `hampel_causal`）は C++/MEX 実装へ移行済みで、MATLAB 上では MEX ラッパーを経由して実行されるようになっています。元の MATLAB 実装はラッパー化されており、直接の純 MATLAB 実装は実行されません。
- Phase2（`BiquadFilter`, `AccelFilter` 等）は移行完了・MATLAB 実装はアーカイブされ、実行は MEX のみが行われる状態です。`SensorGyroFilter` は廃止されています。

## 主要な証拠（リポジトリ内ファイル）
- Phase1 完了ノート: [kalman/KF/Utils/PHASE0_COMPLETE.md](kalman/KF/Utils/PHASE0_COMPLETE.md#L12-L25)
- MEX ハンドラ（Phase1 関数を含む）: [kalman/cpp/MEX/mex_filter_utils.cpp](kalman/cpp/MEX/mex_filter_utils.cpp#L13-L24)
- MATLAB ラッパー（MEX 呼び出し）:
  - [kalman/KF/Utils/alpha_beta_step.m](kalman/KF/Utils/alpha_beta_step.m#L1-L10)
  - [kalman/KF/Utils/ema_update.m](kalman/KF/Utils/ema_update.m#L1-L10)
  - [kalman/KF/Utils/hampel_causal.m](kalman/KF/Utils/hampel_causal.m#L1-L10)
- Phase2 完了記録（計画書）: [kalman/KF/Utils/cpp_migration_plan.md](kalman/KF/Utils/cpp_migration_plan.md#L236-L261)
- ESKF での設定（MATLAB 側で Gyro/Accel の MATLAB 実装を作成していない）:
  - `obj.sensor_filters.gyro` が空: [kalman/ESKF/@ESKF/ESKF.m](kalman/ESKF/@ESKF/ESKF.m#L178-L178)
  - `obj.accel_filter` が空（MEX 側で処理）: [kalman/ESKF/@ESKF/ESKF.m](kalman/ESKF/@ESKF/ESKF.m#L184-L184)

## 実行の振る舞い確認
- MATLAB の高レベル関数やクラスは、実行時に MEX バイナリを呼び出すラッパーへフォールバックしています（例: `alpha_beta_step` → `alpha_beta_step_cpp` → `mex_filter_utils('alpha_beta_step', ...)`）。
- Phase2 の MATLAB 実装は `archive/` に移動され、通常の実行パスには存在しません（実行は MEX 実装のみ）。

## 推奨アクション
- ドキュメント整合性のため、`kalman/KF/Utils/cpp_migration_plan.md` の Phase1 チェックボックスを完了に更新することを推奨します。
- 追加検証として、`run_batch_10sets.m` を実行して MEX 実行パスが確実に使われていることを再確認すると安心です（既に 10/10 PASS ログあり）。

---
Generated: 2025-12-17
