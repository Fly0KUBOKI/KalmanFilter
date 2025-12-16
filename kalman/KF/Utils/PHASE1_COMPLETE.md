# Phase1 完了ステータス

以下は現在の作業状況の簡潔なサマリです（更新日時: 2025-12-16）。

- **ブランチ**: `phase2` を作成してローカル変更をコミット・`origin/phase2` に push 済み。
- **重要変更**:
	- `kalman/run_batch_10sets.m`: MEX 初期化 (`clear mex` + `mex_sensor_filter('reset'|'reset_zero')`) と `FORCE_MATLAB_FILTERS` の自動セットを追加。さらに、PASS 判定に各姿勢軸（Roll/Pitch/Yaw）ごとの厳格閾値（1.0 deg）条件を追加。
	- `kalman/mardown/PHASE2_ROOT_CAUSE_AND_ACTIONS.md`: Pitch/Roll 回帰の原因と今回の成功要因を追記。

- **検証結果（簡易）**:
	- バッチ実行（10セット）で Run1 の Roll/Pitch/RMSE は約 `0.28 / 0.29` deg に改善。ログは `kalman/Results/batch_10sets_log.txt` に保存。

- **次の推奨アクション**:
	1. `compare_accel_filters.m` と `compare_gyro_filters.m` を実行して C++ MEX と MATLAB 実装の出力一致を確認。
	2. `mex_sensor_filter` に明示的 `init`/`reset` API を追加して MEX 側の初期化を堅牢化。
	3. CI にフィルタ一致チェックを組み込み（PR 時に自動検証）。

---

（このファイルは自動更新されています。追加で記載したい項目があれば指示ください。）

