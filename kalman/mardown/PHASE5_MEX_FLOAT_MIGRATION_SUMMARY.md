# PHASE5: MEX 内部 float 移行サマリ

最終更新: 2025-12-22

## 概要
- 目的: MATLAB 側は double API を維持しつつ、MEX 内部の計算を `float` (float32) に移行して性能と一貫性を確保。
- 方針: MEX 入力を double で受け取り、内部で double→float に変換して float コアを呼び出し、結果を float→double に戻して MATLAB に返す。

## 追加したファイル
- `kalman/cpp/MEX/mex_type_conv.hpp` — mxArray⇄float バッファの安全な変換ヘルパー（NULLセーフ、zero-fill）
- `kalman/cpp/TYPE_MIX_REPORT.md` — 種別混在の現状と移行方針の報告

## 主な修正（ファイル一覧）
以下の MEX ラッパーを順次修正し、入力の double 受け取り → float 暫定バッファへ変換 → float コア呼び出し → double 出力へ変換、というパターンに統一しました。

- mex_common_lib.cpp
- mex_eskf_helper.cpp
- mex_eskf_math.cpp
- mex_eskf_core_v2.cpp
- mex_eskf_core.cpp
- mex_ekf.cpp
- mex_ukf_sigma_points.cpp
- mex_ukf.cpp
- mex_sensor_filter.cpp
- mex_quaternion_lib.cpp
- mex_meukf_step.cpp
- mex_kf_core.cpp
- mex_ukf_update.cpp
- mex_kalman_filter_core.cpp
- mex_kalman_compute.cpp
- mex_filter_utils.cpp
- mex_eskf_step.cpp
- mex_unified_filter.cpp
- mex_ukf_update_minimal.cpp

（上記は主要な変更ファイル。小さなヘッダ追加やNULL安全化パッチも多数適用。）

## 技術的な変更ポイント
- 中央化された変換ヘルパー `mex_type_conv.hpp` を導入し、以下を提供:
  - `mxArrayToFloatArray(const mxArray*, float*, size_t)`
  - `floatArrayToMxArray(const float*, mxArray*, rows, cols)`
  - `mxGetScalarAsFloat(const mxArray*)`
- 直接 `mxGetPr`/`mxGetScalar` を多用するのではなく、上記ヘルパー経由で NULL チェック・zero-fill を行うように変更。
- 生配列管理を `std::vector<float>` に置換して自動的メモリ管理に。
- MATLAB の列優先（column-major）と C++ 内部の行/列扱いの違いを正しく扱うため、行列転置やインデックス変換を統一。

## 解決した主な問題
- NULL ポインタ参照による MATLAB 停止クラッシュを防止（フィールド欠落や空配列に対してデフォルト値を返す）
- ビルドエラー（`<vector>` の未インクルードなど）を修正
- `prS` 未定義やスカラーの重複定義などの小さなバグを修正

## テスト結果（バッチ: 10/10）
成功: 10/10 (100.0%)

主要統計:

- Position RMSE (overall): Mean=0.5559, Std=0.0233, Max=0.5836 m
- Position RMSE by axis: X Mean=0.1578, Y Mean=0.1484, Z Mean=0.5118 m
- Velocity RMSE: Mean=0.5687, Std=0.0012, Max=0.5705 m/s
- Roll RMSE: Mean=0.2732°, Pitch RMSE: Mean=0.2834°, Yaw RMSE: Mean=0.6704°

個別 Run のサマリ（抜粋）:

```
Run  1: PASS (Pos Overall=0.5624m, X=0.1625m, Y=0.1399m, Z=0.5199m, Att=0.27/0.27/0.65 deg)
... (Run 2..9省略)
Run 10: PASS (Pos Overall=0.5836m, X=0.1638m, Y=0.1599m, Z=0.5369m, Att=0.27/0.29/0.69 deg)
```

ログと保存先:

- バッチログ: `kalman/Results/log/batch_10sets_log_mex_20251222_213553.txt`
- 結果ファイル: `kalman/Results/batch_10sets_results.mat`
- CSV サマリ: `kalman/Results/batch_10sets_summary.csv`

## 今後の推奨アクション
1. 最終回帰を CI で自動化（`build_mex()` → `run_batch_10sets(true)`）
2. Phase5 の残タスクとして Eigen 依存の洗い出し & 標準C++化検討
3. MATLAB 側の冗長ラッパー削減（Phase6）および `mex_run_simulation` へ段階的統合

---
補足: 詳細なパッチ履歴や個々のコミットコメントは git 履歴（ローカル）に記録されています。追加で「変更差分の抜粋」や「個別ファイルごとの説明」が必要なら教えてください。
