# ESKF MEX 回帰（Z軸 RMSE） — 原因解析と予防

## 概要
シミュレーションで `Position Z RMSE ≈ 1.73 m` の悪化が発生しました。MATLAB 実装・MEX 実装の両方で同じ症状が出ているため、MEX ビルドの単純ミスではなくコード内の座標順序/符号不整合が原因でした。

## 根本原因
- GPS 前処理の軸順・符号がコード内で統一されていませんでした。
  - `reset_filter_impl`（初期化）では座標を ENU として `obj.p = [x_m; y_m; z_m]`（x=East, y=North, z=Up）で設定している。
  - 一方、センサー更新の既存実装（`update_sensor_impl` / thin wrapper の以前の実装）は GPS を `[y_m; x_m; -z_m]`（North, East, Down）で C++ に渡しており、状態と観測の軸順・符号が不一致だった。
- そのため GPS 更新が Z 軸（高度/上下）に逆符号・軸ソートによる大きな誤差を導入していました。

## 修正内容（リポジトリに適用済み）
- `kalman/ESKF.m`
  - `update_sensor_impl` の `gps` ケースを ENU（[x(East); y(North); z(Up)]）で C++ に渡すよう修正しました。
- `kalman/cpp/MEX/mex_sensor_preprocessor.cpp`
  - `preprocess_gps` を ENU 出力に変更（戻り値を `[x_m; y_m; z_m]`、高度は正を上向きに）。
- これにより MATLAB 経路（`update_sensor_impl`）と thin MEX 前処理経路の双方で同一の座標順序が保証されます。

## 予防策（チェックリスト）
- コードベースでは座標データの表記を明文化し、全ファイルで同じ順序／符号を使うこと。
  - 推奨: 明示的に `GPS in ENU [x(E), y(N), z(Up)]` を採用。
- 変更時は以下の自動チェックを実行する Git pre-commit hook を追加することを推奨：
  - `gps`, `gps_pos`, `x_m`, `y_m`, `z_m` を含む行で `-`（negation）や順序入れ替えの痕跡がないか正規表現で検出。
  - MEX と MATLAB の差分検査を自動実行する小スクリプト（下記参照）を用意。

## 推奨の簡易テスト手順
MATLAB 上でローカルにて次を実行してください。

```matlab
addpath('kalman/cpp/bin')
run_simulation(42, true)        % 単一試行を実行
compare_mex_matlab_detailed()   % MEX/MATLAB 差分確認
```
- 期待結果: Z軸の RMSE が改善される（以前の ~1.73m は解消されるはず）。

## 変更ファイル一覧
- Modified: `kalman/ESKF.m` — GPS 更新の ENU 統一
- Modified: `kalman/cpp/MEX/mex_sensor_preprocessor.cpp` — GPS 出力を ENU に変更
- Added: `kalman/cpp/markdown/ESKF_MEX_REGRESSION_ROOT_CAUSE.md`（本書）

## 次の推奨作業
- `compare_mex_matlab_detailed()` の出力と `Results/estimation_01.csv` を確認してください。差分が残る場合、同様の軸混在が残っているはずなので、該当行（CSV の位置列順）を突き合わせます。
- 長期: pre-commit hook と簡易自動差分チェック（MATLAB vs MEX）を作成して回帰を防止してください。

---
作業が完了したらシミュレーション結果（差分レポート）を共有してください。追加で修正が必要であれば続けて対応します。
