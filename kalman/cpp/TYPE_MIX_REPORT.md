# 型混在レポート（kalman/cpp）

作成日: 2025-12-22

概要:
- `kalman/cpp` 以下に `float` と `double` が混在しています。
- MEX 入出力（MATLAB API）は `double` 前提の箇所が多数あり、内部実装は `float` を使っている箇所が多いです。

重点ファイル（確認済み）:
- MEX ラッパ/ユーティリティ:
  - `kalman/cpp/MEX/mex_eskf_helper.cpp` (mxGetPr → `double*`, 内部 `ESKFHelper<float>` を使用)
  - `kalman/cpp/MEX/mex_eskf_core_v2.cpp` (mxGetPr/戻り値は `double*`、内部で `float` に変換)
  - `kalman/cpp/MEX/mex_meukf_step.cpp` (出力を `double` に詰める処理あり)
  - `kalman/cpp/MEX/mex_common_lib.cpp` (入力/出力 `double*` だが内部で `float` キャストあり)
  - `kalman/cpp/MEX/mex_ukf_update_minimal.cpp`, `mex_ukf_update.cpp`, `mex_ukf_sigma_points.cpp` 等

- コアライブラリ / MEUKF:
  - `kalman/cpp/MEUKF/meukf_types.hpp` (State 構造体は `float`)
  - `kalman/cpp/MEUKF/meukf_core.cpp` (多数の `float` 演算)
  - `kalman/cpp/MEUKF/unified_filter.cpp` (一部 `double` 使用あり in include/MEUKF)

- UKF:
  - `kalman/cpp/UKF/Core/ukf_sigma_points.cpp` / `ukf_update.*` (`float` のリテラル / テンプレート)
  - `kalman/cpp/src/UKF/` も同様

- その他（テスト / ドキュメント）:
  - `kalman/cpp/tests/compare_cholesky.cpp` (double)
  - ドキュメント: `kalman/cpp/tests/api_diff_phase3.md`, `kalman/cpp/Common/README.md` (float/double に言及)

推奨短期方針:
1. MEX API は `double` のまま維持し、MEX内で明示的に `double`→`float` / `float`→`double` の変換を行うラッパーを導入する（互換性維持、最小破壊）。
2. コア実装を全面的に `float` に統一する場合は、テンプレート/using を利用して `float` に固定する（段階的に実施）。
3. 変換パッチはまず MEX ラッパ周りで行い、ビルド→`run_batch_10sets()` で検証する。

次のアクション候補:
- `A`: 全ソースの `float` 一括変換パッチを作成（破壊的、推奨しない）
- `B`: MEXラッパーで変換を明示的に実装→内部を `float` に統一（推奨、段階的）
- `C`: さらに詳細なファイル一覧（全match行のフルダンプ）を生成

-- end --
