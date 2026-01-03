# KalmanFilter C++ リファクタリング概要

このファイルはリファクタリング進行の短い実行ガイドです。主要変更点は以下の通りです。

- Phase 3: 共通インターフェース
  - `Lib/Common/inc/interface.hpp` を追加し、`Filter` 基底クラスを定義しました。
  - `Lib/ESKF` に `filter.hpp` / `filter.cpp`（最小実装）を追加。
  - スタンドアロン API: `Lib/Common/inc/standalone.hpp` と `Lib/Common/src/standalone.cpp` を追加。

- Phase 4: マスターヘッダーとインクルード統合
  - `Inc/kalman_all.hpp` を整備し、主要ヘッダーを集約しました。
  - `build/build_mex.m` を更新して `Lib/*/inc` を自動で -I に追加、MEX ビルドで `standalone` を除外する定義を追加しました。

- Phase 5: スタンドアロン例
  - `examples/main_standalone.cpp` を追加（`kalman::filter_init()` を使う簡易例）。
  - `examples/test_interface.cpp` など、ビルド確認用のサンプルを用意済み。

次の推奨アクション
- `build_mex()` を実行して MEX ビルドログを確認（既に成功ログあり）。
- C++ 側の本格実装: `Lib/ESKF` の本実装（predict/update）を `standalone` に接続。

問題や追加自動化したいチェックがあれば教えてください。
# REFACTORING - 実行ガイド

このディレクトリのリファクタリング作業手順と進め方を記載します。

主要ファイル:
- `REFACTORING_PLAN.md` : 詳細なフェーズ別計画（実施済）

目的:
- 不要ドキュメントの整理
- C++ ライブラリ構造の標準化
- スタンドアロン API の実装

作業手順（概要）:
1. `archive/` に過時ファイルを移動済み
2. `REFACTORING_PLAN.md` に従ってフェーズ2以降を実施

注意:
- 既存の Git 履歴は `git mv` により保持しています（可能な場合）。
- 実施中は重要な変更を小さなコミットで残してください。

進捗は `kalman/cpp/markdown/REFACTORING_PLAN.md` を更新してください。