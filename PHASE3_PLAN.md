# Phase 3: 品質・性能向上計画 (開始)

**開始日**: 2026年1月5日

## 目的
Lib 内の重複実装を精査・統一し、信頼性と保守性を向上させる。
高優先度は「四元数正規化」「共分散対称化」「Mahalanobis/Innovation の統一」。

## スコープ（短期, 2 週間目標）
1. 四元数正規化の統一
   - 標準実装: `cquat::normalize_quat()`
   - 影響範囲: `ESKF`, `MEUKF`, `MEX` の呼び出しを検査し置換
2. 共分散対称化の一本化
   - 標準実装: `common::filter::symmetrize_covariance()`
   - 影響範囲: P 更新箇所、postprocess、runner
3. Mahalanobis / Innovation 計算の集約
   - 標準実装: `Common::Math::MathUtils::mahalanobis_distance_*` と `compute_innovation_and_S`
   - 影響範囲: sensor_filter, eskf_sensor_updates, ekf_linear_update
4. 重複コード検出・一覧作成
   - grep による自動一覧生成
5. 回帰テスト拡充
   - 外れ値ケース（IMU spike, GPS dropout）を 3 ケース追加
6. ドキュメント更新
   - `docs/LIB_STRUCTURE_ANALYSIS.md` と `PROGRESS_PHASE2.md` に反映

## 実行手順（実務）
- Step A: 影響箇所の一覧化（自動 grep）
  - コマンド例:
    ```bash
    grep -R "normalize_quat\|normalize_quaternion\|quat_normalize" -n kalman/cpp | sort
    grep -R "symmetrize_covariance\|symmetrizeCov" -n kalman/cpp | sort
    grep -R "mahalanobis_distance" -n kalman/cpp | sort
    ```

- Step B: 安全な一括置換（apply_patch を使い、1 ファイルずつ検証）
- Step C: ビルド & 単体テスト（`build_mex()` → `run_simulation()`）
- Step D: 回帰テスト（`run_batch_10sets()`）
- Step E: ドキュメント更新とコミット

## リスクと対策
- 既存 API に影響する可能性 → ラッパー関数を残しつつ内部で統一実装を呼ぶ
- ビルド破壊リスク → 小さなチャンクでコミット、回帰テストを常時実行

## 期待成果（2週目終了時）
- 四元数正規化が単一実装へ移行（コード行数削減）
- 共分散対称化が単一実装へ移行
- Mahalanobis/Innovation の統一により外れ値判定の一貫性向上
- 追加テストでロバスト性を検証済み
- 更新内容を反映した `PHASE3_PLAN.md` と PR

---

次のアクション: `四元数正規化の統一` を実行して、小さな変更を 1-2 ファイルに適用しビルド・テストを行います。

## 進捗（2026-01-05）

- `四元数正規化の統一` を適用: `cquat::normalize_quat()` を主要呼び出しに統一しました。
- ビルド: `build_mex()` 実行 → 成功（mex バイナリ出力）
- 回帰テスト: `run_batch_10sets()` 実行 → 10/10 PASS（詳細は `kalman/Results/log/batch_10sets_log_20260105_150340.txt`）

次は「共分散対称化の統一」を進めます。
 
## 進捗（共分散対称化）


結論: 現状で「共分散対称化」は実質的に統一済みのため、追加変更は最小限（ラッパーの整理とドキュメント明確化）で済みます。

次アクション: ラッパー整理と `docs/LIB_STRUCTURE_ANALYSIS.md` の更新を行います。

## Mahalanobis / Innovation 統一の所見

- 解析結果: `common::math::MathUtils::compute_innovation_and_S` と `MathUtils::mahalanobis_distance_squared` が既に主要実装として存在します。
- ただし `KF/inc/kalman_filter_core.hpp` にテンプレート実装の `compute_innovation_and_S` が残っており、実体の重複があるため将来的に `KF` 側を MathUtils 呼び出しに置換するリファクタを推奨します。
- すぐに破壊的変更を行わず、まずはドキュメントに統一方針を明記し、小さな段階的リファクタ（ラッパー作成→置換）で進めます。

次アクション: `KF` ヘッダのテンプレート実装を段階的に MathUtils に委任する作業をスケジュールします。
