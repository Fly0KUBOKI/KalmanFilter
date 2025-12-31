# MEX 統合計画 — 全体ガイドマップ

**版**: 1.0  
**作成日**: 2025年12月31日  
**対象**: KalmanFilter プロジェクトの mex_meukf_step_v2 + mex_sensor_filter → mex_run_eskf 統合

---

## 📋 ドキュメント構成

統合計画は以下の5つのドキュメントで構成されています。用途に応じて参照してください。

### 1. **[MEX_INTEGRATION_PRIORITY.md](MEX_INTEGRATION_PRIORITY.md)** 
   📌 **最初に読むドキュメント**

   **内容**:
   - 現状のMEX構成確認
   - 統合優先度の判定基準と計算結果
   - 統合による効果の見積り
   - リスク評価と対策

   **対象者**: プロジェクトマネージャー、意思決定者

   **読む順序**: 最初（全体方針の理解）

   ---

### 2. **[MEX_COMPLETE_INTEGRATION_PLAN.md](MEX_COMPLETE_INTEGRATION_PLAN.md)** 
   📌 **実装者向けの詳細計画書**

   **内容**:
   - Phase 0-5 の詳細な実装ステップ
   - C++ コード実装例
   - ビルド設定変更
   - テスト方法（3段階）
   - トラブルシューティング
   - ロールバック手順

   **対象者**: C++ 開発者、統合実装者

   **読む順序**: 2番目（詳細理解・実装準備）

   ---

### 3. **[MEX_INTEGRATION_CHECKLIST.md](MEX_INTEGRATION_CHECKLIST.md)** 
   📌 **実装時の実行チェックリスト**

   **内容**:
   - Phase ごとの詳細なチェックリスト
   - 各ステップの確認項目
   - テスト結果の記録フォーマット
   - トラブルシューティング記録テンプレート

   **対象者**: 実装者（実行時に参照）

   **読む順序**: 3番目（実装開始時）

   ---

### 4. **[INTEGRATION_FAILURE_ROOT_CAUSE.md](INTEGRATION_FAILURE_ROOT_CAUSE.md)** 
   📌 **過去の失敗から学ぶドキュメント**

   **内容**:
   - 過去の統合失敗の根本原因分析
   - 失われた要素の詳細
   - 再発防止策（Phase 1-4）
   - 標準チェックリスト
   - 緊急対応手順

   **対象者**: レビューア、品質管理

   **読む順序**: 随時（予防・問題発生時）

   ---

### 5. **[MEX_INTEGRATION_STANDARD_PROCESS.md](MEX_INTEGRATION_STANDARD_PROCESS.md)** 
   📌 **一般的な統合手順の参考資料**

   **内容**:
   - MEX統合の標準的なプロセス（Phase 1-6）
   - トラブルシューティング
   - チェックリスト

   **対象者**: 将来の統合プロジェクト

   **読む順序**: 必要に応じて（他のプロジェクト参照用）

---

## 🎯 実装フロー

```
┌─────────────────────────────────────────────────────┐
│ Step 1: 全体方針理解                                │
│ → MEX_INTEGRATION_PRIORITY.md を読む                │
│   (優先度、効果、リスク評価)                         │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│ Step 2: 詳細計画確認                                │
│ → MEX_COMPLETE_INTEGRATION_PLAN.md を読む            │
│   (Phase 0-5 の詳細、実装例、テスト方法)            │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│ Step 3: 実装開始（本番実行）                        │
│ → MEX_INTEGRATION_CHECKLIST.md を開く (プリント)    │
│ → Phase 0 から順に実行、各項目をチェック            │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│ Step 4: トラブル発生時                              │
│ → INTEGRATION_FAILURE_ROOT_CAUSE.md で類似例を検索  │
│ → MEX_COMPLETE_INTEGRATION_PLAN.md のTSセクション   │
│ → ロールバック手順を実行                            │
└─────────────────────────────────────────────────────┘
```

---

## ⚡ クイックスタート（今すぐ実行）

### Phase 0（本日実施推奨）

```bash
# 1. 現在の状態をバックアップ
cd kalman
git add -A
git commit -m "[backup] 統合前のクリーンな状態を記録"
git stash create "phase0-baseline"

# 2. ベースラインテストを実行
clear mex
run_simulation(42, true)
cp kalman/Results/estimation_01.csv estimation_phase0.csv

# 3. バージョン確認
git log --oneline -1
git stash list  # バックアップが存在するか確認
```

**期待結果**: ✅ バックアップ完成、復旧可能な状態に

---

## 📊 計画概要

### 統合対象

```
現在（独立）:
  ┌─────────────────────┐
  │ mex_meukf_step_v2   │  ← MEUKF状態更新
  │                     │
  │ mex_sensor_filter   │  ← センサーフィルタ
  │                     │
  │ mex_run_eskf        │  ← ESKF Predict+Update
  └─────────────────────┘

目標（統合）:
  ┌─────────────────────────────────┐
  │   mex_run_eskf (統合版)         │
  │ ┌──────────────────────────────┤
  │ │ - ESKF Predict + Update       │
  │ │ - MEUKF Step (旧 v2)         │
  │ │ - Sensor Filter              │
  │ └──────────────────────────────┤
  └─────────────────────────────────┘
```

### 推奨スケジュール

| Phase | 内容 | 所要時間 | ステータス |
|-------|------|---------|----------|
| 0 | 準備・バックアップ | 1h | ✅ 今日実施可 |
| 1 | MEUKF統合 | 2-3h | 📅 明日以降 |
| 2 | センサーフィルタ統合 | 2-3h | 📅 明日以降 |
| 3 | 検証（単体・非回帰・バッチ） | 1-2h | 📅 その後 |
| 4 | 確定・コミット | 0.5h | 📅 その後 |
| 5 | 後始末（バイナリ削除） | 0.25h | 📅 1週間後 |
| **合計** | | **~10h** | |

---

## 🔒 安全性と復旧

### バックアップ戦略

各 Phase の完了後に **git stash** でバックアップを作成：

```bash
Phase 0 完了時:
$ git stash create "phase0-baseline"

Phase 1 完了時:
$ git stash create "phase1-meukf-integrated"

Phase 2 完了時:
$ git stash create "phase2-sensor-filter-integrated"

Phase 3 完了時:
$ git stash create "phase3-integration-verified"

Phase 4 完了時:
$ git stash create "phase4-integration-final"
```

### 復旧方法（何かうまくいかないとき）

```bash
# バックアップ一覧を確認
git stash list

# 最後に成功した状態へ復旧
git stash apply phase3-integration-verified

# 復旧後、テスト実行
clear mex
run_batch_10sets()
```

---

## ✅ 成功判定基準

統合が成功したと言えるのは以下の条件をすべて満たすとき：

### Phase 3 の検証
- ✅ 非回帰テスト: `max_diff < 1e-4`
- ✅ バッチテスト: `10/10 成功（100%）`
- ✅ エラー検出: `NaN/Inf なし`

### Phase 4 の確定
- ✅ 最終コミット: 詳細メッセージ付き
- ✅ git tag: `v1.0-integrated-mex` で記録

### Phase 5（1週間後）
- ✅ 継続テスト: `5回以上の run_batch_10sets()` で成功
- ✅ 旧MEX削除: バイナリをリポジトリから除去

---

## ⚠️ 重要な注意事項

### ❌ やってはいけないこと

```
❌ 実装なしで MEX バイナリを削除する
   → 復旧不能、全シミュレーション失敗

❌ 初期化コード（mex_sensor_filter('reset_zero')）を削除する
   → NaN/Inf 確定

❌ 複数の Phase を同時に実装する
   → 問題の切り分けが不可能

❌ テストなしでコミットする
   → バグが本番に混入
```

### ✅ 推奨される進め方

```
✅ 実装 → テスト → コミット → バックアップ の順序
✅ Phase ごとに分割実装
✅ 単体テスト（差分 < 1e-10）で確認
✅ 非回帰テスト（差分 < 1e-4）で確認
✅ バッチテスト（10/10）で確認
✅ 各 Phase 後に git stash で復旧ポイント作成
```

---

## 📞 トラブル時の相談先

| トラブル | 参照ドキュメント | セクション |
|---------|----------------|----------|
| ビルドエラー | MEX_COMPLETE_INTEGRATION_PLAN.md | Phase 1.4, トラブル1 |
| NaN/Inf 発生 | INTEGRATION_FAILURE_ROOT_CAUSE.md | 再発防止策 |
| 非回帰テスト失敗 | MEX_COMPLETE_INTEGRATION_PLAN.md | トラブル2-3 |
| 実装方法不明 | MEX_COMPLETE_INTEGRATION_PLAN.md | Step 1.2, 2.2 |
| ロールバック方法 | MEX_COMPLETE_INTEGRATION_PLAN.md | トラブルシューティング |

---

## 🎓 参考資料

### 過去のプロジェクト
- `kalman/cpp/build/04_INTEGRATION_REFACTORING_PLAN.md` — 統合の高レベル設計
- `kalman/cpp/build/05_CURRENT_STATUS_AND_FAILURE_ANALYSIS.md` — 現状分析

### ソースコード
- [kalman/cpp/MEX/mex_meukf_step.cpp](../kalman/cpp/MEX/mex_meukf_step.cpp)
- [kalman/cpp/MEX/mex_sensor_filter.cpp](../kalman/cpp/MEX/mex_sensor_filter.cpp)
- [kalman/cpp/MEX/mex_run_eskf.cpp](../kalman/cpp/MEX/mex_run_eskf.cpp)

### ビルド設定
- [kalman/cpp/build/build_mex.m](../kalman/cpp/build/build_mex.m)

### テスト環境
- [kalman/run_batch_10sets.m](../kalman/run_batch_10sets.m)
- [kalman/run_simulation.m](../kalman/run_simulation.m)

---

## 📝 ステータス追跡

現在の進捗を記録：

```
準備状況:
  □ ドキュメント確認完了
  □ Phase 0 実装完了
  □ バックアップ作成完了
  
実装中（チェック状況）:
  □ Phase 1: _____________________________
  □ Phase 2: _____________________________
  □ Phase 3: _____________________________
  □ Phase 4: _____________________________
  □ Phase 5: _____________________________

最終ステータス:
  □ 統合完了
  □ テスト合格
  □ 本番運用開始
```

---

## 📞 サポート連絡先

質問やトラブルが発生した場合：

1. **ドキュメント検索**: 上記の「参照セクション」を参照
2. **ロールバック**: git stash で復旧
3. **再実装**: Phase を一つ前に戻す

---

**最終更新**: 2025年12月31日  
**ドキュメント版**: 1.0  
**対象バージョン**: KalmanFilter phase6+

---

## 🚀 次のアクション

```
【今すぐ（本日）】
1. MEX_INTEGRATION_PRIORITY.md を読む
2. MEX_COMPLETE_INTEGRATION_PLAN.md を確認
3. Phase 0 を実行 → バックアップコミット

【明日以降】
4. MEX_INTEGRATION_CHECKLIST.md をプリント
5. Phase 1 を実行開始（MEUKF統合）
6. Phase 2 を実行（センサーフィルタ統合）

【その後】
7. Phase 3-4 実行（検証・確定）
8. 1週間テスト後、Phase 5 実行（後始末）
```

**Good luck! 統合が成功することを祈ります。** 🍀
