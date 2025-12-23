# Utils MEX化 計画 - ドキュメント索引

**作成日**: 2025年12月23日  
**計画状態**: ✅ 完成 - 実装準備完了  
**次のステップ**: Phase 0 実行

---

## 📚 計画書一覧

### 1. 📄 [UTILS_MEX_SUMMARY.md](UTILS_MEX_SUMMARY.md) - **最初に読むドキュメント ⭐**

| 対象者 | マネージャー / リーダー | 時間 | 5分 |
|--------|-----------|------|-------|
| **内容** | プロジェクト全体の概要、リスク、タイムライン |
| **含まれる情報** | 削除予定ファイル、実行手順、所要時間 |
| **利用シーン** | 全体像の把握、ステークホルダーへの説明 |

**キーポイント**:
- ✅ 呼び出し関係の完全把握済み
- 🟢 即座に削除可能: BiquadFilter.m, OutlierGuard.m
- 🟠 段階的 MEX 化: 4つの SensorFilter クラス
- ⏱️ 所要時間: ~3 時間

---

### 2. 🔍 [UTILS_DEPENDENCY_FINAL_REPORT.md](UTILS_DEPENDENCY_FINAL_REPORT.md) - **実装開始時に読む**

| 対象者 | 開発者 | 時間 | 10分 |
|--------|--------|------|-------|
| **内容** | 呼び出し関係の詳細分析、ESKF での使用状況 |
| **含まれる情報** | 各ファイルの削除/保持判定、修正コード例 |
| **利用シーン** | 実装前のチェックリスト、修正内容の確認 |

**キーポイント**:
- ESKF.m が全ての Sensor*Filter を生成していることを確認
- 各段階での修正内容を記載
- テスト実行手順を明記

---

### 3. 📊 [UTILS_MEXIFICATION_PLAN.md](UTILS_MEXIFICATION_PLAN.md) - **詳細な実装計画書**

| 対象者 | 開発者 / テスター | 時間 | 20分 |
|--------|-----------|------|-------|
| **内容** | 段階的な MEX 化の詳細、リスク評価、チェックリスト |
| **含まれる情報** | 低レイヤーから順番の実装手順、検証方法 |
| **利用シーン** | 実装中の参考、各フェーズの確認 |

**キーポイント**:
- 原則: 低レイヤーから順番にコメント化
- 各段階で回帰テスト必須
- リスク対策を記載

---

### 4. 📈 [UTILS_DEPENDENCY_VISUAL.md](UTILS_DEPENDENCY_VISUAL.md) - **ビジュアル図解**

| 対象者 | 全員 | 時間 | 5分 |
|--------|------|------|--------|
| **内容** | 現在の構成図、データフロー、ファイル削減効果 |
| **含まれる情報** | ASCII図、フェーズタイムライン、トラブルシューティング |
| **利用シーン** | 仕組みの理解、テーブルへの説明 |

**キーポイント**:
- 現在: 3層のラッパー構造 → 完了後: 1層
- 削減ファイル数: 9個 (60%)
- トラブルシューティング付き

---

## 🎯 ドキュメント選択ガイド

### 「全体像を 5 分で知りたい」
→ [UTILS_MEX_SUMMARY.md](UTILS_MEX_SUMMARY.md)

### 「実装を開始する」
→ [UTILS_DEPENDENCY_FINAL_REPORT.md](UTILS_DEPENDENCY_FINAL_REPORT.md) + [UTILS_MEXIFICATION_PLAN.md](UTILS_MEXIFICATION_PLAN.md)

### 「ファイル依存関係を図解で理解したい」
→ [UTILS_DEPENDENCY_VISUAL.md](UTILS_DEPENDENCY_VISUAL.md)

### 「各フェーズの詳細を知りたい」
→ [UTILS_MEXIFICATION_PLAN.md](UTILS_MEXIFICATION_PLAN.md)

---

## 📋 実行手順チェックリスト

### 【事前準備】
```
[ ] UTILS_MEX_SUMMARY.md を読む
[ ] 依存関係を理解する
[ ] TYPE_MIX_REPORT.md で型混在をチェック
```

### 【Phase 0 - 即座削除（15分）】
```
[ ] BiquadFilter.m 削除
[ ] OutlierGuard.m 削除
[ ] run_simulation(42, true) で動作確認
```

### 【Phase 1 - 段階的 MEX 化（110分）】
```
段階1-1: SensorAccelFilter
  [ ] apply() を SensorFilters.accel() に統一
  [ ] run_batch_10sets() 実行
  [ ] compare_mex_matlab_detailed() で確認

段階1-2: SensorMagFilter
  [ ] apply() を SensorFilters.mag() に統一
  [ ] run_batch_10sets() 実行
  [ ] 確認

段階1-3: SensorGPSFilter
  [ ] apply() を SensorFilters.gps() に統一
  [ ] run_batch_10sets() 実行
  [ ] 確認

段階1-4: SensorBaroFilter
  [ ] apply() を SensorFilters.baro() に統一
  [ ] run_batch_10sets() 実行
  [ ] 確認

確認事項:
  [ ] RMSE (Roll/Pitch) < 0.30°
  [ ] GPS/Baro/Mag パリティ保持
```

### 【Phase 2 - 最終削除（35分）】
```
[ ] AccelFilter.m 削除
[ ] SensorAccelFilter.m 削除
[ ] SensorMagFilter.m 削除
[ ] SensorGPSFilter.m 削除
[ ] SensorBaroFilter.m 削除
[ ] run_batch_10sets() 実行（最終確認）
```

---

## 🔄 参照関係

```
UTILS_MEX_SUMMARY.md (概要)
  │
  ├─→ UTILS_DEPENDENCY_FINAL_REPORT.md (詳細)
  │   └─→ 各ファイルの修正方法
  │
  ├─→ UTILS_MEXIFICATION_PLAN.md (詳細計画)
  │   └─→ 段階的実装手順
  │
  └─→ UTILS_DEPENDENCY_VISUAL.md (図解)
      └─→ 仕組みの理解
```

---

## 📊 計画内容サマリー

### 削除対象（全9ファイル）

| ファイル | 理由 | タイミング |
|---------|------|---------|
| BiquadFilter.m | 呼び出し元なし | Phase 0 |
| OutlierGuard.m | 呼び出し元なし | Phase 0 |
| AccelFilter.m | 呼び出し元なし | Phase 2 |
| SensorAccelFilter.m | ESKF で直接新規化 → SensorFilters に統一 | Phase 2 |
| SensorMagFilter.m | ESKF で直接新規化 → SensorFilters に統一 | Phase 2 |
| SensorGPSFilter.m | ESKF で直接新規化 → SensorFilters に統一 | Phase 2 |
| SensorBaroFilter.m | ESKF で直接新規化 → SensorFilters に統一 | Phase 2 |
| （計9個） | | |

### 保持対象（全6ファイル）

| ファイル | 役割 |
|---------|------|
| SensorFilters.m | MEX ラッパー統合ハブ |
| NoiseEstimator.m | 互換性層 + MEX 委譲 |
| DivergenceGuard.m | MEX 委譲完了 |
| alpha_beta_step.m | mex_filter_utils 呼び出し |
| ema_update.m | mex_filter_utils 呼び出し |
| hampel_causal.m | mex_filter_utils 呼び出し |

---

## ⏰ タイムライン

```
Phase 0 (即座)
  ├─ 削除: 5分
  ├─ テスト: 10分
  └─ 合計: 15分 ← 【今ここから開始推奨】

Phase 1 (段階的)
  ├─ SensorAccelFilter: 30分
  ├─ SensorMagFilter: 30分
  ├─ SensorGPSFilter: 30分
  ├─ SensorBaroFilter: 30分
  └─ 合計: 110分

Phase 2 (最終)
  ├─ 削除: 15分
  ├─ テスト: 20分
  └─ 合計: 35分

全体: 約 3 時間
```

---

## 🚀 次のステップ

### 今すぐ（5分）
1. [UTILS_MEX_SUMMARY.md](UTILS_MEX_SUMMARY.md) を読む
2. 全体像を理解する

### 準備（15分）
3. [UTILS_DEPENDENCY_FINAL_REPORT.md](UTILS_DEPENDENCY_FINAL_REPORT.md) を読む
4. TYPE_MIX_REPORT.md で型混在をチェック
5. 修正対象ファイルを確認

### 実装開始（20分後）
6. Phase 0 を実行（BiquadFilter + OutlierGuard 削除）
7. `run_simulation(42, true)` で確認
8. Phase 1 開始

---

## 📞 質問・問題時の参照

| 問題 | 参照先 |
|------|--------|
| 全体像が不明 | UTILS_MEX_SUMMARY.md |
| 実装方法が不明 | UTILS_DEPENDENCY_FINAL_REPORT.md |
| 段階的手順が不明 | UTILS_MEXIFICATION_PLAN.md |
| 仕組みが不明 | UTILS_DEPENDENCY_VISUAL.md |
| MEX の型問題 | TYPE_MIX_REPORT.md |
| MATLAB/MEX パリティ | MATLAB_MEX_PARITY_CHECKLIST.md |

---

## ✅ 最後に確認すること

- [ ] 全4つの計画ドキュメントを認識している
- [ ] UTILS_MEX_SUMMARY.md で 5 分で理解できた
- [ ] 削除対象と保持対象が明確である
- [ ] タイムラインが理解できた
- [ ] 次のステップが明確である

---

**計画完成日**: 2025年12月23日  
**実装準備状態**: ✅ 完了  
**推奨開始日**: 本日から（Phase 0 から順に）

