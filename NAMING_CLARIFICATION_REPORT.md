# 実装構造の整理完了レポート

**作成日**: 2026年1月12日  
**目的**: ESKFとMEUKFの実装混在を整理し、ドキュメントを更新

---

## 1. 実施内容サマリー

### 1.1 発見された問題

| 問題 | 詳細 |
|-----|------|
| 名前と実装の不一致 | `mex_run_eskf` という名前だが、実際は **ESKF予測 + MEUKF更新のハイブリッド** |
| 誤解を招く構造 | ESKFが使われていないと誤認識されていた |
| ドキュメント不足 | ハイブリッド構造が明示されていなかった |

### 1.2 実施した修正

| 修正内容 | ファイル | 状態 |
|---------|---------|------|
| **ドキュメント更新** | `.github/copilot-instructions.md` | ✅ 完了 |
| **詳細分析作成** | `IMPLEMENTATION_ANALYSIS.md` | ✅ 完了 |
| **コメント追加** | `MEX/mex_run_eskf.cpp` | ✅ 完了 |
| **コメント追加** | `MEX/Impl/mex_run_eskf_impl.hpp` | ✅ 完了 |
| **未使用関数マーク** | `Lib/ESKF/inc/eskf_core.hpp` | ✅ 完了 |
| **Phase計画修正** | `REFACTORING_PLAN.md` | ✅ 完了 |

---

## 2. 実装の実態（確認結果）

### 2.1 ハイブリッドフィルタ構成

```
mex_run_eskf('step')
  │
  ├─ ESKFRunner::predict()               ← ESKF予測ステップ
  │   ├─ ESKFCore::integrate_nominal()   (RK2積分)
  │   └─ ESKFCore::predict_covariance()  (共分散予測)
  │
  └─ call_sensor_update()                ← MEUKF更新ステップ
      └─ handle_sensor_update_internal()
          └─ do_sensor_update_meukf()
              └─ MEUKFCore::step()       (UKFベース更新)
                  ├─ update_accel_meukf()
                  ├─ update_mag_meukf()
                  ├─ update_gps_meukf_ukf_version()
                  └─ update_baro_meukf_ukf_version()
```

### 2.2 各コンポーネントの使用状況

| コンポーネント | 実装 | 使用中 | 役割 |
|-------------|------|-------|------|
| **ESKFCore** | ✅ | ✅ | 予測（integrate_nominal, predict_covariance, update_zupt） |
| **ESKFRunner** | ✅ | ✅ | 予測ステップの統合実行 |
| **MEUKFCore** | ✅ | ✅ | センサー更新（Accel/Mag/GPS/Baro） |
| `ESKFCore::update_accel` | ✅ | ❌ | MEUKFで置き換え済み（未使用） |
| `ESKFCore::update_mag` | ✅ | ❌ | MEUKFで置き換え済み（未使用） |
| `ESKFCore::update_gps` | ✅ | ❌ | MEUKFで置き換え済み（未使用） |
| `ESKFCore::update_baro` | ✅ | ❌ | MEUKFで置き換え済み（未使用） |
| `mex_meukf_step_v2` | ✅ | ❌ | レガシー（ビルドから除外済み） |

---

## 3. 更新されたドキュメント

### 3.1 `.github/copilot-instructions.md`

新セクション追加：
```markdown
## 【最優先ルール】フィルタ実装の構造

### ハイブリッドフィルタ構成（重要）
**注意**: `mex_run_eskf` という名前だが、実装は **ESKF予測 + MEUKF更新のハイブリッド** です。

- **予測ステップ**: ESKFCore（誤差状態カルマンフィルタ）
- **更新ステップ**: MEUKFCore（UKFベース測定更新）
```

### 3.2 `IMPLEMENTATION_ANALYSIS.md`（新規作成）

以下の情報を含む詳細分析：
- 実装フローの図解
- ESKFとMEUKFの役割分担
- 命名の修正提案（3つのオプション）
- 削除可能/不可なコンポーネント一覧
- 技術的補足

### 3.3 `REFACTORING_PLAN.md`

修正箇所：
- セクション1.0 追加（フィルタ実装の実態）
- Phase 4: 保持するクラスに「削除不可」マーク追加
- Phase 5: 削除対象に注意書き追加

---

## 4. コードへのコメント追加

### 4.1 `mex_run_eskf.cpp`

```cpp
/* mex_run_eskf.cpp
 * Hybrid Filter MEX Interface (ESKF Prediction + MEUKF Update)
 *
 * ARCHITECTURE:
 *   - Prediction Step: ESKF (Error-State Kalman Filter)
 *   - Update Step: MEUKF (Multiplicative Extended UKF)
 *
 * NOTE: Despite the name "eskf", this implementation uses MEUKF for all
 *       sensor updates (Accel, Mag, GPS, Baro).
 */
```

### 4.2 `mex_run_eskf_impl.hpp` (do_sensor_update_meukf)

```cpp
inline void do_sensor_update_meukf(...) {
    // MEUKF-based sensor update (UKF core)
    // This function performs Kalman gain calculation using MEUKFCore::step()
    // which implements Unscented Kalman Filter for sensor updates.
    // Called from handle_sensor_update_internal()
```

### 4.3 `eskf_core.hpp`

未使用のセンサー更新関数にマーク追加：
```cpp
// ====================================================================
// NOTE: 以下のセンサー更新関数は MEUKF で置き換え済み（未使用）
// 実際のセンサー更新は MEUKFCore::step() で実行される
// ====================================================================

// update_accel - UNUSED (replaced by MEUKFCore::update_accel_meukf)
// update_mag - UNUSED (replaced by MEUKFCore::update_mag_meukf)
// update_gps - UNUSED (replaced by MEUKFCore::update_gps_meukf_ukf_version)
// update_baro - UNUSED (replaced by MEUKFCore::update_baro_meukf_ukf_version)
```

---

## 5. 削除不可なコンポーネント（重要）

以下のコンポーネントは **絶対に削除してはいけない**：

| コンポーネント | 理由 |
|-------------|------|
| `Lib/ESKF/` ディレクトリ | 予測ステップで使用中 |
| `Lib/MEUKF/` ディレクトリ | センサー更新で使用中 |
| `ESKFCore` クラス | 予測・ZUPT更新で必須 |
| `ESKFRunner` クラス | 予測統合で必須 |
| `MEUKFCore` クラス | センサー更新で必須 |
| `do_sensor_update_meukf()` 関数 | MEUKFCore::step() の呼び出しに必須 |

---

## 6. 削除可能なコンポーネント（オプション）

以下は削除可能だが、保持推奨：

| コンポーネント | 理由 | 推奨 |
|-------------|------|------|
| `ESKFCore::update_accel/mag/gps/baro` | MEUKFで置き換え済み | ⚠️ 保持（将来的な切り替え用） |
| `mex_meukf_step.cpp` | レガシー互換性用 | ✅ 保持（互換性のため） |

完全に削除して問題ないもの：
- `KF/`, `EKF/`, `UKF/` ディレクトリ（未使用）

---

## 7. 将来の検討事項（低優先度）

### 7.1 命名の統一（Option A）

**提案**:
- `mex_run_eskf` → `mex_run_hybrid_filter`
- `ESKFState` → `FilterState` または `HybridFilterState`

**メリット**: 実装と名前が一致  
**デメリット**: MATLAB側の大幅変更が必要  
**結論**: 現時点では **保留**（ドキュメントで対応済み）

### 7.2 未使用関数の削除

**提案**: `ESKFCore::update_accel/mag/gps/baro` を削除

**メリット**: コードの簡潔化  
**デメリット**: 将来的にMEUKFからESKFに戻す際の再実装が必要  
**結論**: 現時点では **保留**（コメントで「未使用」とマーク済み）

---

## 8. まとめ

### 8.1 達成した目標

- ✅ 実装の実態を詳細に分析・文書化
- ✅ 紛らわしい名前を明確化（コメント追加）
- ✅ ドキュメントを更新（ハイブリッド構成を明記）
- ✅ Phase計画を修正（ESKF/MEUKF削除不可を明記）
- ✅ 未使用関数をマーク（将来の混乱を防止）

### 8.2 現在の構造（確定）

**フィルタ名**: `mex_run_eskf`（名前は変更せず）  
**実装**: ESKF予測 + MEUKF更新のハイブリッド  
**理由**: ESKFの効率的な予測 + MEUKFの頑健な更新を組み合わせた設計  

### 8.3 ドキュメント階層

```
1. 即座に理解: .github/copilot-instructions.md（ハイブリッド構成の概要）
2. 詳細分析: IMPLEMENTATION_ANALYSIS.md（実装フロー・技術詳細）
3. 計画修正: REFACTORING_PLAN.md（Phase計画の更新版）
4. 完了報告: NAMING_CLARIFICATION_REPORT.md（本ドキュメント）
```

---

## 9. 次のステップ（オプション）

1. **即時不要**: 現在の構造で問題なく動作している
2. **短期オプション**: 未使用のESKFセンサー更新関数を削除（リスク低）
3. **長期オプション**: `mex_run_hybrid_filter` への名前変更（リスク高、要MATLAB側変更）

**現時点の結論**: **現状維持を推奨**。ドキュメントとコメントで十分に明確化済み。

---

**完了日**: 2026年1月12日  
**ビルド確認**: ✅ 成功（コメント追加のみのため影響なし）  
**テスト確認**: 次回の `run_batch_10sets()` で確認推奨
