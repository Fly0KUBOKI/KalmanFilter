# 実装構造の詳細分析

**作成日**: 2026年1月12日  
**目的**: ESKFとMEUKFの実装の混在状況を整理し、命名の不一致を修正する

---

## 1. 発見された問題

### 1.1 名前と実装の不一致

| ファイル/関数名 | 期待される実装 | 実際の実装 | 問題 |
|--------------|-------------|----------|------|
| `mex_run_eskf.cpp` | ESKF | **MEUKFベース** | ❌ 名前詐称 |
| `do_sensor_update_meukf()` | MEUKF | MEUKF | ✅ 正しい |
| `MEUKFCore::step()` | MEUKF | MEUKF | ✅ 正しい |
| `mex_meukf_step_v2` | MEUKF | 無効化済み | ⚠️ レガシー |
| `ESKFCore` | ESKF | ESKF | ✅ 正しい（未使用） |
| `ESKFRunner` | ESKF | ESKF | ✅ 正しい（未使用） |

### 1.2 実際の呼び出しフロー

```
run_simulation.m
  ↓
mex_run_eskf('init')  ← 名前は ESKF だが...
  ↓
ESKFState* 初期化
  ↓
mex_run_eskf('step')
  ↓
do_step(ESKFState*, obs, k)
  ├─ call_predict(s, a_f, w_f)  ← ESKFRunner::predict() を呼ぶ（ESKF予測）
  │   └─ ESKFCore::integrate_nominal()
  │   └─ ESKFCore::predict_covariance()
  ├─ zupt_check_and_update(s, a_d, w_d)  ← ESKFCore::update_zupt()
  └─ call_sensor_update(s, "accel", a_d, 3, k)  ← ★ここからMEUKFに切り替わる
        └─ handle_sensor_update_internal()
          └─ do_sensor_update_meukf(state_s, sensor_data, mex_params, ...)  ← ★MEUKF実行
            └─ MEUKFCore::step(input, output)  ← ★MEUKF更新
```

**結論**: 
- **予測ステップ**: ESKFCore（正しいESKF実装）
- **更新ステップ**: MEUKFCore（MATLABから移植したMEUKF）
- **フィルタ名**: `mex_run_eskf` だが、実際は **ESKF予測 + MEUKF更新のハイブリッド**

---

## 2. ESKFとMEUKFの役割分担

### 2.1 ESKFが担当する処理

| 処理 | ファイル | 関数 |
|-----|---------|------|
| 予測（積分） | `ESKF/src/eskf_core.cpp` | `ESKFCore::integrate_nominal()` |
| 共分散予測 | `ESKF/src/eskf_core.cpp` | `ESKFCore::predict_covariance()` |
| ZUPT更新 | `ESKF/src/eskf_core.cpp` | `ESKFCore::update_zupt()` |
| 状態注入 | `ESKF/src/eskf_core.cpp` | `ESKFCore::inject_error_state()` |
| 予測後処理 | `ESKF/src/eskf_runner.cpp` | `ESKFRunner::predict()` |

### 2.2 MEUKFが担当する処理

| 処理 | ファイル | 関数 |
|-----|---------|------|
| センサー更新（Accel） | `MEUKF/src/meukf_core.cpp` | `MEUKFCore::update_accel_meukf()` |
| センサー更新（Mag） | `MEUKF/src/meukf_core.cpp` | `MEUKFCore::update_mag_meukf()` |
| センサー更新（GPS） | `MEUKF/src/meukf_core.cpp` | `MEUKFCore::update_gps_meukf_ukf_version()` |
| センサー更新（Baro） | `MEUKF/src/meukf_core.cpp` | `MEUKFCore::update_baro_meukf_ukf_version()` |
| カルマンゲイン計算 | `MEUKF/src/meukf_update.cpp` | `MEUKFCore::step()` 内部 |
| イノベーション計算 | `MEUKF/src/meukf_update.cpp` | `MEUKFCore::step()` 内部 |

### 2.3 混在の理由

**歴史的経緯（推測）**:
1. 最初にMATLABでMEUKF実装があった
2. C++に移植する際、MEXインターフェースに `eskf` という名前を付けた
3. 予測ステップだけESKFCoreで実装したが、更新ステップはMEUKFのまま残った
4. 結果的に **ハイブリッドフィルタ** になった

**技術的理由**:
- MEUKF は UKF (Unscented Kalman Filter) ベース → 非線形センサーモデルに強い
- ESKF は誤差状態フィルタ → 四元数の扱いに適している
- 両方の利点を組み合わせた設計と思われる

---

## 3. 命名の修正提案

### 3.1 Option A: 正確な名前に変更（推奨）

**変更内容**:
- `mex_run_eskf` → `mex_run_hybrid_filter` または `mex_run_unified_filter`
- `do_sensor_update_meukf` → `do_sensor_update` （MEUKFを隠蔽）
- `ESKFState` → `FilterState` または `HybridFilterState`

**メリット**:
- 実装と名前が一致
- 混乱が解消される

**デメリット**:
- MATLAB側のコード変更が必要（`run_simulation.m` 等）
- 既存のドキュメントも変更が必要

### 3.2 Option B: 実装をESKFに統一（大規模変更）

**変更内容**:
- `do_sensor_update_meukf` を削除
- センサー更新を全て `ESKFCore::update_*` に置き換え
- `MEUKF/` ディレクトリを削除

**メリット**:
- 名前と実装が一致
- コードがシンプルになる

**デメリット**:
- 大規模な実装変更が必要
- MEUKF の UKF ベース更新の利点を失う
- 高リスク

### 3.3 Option C: コメントで明示（最小変更）

**変更内容**:
- ファイルヘッダーに「ESKF予測 + MEUKF更新のハイブリッド」と明記
- `mex_run_eskf` のヘルプテキストに実装の詳細を追加

**メリット**:
- コード変更が最小限
- リスクが低い

**デメリット**:
- 根本的な混乱は解消されない

---

## 4. 推奨アクション

### Phase 1: ドキュメント整理（即時実施）

1. **`.github/copilot-instructions.md` 更新**:
   ```markdown
   ## 【最優先ルール】フィルタ実装の構造
   
   ### ハイブリッドフィルタ構成
   - **予測ステップ**: ESKFCore（誤差状態カルマンフィルタ）
   - **更新ステップ**: MEUKFCore（UKFベース測定更新）
   - **MEXインターフェース**: `mex_run_eskf`（名前はESKFだが実際はハイブリッド）
   
   ### 呼び出しフロー
   1. `mex_run_eskf('step')` → `do_step()`
   2. `ESKFRunner::predict()` → 状態積分・共分散予測（ESKF）
  3. `call_sensor_update()` → `do_sensor_update_meukf()` → `MEUKFCore::step()`（MEUKF更新）
   ```

2. **`IMPLEMENTATION_ANALYSIS.md` 作成**（このファイル）

3. **`docs/CPP_ARCHITECTURE.md` 更新**:
   - ハイブリッド構成を明記
   - 呼び出しフローを図示

### Phase 2: コメント追加（低リスク）

各ファイルのヘッダーにハイブリッド構成を明記：

```cpp
// kalman/cpp/MEX/mex_run_eskf.cpp
/**
 * Hybrid Filter MEX Interface
 * 
 * Architecture:
 *   - Prediction: ESKF (Error-State Kalman Filter)
 *   - Update: MEUKF (Multiplicative Extended UKF)
 * 
 * Note: Despite the name "eskf", this implementation uses
 *       MEUKF for sensor updates via do_sensor_update_meukf().
 */
```

### Phase 3: 命名の部分的修正（中リスク）

**変更対象**:
1. `do_sensor_update_meukf`（MEUKFベースであることを明示）
2. 内部コメントで「ESKF予測 + MEUKF更新」を強調

**変更しない**:
- `mex_run_eskf` の名前（MATLAB側への影響が大きい）
- `ESKFState` の名前（既に広く使われている）

### Phase 4: 将来的な統一（低優先度）

長期的には以下を検討：
1. `mex_run_hybrid_filter` への名前変更
2. MATLAB側の `run_simulation.m` も対応
3. 全ドキュメントの更新

---

## 5. 実装の詳細（技術的補足）

### 5.1 ESKFCoreの実装（使われている部分）

| 関数 | 呼び出し元 | 用途 |
|-----|----------|------|
| `integrate_nominal()` | `ESKFRunner::predict()` | RK2積分で状態を更新 |
| `predict_covariance()` | `ESKFRunner::predict()` | 共分散行列の予測 |
| `compute_adaptive_Q()` | `ESKFRunner::predict()` | 適応的なプロセスノイズ |
| `update_zupt()` | `mex_run_eskf_filter_ops.hpp` | ZUPT（Zero-velocity Update） |
| `inject_error_state()` | （未使用?） | 誤差状態の注入 |

### 5.2 ESKFCoreの実装（未使用の部分）

| 関数 | 理由 |
|-----|------|
| `update_accel()` | `MEUKFCore::update_accel_meukf()` で置き換えられている |
| `update_mag()` | `MEUKFCore::update_mag_meukf()` で置き換えられている |
| `update_gps()` | `MEUKFCore::update_gps_meukf_ukf_version()` で置き換えられている |
| `update_baro()` | `MEUKFCore::update_baro_meukf_ukf_version()` で置き換えられている |

**結論**: ESKFCoreのセンサー更新関数は **未使用**。MEUKFが全て担当している。

### 5.3 MEUKFCoreの実装

`MEUKFCore::step()` の内部処理:
```cpp
void MEUKFCore::step(const MEUKFInput& input, MEUKFOutput& output) {
    State state = input.prev_state;
    
    // 1. 予測（ここはMEUKF内で実施）
    predict(state, input.sensor, input.params);
    output.pred_P = state.P;  // 予測後の共分散を保存
    
    // 2. センサー更新（UKFベース）
    if (input.sensor.update_accel) {
        update_accel_meukf(state, accel_meas, input.params, output);
    }
    if (input.sensor.update_mag) {
        update_mag_meukf(state, mag_meas, input.params, output);
    }
    if (input.sensor.update_gps) {
        update_gps_meukf_ukf_version(state, gps_meas, input.params, output);
    }
    if (input.sensor.update_baro) {
        update_baro_meukf_ukf_version(state, alt_baro, input.params, output);
    }
    if (input.sensor.update_zupt) {
        update_zupt_meukf_ukf_version(state, input.params, output);
    }
    
    // 3. 結果を出力
    output.new_state = state;
}
```

---

## 6. 削除不可なコンポーネント

| コンポーネント | 理由 |
|-------------|------|
| `MEUKF/` ディレクトリ | センサー更新で必須 |
| `MEUKFCore::step()` | `do_sensor_update_meukf` から呼ばれている |
| `do_sensor_update_meukf()` | `handle_sensor_update_internal` から呼ばれている |
| `mex_meukf_step.cpp` | レガシーだが、互換性のため保持推奨 |
| `ESKF/` ディレクトリ | 予測ステップで必須 |
| `ESKFCore` | 予測・ZUPT更新で使用中 |
| `ESKFRunner` | `do_step` から呼ばれている |

---

## 7. 削除可能なコンポーネント

| コンポーネント | 理由 |
|-------------|------|
| `mex_meukf_step_v2` ビルドターゲット | `run_simulation.m` で未使用（✅ Phase完了済み） |
| `ESKFCore::update_accel()` | MEUKFで置き換え済み |
| `ESKFCore::update_mag()` | MEUKFで置き換え済み |
| `ESKFCore::update_gps()` | MEUKFで置き換え済み |
| `ESKFCore::update_baro()` | MEUKFで置き換え済み |
| `KF/`, `EKF/`, `UKF/` ディレクトリ | 未使用（Phase 5で削除予定） |

---

## 8. まとめ

### 8.1 実装の実態

- **フィルタ名**: `mex_run_eskf`（名前はESKF）
- **実装**: ESKF予測 + MEUKF更新のハイブリッド
- **理由**: ESKFの効率的な予測 + MEUKFの頑健な更新を組み合わせた設計

### 8.2 命名の問題

| 項目 | 名前 | 実態 | 修正の必要性 |
|-----|------|------|------------|
| MEXファイル | `mex_run_eskf` | ハイブリッド | 🟡 低（ドキュメントで対応） |
| 更新関数 | `do_sensor_update_meukf` | MEUKF | ✅ 正しい |
| 予測関数 | `ESKFRunner::predict` | ESKF | ✅ 正しい |
| 状態構造体 | `ESKFState` | 共通 | 🟡 中（`FilterState`が正確） |

### 8.3 推奨アクション（優先順位）

1. **即時**: ドキュメント更新（`.github/copilot-instructions.md` 等）
2. **短期**: コメント追加（`mex_run_eskf.cpp` ヘッダー等）
3. **中期**: 未使用の `ESKFCore::update_*` 関数を削除（オプション）
4. **長期**: `mex_run_hybrid_filter` への名前変更（検討のみ）

---

**次のステップ**: このドキュメントを基に、ドキュメント更新と命名の部分修正を実施
