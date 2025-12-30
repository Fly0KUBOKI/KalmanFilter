# Phase 4 修正版統合計画：センサー更新層の統合

**作成日**: 2025-12-30  
**状態**: Phase 3完了（予測ステップ統合成功）、Phase 4失敗（精度低下により元の実装に復帰）

---

## 背景

### 以前の試み（失敗）

- **目標**: センサー更新を直接C++実装（ESKF）に置き換え
- **結果**: 精度が大幅に低下（Position RMSE: 0.95m → 12.58m）
- **原因**: MEUKFとESKFの違いを理解せずに置き換え

### 現在の状態

- **予測ステップ**: ✅ C++直接実装（`call_predict`関数）
- **初期化**: ✅ C++直接実装（`do_init`関数）
- **センサー更新**: ⚠️ `mexCallMATLAB`経由（元の実装）
  - `mex_sensor_preprocessor` → 前処理
  - `mex_eskf_do_update` → 更新処理（内部で`mex_meukf_step_v2`を呼び出し）
  - `mex_eskf_update_postprocess` → 後処理
  - `mex_sensor_filter` → 発散チェック

---

## 修正版アプローチ：段階的な統合

### 原則

1. **アルゴリズムを変更しない**
   - MEUKFを維持（`mex_meukf_step_v2`を`mexCallMATLAB`経由で呼び出す）
   - ESKFへの置き換えは行わない

2. **段階的に統合**
   - 前処理 → 後処理 → 発散チェックの順に統合
   - 各段階で精度テストを実施

3. **リスクを最小化**
   - 各段階で十分なテストを実施
   - 問題が発生した場合は即座にロールバック

---

## Phase 4A: 前処理の統合（1日）

### 目標

`mex_sensor_preprocessor`をC++実装に置き換える

### 作業内容

1. **前処理関数の確認**
   - `mex_sensor_preprocessor.cpp`の実装を確認
   - 各センサー（accel, gyro, mag, baro）の前処理ロジックを理解

2. **C++実装への移行**
   - `Inc/Common/Sensor/sensor_preprocessor.hpp`に関数を定義
   - `Src/Common/Sensor/sensor_preprocessor.cpp`に実装（既に存在する可能性）

3. **mex_run_eskf.cppの修正**
   - `mexCallMATLAB`で`mex_sensor_preprocessor`を呼び出す部分を削除
   - C++実装の`preprocess_*`関数を直接呼び出す

4. **テスト**
   - ビルドテスト
   - 精度テスト（`run_simulation(42, true)`）
   - 前処理結果の比較（MATLAB実装 vs C++実装）

### チェックリスト

- [ ] `mex_sensor_preprocessor.cpp`の実装を確認
- [ ] `sensor_preprocessor.hpp`に関数定義
- [ ] `sensor_preprocessor.cpp`に実装（必要に応じて）
- [ ] `mex_run_eskf.cpp`を修正
- [ ] ビルドテスト成功
- [ ] 精度テスト成功（Position RMSE < 1.0m）
- [ ] 前処理結果の比較（MATLAB vs C++）

### 期待される結果

- 前処理がC++実装に置き換わる
- 精度が維持される（Position RMSE < 1.0m）
- `mexCallMATLAB`の呼び出しが1つ減る

---

## Phase 4B: 後処理の統合（1日）

### 目標

`mex_eskf_update_postprocess`をC++実装に置き換える

### 作業内容

1. **後処理関数の確認**
   - `mex_eskf_update_postprocess.cpp`の実装を確認
   - `update_state_from_dx`関数の実装を確認

2. **C++実装への移行**
   - `Inc/ESKF/eskf_postprocess.hpp`に関数を定義（既に存在する可能性）
   - `Src/ESKF/eskf_postprocess.cpp`に実装（既に存在する可能性）

3. **mex_run_eskf.cppの修正**
   - `mexCallMATLAB`で`mex_eskf_update_postprocess`を呼び出す部分を削除
   - C++実装の`update_state_from_dx`関数を直接呼び出す

4. **テスト**
   - ビルドテスト
   - 精度テスト（`run_simulation(42, true)`）
   - 後処理結果の比較（MATLAB実装 vs C++実装）

### チェックリスト

- [ ] `mex_eskf_update_postprocess.cpp`の実装を確認
- [ ] `eskf_postprocess.hpp`に関数定義（必要に応じて）
- [ ] `eskf_postprocess.cpp`に実装（必要に応じて）
- [ ] `mex_run_eskf.cpp`を修正
- [ ] ビルドテスト成功
- [ ] 精度テスト成功（Position RMSE < 1.0m）
- [ ] 後処理結果の比較（MATLAB vs C++）

### 期待される結果

- 後処理がC++実装に置き換わる
- 精度が維持される（Position RMSE < 1.0m）
- `mexCallMATLAB`の呼び出しが1つ減る

---

## Phase 4C: 発散チェックの統合（0.5日）

### 目標

`mex_sensor_filter`の発散チェックをC++実装に統合

### 作業内容

1. **発散チェック関数の確認**
   - `mex_sensor_filter.cpp`の実装を確認
   - 発散チェックのロジックを理解

2. **C++実装への移行**
   - `Inc/Common/Sensor/sensor_filter.hpp`に関数を定義（既に存在する可能性）
   - `Src/Common/Sensor/sensor_filter.cpp`に実装（既に存在する可能性）

3. **mex_run_eskf.cppの修正**
   - `mexCallMATLAB`で`mex_sensor_filter`を呼び出す部分を削除
   - C++実装の発散チェック関数を直接呼び出す

4. **テスト**
   - ビルドテスト
   - 精度テスト（`run_simulation(42, true)`）
   - 発散チェックの動作確認

### チェックリスト

- [ ] `mex_sensor_filter.cpp`の実装を確認
- [ ] `sensor_filter.hpp`に関数定義（必要に応じて）
- [ ] `sensor_filter.cpp`に実装（必要に応じて）
- [ ] `mex_run_eskf.cpp`を修正
- [ ] ビルドテスト成功
- [ ] 精度テスト成功（Position RMSE < 1.0m）
- [ ] 発散チェックの動作確認

### 期待される結果

- 発散チェックがC++実装に置き換わる
- 精度が維持される（Position RMSE < 1.0m）
- `mexCallMATLAB`の呼び出しが1つ減る

---

## Phase 4D: 更新処理の統合（将来の検討事項）

### 目標

`mex_meukf_step_v2`を直接C++実装に置き換える

### 注意事項

- **MEUKFアルゴリズムを維持する必要がある**
- ESKFへの置き換えは行わない
- `MEUKFCore::step()`を直接呼び出す

### 作業内容（将来）

1. **MEUKF実装の詳細分析**
   - `mex_meukf_step_v2`の実装を詳細に分析
   - MEUKFの各ステップを理解

2. **C++実装への移行**
   - `MEUKFCore::step()`を直接呼び出す
   - 型変換をC++実装に統合

3. **テスト**
   - ビルドテスト
   - 精度テスト（`run_simulation(42, true)`）
   - 既存実装との完全一致確認

### 現時点での判断

- **Phase 4Dは実施しない**
- 理由: リスクが高く、現在の実装（`mexCallMATLAB`経由）で十分に動作している
- 将来的に、パフォーマンス最適化が必要になった場合に検討

---

## 統合後の状態

### 完了後の実装状態

- **予測ステップ**: ✅ C++直接実装
- **初期化**: ✅ C++直接実装
- **前処理**: ✅ C++直接実装（Phase 4A）
- **後処理**: ✅ C++直接実装（Phase 4B）
- **発散チェック**: ✅ C++直接実装（Phase 4C）
- **更新処理**: ⚠️ `mexCallMATLAB`経由（`mex_meukf_step_v2`）

### `mexCallMATLAB`の呼び出し

- **統合前**: 4回（前処理、更新、後処理、発散チェック）
- **統合後**: 1回（更新処理のみ）

### メリット

- `mexCallMATLAB`のオーバーヘッドが75%削減
- 前処理・後処理・発散チェックがC++実装に統合
- 精度を維持しながら統合可能

---

## リスク管理

### リスク #1: 前処理の実装の違い

**影響度**: 🟡 中程度  
**発生確率**: 🟡 中程度  
**対策**:
- 前処理結果を比較（MATLAB vs C++）
- 問題が発生した場合は即座にロールバック

### リスク #2: 後処理の実装の違い

**影響度**: 🟡 中程度  
**発生確率**: 🟡 中程度  
**対策**:
- 後処理結果を比較（MATLAB vs C++）
- 問題が発生した場合は即座にロールバック

### リスク #3: 発散チェックの動作不良

**影響度**: 🔴 致命的  
**発生確率**: 🟢 低い  
**対策**:
- 発散チェックの動作を詳細に確認
- 問題が発生した場合は即座にロールバック

---

## テスト戦略

### 各フェーズでのテスト

1. **ビルドテスト**
   ```matlab
   cd kalman/cpp/build
   build_mex({'mex_run_eskf'})
   ```

2. **基本動作テスト**
   ```matlab
   clear mex
   run_simulation(42, true)
   ```

3. **精度テスト**
   ```matlab
   run_batch_10sets()
   ```

4. **比較テスト**
   - MATLAB実装 vs C++実装の結果を比較
   - 前処理・後処理の結果を比較

### 成功基準

- **ビルド**: エラーなし
- **基本動作**: エラー・警告なし
- **精度**: Position RMSE < 1.0m（統合前と同等）
- **比較**: MATLAB実装とC++実装の結果が一致（許容誤差内）

---

## スケジュール

| フェーズ | 期間 | 状態 |
|---------|------|------|
| Phase 4A: 前処理の統合 | 1日 | ⏸️ 未開始 |
| Phase 4B: 後処理の統合 | 1日 | ⏸️ 未開始 |
| Phase 4C: 発散チェックの統合 | 0.5日 | ⏸️ 未開始 |
| Phase 4D: 更新処理の統合 | - | ❌ 実施しない |

**合計**: 2.5日（Phase 4Dを除く）

---

## 参考資料

- `INTEGRATED_ANALYSIS_AND_PLAN.md`: 統合分析と計画書
- `MEUKF_VS_ESKF_ANALYSIS.md`: MEUKFとESKFの比較分析
- `05_CURRENT_STATUS_AND_FAILURE_ANALYSIS.md`: 現在の状況と失敗分析

---

**最終更新**: 2025-12-30

