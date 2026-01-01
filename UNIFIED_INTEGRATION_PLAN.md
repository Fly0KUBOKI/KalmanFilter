# 統合完了報告書 & Step_v2 統合計画

**最終更新**: 2026年1月1日  
**著者**: Integration Agent

---

## 📊 現在の成果（Phase 1-2: 完了）

| 指標 | 失敗直後 | 現在 | 改善度 |
|------|----------|------|--------|
| **成功率** | 0/10 (0%) | 10/10 (100%) | ✅ **復帰** |
| **Position RMSE** | 12.5-71m | 0.8-0.9m | ✅ **50倍以上** |
| **Attitude RMSE** | 45-105 deg | 0.25-0.30 deg | ✅ **100倍以上** |
| **Gyro bias** | [0,0,0] 固定 | [-0.23, 0.03, 0.01] deg/s | ✅ **更新可能** |

### 完了した統合

✅ **sensor_filter 統合完了** (`mex_sensor_filter` を `mex_run_eskf` に統合)  
✅ **共分散対称化** + **イノベーション追跡** 機能追加  
✅ **予測ステップ** (C++ 直接実装)  
✅ **初期化** (C++ 直接実装)  

### 未完了（予定中）

⏳ **mex_meukf_step_v2 の統合** (現在は `mexCallMATLAB` 経由)

---

## 🔍 失敗履歴と教訓

### 過去の失敗（commit 477a5d4, e88e940 周辺）

#### 失敗1: ESKFへの無計画な置換
- **試行**: `mex_meukf_step_v2` を `ESKFCore::update_*` に置換
- **結果**: Position RMSE 12.5m → アルゴリズム不一致（MEUKF vs ESKF）
- **原因**: MEUKFとESKFは異なるアルゴリズム。バイアス更新等に差あり。

#### 失敗2: 広範囲な一気統合
- **試行**: 予測・更新・前処理・後処理を同時に置換
- **結果**: ジャイロバイアス更新失敗、共分散発散
- **原因**: 回帰テストなしで複数モジュール変更 → 原因特定困難

#### 失敗3: イノベーション値の損失（CSV ゼロ問題）
- **観察**: `MEUKF_DEBUG` は非ゼロだが、`DEBUG_INNOV` はゼロ
- **原因**: `mexCallMATLAB` 戻り値の出力インデックス誤り（plhs[1] vs plhs[2]）
- **修正済み**: インデックス修正後、イノベーション値が伝搬開始（但し最終的に CSV ゼロのままだが、推定は正常）

### 教訓

1. **MEUKFとESKFは異なるアルゴリズム** → 単純置換不可
2. **統合は小さな単位で** → 予測/更新/フィルタを分離
3. **各ステップでテスト** → 回帰テスト（batch_10sets）必須
4. **型・メモリレイアウト注意** → float/double、列/行優先
5. **mexCallMATLAB の型変換コスト** → 段階的に減らすべき

---

## 📋 Phase 3: mex_meukf_step_v2 統合計画（3段階）

### 設計方針

**目標**: `mexCallMATLAB("mex_meukf_step_v2")` を `MEUKFCore::step()` の直接呼び出しに置換

**前提条件**:
- sensor_filter 既に統合済み（発散チェック機能あり）
- `last_y` / `last_S_inv` / `debug_info` の MEX 出力は存在
- Phase 1-2 の全修正（共分散対称化等）は保持

**リスク対策**:
- 各フェーズは独立した git ブランチまたは cherry-pick 可能な単一コミット
- 回帰テスト（`run_batch_10sets()` で 10/10 PASS）を各フェーズで確認
- ロールバック用に前のコミットを記録

---

### Phase 3-1: 変換ヘルパの実装と動作確認（低リスク）

**目標**: MATLAB 構造体 → C++ 構造体の変換を安全に実装

#### 実施内容

1. **`mex_run_eskf_sensor_updates.hpp` 内に変換関数を追加**
   ```cpp
   void matlab_to_meukf_input(
       mxArray* state_s, mxArray* sensor_data, mxArray* mex_params,
       meukf::MEUKFInput& input)
   {
       // state_s.p, .v, .q, .ba, .bg, .P → input.prev_state
       // sensor_data.* → input.sensor
       // mex_params.* → input.params
       // 型変換: double → float (適切にスケール確認)
   }
   ```

2. **既存の `matlab_to_state()` / `meukf_type_conv.hpp` を流用**
   - `mex_meukf_step.cpp` にある `matlab_to_state()` をコピー
   - または共通ヘッダに統合

3. **テスト（テストメイン）**
   ```matlab
   % kalman/cpp/build/test_meukf_integration.m (拡張)
   % 変換後の MEUKFInput が元の構造体と一致するか検証
   ```

4. **回帰テスト**
   ```matlab
   run_batch_10sets()  % 10/10 PASS を確認
   ```

**リスク**: 低（変換のみで呼び出し変更なし）  
**推定工数**: 1-2 時間

---

### Phase 3-2: `mexCallMATLAB` を `MEUKFCore::step()` に置換（中リスク）

**目標**: 実際に直接呼び出しに置換し、出力を正しく処理

#### 実施内容

1. **`mex_run_eskf_sensor_updates.hpp` 内の mexCallMATLAB を置換**
   ```cpp
   // 現在（Phase 1-2）
   if (mexCallMATLAB(3, plhs_m, 3, prhs_m, "mex_meukf_step_v2") == 0) {
       mxArray* new_state = plhs_m[0];
       mxArray* dbg_info = plhs_m[1];
       mxArray* dbg_out = plhs_m[2];
       // ... 処理 ...
   }

   // Phase 3-2（置換後）
   meukf::MEUKFInput input;
   matlab_to_meukf_input(state_s, sensor_data, mex_params, input);

   meukf::MEUKFOutput output;
   meukf::MEUKFCore::step(input, output);

   // output.new_state, output.debug_info, output.last_y, etc. を直接使用
   s->last_innov_norm = ... // output から計算
   ```

2. **出力の直接処理（mxArray へのコピーは不要に）**
   ```cpp
   // 新しい流れ: C++ struct → ESKFState に直接格納
   s->last_innov_norm = compute_innov_norm(output.last_y, output.last_y_len);
   s->last_maha_dist = compute_maha_dist(output.last_y, output.last_S_inv, ...);
   ```

3. **回帰テスト**
   ```matlab
   run_batch_10sets()  % 10/10 PASS を確認
   ```

4. **追加検証（オプション）**
   - `MEUKF_DEBUG` と CSV イノベーションノルムが一致するか確認
   - メモリ/ 実行時間の改善測定

**リスク**: 中（アルゴリズム動作確認が必要）  
**推定工数**: 2-3 時間

---

### Phase 3-3: メモリレイアウト / 型混在の最適化（低リスク）

**目標**: float/double 変換の削減、不要な mxArray コピーを完全排除

#### 実施内容

1. **double → float 変換の必要部分確認**
   - `meukf_core.cpp` は float ベース
   - MATLAB / MEX インターフェースは double ベース
   - 変換が必須でないなら削減、必須なら明示的にドキュメント化

2. **列/行優先の統一確認**
   - MATLAB: 列優先（column-major）
   - C++: 行優先（row-major）
   - 既に `mex_meukf_step.cpp` で適切に処理されているか確認

3. **テスト**
   ```matlab
   run_batch_10sets()  % 数値誤差がないか確認
   ```

**リスク**: 低（検証のみ）  
**推定工数**: 1 時間

---

## 🎬 実行スケジュール（推奨）

| フェーズ | 内容 | 時間 | 優先度 | ロールバック |
|---------|------|------|--------|-------------|
| **Phase 3-1** | 変換ヘルパ実装 | 1-2h | 🔴 高 | git revert |
| **Phase 3-2** | mexCallMATLAB 置換 | 2-3h | 🔴 高 | git revert |
| **Phase 3-3** | 型最適化 | 1h | 🟡 低 | 不要（ポーリッシュ） |

### 実行チェックリスト

**Phase 3-1 前**
- [ ] 現在の `run_batch_10sets()` が 10/10 PASS
- [ ] `PLAN.md` / `05_CURRENT_STATUS_AND_FAILURE_ANALYSIS.md` を読了
- [ ] commit 0161de2 / 281ff08 の差分を確認

**Phase 3-1 実施**
- [ ] `matlab_to_meukf_input()` を `mex_run_eskf_sensor_updates.hpp` に追加
- [ ] テストプログラムで変換精度確認
- [ ] `build_mex({'mex_meukf_step_v2','mex_run_eskf'})` で再ビルド
- [ ] `clear mex` → `run_batch_10sets()` で 10/10 PASS 確認

**Phase 3-1 成功後**
- [ ] `git commit -m "phase3-1: add MEUKF conversion helpers"`

**Phase 3-2 実施**
- [ ] `mexCallMATLAB` 部分をコメントアウト（回帰用）
- [ ] `MEUKFCore::step()` 直接呼び出しコード追加
- [ ] 出力処理（`last_innov_norm` etc）を修正
- [ ] 再ビルド + `run_batch_10sets()` 確認

**Phase 3-2 失敗時のロールバック**
```matlab
git revert <phase3-2-commit>  % または git reset --hard <phase3-1-commit>
cd kalman/cpp/build
build_mex({'mex_meukf_step_v2','mex_run_eskf'})
clear mex
cd ../..
run_batch_10sets()
```

---

## 📁 クリーンアップ対象（不要ドキュメント）

以下を削除または `docs/archive/` に移動することを推奨：

- `MEX_INTEGRATION_PRIORITY.md` (古い優先度表)
- `MEX_INTEGRATION_GUIDE.md` (古い手引き)
- `MEX_INTEGRATION_CHECKLIST.md` (古いチェックリスト)
- `kalman/cpp/build/01_COMMIT_CHANGES_SUMMARY.md`
- `kalman/cpp/build/02_FAILURE_ROOT_CAUSE_ANALYSIS.md`
- `kalman/cpp/build/03_PREVENTION_STRATEGIES.md`
- `kalman/cpp/build/04_INTEGRATION_REFACTORING_PLAN.md`
- `kalman/cpp/markdown/MEX_*.md` (大量の分析ドキュメント)

**保持すべき**：
- `UNIFIED_INTEGRATION_PLAN.md` (このファイル)
- `PLAN.md` (進捗概要)
- `05_CURRENT_STATUS_AND_FAILURE_ANALYSIS.md` (失敗教訓)
- `.github/copilot-instructions.md` (開発ガイド)

---

## 🔗 参照リンク

- **Phase 1-2 成功時の commit**: 281ff08
- **Phase 2 のステップ**: 0161de2 で `sensor_filter` 統合が確認できる
- **テストスクリプト**: `kalman/cpp/build/test_meukf_integration.m`
- **ビルドスクリプト**: `kalman/cpp/build/build_mex.m`

---

## 📝 次のステップ

1. **このドキュメントレビュー** → フィードバック
2. **Phase 3-1 実装開始** → 変換ヘルパ追加
3. **段階テスト** → 各フェーズで 10/10 PASS 確認
4. **統合完了** → すべての MEX が C++ 直接実装で動作

