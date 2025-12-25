# ESKF完全MEX化計画

**最終更新**: 2025-01-XX  
**目標**: MATLABからはMEXの一番大きな関数の呼び出しのみで実行

---

## 📊 現状サマリー（2025-12-26更新）

### MEXファイルビルド状況
- ✅ Phase 1: `mex_matlab_helpers` - ビルド済み・**統合済み**
- ✅ Phase 2: `mex_sensor_filter` (拡張) - ビルド済み・**統合済み**
- ✅ Phase 3: `mex_sensor_preprocessor` - ビルド済み・**統合済み**
- ✅ Phase 4: `mex_adaptive_predict` - ビルド済み・**統合済み**
- ✅ Phase 5: `mex_filter_management` - ビルド済み・**統合済み**

### ESKF.m統合状況
- ✅ **Phase 1**: `get_field_impl()`, `has_field_impl()` → `mex_matlab_helpers`使用中
- ✅ **Phase 2**: `noiseEstimator`, `divergence_guard` → `mex_sensor_filter`使用中
- ✅ **Phase 3**: `sensor_updates()` → `mex_sensor_preprocessor`使用中
- ✅ **Phase 4**: `predict()` → `mex_adaptive_predict`使用中
- ✅ **Phase 5**: `reset()`, `zupt()` → `mex_filter_management`使用中

### 既存MEX関数（使用中）
- `mex_meukf_step_v2` - センサー更新コア（`do_cpp_update()`内で使用）
- `mex_quaternion_lib` - クォータニオン演算（初期化・予測で使用）
- `mex_sensor_filter` - センサーフィルタ・発散チェック（完全統合）
- `mex_eskf_init` / `mex_eskf_free` - 状態管理（初期化・削除で使用）

### テスト結果（最新: 2025-12-26）
- ✅ **バッチテスト**: 10/10 PASS (100.0%)
- ✅ **Position RMSE**: Mean=0.7831m (X=0.1702m, Y=0.1548m, Z=0.7484m)
- ✅ **Velocity RMSE**: Mean=0.5773 m/s
- ✅ **Attitude RMSE**: Roll=0.2613°, Pitch=0.2815°, Yaw=0.6066°

---

## 🎯 最終目標

**MATLABからはMEXの一番大きな関数の呼び出しのみで実行**

```
【現在の状態（2025-12-26）】
run_simulation.m
  └─ for k=1:N
      └─ eskf.update_filter(obs, k)
          ├─ predict() [mex_adaptive_predict使用 ✅]
          ├─ sensor_updates() [mex_sensor_preprocessor使用 ✅]
          │   └─ do_cpp_update() [mex_meukf_step_v2使用 ✅]
          └─ reset() [mex_filter_management使用 ✅]

【目標（Phase 10）】
run_simulation.m
  └─ result = mex_eskf_run(obs, config, initial_state)
      └─ [C++内で全ループ処理: predict + sensor_updates + reset]
```

---

## 📋 統合フェーズの進捗状況

### ✅ Phase 1-5: 完了済み（2025-12-26時点）

- ✅ **Phase 1**: `get_field_impl()`, `has_field_impl()` → `mex_matlab_helpers`統合済み
- ✅ **Phase 2**: `noiseEstimator`, `divergence_guard` → `mex_sensor_filter`統合済み
- ✅ **Phase 3**: `sensor_updates()` → `mex_sensor_preprocessor`統合済み
- ✅ **Phase 4**: `predict()` → `mex_adaptive_predict`統合済み
- ✅ **Phase 5**: `reset()`, `zupt()` → `mex_filter_management`統合済み

**検証結果**: `run_batch_10sets()` で 10/10 PASS、RMSEも良好

---

### Phase 10: 統合MEX関数の作成（最終目標・未実装）

**優先度**: 最高（最終目標）  
**推定工数**: 3-5日  
**対象**: 全ループ処理をC++に移行

**現状**: 
- 各MEX関数は統合済みだが、MATLAB側でループ処理（`update_filter()`）が残っている
- 各ステップでMEX関数を個別に呼び出している

**実装内容**:
```cpp
// mex_eskf_run.cpp
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: obs (構造体配列), config, initial_state
    // 出力: result (構造体: position, velocity, quaternion, etc.)
    
    // C++内で全ループ処理
    for (int k = 0; k < n_samples; ++k) {
        // predict (mex_adaptive_predict相当)
        // sensor updates (mex_sensor_preprocessor + mex_meukf_step_v2相当)
        // reset check (mex_filter_management相当)
    }
}
```

---

### Phase 10: 統合MEX関数の作成（最終目標）

**優先度**: 最高（最終目標）  
**推定工数**: 3-5日  
**対象**: 全ループ処理をC++に移行

**実装内容**:
```cpp
// mex_eskf_run.cpp
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: obs (構造体配列), config, initial_state
    // 出力: result (構造体: position, velocity, quaternion, etc.)
    
    // C++内で全ループ処理
    for (int k = 0; k < n_samples; ++k) {
        // predict
        // sensor updates
        // reset check
    }
}
```

**MATLAB側**:
```matlab
function result = run_simulation(seed, use_mex)
    % データ生成
    obs = generate_observations(seed);
    
    if use_mex && exist('mex_eskf_run', 'file') == 3
        % 統合MEX関数を呼び出すだけ
        result = mex_eskf_run(obs, config, initial_state);
    else
        % 既存のMATLAB実装（フォールバック）
        eskf = ESKF(obs, static_time, dt);
        for k = 1:length(obs.accel_x)
            eskf.update_filter(obs, k);
        end
    end
end
```

---

## 🔧 技術的制約

### 型システム
- ✅ **float統一**: C++内部は`float`のみ使用
- ✅ **MATLAB API**: 入出力は`double`（互換性維持）
- ✅ **型変換**: MEXラッパーで`double`↔`float`変換

### 状態ベクトル
- ✅ **固定サイズ**: `[p(3), v(3), q(4), ba(3), bg(3)]` = 15要素
- ✅ **クォータニオン**: `[w, x, y, z]` スカラー先頭
- ✅ **共分散**: 出力前に `P = (P + P')/2` で対称化

### 必須チェック
- ✅ **NaN/Inf検出**: 各MEX関数で検証
- ✅ **クォータニオン正規化**: 演算後必ず正規化
- ✅ **共分散対称化**: 出力前必須

---

## 📅 実装スケジュール

### ✅ 完了済み（2025-12-26）
- ✅ Phase 1-5: 全て統合完了・検証済み

### 🔄 残タスク: Phase 10（統合MEX関数）

**Week 1-2: Phase 10実装**
- **Day 1-2**: `mex_eskf_run`の設計・基本実装
- **Day 3-4**: ループ処理のC++化（predict + sensor_updates + reset）
- **Day 5**: 統合テスト・パフォーマンス測定

**総推定工数**: 5日

---

## ✅ 検証チェックリスト

### ✅ Phase 1-5: 完了済み
- [x] MEX関数ビルド成功
- [x] `run_simulation(42, true)` 正常終了
- [x] `run_batch_10sets()` 10/10 PASS（2025-12-26確認）
- [x] NaN/Inf発生なし
- [x] 数値精度確認: Position RMSE Mean=0.7831m, Velocity RMSE Mean=0.5773 m/s

### Phase 10（統合MEX関数）: 未実装
- [ ] `mex_eskf_run`の設計・実装
- [ ] 全処理がC++内で完結
- [ ] MATLAB側は関数呼び出しのみ
- [ ] `run_batch_10sets()` 10/10 PASS（統合MEX版）
- [ ] パフォーマンス測定: 現在比での高速化率

---

## 🚨 リスク管理

| リスク | 対策 |
|--------|------|
| float精度不足 | Phase 7で`compare_mex_matlab_detailed()`実施 |
| 共分散発散 | 対称化ルール遵守・定期的な`run_batch_10sets()` |
| クォータニオン誤り | `[w,x,y,z]`固定、テンプレートで強制 |
| 数値差異累積 | 各フェーズで回帰テスト |

---

## 📚 関連ドキュメント

- [ESKF.m](../ESKF.m) - 現在の実装
- [run_simulation.m](../../run_simulation.m) - メインスクリプト
- [run_batch_10sets.m](../../run_batch_10sets.m) - 回帰テスト

---

## 🎯 次のアクション

### ✅ 完了済み
- Phase 1-5の統合は完了し、`run_batch_10sets()`で10/10 PASSを確認

### 🔄 次のステップ: Phase 10（統合MEX関数）

1. **`mex_eskf_run`の設計**
   - 入力: `obs` (構造体配列), `config`, `initial_state`
   - 出力: `result` (構造体: position, velocity, quaternion, etc.)
   - C++内で全ループ処理を実装

2. **実装内容**
   - `update_filter()`相当の処理をC++に移植
   - 既存のMEX関数（`mex_adaptive_predict`, `mex_sensor_preprocessor`, `mex_meukf_step_v2`, `mex_filter_management`）を内部で呼び出し

3. **検証**
   - `run_batch_10sets()`で10/10 PASS確認
   - パフォーマンス測定（現在比での高速化率）

**現状**: Phase 1-5は統合完了・検証済み  
**次ステップ**: Phase 10（統合MEX関数）の実装開始

