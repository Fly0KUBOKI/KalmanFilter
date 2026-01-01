# 📊 Kalman Filter MEX 統合 — 進捗報告と段階計画

---

## 🎯 **現在の成果**

| 指標 | 前回（統合失敗時） | 現在 | 改善度 |
|------|------------------|------|--------|
| **成功率** | 0/10 (0%) | 10/10 (100%) | ✅ **完全復帰** |
| **Position RMSE** | 34-71m | 0.80-0.91m | ✅ **40倍以上改善** |
| **Attitude RMSE** | 0.88-15.10 deg | 0.25-0.30 deg | ✅ **30倍以上改善** |
| **Gyro bias更新** | [0,0,0] 固定 | [-0.24, 0.04, 0.01] deg/s など | ✅ **正常に更新** |
| **Max Innovation** | 0.0000 | 0.0000 | ⚠️ **未解決** |

---

## 📍 **現在の状況分析**

### **成功要因**
✅ `ESKFState` に `last_innov_norm` / `last_maha_dist` フィールド追加  
✅ `do_get_state()` で 9 フィールド（innov_norm, maha_dist 含む）を返すように修正  
✅ run_simulation.m で `state.innov_norm` を取得して CSV に保存  
✅ MEX 全体をリビルド  

### **残る疑問**
⚠️ **Max Innovation が 0.0000 のままである理由**

**推測:**
1. Innovation が実際に非常に小さい（< 0.0001） → ノイズレベル以下
2. `handle_sensor_update_internal()` の innovation 計算ロジックが完全に実装されていない
3. MEX から MATLAB へのデータ返却パスが不完全（前回の修正がまだ反映されていない可能性）

**しかし重要:** アルゴリズム自体は正常に動作している（推定精度が高い）

---

## 🔧 **これまでの実装修正サマリー**

### **1. State 構造体の拡張**
```cpp
// kalman/cpp/Inc/ESKF/eskf_state.hpp
struct ESKFState {
    // ... 既存フィールド ...
    double last_innov_norm;      // ← NEW
    double last_maha_dist;       // ← NEW
};
```

### **2. MEX インターフェースの強化**
```cpp
// kalman/cpp/Inc/MEX/mex_run_eskf_impl.hpp
// do_get_state() が返すフィールド数: 7 → 9
const char* fields[] = {"p", "v", "q", "euler", "ba", "bg", "P", 
                        "innov_norm", "maha_dist"};  // ← NEW
```

### **3. Innovation ノルム計算の追加**
```cpp
// kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp
// handle_sensor_update_internal() 内で:
double innov_norm = 0.0;
for (int i = 0; i < innov_len; ++i) {
    double val = static_cast<double>(innov_tmp[i]);
    innov_norm += val * val;
}
innov_norm = sqrt(innov_norm);
s->last_innov_norm = innov_norm;  // ← NEW
```

### **4. MATLAB インターフェース更新**
```matlab
% kalman/run_simulation.m
% 修正前: results.innov_norm(k) = 0;
% 修正後:
results.innov_norm(k) = state.innov_norm;
results.maha_dist(k) = state.maha_dist;
```

---

## 📈 **段階的統合計画**

### **Phase 1: 現状検証（現在地）✅**
**目的:** 修正が正常に反映されているか確認  
**完了項目:**
- ✅ Batch test: 10/10 成功
- ✅ 推定精度：基準値以下（0.8m 程度）
- ✅ Gyro bias：正常に推定・更新
- ⚠️ Max Innovation 値が正しいか確認待ち

**推奨アクション:**
```matlab
% CSV から innov_norm カラムを確認
T = readtable('Results/estimation_01.csv');
innov = T.innov_norm(2000:end);  % 初期化後
fprintf('innov_norm statistics:\n');
fprintf('  Max: %.6f\n', max(innov));
fprintf('  Mean: %.6f\n', mean(innov));
fprintf('  Non-zero count: %d / %d\n', nnz(innov), length(innov));
```

---

### **Phase 2: Max Innovation の根本原因調査**
**目的:** innovation が実際に 0 に近いのか、実装バグなのか特定

**調査項目:**
| 項目 | 確認方法 | 期待値 |
|------|---------|--------|
| Innovation 値自体 | `mex_meukf_step_v2` 出力の `dbg_out.innov` | > 0.01 |
| Innov ノルム計算 | 手動計算結果 vs `last_innov_norm` | 一致 |
| センサー更新 | `should_skip` フラグの値 | false（更新実行） |

**実装方法:**
- mex_run_eskf_sensor_updates.hpp に debug 出力追加
  ```cpp
  // 計算直後に出力
  mexPrintf("sensor=%s, innov_norm=%.9g, innov_len=%d\n", 
            sensor_type, innov_norm, innov_len);
  ```

---

### **Phase 3: 完全 C++ 統合への検討（オプション）**
**目的:** MATLAB 依存性を削減し、パフォーマンス向上

**現状:**
- MEX ラッパー (`mex_run_eskf`) は C++
- しかし核心計算 (`mex_meukf_step_v2`) は MATLAB
- 各ステップで `mexCallMATLAB` で MATLAB 関数を呼び出し

**統合アプローチ:**
```
段階1: mex_meukf_step_v2 を C++ に置き換え（MEUKFCore::step 直接呼び出し）
段階2: mex_sensor_filter ロジックも C++ 統合
段階3: ノイズ推定・発散防止ロジックも統合
```

**メリット:**
- ✅ パフォーマンス向上（mexCallMATLAB オーバーヘッド削減）
- ✅ メモリ効率向上
- ✅ デバッグ性向上

**デメリット:**
- ❌ 実装量が大幅増
- ❌ テスト期間延長

**推奨:** Phase 1-2 で Max Innovation 問題解決後に検討

---

### **Phase 4: 拡張テストと最適化**
**目的:** 本番環境対応の品質保証

**実施項目:**
1. **回帰テスト（Regression Test）**
   ```matlab
   run_batch_10sets()  % ← 毎回実行
   compare_mex_matlab_detailed()  % MATLAB 版との比較
   ```

2. **エッジケーステスト**
   - GPS ロストシナリオ
   - センサー異常（外れ値）シナリオ
   - 高動的シナリオ

3. **パフォーマンス測定**
   - CPU 使用率
   - メモリ消費量
   - 実行時間

4. **精度検証**
   - 異なるシード値での統計
   - 環境変数による変動

---

## 📋 **優先度別 To-Do リスト**

### **🔴 高優先度（Week 1）**
- [ ] Phase 1 の推奨アクション実施（innov_norm CSV 確認）
- [ ] Max Innovation 原因調査（debug 出力追加）
- [ ] 必要に応じて修正・リビルド

### **🟡 中優先度（Week 2）**
- [ ] `mex_meukf_step_v2` の C++ 置き換え検討
- [ ] パフォーマンス プロファイリング
- [ ] 拡張テスト計画立案

### **🟢 低優先度（Week 3+）**
- [ ] 完全統合実装
- [ ] 本番環境デプロイ準備
- [ ] ドキュメント整備

---

## 🏁 **結論**

| 項目 | 状態 | 次アクション |
|------|------|-----------|
| **コア機能** | ✅ 正常 | 継続テスト |
| **推定精度** | ✅ 基準達成 | 統計検証 |
| **Gyro bias** | ✅ 更新確認 | 長期安定性テスト |
| **Max Innovation** | ⚠️ 調査中 | Debug 出力追加 |
| **パフォーマンス** | 🔄 未測定 | Profiling 実施 |

---

## 📌 **関連ファイル一覧**

| ファイル | 役割 | 状態 |
|---------|------|------|
| eskf_state.hpp | State 構造体定義 | ✅ 修正済 |
| eskf_initializer.cpp | 初期化ロジック | ✅ 修正済 |
| mex_run_eskf_impl.hpp | MEX インターフェース | ✅ 修正済 |
| mex_run_eskf_sensor_updates.hpp | センサー更新 | ✅ 修正済 |
| run_simulation.m | シミュレーション メインループ | ✅ 修正済 |
| run_batch_10sets.m | Batch テストハーネス | 🔄 検証中 |

---

**推奨次ステップ:** Phase 1 の CSV 確認を実施し、Max Innovation の値を検証してください。