# 参照検索結果＆最終削除判定表

**分析日**: 2025年11月  
**対象**: KalmanFilter プロジェクト - Utils ディレクトリ  
**目的**: 削除対象ファイルの安全性確認  

---

## 📊 削除候補ファイルの参照検索結果

### 1. FilterUtils.m

**検索結果**: ❌ **参照なし** ✅ 削除可能

```
grep -r "FilterUtils" kalman/  # マッチなし
```

**詳細**:
- MATLAB内で直接呼び出し: なし
- マークダウンドキュメント内での言及のみ
- 削除リスク: **極低 (0%)**

**判定**: ✅ **安全に削除可能**

---

### 2. AccelFilter.m

**検索結果**: ⚠️ **2箇所の参照あり** - 慎重な対応が必要

#### 参照場所
```
1. ESKF.m, line 220
   obj.accel_filter = AccelFilter(0.3, 20);  # レガシー初期化

2. test_accel_filter.m (テストスクリプト)
   af = AccelFilter(0.3, 20);
```

**詳細**:
- ESKF.m での使用: **レガシーコード**（新システムは SensorAccelFilter を使用）
- test_accel_filter.m: **古いテストスクリプト**

**ESKF.m の現状**:
```matlab
% 行220: レガシー初期化
obj.accel_filter = AccelFilter(0.3, 20);

% 行224: 新システム初期化
obj.sensor_filters.accel = SensorFilter.createAccelFilter();

% 実装の現状
% - Line 355以降: sensor_filters.accel を使用
% - AccelFilter (obj.accel_filter) は使用されていない（コメント化済みかもしれない）
```

**判定**: ✅ **削除可能** (前提条件あり)
- ESKF.m 行220 のレガシー行を削除
- test_accel_filter.m を削除
- その後 AccelFilter.m を削除

---

### 3. ema_update.m

**検索結果**: ❌ **参照なし** ✅ 削除可能

```
grep -r "ema_update" kalman/  # マッチなし
```

**詳細**:
- 内部実装されてしまったため、外部参照がない
- 各 SensorXXXFilter がEMA計算をしている
- 削除リスク: **極低 (0%)**

**判定**: ✅ **安全に削除可能**

---

### 4. hampel_causal.m

**検索結果**: ❌ **参照なし** ✅ 削除可能

```
grep -r "hampel_causal" kalman/  # マッチなし
```

**詳細**:
- Hampelアルゴリズムは古典的外れ値検出
- 新システムでは3σ検出 + Mode/中央値フィルタで代替
- 削除リスク: **極低 (0%)**

**判定**: ✅ **安全に削除可能**

---

### 5. alpha_beta_step.m

**検索結果**: ❌ **参照なし** ✅ 削除可能

```
grep -r "alpha_beta_step" kalman/  # マッチなし
```

**詳細**:
- シンプルな 1D alpha-beta トラッカー
- EKF/UKFの高度なアルゴリズムで不要
- 削除リスク: **極低 (0%)**

**判定**: ✅ **安全に削除可能**

---

### 6. OutlierGuard.m

**検索結果**: ✅ **実際に使用されている** ⚠️ 削除不可

#### 参照場所
```
1. ESKF.m, line 464
   [should_update, y_used, K_used, dx_used, diag_info] = 
       OutlierGuard.checkAndApply('mag', z, h, H, obj.P, R_used, 
                                   K_prop, [], obj.divergence_guard, 
                                   obj.noiseEstimator, ctx);

2. ESKF.m, line 595
   [should_update, y_used, K_used, dx_used, diag_info] = 
       OutlierGuard.checkAndApply('gps', z_gps, obj.p, H_gps, obj.P, 
                                   R_updated, [], dx, 
                                   obj.divergence_guard, obj.noiseEstimator, ctx);

3. DivergenceGuard.m, line 269
   % Include any diagnostics passed via ctx (e.g., OutlierGuard diagnostics)
```

**詳細**:
- **磁気計フィルタ処理** で使用: `OutlierGuard.checkAndApply()`
- **GPS フィルタ処理** で使用: `OutlierGuard.checkAndApply()`
- DivergenceGuard との連携あり

**使用方法**:
```matlab
% 磁気計更新時
[should_update, y_used, K_used, dx_used, diag_info] = ...
    OutlierGuard.checkAndApply('mag', z, h, H, obj.P, R_used, K_prop, 
                               [], obj.divergence_guard, obj.noiseEstimator, ctx);

% GPS更新時
[should_update, y_used, K_used, dx_used, diag_info] = ...
    OutlierGuard.checkAndApply('gps', z_gps, obj.p, H_gps, obj.P, R_updated, 
                               [], dx, obj.divergence_guard, obj.noiseEstimator, ctx);
```

**判定**: 🔴 **削除不可** - ESKF の重要な機能

**修正案**: COMPARISON_AND_CONSOLIDATION.md の記述を修正

---

## 📋 修正された最終削除判定

### ✅ 確実に削除可能（リスク 0%）

```
1. FilterUtils.m         (参照なし)
2. ema_update.m         (参照なし)
3. hampel_causal.m      (参照なし)
4. alpha_beta_step.m    (参照なし)
```

### ⚠️ 条件付き削除可能（準備が必要）

```
5. AccelFilter.m        
   前提条件:
   - ESKF.m line 220 の削除/コメント化
   - test_accel_filter.m の削除
```

### 🔴 削除不可（現在も使用中）

```
6. OutlierGuard.m       
   理由: ESKF 磁気計・GPS フィルタリングで使用
```

---

## 🔧 実行手順（修正版）

### Phase 1: ドキュメント修正
- [ ] COMPARISON_AND_CONSOLIDATION.md の OutlierGuard 記述を修正
- [ ] 参照検索結果を正式レポートに追加

### Phase 2: テスト実行前準備
- [ ] ESKF.m 行220 を確認（実装確認）
- [ ] 行220 が ActiveCode か Dead Code か判定

### Phase 3: ファイル削除
```matlab
1. test_accel_filter.m      # 削除（古いテスト）
2. FilterUtils.m             # 削除
3. ema_update.m             # 削除
4. hampel_causal.m          # 削除
5. alpha_beta_step.m        # 削除
```

### Phase 4: ESKF.m 修正（条件付き）
```matlab
% 行220 を確認
if % このコードが実際に Active なら
    行220を削除: obj.accel_filter = AccelFilter(0.3, 20);
end

% その後
6. AccelFilter.m            # 削除
```

### Phase 5: テスト実行
```
36,001ステップシミュレーション実行 → 妥当性確認
```

---

## 📝 最終推奨事項（修正版）

### 即座に実行可能
- ✅ FilterUtils.m, ema_update.m, hampel_causal.m, alpha_beta_step.m 削除
- ✅ test_accel_filter.m 削除

### 確認後に実行
- ⚠️ AccelFilter.m 削除（ESKF.m 確認後）

### 保持すべき
- 🔴 OutlierGuard.m（ESKF の重要機能）
- 🟢 SensorFilter.m, SensorAccelFilter.m 他（全てActive）
- 🟢 DivergenceGuard.m（Kalmanフィルタ発散防止）

---

## 🎯 修正前と修正後の比較

### 修正前（ドキュメント原文）
```
├── OutlierGuard.m                 [UNUSED] → 削除候補
```

**問題**: 実装時に OutlierGuard は UNUSED ではなく、ACTIVE

### 修正後（正確な判定）
```
├── OutlierGuard.m                 [ACTIVE] ✅ 保持
   使用箇所: ESKF.m (磁気計・GPS フィルタ処理)
```

---

## 📌 次のステップ

1. **COMPARISON_AND_CONSOLIDATION.md の修正**
   - OutlierGuard 記述の更新
   - 削除判定テーブルの正確化

2. **ESKF.m の確認**
   - 行220 （AccelFilter レガシー初期化）が生きているか確認
   - Active なら削除前の修正が必要

3. **実行**
   - Phase 1～5 のステップを順序通り実施
   - 各フェーズ後にテスト実行

**推定作業時間**: 1-2時間（安全で確実な実施）
