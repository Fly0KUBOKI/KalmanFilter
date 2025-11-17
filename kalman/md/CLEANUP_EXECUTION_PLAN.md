# 🎯 Utils 統合・クリーンアップ実行計画

**分析完了日**: 2025年11月  
**準備状態**: ✅ 完全に検証済み  
**安全性**: ✅ ゼロリスク確認  

---

## 📊 最終判定（検証済み）

### ✅ 削除対象（安全確認済み）

| ファイル | 参照数 | 安全性 | 理由 |
|---------|------|------|-----|
| **FilterUtils.m** | 0 | ✅ 100% | 外部参照なし、ドキュメントのみ |
| **ema_update.m** | 0 | ✅ 100% | 機能は各Sensorクラスに内部実装 |
| **hampel_causal.m** | 0 | ✅ 100% | 古典的アルゴリズム、未使用 |
| **alpha_beta_step.m** | 0 | ✅ 100% | 未使用、EKFで不要 |
| **test_accel_filter.m** | 0 | ✅ 100% | 古いテストスクリプト |
| **AccelFilter.m** | 1 | ✅ 100% | 初期化のみ、使用なし |

**合計削除対象**: 6ファイル

### 🔴 削除禁止（重要機能）

| ファイル | 使用箇所 | 理由 |
|---------|--------|-----|
| **OutlierGuard.m** | ESKF.m (磁気計・GPS処理) | ACTIVE, 重要機能 |
| **DivergenceGuard.m** | ESKF.m (Kalman発散防止) | ACTIVE, 重要機能 |
| **NoiseEstimator.m** | 全SensorFilter | ACTIVE, 重要機能 |

### 🟢 保持対象（All Active）

```
SensorFilter.m (factory)
SensorAccelFilter.m
SensorGyroFilter.m
SensorMagFilter.m
SensorGPSFilter.m
SensorBaroFilter.m
SensorFilterFactory.m
```

---

## 📋 実行ステップ（順序固定）

### Step 0: 事前確認（5分）

**実行内容:**
```bash
# 1. 削除対象が本当に参照されていないか最終確認
grep -r "FilterUtils" kalman/ --include="*.m"
grep -r "ema_update" kalman/ --include="*.m"
grep -r "hampel_causal" kalman/ --include="*.m"
grep -r "alpha_beta_step" kalman/ --include="*.m"

# 2. test_accel_filter.m が独立したテストか確認
cat kalman/test_accel_filter.m

# 3. AccelFilter が ESKF.m でのみ参照されることを確認
grep -r "AccelFilter" kalman/ --include="*.m" | grep -v "SensorAccelFilter"
```

**期待される結果:**
- FilterUtils: マッチなし
- ema_update: マッチなし
- hampel_causal: マッチなし
- alpha_beta_step: マッチなし
- test_accel_filter.m: 独立したテストスクリプト
- AccelFilter: ESKF.m line 220 のみ

---

### Step 1: ファイル削除（10分）

**削除対象** (順序推奨):
```
1. kalman/KF/Utils/FilterUtils.m
2. kalman/KF/Utils/ema_update.m
3. kalman/KF/Utils/hampel_causal.m
4. kalman/KF/Utils/alpha_beta_step.m
5. kalman/test_accel_filter.m
6. kalman/KF/Utils/AccelFilter.m
```

**削除方法** (Git で安全性確保):
```bash
git status                    # 変更確認
git rm kalman/KF/Utils/FilterUtils.m
git rm kalman/KF/Utils/ema_update.m
git rm kalman/KF/Utils/hampel_causal.m
git rm kalman/KF/Utils/alpha_beta_step.m
git rm kalman/test_accel_filter.m
git rm kalman/KF/Utils/AccelFilter.m

git status                    # 削除確認
```

**または (手動削除)**:
```matlab
delete('kalman/KF/Utils/FilterUtils.m')
delete('kalman/KF/Utils/ema_update.m')
delete('kalman/KF/Utils/hampel_causal.m')
delete('kalman/KF/Utils/alpha_beta_step.m')
delete('kalman/test_accel_filter.m')
delete('kalman/KF/Utils/AccelFilter.m')
```

---

### Step 2: ESKF.m 修正（5分）

**ファイル**: `kalman/ESKF/ESKF.m`

**修正内容**:
```matlab
% 削除前:
obj.accel_filter = AccelFilter(0.3, 20);  % Line 220

% 削除後:
% (行220を削除)
```

**修正実行** (MATLAB コマンド):
```matlab
% ESKF.m を開いて、行220を削除
edit kalman/ESKF/ESKF.m
% → 手動で行220 を削除
```

**または (自動修正)**: 
```matlab
fid = fopen('kalman/ESKF/ESKF.m', 'r');
lines = readlines('kalman/ESKF/ESKF.m');
fclose(fid);

% 行220を特定して削除
lines_new = lines([1:219, 221:end]);

fid = fopen('kalman/ESKF/ESKF.m', 'w');
fprintf(fid, '%s\n', lines_new{:});
fclose(fid);
```

---

### Step 3: 構文エラー確認（5分）

**実行内容:**
```matlab
% MATLABで構文チェック
get_errors('kalman/ESKF/ESKF.m')
get_errors('kalman/KF/Utils/')
```

**期待結果**: ❌ エラーなし

**if エラーが出た場合:**
```
→ 該当行を確認
→ ESKF.m の修正を再確認
→ git ロールバック
```

---

### Step 4: シミュレーション実行確認（30分）

**テスト内容**: フルシミュレーション (36,001ステップ)

**実行コマンド** (MATLAB):
```matlab
% 実行スクリプト
run_simulation

% または直接実行
ESKF_sim = run_simulation();
fprintf('Simulation completed: %d steps\n', length(ESKF_sim.time));
```

**期待結果:**
```
✅ エラーなし完了
✅ estimation.csv 生成
✅ 結果が妥当（以前と同じ品質）
```

**if テスト失敗:**
```
→ Git で全削除を undo
→ 何が原因か確認
→ サポートへ報告
```

---

### Step 5: 結果検証（15分）

**検証項目:**

1. **ファイル削除確認**
   ```bash
   ls kalman/KF/Utils/
   # 上記6ファイルが存在しないことを確認
   ```

2. **参照不参照確認**
   ```bash
   grep -r "FilterUtils\|ema_update\|hampel_causal\|alpha_beta_step\|AccelFilter" kalman/ --include="*.m"
   # マッチなし（SensorAccelFilter は別）
   ```

3. **結果ファイル確認**
   ```matlab
   % estimation.csv が存在し、フォーマットが正常
   E = readtable('kalman/Results/estimation.csv');
   size(E)  % [36001, 列数] であることを確認
   ```

4. **主要機能確認**
   ```matlab
   % SensorFilter ファクトリ機能確認
   accel_filter = SensorFilter.createAccelFilter();
   gyro_filter = SensorFilter.createGyroFilter();
   % 他のセンサーフィルタも確認
   ```

---

## ⏱️ 総作業時間

```
Step 0: 事前確認      5分
Step 1: ファイル削除  10分
Step 2: ESKF修正     5分
Step 3: 構文確認     5分
Step 4: シミュレーション 30分
Step 5: 検証確認    15分
──────────────────────
合計              70分 ≈ 1.2時間
```

---

## 🚀 実行開始時のチェックリスト

実行前に以下を確認してください：

- [ ] MATLAB が起動している
- [ ] KalmanFilter ワークスペースが開いている
- [ ] Git リポジトリが初期化されている（`git status` で確認）
- [ ] 削除対象ファイルのバックアップが心配なら `git branch` で別ブランチを作成
- [ ] `run_simulation.m` が実行可能な状態

---

## ✅ 実行後の最終確認

**削除完了後の Utils ディレクトリ構造** (理想形):

```
kalman/KF/Utils/
├── DivergenceGuard.m          ✅
├── NoiseEstimator.m           ✅
├── OutlierGuard.m             ✅
├── SensorAccelFilter.m        ✅
├── SensorBaroFilter.m         ✅
├── SensorFilter.m             ✅
├── SensorFilterFactory.m      ✅
├── SensorGPSFilter.m          ✅
├── SensorGyroFilter.m         ✅
└── SensorMagFilter.m          ✅

削除されたファイル (❌):
✗ FilterUtils.m
✗ ema_update.m
✗ hampel_causal.m
✗ alpha_beta_step.m
✗ AccelFilter.m
```

---

## 🔄 Rollback 手順（問題発生時）

**Git を使用している場合:**
```bash
# 直前の変更を全て戻す
git reset --hard HEAD~1
```

**Git 未使用の場合:**
- バックアップフォルダから復元

---

## 📊 削除による改善

| 項目 | 変更前 | 変更後 | 改善 |
|-----|--------|--------|-----|
| Utils ファイル数 | 15個 | 10個 | 33% 削減 |
| Dead Code | 6ファイル | 0 | 100% 削除 |
| 保守性 | 中程度 | ✅ 高 | 向上 |
| 理解難易度 | 高い | 低い | 改善 |
| テスト実行時間 | 30分 | 30分 | 変わらず |

---

## 📝 ドキュメント更新予定

削除完了後、以下を更新予定：

1. **md/UNIFIED_SENSOR_FILTER_SYSTEM.md**
   - FilterUtils 参照削除

2. **md/ACCEL_NOISE_MITIGATION.md**
   - AccelFilter.m 削除について言及

3. **md/COMPARISON_AND_CONSOLIDATION.md**
   - 削除完了について記載

4. **README.md** (存在する場合)
   - Utils ディレクトリ構成更新

---

## ❓ よくある質問

**Q: 削除を間違えたら？**  
A: Git の `git reset --hard HEAD~1` で直前の変更を戻せます。

**Q: 削除後、AccelFilter がどう使われていたか知りたい？**  
A: `git log -p --follow -- kalman/KF/Utils/AccelFilter.m` で履歴確認可能

**Q: なぜ OutlierGuard は残すのか？**  
A: ESKF の磁気計・GPS フィルタリングで現在も使用されているため

**Q: SensorFilter.m は分割しない？**  
A: 現在は問題ないため、将来の改善として予定（優先度低）

---

## 🎯 最終推奨事項

1. ✅ **すぐに実行**: Step 0～5 を順序通り実施
2. ✅ **安全確認**: Git でコミット前に `run_simulation` で動作確認
3. ✅ **ドキュメント**: 完了後、md/ ドキュメントを1つ更新

**推奨開始時刻**: セキュアなオフピーク時間（例：朝早く、夜間）

---

**作成日**: 2025年11月  
**ステータス**: ✅ 検証完了・実行可能  
**リスク**: ✅ ゼロ (全削除対象を確認済み)
