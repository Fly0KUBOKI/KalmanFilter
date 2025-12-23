# Utils MEX化 計画 - エグゼクティブサマリー

**最終目標**: Utils フォルダの MATLAB ファイルを削除し、SensorFilters クラスで全ての処理を完結させる

**ステータス**: ✅ 計画完成、実装準備完了

---

## 🎯 一目でわかる依存関係

### 【呼び出し元の調査結果】

```
ESKF.m が全ての Sensor*Filter を直接生成:
  ├─ SensorAccelFilter()  ← MEX 委譲中
  ├─ SensorMagFilter()    ← MATLAB 純実装 (MEX化必要)
  ├─ SensorGPSFilter()    ← MATLAB 純実装 (MEX化必要)
  ├─ SensorBaroFilter()   ← MATLAB 純実装 (MEX化必要)
  ├─ NoiseEstimator()     ← MEX 委譲中
  └─ DivergenceGuard()    ← MEX 委譲完了

呼び出し元がないファイル（即座に削除可能）:
  ├─ BiquadFilter.m       ← 削除 ✓
  └─ OutlierGuard.m       ← 削除 ✓
```

---

## 📊 削除・保持の最終判定

| ファイル | 呼び出し元 | 最終判定 | タイミング |
|---------|---------|--------|---------|
| BiquadFilter.m | なし | 🟢 **削除** | 即座 |
| OutlierGuard.m | なし | 🟢 **削除** | 即座 |
| AccelFilter.m | なし | 🟢 **削除** | 段階2 |
| SensorAccelFilter.m | ESKF.m | 🟠 **段階削除** | MEX確認後 |
| SensorMagFilter.m | ESKF.m | 🟠 **段階削除** | MEX化後 |
| SensorGPSFilter.m | ESKF.m | 🟠 **段階削除** | MEX化後 |
| SensorBaroFilter.m | ESKF.m | 🟠 **段階削除** | MEX化後 |
| SensorFilters.m | 複数 | 🟡 **保持** | 永続 |
| NoiseEstimator.m | ESKF.m | 🟡 **保持** | 永続 |
| DivergenceGuard.m | ESKF.m | 🟡 **保持** | 永続 |

---

## 🚀 実装ロードマップ

### **Phase 0: 即座（リスクゼロ）**

**削除対象**: BiquadFilter.m, OutlierGuard.m  
**時間**: 5 分  
**テスト**: run_simulation(42, true)

```bash
rm kalman/KF/Utils/BiquadFilter.m
rm kalman/KF/Utils/OutlierGuard.m
```

### **Phase 1: 段階的 MEX 化（低レイヤーから）**

**対象**: SensorAccelFilter → SensorMagFilter → SensorGPSFilter → SensorBaroFilter  
**方法**: apply() を SensorFilters.accel/mag/gps/baro() に完全委譲  
**テスト**: 各段階で `run_batch_10sets()` + `compare_mex_matlab_detailed()`  
**時間**: 各 30 分 × 4 = 2 時間

```matlab
% 例: SensorAccelFilter.m
function [a_out, is_outlier] = apply(obj, a_meas, a_expected)
    % ✅ MEX 完全委譲
    [a_out, is_outlier] = SensorFilters.accel(a_meas, a_expected);
end

% 同様に SensorMagFilter, SensorGPSFilter, SensorBaroFilter を修正
```

### **Phase 2: 最終削除（テスト成功後）**

**削除対象**: AccelFilter, SensorAccelFilter, SensorMagFilter, SensorGPSFilter, SensorBaroFilter  
**時間**: 15 分  
**テスト**: 最終回帰テスト

```bash
rm kalman/KF/Utils/AccelFilter.m
rm kalman/KF/Utils/SensorAccelFilter.m
rm kalman/KF/Utils/SensorMagFilter.m
rm kalman/KF/Utils/SensorGPSFilter.m
rm kalman/KF/Utils/SensorBaroFilter.m
```

### **Phase 3: ESKF 簡潔化（オプション）**

ESKF.m の不要な struct 参照を削除し、直接 SensorFilters 呼び出しに統一

---

## ✅ 最小実行可能パス（MVP）

### ステップ 1: 事前準備（確認のみ）

```matlab
% TYPE_MIX_REPORT.md を読む
% MATLAB/MEX パリティを確認
edit kalman/cpp/TYPE_MIX_REPORT.md
```

### ステップ 2: Phase 0 実行（即座削除）

```bash
rm kalman/KF/Utils/BiquadFilter.m
rm kalman/KF/Utils/OutlierGuard.m
cd kalman
run_simulation(42, true)  % 確認テスト
```

### ステップ 3: Phase 1 実行（段階的 MEX 化）

**3-1. SensorAccelFilter の修正**
- [kalman/KF/Utils/SensorAccelFilter.m](kalman/KF/Utils/SensorAccelFilter.m) の apply() を SensorFilters.accel() に統一
- `run_batch_10sets()`
- `compare_mex_matlab_detailed()` で RMSE 確認

**3-2. SensorMagFilter の修正**
- 同様に apply() を SensorFilters.mag() に統一
- テスト実行

**3-3. SensorGPSFilter の修正**
- 同様に apply() を SensorFilters.gps() に統一
- テスト実行

**3-4. SensorBaroFilter の修正**
- 同様に apply() を SensorFilters.baro() に統一
- テスト実行

### ステップ 4: Phase 2 実行（最終削除）

```bash
rm kalman/KF/Utils/AccelFilter.m
rm kalman/KF/Utils/SensorAccelFilter.m
rm kalman/KF/Utils/SensorMagFilter.m
rm kalman/KF/Utils/SensorGPSFilter.m
rm kalman/KF/Utils/SensorBaroFilter.m
run_batch_10sets()  % 最終確認
```

---

## 📈 完了後の構成

### 削除されるファイル（全 9 個）

```
kalman/KF/Utils/
├─ ✗ BiquadFilter.m
├─ ✗ OutlierGuard.m
├─ ✗ AccelFilter.m
├─ ✗ SensorAccelFilter.m
├─ ✗ SensorMagFilter.m
├─ ✗ SensorGPSFilter.m
├─ ✗ SensorBaroFilter.m
├─ ✗ SensorGPSFilter.m
├─ ✗ SensorMagFilter.m
```

### 保持されるファイル（最小構成）

```
kalman/KF/Utils/
├─ ✓ SensorFilters.m        (MEX ラッパー - 全センサー統合)
├─ ✓ NoiseEstimator.m       (互換性層 + MEX 委譲)
├─ ✓ DivergenceGuard.m      (MEX 委譲完了)
├─ ✓ alpha_beta_step.m      (mex_filter_utils 呼び出し)
├─ ✓ ema_update.m           (mex_filter_utils 呼び出し)
├─ ✓ hampel_causal.m        (mex_filter_utils 呼び出し)
```

---

## ⏱️ 予想所要時間

| フェーズ | 作業 | テスト | 合計 |
|---------|------|------|------|
| Phase 0 | 5分 | 10分 | **15分** |
| Phase 1 | 20分 | 90分 | **110分** |
| Phase 2 | 15分 | 20分 | **35分** |
| **合計** | | | **~3時間** |

---

## 🔐 リスク対策

### リスク 1: MEX の型混在

**対策**: [TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md) で確認 → double に統一

### リスク 2: パリティ喪失

**対策**: 各段階で `compare_mex_matlab_detailed()` で RMSE 確認
- 期待値: Roll/Pitch < 0.30°

### リスク 3: 予期しない呼び出し元

**対策**: grep で 3 回検索確認済み

---

## 📚 参考文書

| 文書 | 目的 |
|-----|------|
| [UTILS_MEXIFICATION_PLAN.md](kalman/mardown/UTILS_MEXIFICATION_PLAN.md) | 詳細計画 |
| [UTILS_DEPENDENCY_FINAL_REPORT.md](kalman/mardown/UTILS_DEPENDENCY_FINAL_REPORT.md) | 依存関係の詳細分析 |
| [TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md) | 型混在チェック |
| [MATLAB_MEX_PARITY_CHECKLIST.md](kalman/mardown/MATLAB_MEX_PARITY_CHECKLIST.md) | パリティチェックリスト |

---

## 次のアクション

### 🟢 即座に実行可能

1. BiquadFilter.m と OutlierGuard.m を削除
2. `run_simulation(42, true)` で動作確認
3. Phase 1 開始

### 🟡 計画に基づいて順序で実行

1. SensorAccelFilter → SensorMagFilter → SensorGPSFilter → SensorBaroFilter の順で段階的修正
2. 各段階で `run_batch_10sets()` + `compare_mex_matlab_detailed()` を実行
3. RMSE < 0.30° を確認したら次へ進む

### 🔴 注意事項

- 各フェーズ後に **必ずテストを実行**
- 問題があれば **git でロールバック**
- NoiseEstimator の状態管理（R_accel, R_gyro 等）は同期を確認

---

**計画書作成**: 2025年12月23日  
**計画状態**: ✅ 完成・実装準備完了  
**推奨開始時期**: 即座（Phase 0 から）

