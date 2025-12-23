# Utils フォルダ依存関係の最終確認レポート

**作成日**: 2025年12月23日  
**スキャン対象**: kalman フォルダ全体  
**方法**: grep による参照検索 + MATLAB コード読込

---

## 📋 最終的な呼び出し関係マップ

### 【発見】ESKF.m が **全ての Sensor*Filter クラスを使用中**

#### 行 193-199 の抽出:

```matlab
obj.sensor_filters = struct();
% NOTE: Sensor filters are now MEX-only; MATLAB instances below are placeholders/disabled
obj.sensor_filters.accel = SensorAccelFilter(struct('ema_alpha', 0.3, 'history_size', 20, 'gravity_range', [8.5, 10.5]));
% Gyro MATLAB implementation removed — do not create gyro filter
obj.sensor_filters.gyro  = [];
obj.sensor_filters.mag   = SensorMagFilter(struct('ema_alpha', 0.2, 'history_size', 20, 'mag_norm_expected', 50));
obj.sensor_filters.gps   = SensorGPSFilter(struct('ema_alpha', 0.15, 'history_size', 10, 'horizontal_accuracy', 2.5, 'vertical_accuracy', 5.0));
obj.sensor_filters.baro  = SensorBaroFilter(struct('ema_alpha', 0.1, 'history_size', 50, 'altitude_per_pressure', 44330));
```

---

## 🔍 完全な呼び出し関係

```
ESKF.m (行 193-199)
  ├─→ SensorAccelFilter()       [直接 new instance]
  ├─→ SensorMagFilter()         [直接 new instance]
  ├─→ SensorGPSFilter()         [直接 new instance]
  ├─→ SensorBaroFilter()        [直接 new instance]
  ├─→ NoiseEstimator()          [行 191]
  ├─→ DivergenceGuard()         [行 210]
  └─→ その他ヘルパークラス

run_batch_10sets.m (行 52-54)
  ├─→ SensorFilters.reset_zero()
  └─→ SensorFilters.reset()

AccelFilter.m (内部使用)
  └─→ SensorFilters.accel()

SensorAccelFilter.m (内部使用)
  └─→ SensorFilters.accel()

DivergenceGuard.m (内部使用)
  ├─→ SensorFilters.divergence_check()
  └─→ SensorFilters.divergence_regularize()
```

---

## 📊 ファイル別の削除可否判定

| ファイル | 呼び出し元 | 現在の状態 | 最終判定 | 理由 |
|---------|---------|---------|--------|------|
| **AccelFilter.m** | なし | 純 MATLAB | 🟢 **削除可** | 呼び出し元が確認できない。SensorAccelFilter/SensorFilters で代替可能 |
| **SensorAccelFilter.m** | ESKF.m (L193) | MEX 委譲 | 🟠 **段階削除** | ESKF で直接 new 可能。段階1で MATLAB 実装コメント化 → 段階2で MEX 化決定後に削除 |
| **SensorMagFilter.m** | ESKF.m (L197) | 純 MATLAB | 🟠 **段階削除** | ESKF で直接 new 可能。MEX 化が必要 |
| **SensorGPSFilter.m** | ESKF.m (L198) | 純 MATLAB | 🟠 **段階削除** | ESKF で直接 new 可能。MEX 化が必要 |
| **SensorBaroFilter.m** | ESKF.m (L199) | 純 MATLAB | 🟠 **段階削除** | ESKF で直接 new 可能。MEX 化が必要 |
| **BiquadFilter.m** | **なし** | 純 MATLAB | 🟢 **削除可** | 呼び出し元が完全に見つからない。不要 |
| **OutlierGuard.m** | **なし** | 純 MATLAB | 🟢 **削除可** | 直接呼び出し元が見つからない。不要 |
| **NoiseEstimator.m** | ESKF.m (L191) | MEX 委譲中 | 🟡 **保持** | ESKF で使用中。MEX 化を完成させて保持 |
| **DivergenceGuard.m** | ESKF.m (L210) | MEX 委譲 | 🟡 **保持** | ESKF で使用中。MEX 化完成 |
| **SensorFilters.m** | 複数 | MEX 委譲 | 🟡 **保持** | 全てのセンサーフィルタ統合ラッパー |

---

## 🎯 段階的削除・MEX 化スケジュール

### **段階 0：即座に削除（リスクゼロ）**

```
削除対象（呼び出し元なし）:
  ✓ BiquadFilter.m
  ✓ OutlierGuard.m
```

**実行方法**:
```bash
rm kalman/KF/Utils/BiquadFilter.m
rm kalman/KF/Utils/OutlierGuard.m
```

---

### **段階 1：MATLAB 実装をコメント化してテスト**

#### 1-1. SensorAccelFilter の MATLAB 実装削除

**ファイル**: [kalman/KF/Utils/SensorAccelFilter.m](kalman/KF/Utils/SensorAccelFilter.m)

**対象行**: apply() メソッド

**変更前**:
```matlab
function [a_out, is_outlier, info] = apply(obj, a_meas, a_expected)
    % Delegate to SensorFilters (now a MEX-only wrapper)
    if nargout >= 2
        [a_filt, is_outlier] = SensorFilters.accel(a_meas, a_expected);
    else
        a_filt = SensorFilters.accel(a_meas, a_expected);
        is_outlier = false;
    end
    obj.a_filtered = a_filt;
    a_out = a_filt;
    info = struct('is_outlier', is_outlier, 'is_gravity_mismatch', false, 'scale_factor', 1.0);
end
```

**変更後**:
```matlab
% ✅ MEX 化完了: SensorFilters.accel() のみを使用
function [a_out, is_outlier, info] = apply(obj, a_meas, a_expected)
    % Pure MEX delegation (no fallback)
    if nargout >= 2
        [a_filt, is_outlier] = SensorFilters.accel(a_meas, a_expected);
    else
        a_filt = SensorFilters.accel(a_meas, a_expected);
        is_outlier = false;
    end
    obj.a_filtered = a_filt;
    a_out = a_filt;
    info = struct('is_outlier', is_outlier, 'is_gravity_mismatch', false, 'scale_factor', 1.0);
end
```

**テスト実行**:
```matlab
run_simulation(42, true);
run_batch_10sets();
compare_mex_matlab_detailed();
```

---

#### 1-2. SensorMagFilter を SensorFilters に統合

**ファイル**: [kalman/KF/Utils/SensorMagFilter.m](kalman/KF/Utils/SensorMagFilter.m)

**方針**: apply() メソッドを `SensorFilters.mag()` に委譲

**変更例**:
```matlab
classdef SensorMagFilter < handle
    methods
        function [m_filt, is_outlier, info] = apply(obj, m_meas, m_expected)
            % ✅ MEX 化: SensorFilters.mag() のみを使用
            if nargout >= 2
                [m_filt, is_outlier] = SensorFilters.mag(m_meas, m_expected);
            else
                m_filt = SensorFilters.mag(m_meas, m_expected);
                is_outlier = false;
            end
            info = struct('is_outlier', is_outlier);
        end
    end
end
```

---

#### 1-3. SensorGPSFilter を SensorFilters に統合

**同様に** apply() メソッドを `SensorFilters.gps()` に委譲

---

#### 1-4. SensorBaroFilter を SensorFilters に統合

**同様に** apply() メソッドを `SensorFilters.baro()` に委譲

---

### **段階 2：回帰テスト**

各段階後に必ず実行:

```matlab
% 単体テスト
run_simulation(42, true)

% バッチテスト
run_batch_10sets()

% 詳細比較
compare_mex_matlab_detailed()
```

**期待値**:
- RMSE (Roll/Pitch) < 0.30°
- GPS/Baro/Mag パリティ保持

---

### **段階 3：完全削除（MEX 化確認後）**

**タイミング**: 全段階2テストが成功後

**削除対象**:
```
削除候補（MEX 化確認後）:
  [ ] AccelFilter.m              (使用元なし)
  [ ] SensorAccelFilter.m        (ESKF から直接 SensorFilters に切り替え後)
  [ ] SensorMagFilter.m          (ESKF から直接 SensorFilters に切り替え後)
  [ ] SensorGPSFilter.m          (ESKF から直接 SensorFilters に切り替え後)
  [ ] SensorBaroFilter.m         (ESKF から直接 SensorFilters に切り替え後)
```

---

## 🚀 ESKF の修正計画

### 現在のコード（L193-199）

```matlab
obj.sensor_filters.accel = SensorAccelFilter(...);
obj.sensor_filters.mag   = SensorMagFilter(...);
obj.sensor_filters.gps   = SensorGPSFilter(...);
obj.sensor_filters.baro  = SensorBaroFilter(...);
```

### 最終形（完全削除後）

```matlab
% NOTE: All sensor filters are now MEX-based through SensorFilters
% Individual wrapper classes (SensorAccelFilter, etc.) removed
% MATLAB側では struct のメタデータのみ保持
obj.sensor_filters = struct('accel', struct(), 'mag', struct(), 'gps', struct(), 'baro', struct());
```

### または直接呼び出しに統一

```matlab
% sensor_updates.m 内で直接呼び出し
[a_filt, is_out] = SensorFilters.accel(a_meas, a_expected);
[m_filt, is_out] = SensorFilters.mag(m_meas, m_expected);
```

---

## 📈 最終的な層構成

### ✅ 完了後

```
【削除】
├─ BiquadFilter.m              ❌ 削除
├─ OutlierGuard.m              ❌ 削除
├─ AccelFilter.m               ❌ 削除
├─ SensorAccelFilter.m         ❌ 削除
├─ SensorMagFilter.m           ❌ 削除
├─ SensorGPSFilter.m           ❌ 削除
└─ SensorBaroFilter.m          ❌ 削除

【保持】
├─ SensorFilters.m             ✅ 保持（MEX 統合ラッパー）
├─ NoiseEstimator.m            ✅ 保持（互換性層）
└─ DivergenceGuard.m           ✅ 保持（MEX 委譲）

【簡潔化】
├─ ESKF.m
│  └─ sensor_filters = struct()  [メタデータのみ]
│     または
│  └─ 直接 SensorFilters.accel/mag/gps/baro() 呼び出し
└─ sensor_updates.m
   └─ 直接 SensorFilters.accel(...) 等を呼び出し
```

---

## ⚠️ 重要な注意事項

1. **MEX の型チェック**: float32 vs float64 の混在を避ける
   - 確認先: [TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md)

2. **NoiseEstimator の状態管理**: MATLAB ↔ MEX 間の R_accel/R_gyro 同期が必要
   - 確認先: [kalman/KF/Utils/NoiseEstimator.m](kalman/KF/Utils/NoiseEstimator.m)

3. **各段階後のテスト必須**: `run_batch_10sets()` で RMSE 変化を確認

4. **ロールバック計画**: 問題があれば git で即座に元に戻す

---

## 📝 実行手順チェックリスト

```
【段階0】
[ ] BiquadFilter.m 削除
[ ] OutlierGuard.m 削除
[ ] テスト: run_simulation(42, true)

【段階1-1】
[ ] SensorAccelFilter.m の MATLAB フォールバック削除
[ ] テスト: run_batch_10sets()
[ ] 比較: compare_mex_matlab_detailed()

【段階1-2～1-4】
[ ] SensorMagFilter.m の MEX 委譲確認
[ ] SensorGPSFilter.m の MEX 委譲確認
[ ] SensorBaroFilter.m の MEX 委譲確認
[ ] テスト: run_batch_10sets() ✓

【段階2】
[ ] RMSE 確認 < 0.30°
[ ] GPS/Baro/Mag パリティ ✓
[ ] Divergence Guard フラグ確認 ✓

【段階3】
[ ] 4 つの SensorFilter クラス削除
[ ] ESKF.m の sensor_filters struct 簡潔化
[ ] 最終テスト: run_batch_10sets() ✓

【完了】
[ ] 計画書のレビュー
[ ] commit 実行
```

---

## 参考リソース

- MEX ビルド: [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m)
- テストスクリプト: [kalman/run_batch_10sets.m](kalman/run_batch_10sets.m)
- 詳細比較: [kalman/compare_mex_matlab_detailed.m](kalman/compare_mex_matlab_detailed.m)
- MATLAB/MEX パリティ: [kalman/mardown/MATLAB_MEX_PARITY_CHECKLIST.md](kalman/mardown/MATLAB_MEX_PARITY_CHECKLIST.md)
- 型チェック: [kalman/cpp/TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md)
