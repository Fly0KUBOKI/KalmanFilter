# Utils フォルダの依存関係 - ビジュアル図解

## 【現在の構成】

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│                                                               │
│  ┌──────────────────┐  ┌──────────────────┐                │
│  │ run_batch_10sets │  │  ESKF.sensor_    │                │
│  │      .m          │  │    updates.m     │                │
│  └────────┬─────────┘  └────────┬─────────┘                │
└───────────┼──────────────────────┼────────────────────────────┘
            │                      │
            │  calls               │  calls
            ▼                      ▼
┌─────────────────────────────────────────────────────────────┐
│           Sensor Filter Wrapper Layer                       │
│                                                               │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  SensorFilters.m (MEX Wrapper - 全て MEX 委譲)         │ │
│  │  - reset()                                              │ │
│  │  - accel(a_meas, a_expected)     → SensorAccelFilter   │ │
│  │  - mag(m_meas, m_expected)        → SensorMagFilter    │ │
│  │  - gps(gps_pos, dt)               → SensorGPSFilter    │ │
│  │  - baro(pressure)                 → SensorBaroFilter   │ │
│  │  - divergence_check(...)          → DivergenceGuard    │ │
│  │  - get_R(sensor_type)             → NoiseEstimator    │ │
│  └────────────────────────────────────────────────────────┘ │
│                                 ▲                             │
└─────────────────────────────────┼─────────────────────────────┘
                                  │
                    ┌─────────────┴──────────────┐
                    │                            │
                    │ (MATLAB filter classes)    │ (Stats)
                    ▼                            ▼
          ┌──────────────────────┐    ┌─────────────────┐
          │ Intermediate Layer   │    │ Helper Classes  │
          │                      │    │                 │
          │ - SensorAccelFilter  │    │ - NoiseEstimator│
          │ - SensorMagFilter    │    │ - DivergenceG.. │
          │ - SensorGPSFilter    │    │                 │
          │ - SensorBaroFilter   │    │ (Obsolete)      │
          │ - AccelFilter        │    │ - OutlierGuard  │
          │                      │    │ - BiquadFilter  │
          │ (all delegate to     │    │                 │
          │  SensorFilters)      │    │                 │
          └──────────────────────┘    └─────────────────┘
                    │
                    │
                    ▼
          ┌──────────────────────┐
          │   MEX Layer          │
          │                      │
          │ mex_sensor_filter    │
          │ (C++ implementation) │
          └──────────────────────┘
```

---

## 【データフロー - 加速度フィルタの例】

### 現在（Phase 0）:

```
ESKF.sensor_updates()
  │
  ├─→ a_meas = [ax, ay, az]
  │
  ├─→ obj.sensor_filters.accel.apply(a_meas, a_expected)
  │     │
  │     └─→ SensorAccelFilter.apply()
  │           │
  │           └─→ SensorFilters.accel(a_meas, a_expected)
  │                 │
  │                 └─→ mex_sensor_filter('accel', a_meas, a_expected)
  │                       │
  │                       └─→ C++ SensorFilterLib::accel()
  │                             │
  │                             └─→ [a_filt, is_outlier]
  │
  └─→ [a_filt, is_outlier] ✓
```

### 完了後（Phase 2）:

```
ESKF.sensor_updates()
  │
  ├─→ a_meas = [ax, ay, az]
  │
  ├─→ SensorFilters.accel(a_meas, a_expected)  ← 直接呼び出し
  │     │
  │     └─→ mex_sensor_filter('accel', a_meas, a_expected)
  │           │
  │           └─→ C++ SensorFilterLib::accel()
  │                 │
  │                 └─→ [a_filt, is_outlier]
  │
  └─→ [a_filt, is_outlier] ✓
```

**削減**: 2 層分のラッパー呼び出しを削除

---

## 【ファイル削除タイムライン】

```
Phase 0 (即座)
│
├─ BiquadFilter.m          ❌ DELETE
├─ OutlierGuard.m          ❌ DELETE
│
└─→ test: run_simulation(42, true)


Phase 1 (段階的 MEX 化 + テスト)
│
├─ SensorAccelFilter.m     → SensorFilters.accel() に統一
│  └─→ test: run_batch_10sets()
│
├─ SensorMagFilter.m       → SensorFilters.mag() に統一
│  └─→ test: run_batch_10sets()
│
├─ SensorGPSFilter.m       → SensorFilters.gps() に統一
│  └─→ test: run_batch_10sets()
│
├─ SensorBaroFilter.m      → SensorFilters.baro() に統一
│  └─→ test: run_batch_10sets()
│
└─→ RMSE < 0.30° ✓ 確認


Phase 2 (最終削除)
│
├─ AccelFilter.m           ❌ DELETE
├─ SensorAccelFilter.m     ❌ DELETE
├─ SensorMagFilter.m       ❌ DELETE
├─ SensorGPSFilter.m       ❌ DELETE
├─ SensorBaroFilter.m      ❌ DELETE
│
└─→ test: run_batch_10sets() (final check)


保持
│
├─ SensorFilters.m         ✅ KEEP (MEX wrapper hub)
├─ NoiseEstimator.m        ✅ KEEP (compatibility layer)
├─ DivergenceGuard.m       ✅ KEEP (MEX delegation)
├─ alpha_beta_step.m       ✅ KEEP (mex_filter_utils)
├─ ema_update.m            ✅ KEEP (mex_filter_utils)
└─ hampel_causal.m         ✅ KEEP (mex_filter_utils)
```

---

## 【依存グラフ - 呼び出し関係】

### 【PHASE 0 直後】(3 層削減)

```
Before:  ESKF → SensorAccelFilter → SensorFilters → MEX
After:   ESKF → SensorFilters → MEX
         └─────────────────────┘
         (SensorAccelFilter 削除時点)
```

### 【PHASE 1 後】(全て統一 + テスト済み)

```
Before:  
  ESKF → SensorAccelFilter → SensorFilters → MEX
  ESKF → SensorMagFilter → SensorFilters → MEX
  ESKF → SensorGPSFilter → SensorFilters → MEX
  ESKF → SensorBaroFilter → SensorFilters → MEX

After:   
  ESKF → SensorFilters → MEX  (全て統一)
```

### 【PHASE 2 後】(最小レイヤー)

```
ESKF → SensorFilters → MEX (最小構成)
```

---

## 【ファイル削減の効果】

```
削除ファイル数:     9個
├─ BiquadFilter.m
├─ OutlierGuard.m
├─ AccelFilter.m
├─ SensorAccelFilter.m
├─ SensorMagFilter.m
├─ SensorGPSFilter.m
├─ SensorBaroFilter.m
├─ (2 duplicates in visualization)
└─ ...

保持ファイル数:     6個
├─ SensorFilters.m
├─ NoiseEstimator.m
├─ DivergenceGuard.m
├─ alpha_beta_step.m
├─ ema_update.m
└─ hampel_causal.m

削減率:            60% (15個中9個削除)

コードパス短縮:    3層 → 1層 (66% 削減)
```

---

## 【テストチェックポイント】

```
Phase 0 後:
  ✓ run_simulation(42, true)

Phase 1 段階1 後:
  ✓ run_batch_10sets()
  ✓ compare_mex_matlab_detailed()
  ✓ RMSE (Roll/Pitch) < 0.30°

Phase 1 段階2-4 後:
  ✓ 各段階で同様にテスト

Phase 2 後:
  ✓ run_batch_10sets() (final)
  ✓ all RMSE metrics OK
  ✓ パリティ保持確認

最終チェック:
  ✓ git diff で不要な削除がないか確認
  ✓ mex_sensor_filter の全機能が使われているか確認
```

---

## 【トラブルシューティング】

### Q: Phase 1 でテストが失敗した場合

```
A: 即座に git revert で該当 phase をロールバック
   $ git revert HEAD
   $ run_batch_10sets()  # 確認
   
   原因を特定:
   - TYPE_MIX_REPORT.md で float32/float64 混在を確認
   - NoiseEstimator の R_accel/R_gyro 同期を確認
   - mex_sensor_filter のコマンド引数を確認
```

### Q: AccelFilter/Sensor*Filter の呼び出し元が他にもあった場合

```
A: grep -r で完全スキャン
   $ grep -r "AccelFilter\|SensorAccelFilter" kalman --include="*.m"
   
   もし発見されたら:
   - 該当箇所を SensorFilters.accel() に置き換え
   - テスト再実行
```

### Q: 完全削除後に問題が発生した場合

```
A: git リポジトリからファイルを復元
   $ git checkout HEAD -- kalman/KF/Utils/
   $ run_batch_10sets()  # 確認
   
   その後、段階的に再実施
```

---

## 参照

- 詳細計画: [UTILS_MEXIFICATION_PLAN.md](kalman/mardown/UTILS_MEXIFICATION_PLAN.md)
- 依存分析: [UTILS_DEPENDENCY_FINAL_REPORT.md](kalman/mardown/UTILS_DEPENDENCY_FINAL_REPORT.md)
- サマリー: [UTILS_MEX_SUMMARY.md](kalman/mardown/UTILS_MEX_SUMMARY.md)

