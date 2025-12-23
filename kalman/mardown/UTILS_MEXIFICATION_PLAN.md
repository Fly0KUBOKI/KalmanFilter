# Utils フォルダの Mex 化に向けた計画書

**最終目標**: MATLAB ファイルを削除し、呼び出し元で直接 MEX を呼び出す。SensorFilter クラスで全ての処理を完結させる

**作成日**: 2025年12月23日  
**ステータス**: 計画段階

---

## 1. 呼び出し関係図

### 依存構造（低レイヤーから上位へ）

```
【層1】基本計算関数（低レイヤー）
├── alpha_beta_step.m          → mex_sensor_filter('alpha_beta_step', ...)
├── ema_update.m               → mex_sensor_filter('ema_update', ...)
├── hampel_causal.m            → mex_sensor_filter('hampel_causal', ...)
└── BiquadFilter.m             → MATLAB 純実装（削除候補）

【層2】フィルタラッパークラス（中間層）
├── AccelFilter.m              → SensorFilters.accel(...) → mex_sensor_filter
├── SensorAccelFilter.m        → SensorFilters.accel(...) → mex_sensor_filter
├── NoiseEstimator.m           → mex_sensor_filter('noise_estimate', ...)
├── DivergenceGuard.m          → SensorFilters.divergence_check(...) → mex_sensor_filter
└── OutlierGuard.m            → 複数クラスを組み合わせる（最上位）

【層3】統合ラッパー（最上位）
└── SensorFilters.m            → mex_sensor_filter (全て委譲)

【層4】呼び出し元（アプリケーション層）
├── run_batch_10sets.m         → SensorFilters.reset/reset_zero
├── sensor_updates.m (ESKF)    → NoiseEstimator.getRnoise()
└── その他フィルタリング処理
```

---

## 2. 各ファイルの役割と呼び出し元

| ファイル名 | 役割 | 呼び出し元 | MEX 状態 | 優先度 |
|-----------|------|---------|--------|--------|
| **SensorFilters.m** | 統合ラッパー（全て MEX に委譲） | AccelFilter, SensorAccelFilter, DivergenceGuard, OutlierGuard, run_batch_10sets | ✅ MEX | ★★★ |
| **AccelFilter.m** | 加速度フィルタ（MATLAB） | 独立呼び出し未確認（可能性あり） | ✅ MEX 委譲 | ★★ |
| **SensorAccelFilter.m** | 加速度フィルタ（クラス） | ESKF（可能性） | ✅ MEX 委譲 | ★★ |
| **NoiseEstimator.m** | ノイズ推定（クラス） | ESKF.sensor_updates.m | ✅ MEX 対応中 | ★★★ |
| **DivergenceGuard.m** | 発散防止（クラス） | ESKF,OutlierGuard | ✅ MEX 委譲 | ★★★ |
| **OutlierGuard.m** | 外れ値判定ラッパー | 直接呼び出し不明 | ❌ 未実装 | ★ |
| **BiquadFilter.m** | 2次IIRフィルタ（MATLAB） | 未確認（削除候補） | ❌ 未実装 | ★ |
| **SensorBaroFilter.m** | 気圧フィルタ | 未確認（削除予定） | ❌ | ☐ |
| **SensorGPSFilter.m** | GPS フィルタ | 未確認（削除予定） | ❌ | ☐ |
| **SensorMagFilter.m** | 磁気フィルタ | 未確認（削除予定） | ❌ | ☐ |

---

## 3. ファイル間の直接的な依存関係

### 【相互参照検索結果】

```
呼び出し元 → 呼び出し先

run_batch_10sets.m
  └─→ SensorFilters.reset()
  └─→ SensorFilters.reset_zero()

ESKF/@ESKF/sensor_updates.m
  └─→ obj.noiseEstimator.getRnoise('accel')
  └─→ obj.noiseEstimator.getRnoise('mag')
  └─→ obj.noiseEstimator.getRnoise('gps')
  └─→ obj.noiseEstimator.getRnoise('baro')

AccelFilter.m
  └─→ SensorFilters.accel(a_meas, a_expected)

SensorAccelFilter.m
  └─→ SensorFilters.accel(a_meas, a_expected)

DivergenceGuard.m
  └─→ SensorFilters.divergence_check(sensor_name, innovation, dx_in)
  └─→ SensorFilters.divergence_regularize(P_in)

OutlierGuard.m
  └─→ SensorFilter.filterInnovation()  ← 不完全な参照（コメントのみ）
  └─→ DivergenceGuard.check_and_attenuate_update()

SensorFilters.m
  └─→ mex_sensor_filter(command, ...)  ← 全て MEX に委譲
```

---

## 4. 現在の MEX 実装状況

### ✅ 既に MEX 化されている機能

| MEX ファイル | 提供する機能 | MATLAB ラッパー |
|-----------|----------|----------|
| **mex_sensor_filter** | reset, accel, mag, gps, baro, noise_estimate, divergence_check, divergence_regularize, get_R | SensorFilters.m |
| **mex_filter_utils** | alpha_beta_step, ema_update, hampel_causal | （直接使用なし） |

### ❌ まだ MEX 化されていない機能

| 機能 | MATLAB ファイル | MEX 候補 |
|------|----------|---------|
| BiquadFilter | BiquadFilter.m | mex_biquad_filter (未作成) |
| OutlierGuard 統合 | OutlierGuard.m | mex_outlier_guard (未作成) |
| 気圧フィルタ | SensorBaroFilter.m | 削除予定 |
| GPS フィルタ | SensorGPSFilter.m | 削除予定 |
| 磁気フィルタ | SensorMagFilter.m | 削除予定 |

---

## 5. MEX 化の段階的計画

### **Phase 0（基盤準備）**
**目標**: 低レイヤーの最小 MEX ユニットを確認  
**作業内容**:
1. `mex_sensor_filter` の機能網羅性を確認
2. `mex_filter_utils` で `alpha_beta_step`, `ema_update`, `hampel_causal` が実装済みか確認
3. C++ の `sensor_filter.hpp` が全機能を包含しているか確認

**確認項目**:
- [ ] `mex_sensor_filter` がすべてのセンサーフィルタ機能を提供
- [ ] 型混在チェック: `float32` vs `float64` の一貫性
- [ ] C++ の境界値と MATLAB の期待値が一致

---

### **Phase 1（最下層から段階的 MEX 化）**

#### **1-1. BiquadFilter の MEX 化評価**
**優先度**: ★★（中） | **難易度**: ★★（中）

**作業内容**:
1. 現在 BiquadFilter.m が使われているかコード検索
   - 呼び出し元が存在しない場合は**削除**
   - 使われている場合は MEX 化判定
2. C++ 実装を準備（`biquad_filter.hpp`）
3. MEX ラッパー作成（`mex_biquad_filter.cpp`）
4. ビルド・テスト

**テストポイント**:
```matlab
% テスト例
bf = BiquadFilter(200, 20);  % 200Hz, 20Hz カットオフ
output = bf.apply(input_signal);
```

**実装方針**:
- 削除か MEX 化かは **呼び出し元リストで決定**
- MEX 化する場合: C++ の `std::vector<float>` で状態管理
- クラスインターフェースは MATLAB 側で保持

---

#### **1-2. OutlierGuard の完全 MEX 化検討**
**優先度**: ★（低）| **難易度**: ★★★（高）

**作業内容**:
1. OutlierGuard.m の実際の使用箇所を検索
   - 現在 **直接呼び出し不明** → 検索が必要
2. 使用されていない場合: **削除**
3. 使用されている場合: 以下の統合を検討
   - SensorFilters の拡張機能として MEX 化
   - または MATLAB 実装を残す

**検索コマンド**:
```bash
grep -r "OutlierGuard" kalman/*.m
grep -r "checkAndApply" kalman/*.m
```

---

### **Phase 2（中間層の MATLAB ラッパー削除）**

#### **2-1. SensorAccelFilter/AccelFilter の削除**
**優先度**: ★★★（高）| **難易度**: ★（低）

**作業内容**:
1. 呼び出し元を特定：
   - ESKF で使われているか確認
   - 独立呼び出しがあるか確認
2. 呼び出し元で **直接 `SensorFilters.accel()` に置き換え**
3. ファイルを削除

**具体例**:
```matlab
% Before
accel_filter = AccelFilter(0.3, 20);
[a_filt, is_out] = accel_filter.filter(a_meas, a_expected);

% After（呼び出し元で直接）
[a_filt, is_out] = SensorFilters.accel(a_meas, a_expected);
```

**削除ファイル**:
- [ ] AccelFilter.m
- [ ] SensorAccelFilter.m

---

#### **2-2. DivergenceGuard の MATLAB 実装を削除**
**優先度**: ★★★（高）| **難易度**: ★★（中）

**作業内容**:
1. DivergenceGuard.m の MATLAB フォールバックをコメントアウト
2. 全て MEX 呼び出しに統一
3. テスト: `run_batch_10sets()` で MATLAB vs MEX 比較が一致

**実装方針**:
```matlab
% DivergenceGuard.m - MEX 完全委譲版
function [dx_out, should_skip, was_attenuated] = check_and_attenuate_update(obj, sensor_name, innovation, dx_in, ctx)
    % MEX 呼び出しのみ（MATLAB フォールバック削除）
    [dx_out, should_skip, was_attenuated] = SensorFilters.divergence_check(sensor_name, innovation, dx_in);
end
```

---

#### **2-3. NoiseEstimator のコメントアウト検討**
**優先度**: ★★（中）| **難易度**: ★★★（高）

**作業内容**:
1. MATLAB 実装を段階的にコメントアウト
2. MEX 呼び出しに統一
3. MATLAB 側の状態管理（R_accel, R_gyro 等）は互換性のため保持

**注意**:
- 現在 MATLAB 側の `NoiseEstimator` が ESKF に統合されている
- MEX 化によるパリティを十分にテストしてから進める

---

### **Phase 3（不要なクラスの削除）**

#### **3-1. 削除予定ファイル**
**優先度**: ★（低）| **難易度**: ★（低）

```
[ ] SensorBaroFilter.m       # 気圧フィルタ（未使用）
[ ] SensorGPSFilter.m        # GPS フィルタ（未使用）
[ ] SensorMagFilter.m        # 磁気フィルタ（未使用）
[ ] OutlierGuard.m           # 直接呼び出し確認後判定
[ ] BiquadFilter.m           # 呼び出し確認後判定
```

**確認手順**:
```bash
# 各ファイルの呼び出し元を検索
grep -r "SensorBaroFilter\|SensorGPSFilter\|SensorMagFilter" kalman/*.m
grep -r "BiquadFilter" kalman/*.m
grep -r "OutlierGuard" kalman/*.m
```

---

## 6. 実装チェックリスト

### 最小実行可能パス（MVP）

- [ ] Phase 0: MEX 機能確認
- [ ] Phase 1-1: BiquadFilter 削除判定
- [ ] Phase 1-2: OutlierGuard 削除判定
- [ ] Phase 2-1: AccelFilter/SensorAccelFilter を呼び出し元で置き換え＆削除
- [ ] Phase 2-2: DivergenceGuard から MATLAB フォールバック削除
- [ ] テスト: `run_batch_10sets()` で RMSE が変わらないことを確認
- [ ] テスト: `compare_mex_matlab_detailed()` でロールバック確認

---

## 7. 低リスク段階的アプローチ

**原則**:
> 低レイヤーから順番に MEX 化してMATLAB 実装をコメントアウトし、問題が無ければ次へ進む

**各段階での検証**:

```
段階1: 呼び出し元リストを完成させる
    ├─ grep で全参照を抽出
    ├─ 呼び出しがない → 即削除
    └─ 呼び出しあり → MEX 化判定

段階2: MEX ラッパーで MATLAB フォールバック化
    ├─ MATLAB 実装をコメントアウト
    ├─ try-catch で MEX 呼び出しのみ
    └─ 失敗時は error (段階的削除ではなく)

段階3: 回帰テスト（毎段階）
    ├─ run_simulation(42, true)         # 単一テスト
    ├─ run_batch_10sets()               # 10セット
    └─ compare_mex_matlab_detailed()   # 数値確認
        └─ RMSE < 0.30° (Roll/Pitch)
        └─ GPS/Baro/Mag パリティ確認

段階4: クリーンアップ
    ├─ テスト OK なら .m ファイル削除
    └─ 問題 → ロールバック & 原因特定
```

---

## 8. リスク評価と対策

| リスク | 影響度 | 対策 |
|------|------|------|
| MEX の型混在（float32↔float64） | 高 | TYPE_MIX_REPORT.md 確認 → double に統一 |
| NoiseEstimator の状態管理 | 中 | MATLAB 側キャッシュ保持 → MEX と同期 |
| 呼び出し関係の漏れ | 中 | grep で 3 回検索確認 |
| OutlierGuard の直接呼び出し | 低 | 見つからなければ削除 |
| BiquadFilter の使用 | 低 | 見つからなければ削除 |

---

## 9. 最終的な構成イメージ

### ✅ 完了後の構造

```
【削除】
├─ AccelFilter.m              ❌ 削除（直接 SensorFilters に置き換え）
├─ SensorAccelFilter.m        ❌ 削除（直接 SensorFilters に置き換え）
├─ BiquadFilter.m             ❌ 削除（呼び出しなし or MEX 化）
├─ OutlierGuard.m             ❌ 削除 or MEX 統合
├─ SensorBaroFilter.m         ❌ 削除
├─ SensorGPSFilter.m          ❌ 削除
└─ SensorMagFilter.m          ❌ 削除

【保持】
├─ SensorFilters.m            ✅ 保持（MEX ラッパーのみ）
├─ NoiseEstimator.m           ✅ 保持（互換性層 + MEX 委譲）
└─ DivergenceGuard.m          ✅ 保持（MEX 委譲のみ）

【呼び出し元の変更】
├─ run_batch_10sets.m
│  └─ SensorFilters.reset/reset_zero() のまま
├─ ESKF/@ESKF/sensor_updates.m
│  └─ NoiseEstimator.getRnoise() のまま（互換性層で MEX に委譲）
└─ その他
   └─ 不要な中間ラッパー削除 → 直接 SensorFilters 呼び出し
```

---

## 10. 次のステップ

### 即座の調査タスク

```matlab
% Task 1: 各ファイルの呼び出し元リストを完成させる
grep -r "BiquadFilter\|OutlierGuard\|SensorBaroFilter\|SensorGPSFilter\|SensorMagFilter" kalman/*.m

% Task 2: MEX 機能確認
% mex_sensor_filter の全コマンドをリストアップ
% mex_filter_utils の全コマンドをリストアップ

% Task 3: 型混在チェック確認
% TYPE_MIX_REPORT.md を読み込む
```

### 推奨実行順序

1. **呼び出し関係の完全把握** (1-2 時間)
   - grep で最終確認
   - SensorFilters 外のクラスの使用有無判定

2. **BiquadFilter/OutlierGuard の削除 or MEX 化決定** (30 分)
   - 呼び出しなし → 即削除
   - 呼び出しあり → MEX 化スケジュール

3. **AccelFilter/SensorAccelFilter 削除** (1-2 時間)
   - 呼び出し元で直接 SensorFilters に置き換え
   - テスト実行

4. **DivergenceGuard MATLAB フォールバック削除** (1-2 時間)
   - MEX 呼び出しのみに統一
   - 回帰テスト

5. **NoiseEstimator の段階的コメントアウト** (2-4 時間)
   - MATLAB 実装をコメントアウト
   - MEX 同期確認
   - 回帰テスト

---

## 参考資料

- 現在のファイル構成: `FILE_STRUCTURE_AND_MEX_STATUS.md`
- 型混在チェック: `TYPE_MIX_REPORT.md`
- MATLAB/MEX パリティ: `MATLAB_MEX_PARITY_CHECKLIST.md`
- MEX ビルド手順: `kalman/cpp/build/build_mex.m`
