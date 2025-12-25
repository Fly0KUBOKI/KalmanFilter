# ESKF MEX移行計画 - エグゼクティブサマリー

**対象**: ESKFクラスのMATLAB実装をC++ MEX化する段階的なロードマップ  
**期間**: 推定 8-14日間  
**目標**: 計算性能向上（1.5倍以上）＋ 数値精度安定化（float統一）

---

## 概要表

| 項目 | 内容 |
|------|------|
| **現状** | MATLAB実装8個（計~1600行）、MEX実装13個 |
| **目標状態** | MATLAB実装2個（薄い分岐のみ）、MEX実装18個 |
| **削減対象** | 厚いMATLAB関数の段階的MEX化 |
| **リスク** | float精度誤差、クォータニオン順序、共分散発散 |
| **検証方法** | `run_simulation()` + `compare_mex_matlab_detailed()` + `run_batch_10sets()` |

---

## 関数MEX化状況マトリックス

```
┌─────────────────────────────────────┬──────────┬──────┬─────────┐
│ 関数名                              │ 現状     │優先度│Phase   │
├─────────────────────────────────────┼──────────┼──────┼─────────┤
│ get_field_impl()                    │ MATLAB   │  低  │ Phase 1 │
│ has_field_impl()                    │ MATLAB   │  低  │ Phase 1 │
│ get_euler_impl()                    │ MATLAB   │  低  │ Phase 1 │
│ divergence_check_velocity_impl()    │ MATLAB   │  中  │ Phase 2 │
│ estimate_noise()                    │ MATLAB   │  低  │ Phase 2 │
│ get_sensor_R()                      │ MATLAB   │  低  │ Phase 2 │
│ reset_sensor_filters()              │ MATLAB   │  低  │ Phase 2 │
│ update_sensor_impl()                │ MATLAB   │  高  │ Phase 3 │
│ do_cpp_update()                     │ MATLAB   │  中  │ Phase 3 │
│ check_and_reset_impl()              │ MATLAB   │  中  │ Phase 5 │
│ reset_filter_impl()                 │ MATLAB   │  低  │ Phase 5 │
│ check_stationary_impl()             │ MATLAB   │  低  │ Phase 5 │
│ update_zupt_impl()                  │ MATLAB   │  低  │ Phase 5 │
│ predict()                           │ MATLAB   │⭐⭐⭐ │ Phase 4 │
├─────────────────────────────────────┼──────────┼──────┼─────────┤
│ call_meukf_step()                   │ MEX呼出  │      │ 既存    │
│ divergence_check()                  │ MEX呼出  │      │ 既存    │
│ utils()/各ユーティリティ            │ MEX呼出  │      │ 既存    │
└─────────────────────────────────────┴──────────┴──────┴─────────┘

⭐⭐⭐ = 最優先（core logic、最大効果）
```

---

## 5段階移行計画ダイアグラム

### 進行フロー

```
【既存状態】
┌────────────────────────────────┐
│ MATLAB: 8関数                  │
│ - predict()         [1600行]   │
│ - update_sensor_impl()         │
│ - do_cpp_update()              │
│ - check_and_reset_impl()       │
│ - divergence_check_velocity... │
│ - zupt(), ...その他            │
│ MEX: mex_meukf_step_v2         │
│      mex_sensor_filter         │
│      ... (13個)                │
└────────────────────────────────┘

        ↓ Phase 1-5進行

【最終状態】
┌────────────────────────────────┐
│ MATLAB: 薄い分岐のみ           │
│ - update_filter() [分岐]       │
│ - sensor_updates() [分岐]      │
│ - reset() [分岐]               │
│ MEX: 18個                      │
│ - mex_matlab_helpers           │
│ - mex_sensor_preprocessor      │
│ - mex_adaptive_predict         │
│ - mex_filter_management        │
│ - mex_meukf_step_v2 [改良]     │
│ - その他既存 (13個)            │
└────────────────────────────────┘
```

### フェーズ進行

```
【Phase 1】基盤層 (1-2日)
  ↓ mex_matlab_helpers作成 (get_field, has_field, get_euler)
  ↓ ビルド ✓
  ↓ run_simulation(42, true) ✓
  ↓ 数値差分確認 (差分=0) ✓

【Phase 2】ユーティリティ拡張 (1-2日)
  ↓ mex_sensor_filter 拡張 (divergence_clip_velocity, ...)
  ↓ ビルド ✓
  ↓ run_simulation(42, true) ✓
  ↓ compare_mex_matlab_detailed() ✓
  ↓ run_batch_10sets() ✓

【Phase 3】センサー前処理 (2-3日)
  ↓ mex_sensor_preprocessor 作成
  ↓ update_sensor_impl() の前処理ロジックをMEX化
  ↓ ビルド ✓
  ↓ run_simulation(42, true) ✓
  ↓ compare_mex_matlab_detailed() (RMS < 1e-4) ✓
  ↓ run_batch_10sets() ✓

【Phase 4】予測ステップ最適化 ⭐⭐⭐ (3-5日)
  ↓ mex_adaptive_predict 作成 (predict()全体をC++化)
  ↓ predict() をMEX呼び出しに置き換え
  ↓ ビルド ✓
  ↓ run_simulation(42, true) ✓
  ↓ compare_mex_matlab_detailed() (詳細検証) ✓
  ↓ run_batch_10sets() (回帰テスト) ✓
  ↓ パフォーマンス測定 (1.5倍高速化?) ✓

【Phase 5】フィルタ管理 (1-2日)
  ↓ mex_filter_management 作成
  ↓ check_and_reset_impl(), zupt() 系をMEX化
  ↓ ビルド ✓
  ↓ run_batch_10sets() ✓
```

---

## MEX関数マッピング表

### Phase 1: 基盤ユーティリティ

| MEX関数 | 機能 | 入力 | 出力 | 依存 |
|--------|------|------|------|------|
| **mex_matlab_helpers** | フィールド抽出・確認 | obs struct | data | fixed_matrix |
|  | get_field() | (struct, names, idx) | vector/scalar | |
|  | has_field() | (struct, names) | bool | |
|  | get_euler() | quat [w,x,y,z] | euler[°] | quaternion_lib |

### Phase 2: センサー・発散ガード拡張

| MEX関数 | 機能 | 前提MEX |
|--------|------|---------|
| **mex_sensor_filter** | センサー異常検知 | 既存 |
| (拡張) | divergence_clip_velocity() | fixed_matrix |
| (拡張) | divergence_regularize_covariance() | fixed_matrix |

### Phase 3: センサー前処理

| MEX関数 | 機能 | 入力 | 出力 |
|--------|------|------|------|
| **mex_sensor_preprocessor** | 変更検知＋前処理 | sensor_meas | (is_outlier, corrected) |
|  | preprocess_accel() | a_meas, prev_a | SensorPreprocessOutput |
|  | preprocess_mag() | m_meas, prev_m | SensorPreprocessOutput |
|  | preprocess_gps() | lat, lon, alt, origin | z_gps |
|  | preprocess_baro() | pressure | z_baro |

### Phase 4: 予測ステップ

| MEX関数 | 機能 | 計算量 | 効果 |
|--------|------|--------|------|
| **mex_adaptive_predict** | IMU積分＋Q適応 | O(n²) → O(1) | **1.5倍高速化** |
| (新規) | predict_step() | state, IMU | state_new |

### Phase 5: フィルタ管理

| MEX関数 | 機能 | 用途 |
|--------|------|------|
| **mex_filter_management** | 発散チェック・リセット | 補助 |
| (新規) | check_divergence() | 状態検証 |
| (新規) | apply_zupt() | 静止更新 |

---

## 実装スケジュール（ガントチャート）

```
Week 1
├─ 2025/12/24 (Wed)  Phase 1 開始
│  ├─ mex_matlab_helpers.cpp 実装
│  └─ ビルド・テスト
├─ 2025/12/25 (Thu)  Phase 2 開始
│  ├─ mex_sensor_filter 拡張
│  └─ テスト
├─ 2025/12/26-27 (Fri-Sat)  Phase 3
│  ├─ mex_sensor_preprocessor 実装
│  └─ 詳細検証
└─ 2025/12/28-30 (Sun-Tue)  Phase 4 ⭐
   ├─ mex_adaptive_predict 実装
   └─ 厳密テスト (compare_mex_matlab_detailed)

Week 2
└─ 2025/12/31-01/02 (Wed-Fri)  Phase 5 + 最終検証
   ├─ mex_filter_management 実装
   └─ run_batch_10sets() 全セット検証
```

---

## 技術スタック

### C++側

```
既存構造を完全に活用:
├─ include/Common/Math/fixed_matrix.hpp
│  └─ Vector<N, float>, Matrix<R,C,float> ✅
├─ include/Common/Math/quaternion.hpp
│  └─ 四元数演算 (multiply, normalize, to_rotm) ✅
├─ include/Common/filter_interface.hpp
│  └─ SensorInput, FilterOutput構造体 ✅
└─ src/ESKF/eskf_core.cpp
   └─ 既存の積分・更新ロジック ✅

新規追加:
├─ src/ESKF/sensor_preprocessor.cpp
├─ src/ESKF/adaptive_predict.cpp
├─ src/ESKF/filter_management.cpp
└─ MEX/mex_*.cpp (4個新規)
```

### ビルドシステム

```
MATLAB: build_mex.m
├─ targets: {..., 'mex_matlab_helpers', 'mex_sensor_preprocessor', ...}
├─ compile_opts: -O -DNDEBUG
├─ include_paths: [.../include, .../src]
└─ output: .../bin/mex_*.mexw64
```

### テストツール

```
検証スクリプト:
├─ run_simulation(seed, verbose) → Results/estimation_*.csv
├─ compare_mex_matlab_detailed() → (diff_pos, diff_vel, diff_att)
├─ run_batch_10sets() → 10回実行・RMS集計
└─ ZUPT検証スクリプト（必要に応じて）
```

---

## 重要な制約・ルール

### ✅ 必須遵守事項

```
型システム:
  ✅ float のみ使用（double禁止）
  ✅ cmath_fx::Vector/Matrix使用
  ✅ Eigen禁止
  ✅ STL vector<double> 禁止

状態ベクトル:
  ✅ [p(3), v(3), q(4), ba(3), bg(3)] = 15要素
  ✅ q = [w, x, y, z] (スカラー先頭)
  ✅ 出力時に P = (P+P')/2 対称化

MEXインタフェース:
  ✅ MATLAB struct → C++型変換は mex_type_conv.hpp
  ✅ NaN/Inf チェック必須
  ✅ クォータニオン正規化必須
```

### ❌ 禁止事項

```
  ❌ Eigen ライブラリ使用
  ❌ double 型使用
  ❌ 動的メモリ割り当て
  ❌ STL コンテナ（vector等）
  ❌ クォータニオン順序の変更 [w,x,y,z]固定
  ❌ 共分散の非対称化のまま出力
```

---

## リスク・対策

| リスク | 確率 | 影響 | 対策 |
|--------|------|------|------|
| float精度不足 | 中 | 高 | Phase 4で compare_mex_matlab_detailed() |
| 共分散発散 | 低 | 高 | 対称化ルール遵守・定期的な run_batch |
| クォータニオン誤り | 低 | 致命 | テンプレートで [w,x,y,z]固定 |
| ビルド失敗 | 低 | 中 | build_mex() エラーハンドリング改善 |
| 数値差異累積 | 中 | 中 | 各フェーズで回帰テスト |

---

## 検証チェックリスト

### Phase 1

- [ ] build_mex({'mex_matlab_helpers'}) 成功
- [ ] mex_matlab_helpers.mexw64 生成確認
- [ ] run_simulation(42, true) 正常終了
- [ ] Results/estimation_0.csv 生成確認
- [ ] MATLAB側の get_field_impl() をMEX呼び出しに置き換え完了

### Phase 2-3

- [ ] 個別MEX ビルド成功
- [ ] run_simulation(42, true) 実行
- [ ] compare_mex_matlab_detailed() で数値確認 (RMS < 1e-4)
- [ ] run_batch_10sets() すべてのセット成功

### Phase 4 ⭐重要⭐

- [ ] mex_adaptive_predict ビルド成功
- [ ] run_simulation(42, true) 正常終了
- [ ] **compare_mex_matlab_detailed() 詳細検証**
  - [ ] position RMS < 1e-3 m
  - [ ] velocity RMS < 1e-4 m/s
  - [ ] attitude RMS < 0.1 deg
  - [ ] NaN/Inf なし
- [ ] run_batch_10sets() すべてのセット成功
- [ ] パフォーマンス測定: >1.4倍高速化

### Phase 5

- [ ] mex_filter_management ビルド成功
- [ ] run_batch_10sets() 最終検証パス
- [ ] 全フェーズのメトリクス記録

---

## 出力ドキュメント一覧

作成されたドキュメント:

1. **ESKF_MATLAB_TO_MEX_MIGRATION_PLAN.md** (このファイルの詳細版)
   - 関数一覧・役割表
   - C++構造分析
   - 5段階移行計画
   - ビルド・テスト戦略

2. **ESKF_MEX_MIGRATION_DETAILED_GUIDE.md**
   - 関数依存グラフ（詳細）
   - Phase別実装ガイド
   - C++コード例（Phase 1-4）
   - テスト検証フロー
   - ビルド設定

3. **ESKF_MIGRATION_EXECUTIVE_SUMMARY.md** (このファイル)
   - エグゼクティブサマリー
   - 進行フロー
   - スケジュール
   - リスク管理

---

## 開始コマンド

```matlab
% Phase 1 開始
cd kalman/cpp/build
build_mex({'mex_matlab_helpers'})
clear mex
cd ../..
run_simulation(42, true)
```

---

**状態**: ⚠️ Phase 1-5のMEXファイルはビルド済みだが、ESKF.mに統合されていない  
**次ステップ**: Phase 6（Phase 1統合）またはPhase 7（Phase 4統合）から開始

---

## 統合フェーズ（Phase 6-10）

**現状の問題**: MEXファイルはビルド済みだが、ESKF.mで使用されていないため、パフォーマンス向上の効果が得られていない。

### 統合優先順位

1. **Phase 7: Phase 4統合（predict）** ⭐⭐⭐ **最重要**
   - 最大のパフォーマンス向上が期待される
   - 推定工数: 2-3日
   - 影響: 1.5倍以上の高速化が期待

2. **Phase 8: Phase 3統合（update_sensor_impl前処理）**
   - 推定工数: 1-2日
   - 影響: センサー前処理の高速化

3. **Phase 9: Phase 5統合（reset, ZUPT）**
   - 推定工数: 1日
   - 影響: フィルタ管理の高速化

4. **Phase 6: Phase 1統合（get_field, has_field）**
   - 推定工数: 0.5日
   - 影響: 軽微な高速化

5. **Phase 10: Phase 2完了（divergence_check_velocity）**
   - 推定工数: 0.5日
   - 影響: 軽微な高速化

### 統合スケジュール（再計画）

```
Week 1
├─ Day 1-2: Phase 7統合（predict）⭐最重要⭐
│  └─ テスト: run_simulation + run_batch_10sets + 数値精度確認
├─ Day 3: Phase 8統合（update_sensor_impl前処理）
│  └─ テスト: run_simulation + run_batch_10sets
└─ Day 4-5: Phase 9統合（reset, ZUPT）
   └─ テスト: run_batch_10sets

Week 2
├─ Day 1: Phase 6統合（get_field, has_field）
└─ Day 2: Phase 10完了（divergence_check_velocity）
   └─ 最終検証: run_batch_10sets + パフォーマンス測定
```

**総推定工数**: 5-7日

