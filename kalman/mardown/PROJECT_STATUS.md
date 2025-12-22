# KalmanFilter プロジェクト — 統合ステータスレポート

**最終更新**: 2025年12月22日  
**プロジェクトステータス**: ✅ **PHASE 4 完了** — OutlierDetector バグ修正・MATLAB/MEX パリティ達成

---

## 📋 概要

MATLAB + C++ MEX ハイブリッド実装による高精度カルマンフィルタ（ESKF/MEUKF）開発プロジェクト。MATLAB 実装と C++ 実装の完全なパリティ達成を目標。

### 現在の成果
- ✅ **PHASE 0-2**: C++ 基本関数・フィルタの MEX 化完了
- ✅ **PHASE 3 準備**: NoiseEstimator / DivergenceGuard の C++ 実装確認
- ✅ **PHASE 4 完了**: OutlierDetector の 2 つのバグ修正 → MATLAB/MEX パリティ達成 🎉
  - 修正内容: `noise_std` 計算と外れ値履歴管理の不一致を解決
  - 結果: Roll/Pitch RMSE 1.5° → 0.27°（83% 改善）
  - 検証: 10セット バッチテスト 100% PASS

---

## � PHASE 4 実施内容と結果（2025/12/22）

### 問題特定
**症状**: MEX実装が MATLAB比で 5-6倍精度が低い（Roll/Pitch RMSE 1.5° vs 0.27°）

**根本原因**: C++ `OutlierDetector::detect()` に 2 つのバグが存在
1. `noise_std` の下限計算が不完全（`residual_norm / 3.0` が欠落）
2. 外れ値も履歴に追加（MATLAB では外れ値を除外）

**定量的影響**: 999/1000 サンプル（99.5%）が不正に外れ値として判定

### 修正実装
- [sensor_filter.hpp L279](../cpp/include/Common/Sensor/sensor_filter.hpp#L279): `noise_std` 計算を MATLAB と同一に
- [sensor_filter.hpp L285-293](../cpp/include/Common/Sensor/sensor_filter.hpp#L285): 外れ値を履歴から除外

### 検証結果
✅ **10セット バッチテスト: 100% PASS（10/10）**

| 指標 | 修正前 | 修正後 | 期待値(MATLAB) | 判定 |
|------|--------|--------|----------------|------|
| Roll RMSE | 1.5-1.7° | **0.2732°** | 0.2734° | ✅ 一致 |
| Pitch RMSE | 1.5-1.7° | **0.2834°** | 0.2835° | ✅ 一致 |
| 外れ値率 | 99.5% | **0.3%** | 0.3% | ✅ 一致 |
| バッチ合格率 | 0% | **100%** | 100% | ✅ 一致 |

---

## 🔴 過去に特定した問題（PHASE 4 前半）

### 初期診断: 浮動小数点精度（32-bit 誤検）
当初は GPS Kalman Gain の誤差を浮動小数点精度の問題と判断していたが、実際の根本原因は OutlierDetector のロジック不一致だった。

**教訓**: 数値差異が見られた際は、アルゴリズムレベルの差異を先に確認すべき

---

## 📊 プロジェクト構成

```
┌─────────────────────────────────────┐
│  MATLAB 層 (double64 精度)          │
│  ├─ run_simulation.m                │
│  ├─ run_batch_10sets.m              │
│  └─ ESKF/@ESKF/*                    │
├─────────────────────────────────────┤
│  MEX インターフェース                │
│  └─ mex_meukf_step_v2.mexw64        │
├─────────────────────────────────────┤
│  C++ 層 (現在: float32)             │
│  ├─ meukf_core.cpp                  │
│  ├─ sensor_filter.hpp               │
│  └─ kalman/cpp/build/build_mex.m    │
└─────────────────────────────────────┘
```

### 状態ベクトル（15次元）
```
x = [位置(3), 速度(3), クォータニオン(4), 
     加速度計バイアス(3), ジャイロバイアス(3)]
```

---

## ✅ 完了した作業

### Phase 0-2: C++化移行（完了）
- ✅ 基本関数 (`alpha_beta_step`, `ema_update`, `hampel_causal`) MEX 化
- ✅ `BiquadFilter`, `AccelFilter` MEX 化
- ✅ `SensorGyroFilter` 廃止
- ✅ バッチテスト 10/10 PASS 検証済み

### Phase 3 準備: 統合テスト設計
- ✅ `NoiseEstimator` / `DivergenceGuard` C++ 実装確認
- ✅ MATLAB 側初期化復帰（バグ修正）
- ✅ `run_batch_10sets` で 10/10 PASS 確認

### Phase 4: 根本原因分析（完了）
- ✅ GPS debug 出力収集 (20.4 MB)
- ✅ MEX 側 Kalman Gain 抽出
- ✅ MATLAB 側 Kalman Gain 抽出
- ✅ 浮動小数点精度差を定量化
- ✅ **根本原因特定**: float32 vs float64

---

## 📋 現在のクリーンアップ状況（2025-12-21）

### 削除した主要ファイル

**デバッグツール & 分析スクリプト** (12個)
- `run_simulation_with_debug.m`, `run_noise_estimate_debug.m`
- `analyze_parity_phase4.m`, `phase4_run_debug_sim.m`
- その他 GPS/ノイズ推定デバッグスクリプト

**詳細なドキュメント** (20個)
- `PHASE0_COMPLETE.md`, `PHASE1_COMPLETE.md`, `PHASE2_ROOT_CAUSE_AND_ACTIONS.md`
- `PHASE3_PARITY_ANALYSIS_SUMMARY.md`, `PHASE4_ROOT_CAUSE_GPS_K_ANALYSIS.md`
- `PHASE4_ACTION_PLAN.md`, `PHASE4_PROGRESS_REPORT.md`
- その他 `*_consolidated.md`, `*_analysis_summary.md` など

### 保持したコア ドキュメント

| ファイル | 用途 |
|---------|------|
| `PROJECT_STATUS.md` | 📊 本レポート（統合版） |
| `cpp_migration_plan.md` | 🔧 C++化移行の現在進行中タスク |
| `CLEANUP_FINAL_2025_12_21.md` | 📝 クリーンアップ詳細記録 |
| `PROJECT_STRUCTURE_AND_CLEANUP_PLAN.md` | 📐 プロジェクト構造・整理計画 |
| `phase3_migration_notes.md` | 📌 Phase 3 実装ノート |

---

## 🔧 次のアクション（推奨順序）

### 短期（1-2 時間）
1. **C++ 精度を double に変更**
   ```cpp
   // kalman/cpp/MEUKF/meukf_types.hpp
   typedef Eigen::Matrix<double, 15, 15> Matrix15x15;  // float → double
   ```
2. ビルドとテスト
   ```matlab
   cd kalman/cpp/build
   build_mex('mex_meukf_step')
   clear mex
   run_batch_10sets()
   ```
3. 検証: GPS 速度更新の誤差が `< 1e-10 m/s` に低下することを確認

### 中期（Phase 3 続行）
1. NoiseEstimator / DivergenceGuard のインターフェース突合
2. MEX API の不足メソッド実装
3. 単体テスト追加 (`kalman/cpp/tests/`)
4. フルバッチ回帰検証

### 長期（Phase 5+）
1. 残る Phase 4+ の C++化移行
2. 性能最適化（必要に応じて）

---

## 📈 性能指標（最後の PASS 結果）

```
Batch: 10 seeds × 1 filter
Result: 10/10 PASS ✅

Position RMSE: 0.50-0.59 m (許容範囲内)
Roll/Pitch RMSE: 0.27-0.30 deg (許容範囲内)
```

---

## 🏗️ 現在のファイル構成（`kalman/mardown/`）

```
mardown/
├── PROJECT_STATUS.md               ← 本ファイル
├── cpp_migration_plan.md           ← 移行計画（進行中）
├── CLEANUP_FINAL_2025_12_21.md     ← クリーンアップ詳細
├── PROJECT_STRUCTURE_AND_CLEANUP_PLAN.md
├── phase3_migration_notes.md       ← Phase 3 ノート
└── プロンプト/                      ← 開発ノート（参考）
```

---

## 📝 重要な技術ノート

### 状態ベクトル順序
- Quaternion は **スカラー先頭**: `q_w, q_x, q_y, q_z`
- 状態: `[p(3), v(3), q(4), ba(3), bg(3)]` = 15次元

### MEX 呼び出しパターン
```matlab
[x_new, P_new] = mex_meukf_step_v2(x, P, u, sensor_data, params)
```

### 共分散対称性
更新後は対称化: `P = (P + P')/2`

### クォータニオン正規化
C++ 側で実施（MATLAB 側で二重正規化しない）

---

## ✨ 知見・教訓

1. **32-bit vs 64-bit**: リアルタイムシステムでも精度差が問題になる
2. **段階的テスト**: `run_batch_10sets` のような回帰テストが問題早期発見に有効
3. **ドキュメント管理**: 詳細ログは削除し、重要な結果のみを記録すべき

---

**補足**: 詳細な実装やテスト手順は `cpp_migration_plan.md` と `CLEANUP_FINAL_2025_12_21.md` を参照してください。
