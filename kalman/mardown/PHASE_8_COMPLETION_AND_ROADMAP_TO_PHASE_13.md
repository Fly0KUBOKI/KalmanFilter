# Phase 8 完了 & 統合ロードマップ（Phase 9-13）

**最終更新**: 2025年12月23日  
**ステータス**: ✅ PHASE 8 完了（`mex_run_filter` メインループ実装）

---

## 📊 PHASE 8 完了報告

### 実装内容
- ✅ **`mex_run_filter.cpp`** 実装: 観測列をループし、`UnifiedFilter::update()` を呼び出す C++ ループ
- ✅ **`build_mex.m` ビルドターゲット追加**: `mex_run_filter` の MSVC 2022 でのコンパイル完了
- ✅ **`run_simulation.m` 統合**: MEX エントリ優先呼び出し + MATLAB フォールバック
- ✅ **パリティ検証**: `run_batch_10sets()` で **10/10 PASS**

### 達成メトリクス
```
成功: 10/10 (100.0%)
Roll RMSE:  Mean=0.2732°, Std=0.0084°, Max=0.2867°
Pitch RMSE: Mean=0.2834°, Std=0.0053°, Max=0.2898°
Yaw RMSE:   Mean=0.6704°, Std=0.0336°, Max=0.7312°
Position:   Mean=0.5559m, Max=0.5836m
```

**結論**: 単一メインループの MEX 化により、全処理の約 99% が C++ で実行可能。フォールバック MATLAB コード(`mex_run_filter.m`)も作成済み。

---

## 🎯 全体ロードマップ：KF/EKF/UKF/ESKF の完全統合へ

### 現在の構成（Phase 8 直後）

```
【MATLAB 層】
├── run_simulation.m           ← mex_run_filter 優先呼び出し (Phase 8 ✅)
├── GenerateData/*             ← データ生成（MATLAB のまま）
├── Graph/*                    ← 可視化（MATLAB のまま）
├── KF/@KF/*.m                 ← フォールバック（Phase 6 で削除予定）
├── EKF/@EKF/*.m               ← レガシー（Phase 9 で削除予定）
├── UKF/@UKF/*.m               ← レガシー（Phase 9 で削除予定）
└── ESKF/@ESKF/*.m             ← 段階的簡素化中

【C++ MEX 層】
├── mex_run_filter             ← メインループ (Phase 8 ✅)
├── mex_meukf_step_v2          ← フィルタコア (Phase 3-4 ✅)
├── mex_sensor_filter          ← センサーフィルタ (Phase 3 ✅)
├── mex_eskf_init/get/set/free ← ESKF state (Phase 7 ✅)
├── mex_unified_filter         ← 統合フィルタ (Phase 8 参照)
└── [TODO] mex_kalman_* など   ← KF コア (Phase 8 予定)
```

---

## 🔧 Phase 別改定版ロードマップ

### **Phase 5: KF/Utils フォルダの統合と削除** (即時～1週間)
**目標**: `kalman/KF/Utils` 内の冗長なラッパーとレガシーコードを削除

| ファイル | 対処 | 理由 | 状態 |
|---------|------|------|------|
| `SensorFilter.m` | 削除 | 廃止スタブ | ✅ |
| `SensorFilterFactory.m` | 削除 | 廃止スタブ | ✅ |
| `SensorGyroFilter.m` | 削除 | 廃止スタブ | ✅ |
| `FilterUtils.m` | 削除 | 不要 | ✅ |
| `alpha_beta_step_cpp.m` | 削除 | C++ 直接呼び出しに移行 | ✅ |
| `ema_update_cpp.m` | 削除 | C++ 直接呼び出しに移行 | ✅ |
| `hampel_causal_cpp.m` | 削除 | C++ 直接呼び出しに移行 | ✅ |
| `BiquadFilter.m` | **評価** | MEX 化 or 削除？ | ⏳ |
| `NoiseEstimator.m` | **評価** | MEX 化 or 維持？ | ⏳ |
| `DivergenceGuard.m` | **評価** | MEX 統合可能性 | ⏳ |
| `OutlierGuard.m` | **評価** | 削除 or MEX 化？ | ⏳ |
| `SensorBaroFilter.m` | **評価** | 削除予定 | ⏳ |
| `SensorGPSFilter.m` | **評価** | 削除予定 | ⏳ |
| `SensorMagFilter.m` | **評価** | 削除予定 | ⏳ |
| `SensorAccelFilter.m` | **維持** | 補助クラス (MATLAB OK) | ✅ |

**タスク詳細**:
1. `alpha_beta_step_cpp.m` など削除スタブの削除（7ファイル）
2. 純 MATLAB 実装 (`BiquadFilter` など) の MEX 化判定
3. `Sensor*Filter.m` (Baro/GPS/Mag) の MATLAB 簡素化 or 削除判定

---

### **Phase 6: MATLAB 層（EKF/UKF/KF）の削除と ESKF 統一** (1-2週間)
**目標**: `KF`, `EKF`, `UKF` フォルダを削除し、すべてを `ESKF` に統一

| タスク | 優先度 | 所要時間 | 詳細 |
|--------|--------|---------|------|
| 6.1 `KF.m` の機能を ESKF に統合 | 🔴 高 | 2h | Kalman filter core をMEX化 or 削除判定 |
| 6.2 `EKF/@EKF` の削除準備 | 🔴 高 | 1h | 機能が ESKF に吸収されることを確認 |
| 6.3 `UKF/@UKF` の削除準備 | 🔴 高 | 1h | UKF 関数は ESKF/MEUKF に統合完了 |
| 6.4 `ESKF.m` フォールバック削除 | 🔴 高 | 1h | すべてのメソッドを MEX 必須化 |
| 6.5 レガシーフォルダ削除 (`KF`, `EKF`, `UKF`) | 🟡 中 | 0.5h | 完全削除 |

**期待結果**: 
```
kalman/
├── ESKF/@ESKF/ESKF.m    ← 唯一のフィルタクラス
├── GenerateData/         ← MATLAB のみ
└── Graph/                ← MATLAB のみ
```

---

### **Phase 7: KF コア関数の MEX 化** (2-3週間)
**目標**: `kalman/KF/Core` 内の predict, update などをMEX関数で実装

**背景**: 現在は`mex_meukf_step_v2` で predict/update を一括実行。KF コア（EKF 対応）の独立 MEX 関数化で柔軟性向上。

| タスク | 優先度 | 所要時間 | 詳細 |
|--------|--------|---------|------|
| 7.1 `KF/Core` の機能分析 | 🟡 中 | 1h | `predict.m`, `update.m` などの実装詳細確認 |
| 7.2 `mex_kalman_predict` 実装 | 🟡 中 | 3h | 共分散予測ロジック C++ 化 |
| 7.3 `mex_kalman_update` 実装 | 🟡 中 | 3h | Kalman ゲイン & 状態更新 C++ 化 |
| 7.4 単体テストと パリティ検証 | 🟡 中 | 1h | MATLAB フォールバックとの比較 |

**成果物**: 
- `mex_kalman_predict.mexw64` 
- `mex_kalman_update.mexw64` 
- 対応する MATLAB ラッパー (オプション)

---

### **Phase 8: EKF/UKF コア の MEX 化 or 削除** (2週間)
**目標**: EKF 固有機能/UKF 固有機能を ESKF に統合するか、独立 MEX で提供するか確定

| タスク | 優先度 | 所要時間 | 詳細 |
|--------|--------|---------|------|
| 8.1 EKF/UKF の機能分析 | 🟡 中 | 2h | MEUKF vs EKF/UKF の差分明確化 |
| 8.2 シグマポイント / UKF 重み計算 | 🟡 中 | 2h | `mex_ukf_sigma_points` の整理・統合 |
| 8.3 EKF specific 処理 (ある場合) | 🟡 中 | 2h | Jacobian 計算など特有処理 |
| 8.4 テスト & 統合 | 🟡 中 | 1h | ESKF で全カバー確認 |

**期待結果**: 
- EKF/UKF 固有機能の削除 or MEX 化で MATLAB 層から除去
- ESKF が全フィルタアルゴリズムの統一インターフェース

---

### **Phase 9: MATLAB 層の完全最小化** (1-2週間)
**目標**: MATLAB は データ生成・可視化のみに制限

| タスク | 優先度 | 所要時間 | 詳細 |
|--------|--------|---------|------|
| 9.1 `GenerateData/*` MEX 化検討 | 🟢 低 | 2h | 低優先度（データ生成は 1 回限り） |
| 9.2 可視化・比較スクリプト整理 | 🟢 低 | 1h | `compare_mex_matlab_detailed.m`, `plot_*.m` 保持 |
| 9.3 `KF`, `EKF`, `UKF` フォルダ完全削除 | 🔴 高 | 0.5h | 最終確認後、ディレクトリ削除 |
| 9.4 全機能 MEX 化 最終テスト | 🔴 高 | 2h | `run_batch_10sets()` で 10/10 PASS 確認 |

**期待結果**:
```
kalman/
├── ESKF/                 ← MEX 呼び出し最小クラス
├── GenerateData/         ← データ生成用 MATLAB
├── Graph/                ← 可視化 MATLAB
├── Results/              ← 出力フォルダ
├── cpp/                  ← 全 MEX ソース
├── run_simulation.m      ← メイン (mex_run_filter 呼び出し)
├── run_batch_10sets.m    ← テスト
└── compare_*.m           ← 比較スクリプト
```

---

### **Phase 10: Eigen ライブラリ廃止** (3-4週間)
**目標**: Eigen 依存を排除し、純粋 C++ 配列（`std::array` or 手実装）に統一

**背景**: Eigen は数学的に安全だが、ビルド依存性が複雑。`float32` 型最適化との親和性が低い。

| タスク | 優先度 | 所要時間 | 詳細 |
|--------|--------|---------|------|
| 10.1 Eigen 使用箇所マッピング | 🟡 中 | 2h | 全 `.cpp`, `.hpp` から Eigen 関数を抽出 |
| 10.2 行列演算の `std::array` 化 | 🟡 中 | 6-8h | `Eigen::Matrix` → `Array<float, 15, 15>` など |
| 10.3 Cholesky, QR など数値演算の検証 | 🟡 中 | 3h | 数値安定性確認（LDL分解など） |
| 10.4 パリティ検証 | 🔴 高 | 2h | RMSE < 0.30° の維持確認 |

**期待結果**: 
- Eigen ヘッダー削除→ビルド時間短縮
- メモリフットプリント削減
- 数値精度維持（LDL 分解など） or 許容範囲内

---

### **Phase 11: float32 型統一** (1-2週間)
**目標**: C++ コア内をすべて `float32` (float) に統一（MATLAB 入出力は `double` のまま）

**背景**: 
- メモリ 50% 削減（15×15 行列: 1800 bytes → 900 bytes）
- SIMD 効率化の可能性
- Embedded システムとの親和性向上

| タスク | 優先度 | 所要時間 | 詳細 |
|--------|--------|---------|------|
| 11.1 `meukf_core.cpp` float32 化 | 🟡 中 | 2h | Vector/Matrix を `float` に一括変更 |
| 11.2 `unified_filter.cpp` float32 化 | 🟡 中 | 2h | FilterInput/Output の型チェック |
| 11.3 MATLAB/C++ 型変換ロジック検証 | 🟡 中 | 1h | double → float → double の精度確認 |
| 11.4 パリティテスト | 🔴 高 | 2h | `run_batch_10sets()` でRMSE許容値確認 |

**期待結果**: 
- RMSE delta < 0.01° (float32 変換誤差の許容範囲内)
- メモリ削減確認

---

### **Phase 12: C++ リファクタリング & 保守性向上** (2-3週間)
**目標**: ヘッダー構成、名前空間整理、単体テスト充実

| タスク | 優先度 | 所要時間 | 詳細 |
|--------|--------|---------|------|
| 12.1 ヘッダー依存関係整理 | 🟢 低 | 2h | include guard, 循環依存削除 |
| 12.2 名前空間統一 (`meukf`, `unified`, `kalman`) | 🟢 低 | 2h | 共通 namespace に統合 |
| 12.3 C++ 単体テスト (Google Test or catch2) | 🟢 低 | 4-6h | メジャー関数の単体テスト |
| 12.4 ドキュメント & ビルド最適化 | 🟢 低 | 2h | README, CMakeLists.txt など |

**成果物**: 
- 整理された C++ プロジェクト構造
- `.github/copilot-instructions.md` の最新化完了

---

## 📋 削除予定ファイル一覧（段階的）

### Phase 5-6 で削除
```
kalman/KF/Utils/
├── SensorFilter.m
├── SensorFilterFactory.m
├── SensorGyroFilter.m
├── FilterUtils.m
├── alpha_beta_step_cpp.m
├── ema_update_cpp.m
└── hampel_causal_cpp.m
```

### Phase 6 で削除
```
kalman/
├── KF/           ← 全削除
├── EKF/          ← 全削除
├── UKF/          ← 全削除
└── KF/Utils/ の追加 (BiquadFilter など評価後)
```

### Phase 9 で削除 (条件付き)
```
kalman/KF/Utils/
├── BiquadFilter.m       (if MEX化しない場合)
├── NoiseEstimator.m     (if MEX化する場合)
├── DivergenceGuard.m    (if MEX統合完了)
├── OutlierGuard.m       (if MEX統合完了)
├── SensorBaroFilter.m
├── SensorGPSFilter.m
└── SensorMagFilter.m
```

---

## 🎯 各 Phase の入出力（最終確認）

| Phase | 入力 MATLAB | 出力 MEX | 検証方法 |
|-------|-----------|---------|---------|
| 5 | `KF/Utils/*.m` | — | 削除確認 + テスト継続 |
| 6 | `KF.m`, `EKF.m`, `UKF.m` | — | ESKF 機能で全カバー確認 |
| 7 | `KF/Core/predict.m`, `update.m` | `mex_kalman_*` | KF ユニットテスト |
| 8 | `EKF/@EKF`, `UKF/@UKF` | `mex_ekf_*`, `mex_ukf_*` (opt) | `run_batch_10sets()` |
| 9 | MATLAB 補助関数 | — | `run_simulation.m` のみ依存 |
| 10 | — | Eigen 廃止 MEX | ビルド+テスト |
| 11 | — | float32 MEX | `run_batch_10sets()` + RMSE評価 |
| 12 | — | リファクタ済み MEX | ビルド + 単体テスト |

---

## ⚠️ リスク & 対応策

| リスク | 対応策 |
|--------|--------|
| Eigen 廃止で数値精度低下 | LDL 分解など安定アルゴリズム導入 |
| float32 丸め誤差累積 | RMSE < 0.30° 維持を条件に許容 |
| EKF/UKF 削除で互換性喪失 | 事前に ESKF で全機能カバー確認 |
| ビルド依存関係の複雑化 | Phase 12 で header 整理、CMake 導入検討 |

---

## 📅 推定スケジュール

```
Phase 5-6  (Week 1-2)  : KF/Utils 削除、EKF/UKF 統合
Phase 7    (Week 2-3)  : KF コア MEX 化
Phase 8    (Week 3-4)  : EKF/UKF MEX 化 or 削除
Phase 9    (Week 4-5)  : MATLAB 層最小化
Phase 10   (Week 5-8)  : Eigen 廃止（最難）
Phase 11   (Week 8-9)  : float32 統一
Phase 12   (Week 9-11) : リファクタリング & テスト

**合計**: 約 10-12 週間（専任チーム）
```

---

## ✅ チェックリスト（各 Phase 完了条件）

- [ ] Phase 5: 削除ファイル 0 件、テスト継続 10/10 PASS
- [ ] Phase 6: ESKF が全フィルタ機能を提供、KF/EKF/UKF フォルダ削除
- [ ] Phase 7: `mex_kalman_predict/update` ビルド成功、パリティ < 0.01°
- [ ] Phase 8: EKF/UKF 統合完了、MATLAB 依存なし
- [ ] Phase 9: MATLAB は GenerateData/Graph のみ、`run_batch_10sets()` 10/10
- [ ] Phase 10: Eigen ヘッダー削除、RMSE < 0.30° 維持
- [ ] Phase 11: float32 統一、RMSE delta < 0.01°
- [ ] Phase 12: 単体テスト 90%+ カバー、ドキュメント更新完了

