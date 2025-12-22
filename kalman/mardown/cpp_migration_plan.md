# C++ 化移行計画 — 統合ロードマップ

**最終更新**: 2025年12月22日  
**ステータス**: ✅ PHASE 4 完了（MATLAB/MEXパリティ達成）→ 段階的MEX統合へ

---

## 🎯 最終目標

**MATLABの実行関数1つで全処理をMEXで実行する**

```matlab
% 最終形態（目標）
results = mex_run_simulation(seed, skip_data_gen);
```

### 現在の状態 vs 目標

| 項目 | 現在 | 目標 |
|------|------|------|
| メインループ | MATLAB (`run_simulation.m`) | MEX |
| フィルタ計算 | MEX (`mex_meukf_step_v2`) | MEX |
| センサーフィルタ | MEX (`mex_sensor_filter`) | MEX |
| 初期化 | MATLAB (`ESKF.m`) | MEX |
| データ生成 | MATLAB | MATLAB（維持） |
| 可視化 | MATLAB | MATLAB（維持） |

---

## 📊 10/10 PASS 検証結果（2025/12/22）

MATLABフォールバックをコメントアウトしてMEX-onlyで実行した結果：

```
成功: 10/10 (100.0%)
Roll RMSE:  Mean=0.2732°, Std=0.0084°, Max=0.2867°
Pitch RMSE: Mean=0.2834°, Std=0.0053°, Max=0.2898°
Yaw RMSE:   Mean=0.6704°, Std=0.0336°, Max=0.7312°
Position:   Mean=0.5559m, Max=0.5836m
```

**結論**: MEX実装は十分な精度を達成。段階的にMATLABコードを削減可能。

---

## 🏗️ 段階的移行計画

### Phase 5: Eigenライブラリ依存の整理
**目標**: Eigenを使用しない純粋C++配列への移行準備

| タスク | 優先度 | 所要時間 | 状態 |
|--------|--------|---------|------|
| 5.1 C++ 型を `float64` (double) に統一 | 🔴 高 | 1-2h | ⏳ TODO |
| 5.2 Eigen依存箇所の洗い出し | 🔴 高 | 1h | ⏳ TODO |
| 5.3 配列演算の標準C++化検討 | 🟡 中 | 2-3h | ⏳ TODO |

### Phase 6: MATLABラッパー削減
**目標**: MATLAB側の冗長なラッパーコードを削除

| タスク | 優先度 | 所要時間 | 状態 |
|--------|--------|---------|------|
| 6.1 `SensorFilters.m` フォールバック削除 | 🔴 高 | 0.5h | ⏳ TODO |
| 6.2 `SensorAccelFilter.m` 簡素化 | 🟡 中 | 0.5h | ⏳ TODO |
| 6.3 レガシーファイル削除 | 🟢 低 | 0.5h | ⏳ TODO |

### Phase 7: ESKF初期化のMEX化
**目標**: ESKFコンストラクタをC++で実行

| タスク | 優先度 | 所要時間 | 状態 |
|--------|--------|---------|------|
| 7.1 初期化ロジックのC++移植 | 🟡 中 | 2-3h | ⏳ TODO |
| 7.2 `mex_eskf_init` 新規作成 | 🟡 中 | 2h | ⏳ TODO |
| 7.3 `ESKF.m` から呼び出し統合 | 🟡 中 | 1h | ⏳ TODO |

### Phase 8: メインループのMEX化
**目標**: `run_filter()` 全体をMEXで実行

| タスク | 優先度 | 所要時間 | 状態 |
|--------|--------|---------|------|
| 8.1 ループ構造のC++設計 | 🟡 中 | 3-4h | ⏳ TODO |
| 8.2 `mex_run_filter` 実装 | 🟡 中 | 4-5h | ⏳ TODO |
| 8.3 MATLABとの結果比較検証 | 🟡 中 | 2h | ⏳ TODO |

### Phase 9: C++構造の大規模リファクタリング
**目標**: 保守性・拡張性の向上

| タスク | 優先度 | 所要時間 | 状態 |
|--------|--------|---------|------|
| 9.1 ヘッダー依存関係の整理 | 🟢 低 | 2-3h | ⏳ TODO |
| 9.2 名前空間の統一 | 🟢 低 | 1-2h | ⏳ TODO |
| 9.3 単体テストの充実 | 🟢 低 | 4-5h | ⏳ TODO |

---

## 📁 ファイル依存関係

### 現在のレイヤー構造

```
Layer 0: 基本数学関数 ✅ 完了
├── alpha_beta_step      → C++ 実装済
├── ema_update           → C++ 実装済
├── hampel_causal        → C++ 実装済
└── quaternion_lib       → mex_quaternion_lib

Layer 1: 基本フィルタ ✅ 完了
├── BiquadFilter         → sensor_filter.hpp
├── AccelFilter          → sensor_filter.hpp
└── OutlierDetector      → sensor_filter.hpp

Layer 2: センサーフィルタ ✅ 完了
├── SensorFilters.m      → mex_sensor_filter (MEX呼出)
├── NoiseEstimator       → sensor_filter.hpp
└── DivergenceGuard      → sensor_filter.hpp

Layer 3: フィルタコア ✅ 完了
├── predict.m            → mex_meukf_step_v2
├── sensor_updates.m     → mex_meukf_step_v2
└── zupt.m               → mex_meukf_step_v2

Layer 4: ESKFクラス ⏳ Phase 7
├── ESKF.m 初期化        → mex_eskf_init（予定）
└── ESKF.m プロパティ    → C++ State構造体

Layer 5: メインループ ⏳ Phase 8
├── run_filter()         → mex_run_filter（予定）
└── run_simulation.m     → 最終的に薄いラッパーに
```

### 削除候補ファイル

| ファイル | 理由 |
|---------|------|
| `KF/Utils/alpha_beta_step_cpp.m` | C++直接呼出に移行 |
| `KF/Utils/ema_update_cpp.m` | C++直接呼出に移行 |
| `KF/Utils/hampel_causal_cpp.m` | C++直接呼出に移行 |
| `KF/Utils/SensorFilter.m` | SensorFiltersに統合 |
| `KF/Utils/SensorFilterFactory.m` | 不要 |
| `KF/Utils/SensorGyroFilter.m` | 廃止済み |
| `UKF/@UKF/*.m` | ESKFに統合 |
| `EKF/@EKF/*.m` | ESKFに統合 |

---

## 🔧 実装ガイドライン

### 1. Phase移行時の検証手順

```matlab
% 各Phase完了時に必ず実行
cd kalman/cpp/build
build_mex()              % 全体ビルド
clear mex                % キャッシュクリア
cd ../..
run_batch_10sets()       % 10セット検証
% 期待: Roll/Pitch RMSE < 0.30°
```

### 2. C++ コーディング規約

- **型**: `double` を優先（float32 は避ける）
- **メモリ**: スタック割り当て優先、動的割り当ては最小限
- **Eigen**: 段階的に `std::array` への移行を検討
- **エラー処理**: `mexErrMsgIdAndTxt` で明確なエラーメッセージ

### 3. MATLABラッパー削減方針

```matlab
% 悪い例（冗長）
function out = wrapper(in)
    if exist('mex_func','file') == 3
        out = mex_func(in);
    else
        out = matlab_fallback(in);  % ← 削除対象
    end
end

% 良い例（シンプル）
function out = wrapper(in)
    out = mex_func(in);  % MEX必須
end
```

---

## ⚠️ 重要な注意事項

### Eigen廃止の検討

現状のEigen使用箇所:
- `unified_filter.hpp`: 行列演算
- `eskf_core.hpp`: 共分散計算
- `ukf_sigma_points.hpp`: シグマ点生成

**移行オプション**:
1. **Eigen維持**: ビルド複雑だが数学的に安全
2. **標準C++化**: ビルド簡略化、パフォーマンス向上の可能性
3. **ハイブリッド**: クリティカルパスのみ標準C++化

**推奨**: Phase 9 でEigen依存を評価し、段階的に移行

### MATLAB/MEXパリティ維持

- 各Phase完了時に `run_batch_10sets()` で検証
- Roll/Pitch RMSE が 0.30° 以下であることを確認
- 数値差異が大きい場合は即座にロールバック

---

## 📊 完了したPhase

### Phase 0-2: 基本関数C++化 ✅
| 関数/クラス | 状態 | テスト |
|------------|------|--------|
| `alpha_beta_step` | ✅ MEX化 | 10/10 PASS |
| `ema_update` | ✅ MEX化 | 10/10 PASS |
| `hampel_causal` | ✅ MEX化 | 10/10 PASS |
| `BiquadFilter` | ✅ MEX化 | 10/10 PASS |
| `AccelFilter` | ✅ MEX化 | 10/10 PASS |

### Phase 3: センサーフィルタ統合 ✅
| 関数/クラス | 状態 | テスト |
|------------|------|--------|
| `SensorFilters` | ✅ MEX呼出 | 10/10 PASS |
| `NoiseEstimator` | ✅ MEX統合 | 10/10 PASS |
| `DivergenceGuard` | ✅ MEX統合 | 10/10 PASS |
| `OutlierDetector` | ✅ バグ修正済 | 10/10 PASS |

### Phase 4: パリティ検証 ✅
| 項目 | 結果 |
|------|------|
| OutlierDetector修正 | 99.5%→0.3% 外れ値率 |
| Roll/Pitch RMSE | 1.5°→0.27° (83%改善) |
| MATLABフォールバック不要確認 | ✅ 完了 |

---

## 📚 参考リンク

- [FILE_STRUCTURE_AND_MEX_STATUS.md](FILE_STRUCTURE_AND_MEX_STATUS.md) — ファイル構造詳細
- [PHASE4_COMPLETION_REPORT.md](PHASE4_COMPLETION_REPORT.md) — Phase 4 完了レポート
- [MATLAB_MEX_PARITY_CHECKLIST.md](MATLAB_MEX_PARITY_CHECKLIST.md) — パリティチェックリスト
- [PROJECT_STATUS.md](PROJECT_STATUS.md) — 統合ステータス

---

## 🎯 次のアクション

1. **即座に実行**: Phase 6.1 — `SensorFilters.m` のMATLABフォールバック削除
2. **短期**: Phase 5.1 — C++ 型を `float64` に統一
3. **中期**: Phase 7 — ESKF初期化のMEX化
4. **長期**: Phase 8-9 — メインループMEX化とリファクタリング

