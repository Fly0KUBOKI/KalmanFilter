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
<!-- | 5.1 C++ 型を `float64` (double) に統一 | 🔴 高 | 1-2h | ⏳ TODO | -->
float32に統一してほしい
| 5.2 Eigen依存箇所の洗い出し | 🔴 高 | 1h | ⏳ TODO |
| 5.3 配列演算の標準C++化検討 | 🟡 中 | 2-3h | ⏳ TODO |

### Phase 6: MATLABラッパー削減
**目標**: MATLAB側の冗長なラッパーコードを削除

| タスク | 優先度 | 所要時間 | 状態 |
|--------|--------|---------|------|
| 6.1 `SensorFilters.m` フォールバック削除 | 🔴 高 | 0.5h | ✅ 完了 |
| 6.2 `SensorAccelFilter.m` 簡素化 | 🟡 中 | 0.5h | ⏳ 部分 |
| 6.3 レガシーファイル削除 | 🟢 低 | 0.5h | ✅ 完了 |

**Phase 6 実装状況**: 🟡 70% 完了
- ✅ 完了: DEPRECATED ファイル削除（`SensorFilter.m`, `SensorFilterFactory.m`, `SensorGyroFilter.m`）
- ✅ 完了: `alpha_beta_step_cpp.m`, `ema_update_cpp.m`, `hampel_causal_cpp.m` の冗長ラッパー削除
- ✅ 完了: `FilterUtils.m` の削除
- ⏳ 残存: `BiquadFilter.m`, `NoiseEstimator.m`, `DivergenceGuard.m` の純 MATLAB 実装（MEXへの移行待ち）

### Phase 7: ESKF初期化のMEX化
**目標**: ESKFコンストラクタをC++で実行

| タスク | 優先度 | 所要時間 | 状態 |
|--------|--------|---------|------|
| 7.1 初期化ロジックのC++移植 | 🟡 中 | 2-3h | ✅ 完了 |
| 7.2 `mex_eskf_init` 新規作成 | 🟡 中 | 2h | ✅ 完了 |
| 7.3 `ESKF.m` から呼び出し統合 | 🟡 中 | 1h | ✅ 完了 |

**Phase 7 実装状況**: ✅ 100% 完了
- ✅ 完了: `mex_eskf_init`, `mex_eskf_get_state`, `mex_eskf_set_state`, `mex_eskf_free` の実装
- ✅ 完了: MATLAB `ESKF.m` への統合（ハンドルベース state）
- ✅ 完了: `mex_eskf_step_handle` による中間ステップ実装

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

## � KF/Utils フォルダの現状分析（2025/12/23 更新）

### ファイル構成と実装状況一覧

| ファイル | 役割 | 実装タイプ | MEX委譲 | 状態 |
|---------|------|----------|--------|------|
| `SensorFilters.m` | 統一センサーフィルタラッパー | MEXラッパー | Yes | ✅ 完了（必須） |
| `alpha_beta_step.m` | 1D α-β トラッカー | MEX+MATLAB | Yes | ✅ MEX可、MATLAB FBあり |
| `ema_update.m` | 指数移動平均 | MEX+MATLAB | Yes | ✅ MEX可、MATLAB FBあり |
| `hampel_causal.m` | 因果Hampel外れ値補正 | MEX+MATLAB | Yes | ✅ MEX可、MATLAB FBあり |
| `BiquadFilter.m` | 2次IIRフィルタ | 純MATLAB | No | ⏳ MEX化待ち |
| `NoiseEstimator.m` | センサー雑音推定 | 純MATLAB | Partial | ⏳ MEX化待ち |
| `DivergenceGuard.m` | 発散防止・イノベーション制限 | MEX+MATLAB | Partial | ✅ MEX可、MATLAB FBあり |
| `OutlierGuard.m` | 外れ値判定・統合ラッパー | 混合 | Partial | ✅ MEX可、MATLAB FBあり |
| `AccelFilter.m` | 加速度計専用フィルタ | MEX委譲 | Yes | ✅ 完了（`mex_sensor_filter`） |
| `SensorAccelFilter.m` | 加速度フィルタクラス | MEX委譲 | Yes | ✅ 完了 |
| `SensorBaroFilter.m` | 気圧計フィルタ | 純MATLAB | No | ⏳ MEX化待ち |
| `SensorGPSFilter.m` | GPS フィルタ | 純MATLAB | No | ⏳ MEX化待ち |
| `SensorMagFilter.m` | 磁気計フィルタ | 純MATLAB | No | ⏳ MEX化待ち |
| `SensorFilter.m` | 廃止（スタブ） | — | — | ✅ 削除予定 |
| `SensorFilterFactory.m` | 廃止（スタブ） | — | — | ✅ 削除予定 |
| `SensorGyroFilter.m` | 廃止（スタブ） | — | — | ✅ 削除予定 |
| `FilterUtils.m` | 廃止（スタブ） | — | — | ✅ 削除予定 |

### 残存する純 MATLAB 実装の詳細

#### 1. **BiquadFilter.m** (109行)
- **用途**: 2次ローパスIIRフィルタ（低遅延、位相特性良好）
- **MATLAB実装**: 完全に独立（MEXなし）
- **移行難易度**: 🟡 中（行列演算少、状態変数のみ）
- **提案**: MEXで実装可能だが、処理が軽量なため低優先度

#### 2. **NoiseEstimator.m** (287行)
- **用途**: センサー雑音の逐次推定（EMA、外れ値除外）
- **MATLAB実装**: 主に MATLAB（`estimate()` メソッドで逐次計算）
- **MEX連携**: オプション（`mex_sensor_filter('reset')` 呼び出しあり）
- **移行難易度**: 🟡 中（統計計算が主体）
- **提案**: 現状 MATLAB でOK（パフォーマンス非クリティカル）

#### 3. **DivergenceGuard.m** (230行)
- **用途**: フィルタ発散防止（イノベーション制限、ゲインクランプ、P 正規化）
- **MATLAB実装**: 複数メソッドあり（`check_and_attenuate_update()` → MEX、`regularize_covariance()` → MEX、`clip_*()` → MATLAB）
- **MEX連携**: 部分的（`SensorFilters` 経由で MEX 呼び出し）
- **移行難易度**: 🔴 高（複数の数値安定化ロジック混在）
- **提案**: 既存 MEX（`mex_sensor_filter('divergence_*')`）に統合（優先度低）

#### 4. **OutlierGuard.m** (400+行)
- **用途**: 外れ値判定・Mahalanobis距離・イノベーション加重の統合ラッパー
- **MATLAB実装**: 複雑な条件判定とベクトル演算
- **MEX連携**: 部分的（`SensorFilters.filterInnovation()` → MEX）
- **移行難易度**: 🔴 高（複数の条件判定とパラメータ調整）
- **提案**: 優先度低（複雑性高、呼び出し頻度低）

#### 5. **SensorBaroFilter.m**, **SensorGPSFilter.m**, **SensorMagFilter.m** (各100-200行)
- **用途**: 各センサー専用フィルタ（EMA平滑化、外れ値検出、ノルム検証等）
- **MATLAB実装**: 各センサーの特性に応じたロジック
- **MEX連携**: 多くは `SensorFilters` へ委譲可能
- **移行難易度**: 🟡 中
- **提案**: 統合可能だが、現状 MATLAB でも十分（低優先度）

### 推奨される削除順序（安全性優先）

```
【即座に削除】
1. SensorFilter.m        → 廃止（スタブ化済み）
2. SensorFilterFactory.m → 廃止（スタブ化済み）
3. SensorGyroFilter.m    → 廃止（スタブ化済み）

【統合・簡素化候補】（Phase 9 以降）
4. BiquadFilter.m        → MEX化 or 削除（低優先度）
5. OutlierGuard.m        → MEX化 or 簡素化（複雑度高）
6. DivergenceGuard.m     → 既存 MEX に統合（部分実装）

【維持推奨】
7. NoiseEstimator.m      → MATLAB でOK（パフォーマンス非問題）
8. SensorAccelFilter.m   → 補助クラス（保持）
```

---

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

## 🎯 次のアクション（Phase 8 以降）

### 即座（Phase 8: メインループMEX化）

1. **Phase 8.1**: メインループの構造設計
   - 入力: センサーデータ、初期 state
   - 出力: 最終 state、推定値
   - 処理: 予測→更新ループ（1600ステップ）

2. **Phase 8.2**: `mex_run_filter` C++実装
   - 既存 `mex_meukf_step_v2` をベースにループ化
   - 初期化は `mex_eskf_init` を呼び出し
   - センサーフィルタは `mex_sensor_filter` に委譲

3. **Phase 8.3**: MATLAB/MEX パリティ検証
   - `run_batch_10sets()` で 10/10 PASS を確認
   - 数値誤差 <0.01° (Roll/Pitch RMSE)

### 短期（Phase 9: C++ リファクタリング）

4. **Phase 9.1-9.3**: 構造整理（低優先度）
   - ヘッダー依存の整理
   - Eigen 廃止の検討
   - 単体テスト充実

### 残存する MATLAB 実装の処遇

| ファイル | 次のステップ | 理由 |
|---------|-----------|------|
| `BiquadFilter.m` | Phase 9 で要検討 | 低優先度、処理軽量 |
| `NoiseEstimator.m` | 維持 | パフォーマンス非問題 |
| `DivergenceGuard.m` | Phase 9 で最適化 | 既存 MEX と統合可能 |
| `OutlierGuard.m` | 部分 MEX 化検討 | 複雑度高、低優先度 |
| `Sensor*Filter.m` (Baro,GPS,Mag) | Phase 9 で統合 | 後回し可 |

