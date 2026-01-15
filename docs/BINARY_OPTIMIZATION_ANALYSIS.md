# バイナリ最適化分析レポート

**作成日**: 2026年1月13日  
**分析対象**: `kalman/cpp/bin/mex_hybrid_filter.mexw64` (347 KB)  
**目標**: バイナリサイズ削減＆不要コード除外

---

## 1. 現状分析

### 1.1 ファイル構成の問題点

#### 重複フォルダ構造
```
Lib/
├── Common/
│   └── inc/
│       ├── Math/              ← メイン実装
│       ├── Sensor/            ← メイン実装
│       └── Validation/
│
├── Core/                       ⚠️ 古い wrapper（削除対象）
│   ├── geometry.hpp           → ../Common/inc/Math/geometry.hpp へ redirect 失敗
│   ├── math_utils.hpp
│   ├── statistics.hpp
│   └── ... (全 7 ファイル)    
│
└── Sensor/                     ⚠️ Lib/Sensor と完全重複
    ├── ema_filter.hpp
    ├── biquad_filter.hpp
    └── ... (全 7 ファイル)
```

**発見**:
- `Lib/Core/` は `#include "../../../Core/"` で外部フォルダを参照しようとしているが、そのフォルダは存在しない
- `Lib/Sensor/` は `Lib/Sensor/` に実体が存在する（重複の統一を推奨）
- **結論**: Core は削除、Sensor は一方に統一すべき

#### 未使用テンプレートライブラリ
```
Lib/EKF/inc/
├── ekf_core.hpp             ← MEXで使用されていない（ヘッダーのみ）
└── ... (.cpp なし)

Lib/UKF/inc/
├── ukf_sigma_points.hpp     ← MEXで使用されていない（ヘッダーのみ）
├── ukf_update.hpp
└── ... (.cpp なし)

Lib/KF/inc/
├── kalman_filter_core.hpp   ← 基本テンプレート（呼び出し不明）
├── kf_core.hpp
└── kf_operations.hpp
```

**結論**: KF は ESKF/MEUKF が間接参照している可能性あり。EKF/UKF は明らかに未使用。

---

### 1.2 MEX ビルド対象分析

#### build_mex.m の mex_targets リスト

```matlab
mex_targets = {
    {'mex_hybrid_filter.cpp', {
        % ESKF実装
        'Common/src/filter_mgmt.cpp'
        'ESKF/src/eskf_postprocess.cpp'
        'ESKF/src/eskf_core.cpp'              ✅ 使用確認
        'ESKF/src/eskf_math.cpp'              ✅ 使用確認
        'ESKF/src/eskf_sensor_updates.cpp'    ✅ 使用確認
        'Common/src/Sensor/sensor_preprocessor.cpp' ✅ 使用確認
        'ESKF/src/eskf_runner.cpp'            ✅ 使用確認
        'ESKF/src/eskf_initializer.cpp'       ✅ 使用確認
        'mex_eskf_initializer.cpp'            ✅ 使用確認
        
        % MEUKF実装
        'MEUKF/src/meukf_core.cpp'            ✅ 使用確認（1346行、要分割）
        'MEUKF/src/meukf_predict.cpp'         ✅ 使用確認
        'MEUKF/src/meukf_sigma_points.cpp'    ✅ 使用確認
        'MEUKF/src/meukf_update.cpp'          ✅ 使用確認
        
        % 定数・統計
        'Common/src/robust_statistics_constants.cpp' ✅ 使用確認
        'Common/src/sensor_filter_constants.cpp'     ✅ 使用確認
        'Common/src/math_utils_constants.cpp'        ✅ 使用確認
        'MEUKF/src/meukf_observation_models_constants.cpp' ✅ 使用確認
    }, 'mex_hybrid_filter'}
};
```

**使用状況**:
- ✅ **17 個全て** が実際に MEX のホットパスで呼び出されている
- 削除対象：なし（ビルドリストは最適済み）

---

### 1.3 インクルード分析

#### MEX エントリーポイント(`mex_hybrid_filter_common.hpp`) で include されているファイル

```cpp
// 実際に include されているもの
#include "../../Lib/Matrix/fixed_matrix.hpp"                    ✅ 必須
#include "../../Lib/Common/inc/Math/vector_utils.hpp"          ✅ 必須
#include "../../Lib/Quaternion/quaternion_functions.hpp"       ✅ 必須
#include "../../Lib/Common/inc/Math/statistics.hpp"            ✅ 必須
#include "../../Lib/ESKF/inc/eskf_core.hpp"                    ✅ 必須
#include "../../Lib/ESKF/inc/eskf_postprocess.hpp"             ✅ 必須
#include "../../Lib/ESKF/inc/eskf_state.hpp"                   ✅ 必須
#include "../../Lib/ESKF/inc/eskf_runner.hpp"                  ✅ 必須
#include "../../Lib/Common/inc/filter_mgmt.hpp"                ✅ 必須
#include "../../Lib/Sensor/sensor_filter.hpp"                  ✅ 必須（本体）
#include "../../Lib/Sensor/sensor_preprocessor.hpp"            ✅ 必須
#include "../../Lib/ESKF/inc/eskf_sensor_updates.hpp"          ✅ 必須

// 未使用ヘッダー（推定）
#include "../../Lib/EKF/inc/ekf_core.hpp"                      ❓ 未検出
#include "../../Lib/UKF/inc/ukf_sigma_points.hpp"              ❓ 未検出
#include "../../Lib/KF/inc/kalman_filter_core.hpp"             ❓ 間接参照の可能性
```

**未使用の可能性あり**:
- **Lib/EKF**: メインパスから呼ばれていない（KF テンプレートのみ）
- **Lib/UKF**: メインパスから呼ばれていない（KF テンプレートのみ）

---

## 2. 改善提案

### 2.1 即座に実行可能な削減（容易）

#### A. 古い Lib/Core フォルダを削除
- **対象**: `kalman/cpp/Lib/Core/` フォルダ全体（7ファイル、約5KB）
- **理由**: 機能しない redirect wrapper（`../../../Core/` は存在しない）
- **影響**: なし（実際に include されていない）
- **削減効果**: 最小限（ビルド効率わずかに向上）

#### B. Lib/Sensor フォルダを統一（推奨）
- **対象**: `kalman/cpp/Lib/Sensor/` （7 ファイル）
- **統合先**: `kalman/cpp/Lib/Sensor/`（推奨）
- **理由**: 運用の単純化と明確なヘッダ配置
- **影響**: ドキュメントとサンプルのインクルードパスを `Lib/Sensor/` に更新する必要あり
- **削減効果**: 最小限（ヘッダーのみ、リンク対象外）

### 2.2 段階的改善（要検証）

#### C. Lib/EKF, Lib/UKF の削除（未使用テンプレート）
- **対象**: 
  - `kalman/cpp/Lib/EKF/inc/` (3 ファイル)
  - `kalman/cpp/Lib/UKF/inc/` (3 ファイル)
- **理由**: .cpp 実装なし、MEXで呼び出されていない
- **検証方法**: 
  ```bash
  # ESKF/MEUKF が EKF/UKF を参照していないか確認
  grep -r "ekf_core\|ukf_" kalman/cpp/Lib/ESKF kalman/cpp/Lib/MEUKF --include="*.hpp"
  ```
- **削減効果**: 最小限（テンプレートのみ）

#### D. Lib/KF の詳細分析
- **対象**: `kalman/cpp/Lib/KF/inc/`
- **現状**: ESKF/MEUKF が KF テンプレートを参照している可能性あり
- **検証必須**: `grep -r "KF::" kalman/cpp/Lib` で使用確認

### 2.3 本質的な最適化（高効果）

#### E. メモリ層の削減（最大効果）
- **原因**: `-s` フラグで symbol strip されているが、デバッグシンボル以外にもテンプレート膨張がある
- **対策**: 
  1. `meukf_core.cpp` (1346行) の分割 → 関数粒度の最適化が可能
  2. テンプレートの instantiation 数削減（複数の型・サイズの重複 instantiation を回避）
  3. Inline function の見直し

---

## 3. ファイル構成の推奨状態

### 推奨ファイル構成

```
Lib/
├── Common/                            ← メイン共通ライブラリ
│   ├── inc/
│   │   ├── filter_mgmt.hpp           ← ZUPT, 発散検出, 共分散管理
│   │   ├── interface.hpp              ← MATLAB struct 定義
│   │   ├── Math/
│   │   │   ├── geometry.hpp
│   │   │   ├── math_utils.hpp
│   │   │   ├── statistics.hpp
│   │   │   ├── vector_utils.hpp
│   │   │   ├── mahalanobis.hpp
│   │   │   ├── numerical.hpp
│   │   │   └── portable_math.hpp
│   │   ├── Sensor/                   ← 完全な実装
│   │   │   ├── alpha_beta_filter.hpp
│   │   │   ├── biquad_filter.hpp
│   │   │   ├── ema_filter.hpp
│   │   │   ├── outlier_detector.hpp
│   │   │   ├── robust_statistics.hpp
│   │   │   ├── sensor_filter.hpp
│   │   │   └── sensor_preprocessor.hpp
│   │   └── Validation/
│   │       └── validation.hpp
│   └── src/
│       ├── filter_mgmt.cpp
│       ├── robust_statistics_constants.cpp
│       ├── sensor_filter_constants.cpp
│       ├── math_utils_constants.cpp
│       └── Sensor/
│           └── sensor_preprocessor.cpp
│
├── ESKF/                              ← メインフィルタ実装
│   ├── inc/
│   │   ├── eskf_core.hpp             ← 予測・更新アルゴリズム
│   │   ├── eskf_runner.hpp            ← MEX 型変換ランナー
│   │   ├── eskf_state.hpp             ← 状態定義
│   │   ├── eskf_sensor_updates.hpp    ← センサー更新
│   │   ├── eskf_math.hpp              ← 数学関数
│   │   ├── eskf_postprocess.hpp       ← 出力後処理
│   │   ├── eskf_initializer.hpp       ← 初期化
│   │   ├── eskf_filter.hpp
│   │   ├── eskf_helper.hpp
│   │   └── filter.hpp
│   └── src/
│       ├── eskf_core.cpp
│       ├── eskf_runner.cpp
│       ├── eskf_sensor_updates.cpp
│       ├── eskf_math.cpp
│       ├── eskf_postprocess.cpp
│       ├── eskf_initializer.cpp
│       └── filter.cpp
│
├── MEUKF/                             ← 補助フィルタ実装
│   ├── inc/
│   │   ├── meukf_core.hpp            ← UKF 核（分割予定）
│   │   ├── meukf_helpers.hpp
│   │   ├── meukf_observation_models.hpp
│   │   ├── meukf_types.hpp
│   │   ├── unified_filter.hpp
│   │   └── unified_types.hpp
│   └── src/
│       ├── meukf_core.cpp             ⚠️ 要分割（1346行）
│       ├── meukf_predict.cpp
│       ├── meukf_sigma_points.cpp
│       ├── meukf_update.cpp
│       └── meukf_observation_models_constants.cpp
│
├── KF/                                ← 基本 KF テンプレート（要検証）
│   └── inc/
│       ├── kalman_filter_core.hpp
│       ├── kf_core.hpp
│       └── kf_operations.hpp
│
├── Matrix/                            ← 固定サイズ行列（必須）
│   └── fixed_matrix.hpp
│
└── Quaternion/                        ← 四元数演算（必須）
    └── quaternion_functions.hpp

❌ 削除対象:
├── Core/                              ← 古い wrapper（機能しない）
└── Sensor/ (at Lib/Sensor level)      ← 統一先は `Lib/Sensor/`
```

---

## 4. 関数・シンボル分析

### 4.1 ESKF ホットパス関数群

| 関数 | ファイル | 行数 | 用途 | 使用頻度 |
|------|---------|------|------|---------|
| `HybridFilterRunner::predict()` | eskf_runner.cpp | ~150 | 状態予測（毎ステップ） | ★★★ |
| `HybridFilterCore::integrate_nominal()` | eskf_core.cpp | ~100 | IMU積分 | ★★★ |
| `HybridFilterCore::predict_covariance()` | eskf_core.cpp | ~80 | 共分散伝播 | ★★★ |
| `HybridFilterCore::update_zupt()` | eskf_core.cpp | ~40 | ZUPT 更新 | ★★ |
| `normalize_quaternion()` | quaternion_functions.hpp | ~20 | 四元数正規化 | ★★★ |

### 4.2 MEUKF フィルタパス関数群

| 関数 | ファイル | 行数 | 用途 | 使用頻度 |
|------|---------|------|------|---------|
| `MEUKFCore::step()` | meukf_core.cpp | ~200 | UKF ステップ | ★★★ |
| `meukf_predict()` | meukf_predict.cpp | ~150 | UKF 予測 | ★★★ |
| `meukf_update()` | meukf_update.cpp | ~180 | UKF 更新 | ★★★ |
| `generate_sigma_points()` | meukf_sigma_points.cpp | ~120 | シグマポイント生成 | ★★★ |

### 4.3 完全未使用関数（削除候補）

```cpp
// Lib/EKF （テンプレートのみ、インスタンス化されない）
- ekf_core.hpp: class ekf::EKFCore<T> { ... }

// Lib/UKF （テンプレートのみ、インスタンス化されない）
- ukf_sigma_points.hpp: class ukf::SigmaPoints { ... }
- ukf_update.hpp: class ukf::UKFUpdate { ... }

// Lib/KF （未使用の可能性あり、要確認）
- kalman_filter_core.hpp: テンプレート実装
- kf_operations.hpp: KF基本演算
```

---

## 5. 検証手順

### Step 1: 不要ファイル削除前の baseline
```bash
ls -la kalman/cpp/bin/mex_hybrid_filter.mexw64
# 現在: 347,136 bytes (340 KB)
```

### Step 2: Lib/Core 削除
```bash
rm -rf kalman/cpp/Lib/Core/
# ビルド & 確認
cd kalman/cpp/build && build_mex()
ls -la ../bin/mex_hybrid_filter.mexw64
# 予想: 340-342 KB（ほぼ変化なし）
```

### Step 3: Lib/Common/inc/Sensor -> Lib/Sensor に統一
```bash
# mex_hybrid_filter_common.hpp を編集:
# #include "../../Lib/Sensor/sensor_filter.hpp"
#   ↓
# #include "../../Lib/Sensor/filters.hpp"
# その後 Lib/Sensor を削除
rm -rf kalman/cpp/Lib/Sensor/
cd kalman/cpp/build && build_mex()
# 予想: 340-342 KB
```

### Step 4: EKF/UKF 削除（要検証）
```bash
# ESKF/MEUKF が EKF/UKF を参照していないか確認
grep -r "ekf_core\|ukf_" kalman/cpp/Lib/ESKF kalman/cpp/Lib/MEUKF --include="*.hpp"
# 結果: なければ削除可能
rm -rf kalman/cpp/Lib/EKF kalman/cpp/Lib/UKF
cd kalman/cpp/build && build_mex()
# 予想: 335-340 KB（テンプレートのため大きな削減は期待できない）
```

---

## 6. まとめ

| 改善項目 | 対象 | 削減効果 | 優先度 |
|---------|------|---------|--------|
| **Lib/Core 削除** | 古い wrapper | 最小（~3KB） | P0 |
| **Lib/Sensor 統合** | 重複フォルダ | 最小（~5KB） | P0 |
| **EKF/UKF 削除** | 未使用テンプレート | 最小（~2KB） | P1 |
| **meukf_core 分割** | 超大関数 | 中程度（~10-20KB、最適化効果） | P1 |
| **テンプレート最適化** | 全テンプレート | 大（要分析） | P2 |

**短期目標**: 330-335 KB を目指す（現在 340 KB から 5-10 KB 削減）  
**中期目標**: テンプレート instantiation の削減を検討（要分析）

