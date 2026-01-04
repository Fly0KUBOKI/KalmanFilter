# ファイル重複・依存関係 徹底調査レポート

**調査日**: 2026年1月4日  
**対象フォルダ**: `kalman/cpp/Lib`, `kalman/cpp/Inc`, `kalman/cpp/src`

---

## エグゼクティブサマリー

現在のプロジェクト構造は **3層アーキテクチャ** を採用していますが、**実装は Lib に集約され、Inc と src は転送ラッパーのみ**という状態です。

### 主要な発見
- **実装**: すべて `Lib/` 配下（ヘッダー: `Lib/*/inc/*.hpp`, 実装: `Lib/*/src/*.cpp`）
- **Inc フォルダ**: 完全に転送ヘッダーのみ（`#include "../Lib/.../xxx.hpp"`）
- **src フォルダ**: 完全に転送実装のみ（`#include "../../Lib/.../xxx.cpp"`）
- **MEX**: `Inc/` 経由で `Lib/` を参照（一部直接参照もあり）
- **examples**: `Inc/kalman_all.hpp` のみを参照

### 重複の種類
1. **名前重複**: 同名ファイルが `Lib/ESKF/inc/eskf_core.hpp` ⇔ `Inc/ESKF/eskf_core.hpp` で存在
2. **実装重複**: `src/MEUKF/meukf_core.cpp` (1345行) vs `Lib/MEUKF/src/meukf_core.cpp` (23行・スタブ)

---

## 1. 各フォルダの全ファイルリスト

### 1.1 Lib フォルダ（実装本体）

#### Lib/Matrix/
- `fixed_matrix.hpp` ← **実装本体** (ヘッダーのみライブラリ)

#### Lib/Quaternion/
- `quaternion_functions.hpp` ← **実装本体** (ヘッダーのみライブラリ)

#### Lib/Common/
**ヘッダー (`inc/`):**
- `filter_mgmt.hpp`
- `interface.hpp`
- `standalone.hpp`
- `utils.hpp`
- `types.hpp`
- `Math/math_utils.hpp`
- `Math/statistics.hpp`
- `Math/vector_utils.hpp`
- `Sensor/sensor_filter.hpp`
- `Sensor/sensor_preprocessor.hpp`
- `Validation/validation.hpp`

**実装 (`src/`):**
- `filter_mgmt.cpp`
- `interface_stub.cpp`
- `standalone.cpp`
- `Sensor/sensor_preprocessor.cpp`

#### Lib/ESKF/
**ヘッダー (`inc/`):**
- `eskf_core.hpp` ← **実装本体**
- `eskf_filter.hpp`
- `eskf_helper.hpp`
- `eskf_initializer.hpp` ← **実装本体**
- `eskf_math.hpp` ← **実装本体**
- `eskf_postprocess.hpp` ← **実装本体**
- `eskf_runner.hpp` ← **実装本体**
- `eskf_sensor_updates.hpp` ← **実装本体**
- `eskf_state.hpp`
- `filter.hpp`

**実装 (`src/`):**
- `eskf_core.cpp` (247行) ← **実装本体**
- `eskf_initializer.cpp` (65行) ← **実装本体**
- `eskf_math.cpp` (44行) ← **実装本体**
- `eskf_postprocess.cpp` (22行) ← **実装本体**
- `eskf_runner.cpp` (49行) ← **実装本体**
- `eskf_sensor_updates.cpp` (90行) ← **実装本体**
- `filter.cpp` (159行)

#### Lib/MEUKF/
**ヘッダー (`inc/`):**
- `meukf_core.hpp` ← **実装本体**
- `meukf_types.hpp`
- `unified_filter.hpp` ← **実装本体**
- `unified_types.hpp`

**実装 (`src/`):**
- `meukf_core.cpp` (23行) ← **スタブ実装（コメントのみ）**
- `unified_filter.cpp` (209行) ← **実装本体**

#### Lib/KF/
- `inc/kalman_filter_core.hpp`
- `inc/kf_core.hpp`

#### Lib/UKF/
- `inc/ukf_core.hpp`
- `inc/ukf_sigma_points.hpp`
- `inc/ukf_update.hpp`

#### Lib/EKF/
- `src/ekf_linear_update.cpp`

---

### 1.2 Inc フォルダ（転送ヘッダー層）

#### Inc/ 直下
- `kalman_all.hpp` ← **統合インクルードヘッダー** (`Lib/` を直接参照)
- `kalman_filters.hpp` ← **転送ヘッダー** (`Lib/KF/`, `Lib/UKF/` を参照)
- `matrix.hpp` ← **転送** → `../Lib/Matrix/fixed_matrix.hpp`
- `quaternion.hpp` ← **転送** → `../Lib/Quaternion/quaternion_functions.hpp`

#### Inc/Common/
- `filter_management.hpp` ← **転送** → `../../Lib/Common/inc/filter_mgmt.hpp`
- `Sensor/sensor_preprocessor.hpp` ← **転送** → `../../Lib/Common/inc/Sensor/sensor_preprocessor.hpp`
- `Validation/validation.hpp` (転送の可能性あり)
- `Math/math_utils.hpp`, `Math/statistics.hpp`, `Math/vector_utils.hpp` (調査未完了)

#### Inc/ESKF/
すべて **転送ヘッダー** (`#include "../Lib/ESKF/inc/xxx.hpp"`)
- `eskf_core.hpp`
- `eskf_filter.hpp`
- `eskf_helper.hpp`
- `eskf_initializer.hpp`
- `eskf_math.hpp`
- `eskf_postprocess.hpp`
- `eskf_runner.hpp`
- `eskf_sensor_updates.hpp`
- `eskf_state.hpp`

#### Inc/MEUKF/
すべて **転送ヘッダー** (`#include "../Lib/MEUKF/inc/xxx.hpp"`)
- `meukf_core.hpp`
- `meukf_types.hpp`
- `unified_filter.hpp`
- `unified_types.hpp`

#### Inc/KF/, Inc/UKF/
- 一部 `.bak` ファイルあり（旧バックアップ）

---

### 1.3 src フォルダ（転送実装層）

#### src/Common/
- `filter_management.cpp` ← **転送** → `#include "../../Lib/Common/src/filter_mgmt.cpp"`
- `Sensor/sensor_preprocessor.cpp` ← **転送** → `#include "../../../Lib/Common/src/Sensor/sensor_preprocessor.cpp"`

#### src/ESKF/
すべて **転送実装** (`#include "../../Lib/ESKF/inc/xxx.hpp"` のみ、実装なし)
- `eskf_core.cpp` (3行)
- `eskf_initializer.cpp` (3行)
- `eskf_math.cpp` (2行)
- `eskf_postprocess.cpp` (3行)
- `eskf_runner.cpp` (2行)
- `eskf_sensor_updates.cpp` (2行)

#### src/MEUKF/
- `meukf_core.cpp` (1345行) ← **実装本体が残存** ⚠️
- `unified_filter.cpp` (2行) ← **転送** → `#include "../../Lib/MEUKF/src/unified_filter.cpp"`

---

## 2. 同名ファイル重複マップ

### 2.1 完全重複（削除候補）

#### ESKF関連
| Lib（優先・実装本体） | Inc（転送のみ） | src（転送のみ） | 状態 |
|---|---|---|---|
| `Lib/ESKF/inc/eskf_core.hpp` | `Inc/ESKF/eskf_core.hpp` | `src/ESKF/eskf_core.cpp` | ✅ 正常（転送済み） |
| `Lib/ESKF/inc/eskf_runner.hpp` | `Inc/ESKF/eskf_runner.hpp` | `src/ESKF/eskf_runner.cpp` | ✅ 正常（転送済み） |
| `Lib/ESKF/inc/eskf_sensor_updates.hpp` | `Inc/ESKF/eskf_sensor_updates.hpp` | `src/ESKF/eskf_sensor_updates.cpp` | ✅ 正常（転送済み） |
| `Lib/ESKF/inc/eskf_postprocess.hpp` | `Inc/ESKF/eskf_postprocess.hpp` | `src/ESKF/eskf_postprocess.cpp` | ✅ 正常（転送済み） |
| `Lib/ESKF/inc/eskf_math.hpp` | `Inc/ESKF/eskf_math.hpp` | `src/ESKF/eskf_math.cpp` | ✅ 正常（転送済み） |
| `Lib/ESKF/inc/eskf_initializer.hpp` | `Inc/ESKF/eskf_initializer.hpp` | `src/ESKF/eskf_initializer.cpp` | ✅ 正常（転送済み） |
| `Lib/ESKF/inc/eskf_state.hpp` | `Inc/ESKF/eskf_state.hpp` | - | ✅ 正常 |
| `Lib/ESKF/inc/eskf_filter.hpp` | `Inc/ESKF/eskf_filter.hpp` | - | ✅ 正常 |
| `Lib/ESKF/inc/eskf_helper.hpp` | `Inc/ESKF/eskf_helper.hpp` | - | ✅ 正常 |
| `Lib/ESKF/inc/filter.hpp` | - | - | 独立（Lib のみ） |
| `Lib/ESKF/src/filter.cpp` | - | - | 独立（Lib のみ） |

#### MEUKF関連
| Lib（優先・実装本体） | Inc（転送のみ） | src（重複あり⚠️） | 状態 |
|---|---|---|---|
| `Lib/MEUKF/inc/meukf_core.hpp` | `Inc/MEUKF/meukf_core.hpp` | `src/MEUKF/meukf_core.cpp` | ⚠️ **重大な重複** |
| `Lib/MEUKF/inc/unified_filter.hpp` | `Inc/MEUKF/unified_filter.hpp` | `src/MEUKF/unified_filter.cpp` | ✅ 正常（転送済み） |
| `Lib/MEUKF/inc/meukf_types.hpp` | `Inc/MEUKF/meukf_types.hpp` | - | ✅ 正常 |
| `Lib/MEUKF/inc/unified_types.hpp` | `Inc/MEUKF/unified_types.hpp` | - | ✅ 正常 |

#### Common関連
| Lib（優先・実装本体） | Inc（転送のみ） | src（転送のみ） | 状態 |
|---|---|---|---|
| `Lib/Common/inc/filter_mgmt.hpp` | `Inc/Common/filter_management.hpp` | `src/Common/filter_management.cpp` | ✅ 正常（転送済み） |
| `Lib/Common/inc/Sensor/sensor_preprocessor.hpp` | `Inc/Common/Sensor/sensor_preprocessor.hpp` | `src/Common/Sensor/sensor_preprocessor.cpp` | ✅ 正常（転送済み） |

#### Matrix/Quaternion関連
| Lib（優先・実装本体） | Inc（転送のみ） | src | 状態 |
|---|---|---|---|
| `Lib/Matrix/fixed_matrix.hpp` | `Inc/matrix.hpp` | - | ✅ 正常 |
| `Lib/Quaternion/quaternion_functions.hpp` | `Inc/quaternion.hpp` | - | ✅ 正常 |

---

### 2.2 部分重複・異常な重複（要対応）

#### ⚠️ **重大**: `meukf_core.cpp` の重複

**問題**: 2つの `meukf_core.cpp` が存在し、内容が大きく異なる

| ファイル | 行数 | 内容 | 優先度 |
|---|---|---|---|
| `src/MEUKF/meukf_core.cpp` | **1345行** | **完全な実装** | ❌ 削除候補（旧場所） |
| `Lib/MEUKF/src/meukf_core.cpp` | **23行** | スタブ（コメントのみ） | ⚠️ 要確認 |

**推定原因**: Phase2マイグレーション時に `Lib/MEUKF/src/meukf_core.cpp` への移動が未完了

**差分詳細**:
- `src/MEUKF/meukf_core.cpp`: フル実装（デバッグログ、Cholesky分解、シグマポイント計算など）
- `Lib/MEUKF/src/meukf_core.cpp`: スタブのみ（"Implementation copied from src/MEUKF" とコメントあり）

**影響範囲**:
- MEXファイル `mex_meukf_step.cpp` は `#include "meukf_core.hpp"` を使用
- `Inc/MEUKF/meukf_core.hpp` → `Lib/MEUKF/inc/meukf_core.hpp` を参照
- **ビルドシステムがどちらの `.cpp` をリンクしているか要確認**

**推奨対応**:
1. `src/MEUKF/meukf_core.cpp` の内容を `Lib/MEUKF/src/meukf_core.cpp` に移動
2. `src/MEUKF/meukf_core.cpp` を転送実装に変更
3. CMakeLists.txt でリンク対象を確認

---

## 3. #include依存関係マップ

### 3.1 依存関係グラフ（レイヤー別）

```
┌─────────────────────────────────────────────────┐
│ Layer 4: MEX / examples                         │
├─────────────────────────────────────────────────┤
│ MEX/                                            │
│  ├─ mex_run_eskf.cpp                            │
│  │   └─> Inc/ESKF/eskf_core.hpp (via Inc/)     │
│  ├─ mex_meukf_step.cpp                          │
│  │   └─> meukf_core.hpp (直接参照・曖昧)        │
│  └─ Inc/mex_eskf_common.hpp                     │
│      ├─> Lib/Matrix/fixed_matrix.hpp (直接)    │
│      ├─> Lib/Quaternion/... (直接)             │
│      └─> Inc/ESKF/*, Inc/Common/* (Inc経由)    │
│                                                 │
│ examples/                                       │
│  └─ main_eskf.cpp, test_interface.cpp          │
│      └─> Inc/kalman_all.hpp (統合ヘッダー)     │
└─────────────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────┐
│ Layer 3: Inc (転送レイヤー)                     │
├─────────────────────────────────────────────────┤
│ Inc/kalman_all.hpp                              │
│  └─> Lib/* (直接参照)                           │
│                                                 │
│ Inc/ESKF/*.hpp, Inc/MEUKF/*.hpp                 │
│  └─> #include "../Lib/ESKF/inc/xxx.hpp"        │
│                                                 │
│ Inc/matrix.hpp, Inc/quaternion.hpp              │
│  └─> #include "../Lib/Matrix/..."              │
└─────────────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────┐
│ Layer 2: Lib (実装本体)                         │
├─────────────────────────────────────────────────┤
│ Lib/ESKF/inc/*.hpp                              │
│  ├─> Lib/Matrix/fixed_matrix.hpp               │
│  ├─> Lib/Quaternion/quaternion_functions.hpp   │
│  └─> Lib/Common/inc/Sensor/sensor_filter.hpp   │
│                                                 │
│ Lib/MEUKF/inc/*.hpp                             │
│  ├─> Lib/Matrix/fixed_matrix.hpp               │
│  └─> Lib/Quaternion/quaternion_functions.hpp   │
│                                                 │
│ Lib/Common/inc/*.hpp                            │
│  ├─> Lib/Matrix/fixed_matrix.hpp               │
│  └─> Lib/Quaternion/quaternion_functions.hpp   │
└─────────────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────┐
│ Layer 1: Lib/Matrix, Lib/Quaternion            │
├─────────────────────────────────────────────────┤
│ fixed_matrix.hpp (基礎行列ライブラリ)           │
│ quaternion_functions.hpp (基礎クォータニオン)   │
└─────────────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────┐
│ Layer 0: src (転送のみ)                         │
├─────────────────────────────────────────────────┤
│ src/ESKF/*.cpp, src/Common/*.cpp                │
│  └─> #include "../../Lib/*/inc/*.hpp"          │
│  └─> #include "../../Lib/*/src/*.cpp" (一部)   │
│                                                 │
│ ⚠️ src/MEUKF/meukf_core.cpp                    │
│  └─> Inc/MEUKF/meukf_core.hpp (異常)           │
└─────────────────────────────────────────────────┘
```

### 3.2 詳細な依存関係

#### MEX フォルダ → どこを参照

| MEX ファイル | 参照先 | 参照方法 |
|---|---|---|
| `mex_run_eskf.cpp` | `Inc/mex_eskf_common.hpp` | ローカル |
| | `Inc/mex_run_eskf_impl.hpp` | ローカル |
| `mex_meukf_step.cpp` | `meukf_core.hpp` | **曖昧** (パス未指定) |
| | `Inc/mex_type_conversion.hpp` | ローカル |
| `mex_eskf_initializer.cpp` | `Inc/ESKF/eskf_initializer.hpp` | Inc経由 |
| | `Inc/mex_eskf_initializer.hpp` | ローカル |
| **`Inc/mex_eskf_common.hpp`** | `Lib/Matrix/fixed_matrix.hpp` | **直接参照** |
| | `Lib/Quaternion/quaternion_functions.hpp` | **直接参照** |
| | `Inc/ESKF/*` | Inc経由 |
| | `Inc/Common/*` | Inc経由 |
| `Inc/mex_type_conversion.hpp` | `Lib/Matrix/fixed_matrix.hpp` | **直接参照** |
| `Inc/mex_helpers.hpp` | `Lib/Quaternion/quaternion_functions.hpp` | **直接参照** |
| | `Lib/Matrix/fixed_matrix.hpp` | **直接参照** |
| `Inc/mex_run_eskf_impl.hpp` | `Inc/MEUKF/meukf_core.hpp` | Inc経由 |

**問題点**:
- `mex_meukf_step.cpp` の `#include "meukf_core.hpp"` はパス未指定 → ビルドシステム依存
- MEX ローカルヘッダーは `Lib/` を直接参照しているが、ESKF関連は `Inc/` 経由

#### examples フォルダ → どこを参照

| examples ファイル | 参照先 |
|---|---|
| `main_eskf.cpp` | `Inc/kalman_all.hpp` のみ |
| `main_standalone.cpp` | `Inc/kalman_all.hpp` のみ |
| `test_interface.cpp` | `Inc/kalman_all.hpp` のみ |

**問題点**: なし（統合ヘッダー経由で一貫性あり）

#### Lib 内部の依存

```
Lib/ESKF/inc/eskf_core.hpp
  ├─> ../Lib/Matrix/fixed_matrix.hpp
  └─> ../Lib/Quaternion/quaternion_functions.hpp

Lib/ESKF/inc/eskf_runner.hpp
  ├─> eskf_state.hpp (同一フォルダ)
  ├─> eskf_core.hpp (同一フォルダ)
  ├─> eskf_postprocess.hpp (同一フォルダ)
  ├─> ../Lib/Matrix/fixed_matrix.hpp
  └─> Common/Sensor/sensor_filter.hpp (相対パス)

Lib/MEUKF/inc/meukf_core.hpp
  ├─> unified_types.hpp (同一フォルダ)
  ├─> meukf_types.hpp (同一フォルダ)
  ├─> ../../Matrix/fixed_matrix.hpp
  └─> ../../Quaternion/quaternion_functions.hpp

Lib/Common/inc/Sensor/sensor_filter.hpp
  └─> Lib/Matrix/fixed_matrix.hpp (パス未確認)
```

**問題点**: 相対パス参照の一貫性不足（`../Lib/Matrix/` vs `../../Matrix/`）

#### Inc 内部の依存

すべて転送ヘッダーのため、内部依存なし（`Lib/` への転送のみ）

#### src 内部の依存

すべて転送実装のため、内部依存なし（`Lib/` への転送のみ）

**例外**: `src/MEUKF/meukf_core.cpp` は `Inc/MEUKF/meukf_core.hpp` を参照（異常）

---

## 4. 重複コード・類似コードの詳細分析

### 4.1 完全重複（転送のみ）

以下のファイルは **転送ラッパーのみ** で実装重複なし：

#### Inc → Lib 転送ヘッダー
- `Inc/ESKF/*.hpp` (9ファイル) → `Lib/ESKF/inc/*.hpp`
- `Inc/MEUKF/*.hpp` (4ファイル) → `Lib/MEUKF/inc/*.hpp`
- `Inc/matrix.hpp` → `Lib/Matrix/fixed_matrix.hpp`
- `Inc/quaternion.hpp` → `Lib/Quaternion/quaternion_functions.hpp`
- `Inc/Common/filter_management.hpp` → `Lib/Common/inc/filter_mgmt.hpp`
- `Inc/Common/Sensor/sensor_preprocessor.hpp` → `Lib/Common/inc/Sensor/sensor_preprocessor.hpp`

**削除可能性**: Inc フォルダ自体を削除し、すべて `Lib/` を直接参照可能

#### src → Lib 転送実装
- `src/ESKF/*.cpp` (6ファイル) → すべて2〜3行の転送のみ
- `src/Common/filter_management.cpp` → `Lib/Common/src/filter_mgmt.cpp`
- `src/Common/Sensor/sensor_preprocessor.cpp` → `Lib/Common/src/Sensor/sensor_preprocessor.cpp`
- `src/MEUKF/unified_filter.cpp` → `Lib/MEUKF/src/unified_filter.cpp`

**削除可能性**: src フォルダ自体を削除し、CMakeLists.txt で `Lib/*/src/*.cpp` を直接リンク可能

### 4.2 部分重複（実装差分あり）

#### ⚠️ `meukf_core.cpp` の実装差分

**ファイル1**: `src/MEUKF/meukf_core.cpp` (1345行)
- 完全な実装（デバッグログ、Cholesky分解、UKF計算）
- `#include "../../Inc/MEUKF/meukf_core.hpp"` を参照

**ファイル2**: `Lib/MEUKF/src/meukf_core.cpp` (23行)
- スタブのみ（コメント: "Implementation copied from src/MEUKF"）
- 実装は空

**違い**:
```cpp
// src/MEUKF/meukf_core.cpp (実装あり)
void MEUKFCore::step(const MEUKFInput& input, MEUKFOutput& output) {
    // 1300行以上の完全な実装
    // - シグマポイント生成
    // - 予測ステップ
    // - 更新ステップ
    // - Cholesky分解
    // - デバッグログ出力
}

// Lib/MEUKF/src/meukf_core.cpp (スタブ)
void MEUKFCore::step(const MEUKFInput& input, MEUKFOutput& output) {
    // Delegate to original implementation (copied in migration)
    // Full function body preserved in repository version.
    // Actual content is long; source file added to Lib for phase2 migration.
}
```

**推奨対応**: `src/MEUKF/meukf_core.cpp` の内容を `Lib/MEUKF/src/meukf_core.cpp` に完全移行

### 4.3 名前の不一致

| Lib 名 | Inc/src 名 | 理由 |
|---|---|---|
| `Lib/Common/inc/filter_mgmt.hpp` | `Inc/Common/filter_management.hpp` | リファクタリング時の名前変更 |

---

## 5. MEX・examplesからの参照整理

### 5.1 MEX からの参照パターン

| 参照方法 | ファイル数 | 例 |
|---|---|---|
| `Inc/` 経由 | 多数 | `#include "../../Inc/ESKF/eskf_core.hpp"` |
| `Lib/` 直接 | 3ファイル | `#include "../../Lib/Matrix/fixed_matrix.hpp"` |
| パス未指定（曖昧） | 1ファイル | `#include "meukf_core.hpp"` |

**混在パターン**:
```cpp
// Inc/mex_eskf_common.hpp
#include "../../Lib/Matrix/fixed_matrix.hpp"        // 直接
#include "../../Inc/ESKF/eskf_core.hpp"             // Inc経由
```

**推奨対応**: すべて `Lib/` 直接参照に統一

### 5.2 examples からの参照パターン

すべて `Inc/kalman_all.hpp` のみ → **一貫性あり**

### 5.3 Inc/kalman_all.hpp の参照先

```cpp
// Inc/kalman_all.hpp
#include "Lib/Matrix/fixed_matrix.hpp"              // 直接
#include "Lib/Quaternion/quaternion_functions.hpp"  // 直接
#include "Lib/Common/inc/interface.hpp"             // 直接
#include "Lib/Common/inc/utils.hpp"                 // 直接
#include "Lib/ESKF/inc/eskf_state.hpp"              // 直接
// ... 他すべて Lib/ 直接参照
```

**発見**: `Inc/kalman_all.hpp` 自体が `Lib/` を直接参照している → **Inc レイヤーの意味が薄い**

---

## 6. リファクタリング推奨事項

### 6.1 即時対応（高優先度）

#### ⚠️ **必須**: `meukf_core.cpp` の統合

```bash
# 1. src/MEUKF/meukf_core.cpp の内容を Lib/MEUKF/src/ にコピー
cp src/MEUKF/meukf_core.cpp Lib/MEUKF/src/meukf_core.cpp

# 2. src/MEUKF/meukf_core.cpp を転送ファイルに変更
echo '#include "../../Lib/MEUKF/src/meukf_core.cpp"' > src/MEUKF/meukf_core.cpp

# 3. CMakeLists.txt でリンク対象を確認
# 確認: Lib/MEUKF/src/meukf_core.cpp がビルド対象に含まれているか
```

#### ⚠️ **推奨**: MEX の曖昧な参照を修正

```cpp
// MEX/mex_meukf_step.cpp (修正前)
#include "meukf_core.hpp"  // ← どこの？

// MEX/mex_meukf_step.cpp (修正後)
#include "../Inc/MEUKF/meukf_core.hpp"  // または
#include "../Lib/MEUKF/inc/meukf_core.hpp"
```

### 6.2 中期対応（リファクタリング）

#### オプション A: Inc/src 削除（Lib 一本化）

**メリット**:
- 構造が単純化
- パス参照の一貫性向上
- ビルド時間短縮（転送ファイル不要）

**デメリット**:
- 既存コードの大規模変更が必要

**手順**:
1. MEX、examples のすべての `#include "Inc/..."` を `#include "Lib/..."` に変更
2. `Inc/`, `src/` フォルダを削除
3. CMakeLists.txt を `Lib/` のみビルドに変更

#### オプション B: Inc を Public API 層として維持

**メリット**:
- 外部向け API と内部実装を分離
- バージョンアップ時の互換性維持

**デメリット**:
- 転送ヘッダーのメンテナンスコスト

**手順**:
1. `Inc/kalman_all.hpp` を唯一の外部公開ヘッダーとして定義
2. `Inc/ESKF/`, `Inc/MEUKF/` など個別転送ヘッダーは削除
3. MEX は `Inc/kalman_all.hpp` のみをインクルード

### 6.3 長期対応（アーキテクチャ改善）

#### 1. 相対パス参照の統一

現在:
```cpp
// Lib/ESKF/inc/eskf_core.hpp
#include "../Lib/Matrix/fixed_matrix.hpp"  // パターン1

// Lib/MEUKF/inc/meukf_core.hpp
#include "../../Matrix/fixed_matrix.hpp"   // パターン2
```

推奨:
```cpp
// すべて Lib/ ルートからの相対パスに統一
#include "Matrix/fixed_matrix.hpp"
```

CMakeLists.txt で `Lib/` をインクルードパスに追加:
```cmake
target_include_directories(kalman_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Lib)
```

#### 2. 名前空間の整理

現在の名前空間:
- `eskf::ESKFCore`
- `meukf::MEUKFCore`
- `common::sensor::SensorFilterLib`

推奨:
```cpp
namespace kalman {
    namespace eskf { ... }
    namespace meukf { ... }
    namespace common {
        namespace sensor { ... }
        namespace math { ... }
    }
}
```

---

## 7. 依存関係の循環・問題点

### 7.1 発見された循環依存

**なし** （レイヤー構造が明確）

### 7.2 パス参照の不一致リスト

| ファイル | 参照先 | 問題 |
|---|---|---|
| `MEX/Inc/mex_eskf_common.hpp` | `Lib/Matrix/` と `Inc/ESKF/` の混在 | 一貫性なし |
| `Lib/ESKF/inc/eskf_runner.hpp` | `Common/Sensor/sensor_filter.hpp` | 相対パス未指定 |
| `src/MEUKF/meukf_core.cpp` | `Inc/MEUKF/meukf_core.hpp` | Inc経由（異常） |

---

## 8. CMakeLists.txt リンク対象の推定

**要確認事項** (調査必要):
```bash
# どちらがリンクされている？
Lib/MEUKF/src/meukf_core.cpp  (23行・スタブ)
src/MEUKF/meukf_core.cpp      (1345行・実装本体)
```

**調査方法**:
1. `kalman/cpp/CMakeLists.txt` を確認
2. `kalman/cpp/Lib/CMakeLists.txt` を確認
3. ビルドログ (`build_mex.m` の出力) で実際にコンパイルされたファイルを確認

---

## 9. 削除候補ファイル一覧

### 9.1 完全削除候補（転送のみ）

#### Inc フォルダ（転送ヘッダー）
すべて削除候補（Lib を直接参照すれば不要）:
- `Inc/ESKF/*.hpp` (9ファイル)
- `Inc/MEUKF/*.hpp` (4ファイル)
- `Inc/matrix.hpp`
- `Inc/quaternion.hpp`
- `Inc/Common/filter_management.hpp`
- `Inc/Common/Sensor/sensor_preprocessor.hpp`

**保留**: `Inc/kalman_all.hpp` は統合ヘッダーとして有用（保持推奨）

#### src フォルダ（転送実装）
すべて削除候補（Lib/*/src/*.cpp を直接ビルドすれば不要）:
- `src/ESKF/*.cpp` (6ファイル)
- `src/Common/filter_management.cpp`
- `src/Common/Sensor/sensor_preprocessor.cpp`
- `src/MEUKF/unified_filter.cpp`

**要対応**: `src/MEUKF/meukf_core.cpp` は削除前に Lib へ移行必須

### 9.2 バックアップファイル（即削除可能）

- `Inc/KF/*.bak`
- `Inc/UKF/*.bak`

---

## 10. アクションアイテム

### Phase 1: 緊急対応（今すぐ）

- [ ] **CRITICAL**: `src/MEUKF/meukf_core.cpp` の実装を `Lib/MEUKF/src/meukf_core.cpp` に移行
- [ ] **HIGH**: MEX の `#include "meukf_core.hpp"` を明示的パスに変更
- [ ] **MEDIUM**: CMakeLists.txt でビルド対象ファイルを確認（どちらの meukf_core.cpp がリンクされているか）

### Phase 2: クリーンアップ（1週間以内）

- [ ] `.bak` ファイルをすべて削除
- [ ] 転送ヘッダー・転送実装の一貫性確認
- [ ] ビルドログで警告・重複定義エラーを確認

### Phase 3: リファクタリング（1ヶ月以内）

- [ ] Inc/src フォルダの削除可否を判断
- [ ] 相対パス参照を統一（CMakeLists.txt でインクルードパス設定）
- [ ] MEX の参照を Lib 直接に統一

### Phase 4: ドキュメント化

- [ ] アーキテクチャドキュメント作成（Lib/Inc/src の役割明記）
- [ ] #include ルールを CONTRIBUTING.md に記載
- [ ] CI でパス参照の一貫性をチェック

---

## 11. 結論

### 現状の評価

**良い点**:
- Lib に実装を集約する方針は明確
- examples は統合ヘッダー経由で一貫性あり

**問題点**:
- **重大**: `meukf_core.cpp` の実装が2箇所に分散（1345行 vs 23行）
- Inc/src が完全に転送レイヤーになっており、存在意義が薄い
- MEX の参照パスが Lib 直接と Inc 経由で混在

### 推奨方針

**短期** (今週中):
1. `src/MEUKF/meukf_core.cpp` を `Lib/MEUKF/src/meukf_core.cpp` に移行
2. ビルドシステムの確認

**中期** (1ヶ月):
1. Inc/src フォルダを削除し、Lib 一本化
2. または Inc/kalman_all.hpp のみ残して他を削除

**長期**:
1. 相対パス参照を CMake インクルードパスで管理
2. 名前空間の整理

---

**レポート作成日**: 2026年1月4日  
**調査対象**: kalman/cpp/Lib, Inc, src, MEX, examples  
**総ファイル数**: 95ファイル (Lib: 48, Inc: 24, src: 10, MEX: 10, examples: 3)
