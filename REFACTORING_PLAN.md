# 包括的リファクタリング計画

**作成日**: 2026年1月11日  
**目標**: 安全・安定・最適化されたプログラム構造への再構築

---

## 目次

1. [現状分析サマリー](#1-現状分析サマリー)
2. [Phase 1: ドキュメント整理](#phase-1-ドキュメント整理)
3. [Phase 2: 環境依存・デバッグ機能の廃止](#phase-2-環境依存デバッグ機能の廃止)
4. [Phase 3: 冗長コード・重複の廃止](#phase-3-冗長コード重複の廃止)
5. [Phase 4: クラス設計の再構築](#phase-4-クラス設計の再構築)
6. [Phase 5: ファイル構成の再構築](#phase-5-ファイル構成の再構築)
7. [Phase 6: コメント・コーディング規約の統一](#phase-6-コメントコーディング規約の統一)

---

## 1. 現状分析サマリー

### 1.0 フィルタ実装の実態（重要）

**実装構造**: `mex_run_eskf` は名前に反して **ESKF予測 + MEUKF更新のハイブリッド**

| コンポーネント | 実装 | 役割 |
|-------------|------|------|
| ESKFCore | ✅ 実装済み・使用中 | 予測（状態積分・共分散予測）、ZUPT更新 |
| ESKFRunner | ✅ 実装済み・使用中 | 予測ステップの統合実行 |
| MEUKFCore | ✅ 実装済み・使用中 | センサー更新（Accel/Mag/GPS/Baro） |
| `do_sensor_update_meukf` | ✅ 実装済み・使用中 | MEUKFCore::step() の呼び出しラッパー |
| `mex_meukf_step_v2` | ❌ レガシー・未使用 | 削除済み（ビルドターゲットから除外） |

詳細は [IMPLEMENTATION_ANALYSIS.md](IMPLEMENTATION_ANALYSIS.md) 参照。

**結論**: ESKFとMEUKFは**どちらも必須**。削除不可。

### 1.1 ファイル構成概要

```
kalman/cpp/
├── bin/                    # MEXバイナリ出力
├── build/                  # ビルドスクリプト
├── inc/                    # 旧ヘッダー（使用していない可能性）
├── MEX/                    # MEXエントリーポイント
│   ├── Impl/               # 実装ヘッダー（7ファイル - 重複あり）
│   └── Inc/                # インターフェースヘッダー（7ファイル - Implと重複）
└── Lib/                    # コアライブラリ
    ├── Common/             # 共通ユーティリティ
    │   ├── inc/            # ヘッダー（多階層・複雑）
    │   │   ├── Math/       # 数学関数（7ファイル）
    │   │   ├── Sensor/     # センサー処理（8ファイル - 重複多）
    │   │   └── Validation/ # 検証関数（1ファイル）
    │   └── src/            # 実装ファイル
    ├── ESKF/               # ESKFフィルタ
    ├── MEUKF/              # MEUKFフィルタ
    ├── KF/                 # 基本カルマンフィルタ
    ├── EKF/                # 拡張カルマンフィルタ
    ├── UKF/                # 無香カルマンフィルタ
    ├── Matrix/             # 固定サイズ行列
    └── Quaternion/         # 四元数演算
```

### 1.2 検出された問題

| カテゴリ | 問題 | 深刻度 | ファイル数 |
|---------|------|--------|-----------|
| **クラス重複** | NoiseEstimator: 3箇所, OutlierDetector: 3箇所, DivergenceGuard: 2箇所 | 🔴 高 | 4 |
| **ファイル重複** | MEX/Impl/ と MEX/Inc/ が完全重複（7ファイル） | 🔴 高 | 14 |
| **デバッグコード** | sensor_log, debug_print_R, ファイル出力 | 🟠 中 | 3 |
| **環境依存** | std::atomic, std::chrono, std::fstream | 🟠 中 | 4 |
| **モダンC++過剰** | std::string, std::to_string, lambda | 🟡 低 | 5 |
| **未使用コード** | sensor_filter_base.hpp（846行、sensor_filter.hppと重複） | 🔴 高 | 1 |
| **ヘッダー重複定義** | robust_statistics.hpp内に2つの完全なクラス定義 | 🔴 高 | 1 |

### 1.3 重複クラス一覧

| クラス名 | 定義場所 | 優先度 |
|---------|---------|--------|
| `NoiseEstimator` | robust_statistics.hpp (×2), validation.hpp, sensor_filter_base.hpp | → robust_statistics.hpp に統一 |
| `OutlierDetector` | outlier_detector.hpp, validation.hpp, sensor_filter_base.hpp | → outlier_detector.hpp に統一 |
| `DivergenceGuard` | robust_statistics.hpp (×2), sensor_filter_base.hpp | → robust_statistics.hpp に統一 |
| `EMAFilter` | ema_filter.hpp, sensor_filter_base.hpp | → ema_filter.hpp に統一 |
| `BiquadLowpassFilter` | biquad_filter.hpp, sensor_filter_base.hpp | → biquad_filter.hpp に統一 |
| `AlphaBetaFilter` | alpha_beta_filter.hpp, sensor_filter_base.hpp | → alpha_beta_filter.hpp に統一 |

---

## Phase 1: ドキュメント整理

**目標**: 不要なMarkdownを削除し、必要な情報を統合  
**所要時間**: 30分  
**リスク**: 低

### 1.1 削除対象ファイル

| ファイル | 理由 |
|---------|------|
| `PHASE3_PROGRESS.md` | 過去の進捗記録、現在不要 |
| `PHASE3_CURRENT_REFACTORING.md` | 完了済みタスク、現在不要 |
| `PROJECT_STATUS.md` | 過去のステータス、新計画に置換 |
| `memo.md` | 一時メモ、整理後削除 |
| `docs/BINARY_ANALYSIS_SUMMARY.md` | 過去の分析、不要 |
| `docs/BINARY_MANAGEMENT.md` | 過去の管理情報、不要 |
| `docs/BUILD_OPTIMIZATION_RECOMMENDATIONS.md` | 過去の推奨事項、不要 |
| `docs/COMPILER_ANALYSIS_FINAL.md` | 過去の分析、不要 |
| `docs/COMPILER_DEPENDENCY_ROOT_CAUSE.md` | 過去の分析、不要 |
| `docs/COMPILER_FIX_PLAN.md` | 過去の計画、不要 |
| `docs/COMPILER_ISSUE_ROOT_CAUSE_FINAL.md` | 過去の分析、不要 |
| `docs/ENVIRONMENT_DEPENDENCY_GUIDE.md` | 過去のガイド、不要 |
| `docs/ENVIRONMENT_QA.md` | 過去のQA、不要 |
| `docs/ENVIRONMENT_QUICK_START.md` | 過去のガイド、不要 |
| `docs/ENVIRONMENT_SETUP.md` | 過去のセットアップ、不要 |
| `docs/IMPLEMENTATION_CHECKLIST.md` | 過去のチェックリスト、不要 |
| `docs/IMPLEMENTATION_SUMMARY.md` | 過去のサマリー、不要 |
| `docs/PHASE1_TYPE_UNIFICATION_PLAN.md` | 完了済み計画、不要 |

### 1.2 保持・更新するファイル

| ファイル | アクション |
|---------|----------|
| `.github/copilot-instructions.md` | 更新（新構造を反映） |
| `docs/README.md` | 更新（現在のアーキテクチャを記載） |
| `docs/CODING_STANDARDS.md` | 更新（新規約を反映） |
| `docs/CPP_ARCHITECTURE.md` | 更新（新構造を反映） |
| `docs/CPP_INPUT_OUTPUT_SPEC.md` | 保持（型仕様は有効） |
| `docs/LIB_STRUCTURE.md` | 更新（新構造を反映） |

### 1.3 実施手順

```bash
# 1. 削除対象ファイルをバックアップ（任意）
mkdir -p .archive/docs_backup_20260111
mv PHASE3_*.md PROJECT_STATUS.md memo.md .archive/docs_backup_20260111/

# 2. docs/ 内の不要ファイル削除
cd docs
rm -f BINARY_*.md COMPILER_*.md ENVIRONMENT_*.md IMPLEMENTATION_*.md PHASE1_*.md

# 3. 残ったドキュメントを確認
ls -la
```

---

## Phase 2: 環境依存・デバッグ機能の廃止

**目標**: 動的・環境依存コードを静的・事前定義に変更  
**所要時間**: 2時間  
**リスク**: 中（機能削除のため慎重に）

### 2.1 削除対象: デバッグ・ログ機能

#### 対象ファイル: `Lib/Common/inc/Sensor/sensor_filter.hpp`

**削除する要素**:
```cpp
// 削除: std::atomic グローバル変数
static std::atomic<uint64_t> g_log_counter{0};
static std::atomic<bool> g_enable_sensor_logging{false};

// 削除: ログ関数
inline void sensor_log_enable(bool en);
inline void sensor_log(const char* fmt, ...);
```

**削除する#include**:
```cpp
#include <fstream>
#include <atomic>
#include <chrono>
#include <cstdarg>
```

#### 対象ファイル: `Lib/Common/inc/Sensor/robust_statistics.hpp`

**削除する要素**:
```cpp
// 削除: 全てのdebug_print_R関数（約50行）
// 削除: try-catch ブロック内のファイル出力
// 削除: sensor_log() 呼び出し（約10箇所）
// 削除: std::chrono使用箇所
// 削除: std::ofstream使用箇所
```

### 2.2 静的定義への変換

#### 現在（動的）:
```cpp
static std::atomic<uint64_t> g_log_counter{0};
```

#### 変更後（廃止）:
```cpp
// 完全削除 - ログカウンタは不要
```

### 2.3 MEX条件コンパイルの廃止

#### 現在:
```cpp
#ifdef MATLAB_MEX_FILE
# include "mex.h"
#else
# include <stdio.h>
# define mexPrintf printf
#endif
```

#### 変更後:
```cpp
// 完全削除 - MEX専用ビルドのためprintf不要
```

### 2.4 実施チェックリスト

- [ ] `sensor_filter.hpp`: ログ関連コード削除
- [ ] `sensor_filter_base.hpp`: ファイル全体削除（後述）
- [ ] `robust_statistics.hpp`: debug_print_R, sensor_log削除
- [ ] `robust_statistics.hpp`: std::fstream, std::chrono削除
- [ ] ビルド確認: `build_mex()`
- [ ] 回帰テスト: `run_batch_10sets()`

---

## Phase 3: 冗長コード・重複の廃止

**目標**: 重複定義を排除し、単一責任原則を適用  
**所要時間**: 3時間  
**リスク**: 高（コンパイルエラーの可能性）

### 3.1 削除対象ファイル

| ファイル | 理由 | アクション |
|---------|------|----------|
| `Lib/Common/inc/Sensor/sensor_filter_base.hpp` (846行) | `sensor_filter.hpp`と完全重複 | **削除** |
| `MEX/Inc/*.hpp` (7ファイル) | `MEX/Impl/*.hpp`と完全重複 | **削除** |

### 3.2 重複クラス定義の解消

#### robust_statistics.hpp の修正

現在のファイルには**2つの完全なNoiseEstimatorクラス定義**が存在（行1-200と行340-500）。

**修正手順**:
1. 行1-200のNoiseEstimator定義を保持
2. 行340-750の重複定義を**完全削除**
3. namespace構造を整理

#### validation.hpp の修正

`OutlierDetector`と`NoiseEstimator`が定義されているが、専用ヘッダーと重複。

**修正**:
```cpp
// 削除: OutlierDetector クラス定義（行70-95）
// 削除: NoiseEstimator クラス定義（行204以降）
// 代わりに include を追加
#include "../Sensor/outlier_detector.hpp"
```

### 3.3 冗長なusingエイリアスの統一

#### 現在（複数ファイルで重複定義）:
```cpp
// sensor_filter.hpp
using cm = cmath_fx::FixedMatrix;

// outlier_detector.hpp
using cm = cmath_fx::FixedMatrix;

// robust_statistics.hpp
using cm = cmath_fx::FixedMatrix;

// math_utils.hpp
using cm = cmath_fx::FixedMatrix;
```

#### 変更後:
```cpp
// types.hpp に統一定義
namespace common {
    using cm = cmath_fx::FixedMatrix;
}

// 各ファイルでは
using common::cm;
```

### 3.4 実施チェックリスト

- [ ] `sensor_filter_base.hpp` 削除
- [ ] `MEX/Inc/` ディレクトリ削除
- [ ] `robust_statistics.hpp` 重複定義削除
- [ ] `validation.hpp` 重複クラス削除
- [ ] using エイリアス統一
- [ ] ビルド確認
- [ ] 回帰テスト

---

## Phase 4: クラス設計の再構築

**目標**: クラスを簡素化し、関数ベース設計に移行  
**所要時間**: 4時間  
**リスク**: 中

### 4.1 クラス → 関数への変換対象

| 現在のクラス | 変換後 | 理由 |
|------------|-------|------|
| `MathUtils` (staticメソッドのみ) | 名前空間内関数 | インスタンス化不要 |
| `CovarianceRegularizer` (staticメソッドのみ) | 名前空間内関数 | インスタンス化不要 |
| `StateValidator` (staticメソッドのみ) | 名前空間内関数 | インスタンス化不要 |
| `SensorDataManager` | 削除 | 未使用 |
| `ESKFHelper` | 削除または統合 | ESKFCoreに統合可能 |

### 4.2 MathUtils クラスの変換

#### 現在:
```cpp
class MathUtils {
public:
    static const float EPS;
    static const float PI;
    static float wrap_to_pi(float angle);
    static float wrap_to_180(float angle);
    static cm normalize_vector(const cm& v);
    // ...
};
```

#### 変更後:
```cpp
namespace common {
namespace math {
    // 定数（#define使用）
    #define MATH_EPS 1e-9f
    #define MATH_PI 3.14159265358979323846f
    
    // 関数
    inline float wrap_to_pi(float angle);
    inline float wrap_to_180(float angle);
    inline cm normalize_vector(const cm& v);
}
}
```

### 4.3 保持するクラス（重要：ESKF/MEUKFは削除不可）

| クラス | 理由 |
|-------|------|
| `Matrix<R,C,T>` | テンプレート行列、状態保持が必要 |
| `FixedMatrix` | ランタイムサイズ行列、状態保持が必要 |
| **`ESKFCore`** | **予測ステップで使用中（削除不可）** |
| **`ESKFRunner`** | **予測統合で使用中（削除不可）** |
| **`MEUKFCore`** | **センサー更新で使用中（削除不可）** |
| `EMAFilter` | 状態（filtered_）を保持 |
| `AlphaBetaFilter` | 状態を保持 |
| `OutlierDetector` | 履歴（history_）を保持 |
| `NoiseEstimator` | 状態を保持 |
| `DivergenceGuard` | 状態を保持 |
| `SensorFilterLib` | センサーフィルタ統合 |

**注意**: `ESKFCore::update_accel/mag/gps/baro` は未使用だが、クラス自体は**予測で必須**。

### 4.4 実施チェックリスト

- [ ] `MathUtils` を関数に変換
- [ ] `CovarianceRegularizer` を関数に変換
- [ ] `StateValidator` を関数に変換
- [ ] `SensorDataManager` 削除
- [ ] 不要なクラス階層の削除
- [ ] ビルド確認
- [ ] 回帰テスト

---

## Phase 5: ファイル構成の再構築

**目標**: シンプルで独立したファイル構造  
**所要時間**: 3時間  
**リスク**: 高

### 5.1 新しいファイル構造

```
kalman/cpp/
├── bin/                    # MEXバイナリ出力
├── build/                  # ビルドスクリプト
│   └── build_mex.m
├── MEX/                    # MEXエントリーポイント
│   ├── mex_run_eskf.cpp    # ESKF MEX
│   ├── mex_meukf_step.cpp  # MEUKF MEX
│   └── impl/               # 実装（Impl/Inc統合）
│       ├── mex_common.hpp
│       ├── mex_type_conv.hpp
│       └── mex_helpers.hpp
└── Lib/                    # コアライブラリ
    ├── Matrix/
    │   └── fixed_matrix.hpp      # 行列テンプレート（668行→統合）
    ├── Quaternion/
    │   └── quaternion.hpp        # 四元数演算（統合・簡素化）
    ├── Math/                     # Common/inc/Mathから移動
    │   ├── constants.hpp         # 定数定義（新規）
    │   ├── geometry.hpp          # 幾何計算
    │   ├── numerical.hpp         # 数値計算
    │   └── statistics.hpp        # 統計計算
    ├── Sensor/                   # Common/inc/Sensorから移動
    │   ├── filters.hpp           # EMA/Biquad/AlphaBeta統合
    │   ├── outlier.hpp           # 外れ値検出
    │   ├── noise_estimator.hpp   # ノイズ推定
    │   └── preprocessor.hpp      # 前処理
    ├── Filter/                   # フィルタ共通
    │   ├── covariance.hpp        # 共分散管理（filter_mgmt統合）
    │   └── validation.hpp        # 状態検証
    ├── ESKF/
    │   ├── eskf_core.hpp
    │   ├── eskf_state.hpp
    │   └── src/*.cpp
    └── MEUKF/
        ├── meukf_core.hpp
        ├── meukf_types.hpp
        └── src/*.cpp
```

### 5.2 削除対象ディレクトリ・ファイル

| パス | 理由 |
|-----|------|
| `MEX/Inc/` | `MEX/Impl/`と重複 |
| `Lib/Common/` | 再構成後削除 |
| `Lib/KF/` | 未使用（ESKFが主要） |
| `Lib/EKF/` | 未使用 |
| `Lib/UKF/` | 未使用 |
| `cpp/inc/` | 旧構造、未使用 |

**注意**: `Lib/ESKF/` と `Lib/MEUKF/` は**削除不可**（ハイブリッド実装で両方使用中）。

### 5.3 ファイル統合計画

| 統合元 | 統合先 | 削減効果 |
|-------|-------|---------|
| `ema_filter.hpp` + `biquad_filter.hpp` + `alpha_beta_filter.hpp` | `Sensor/filters.hpp` | 3→1 |
| `math_utils.hpp` + `statistics.hpp` + `numerical.hpp` | 分割維持（ただし簡素化） | - |
| `robust_statistics.hpp` (750行) | `noise_estimator.hpp` (200行) | 75%削減 |
| `validation.hpp` (254行) + `filter_mgmt.hpp` | `Filter/covariance.hpp` + `Filter/validation.hpp` | 構造改善 |

### 5.4 依存関係の簡素化

#### 現在の依存グラフ（複雑）:
```
sensor_filter.hpp
 ├── kf_operations.hpp
 │    └── fixed_matrix.hpp
 ├── statistics.hpp
 │    └── fixed_matrix.hpp
 ├── geometry.hpp
 ├── numerical.hpp
 ├── portable_math.hpp
 ├── ema_filter.hpp
 ├── biquad_filter.hpp
 ├── alpha_beta_filter.hpp
 ├── robust_statistics.hpp
 │    └── (多数の依存)
 └── outlier_detector.hpp
      └── kf_operations.hpp
```

#### 目標の依存グラフ（シンプル）:
```
Sensor/filters.hpp
 └── Matrix/fixed_matrix.hpp

Sensor/outlier.hpp
 └── Matrix/fixed_matrix.hpp

Sensor/noise_estimator.hpp
 └── Matrix/fixed_matrix.hpp
```

### 5.5 実施チェックリスト

- [ ] 新ディレクトリ構造作成
- [ ] ファイル移動・統合
- [ ] include パス更新
- [ ] 旧ディレクトリ削除
- [ ] build_mex.m 更新
- [ ] ビルド確認
- [ ] 回帰テスト

---

## Phase 6: コメント・コーディング規約の統一

**目標**: 最小限の簡潔なコメント、統一されたスタイル  
**所要時間**: 2時間  
**リスク**: 低

### 6.1 コメント規約

#### 許可するコメント:
```cpp
// 関数の目的を1行で説明
inline float wrap_to_pi(float angle);

// 重要な注意事項のみ
// NOTE: q = [w,x,y,z] scalar-first
void normalize_quat(float q[4]);
```

#### 禁止するコメント:
```cpp
// ❌ 冗長な説明
/**
 * @brief Wrap angle to [-pi, pi]
 * @param angle Input angle in radians
 * @return Wrapped angle in radians
 * @note This function uses fmodf for portability
 * @see wrap_to_180 for degree version
 */

// ❌ 変更履歴
// 2025/12/22: コメントアウトを解除し、MATLAB側と同一のロジックを有効化

// ❌ デバッグ用途のコメント
// DEBUG: 以下のコードはデバッグ用
```

### 6.2 命名規約

| 要素 | 規約 | 例 |
|-----|------|-----|
| 名前空間 | lowercase | `common::math` |
| クラス | PascalCase | `ESKFCore` |
| 関数 | snake_case | `wrap_to_pi` |
| 変数 | snake_case | `filtered_value` |
| 定数 | UPPER_SNAKE | `MATH_PI` |
| テンプレート引数 | 単一大文字 | `T`, `R`, `C` |

### 6.3 モダンC++の制限

#### 禁止:
```cpp
// ❌ auto（明示的な型を使用）
auto result = compute();

// ❌ range-based for（インデックスループを使用）
for (auto& item : container);

// ❌ lambda
auto fn = [](int x) { return x * 2; };

// ❌ std::string（固定長配列を使用）
std::string name;

// ❌ std::vector（固定サイズ配列を使用）
std::vector<float> data;
```

#### 許可:
```cpp
// ✅ テンプレート（必須）
template<int R, int C, typename T>

// ✅ inline関数
inline float compute();

// ✅ static const
static const float EPS = 1e-9f;

// ✅ #define マクロ
#define MATH_PI 3.14159265358979323846f
```

### 6.4 実施チェックリスト

- [ ] Doxygenスタイルコメント削除
- [ ] 変更履歴コメント削除
- [ ] デバッグコメント削除
- [ ] 命名規約適用
- [ ] モダンC++使用箇所の修正
- [ ] ビルド確認
- [ ] 回帰テスト

---

## 実施スケジュール

| Phase | 所要時間 | 優先度 | 依存関係 |
|-------|---------|--------|---------|
| Phase 1 | 30分 | 高 | なし |
| Phase 2 | 2時間 | 高 | Phase 1 |
| Phase 3 | 3時間 | 高 | Phase 2 |
| Phase 4 | 4時間 | 中 | Phase 3 |
| Phase 5 | 3時間 | 中 | Phase 4 |
| Phase 6 | 2時間 | 低 | Phase 5 |

**合計所要時間**: 約14.5時間（2日間）

---

## リスク管理

### 各Phaseでのチェックポイント

```matlab
% Phase終了ごとに実施
cd kalman/cpp/build
clear mex
build_mex()
clear mex
cd ../..
run_batch_10sets()

% 期待結果
% Position RMSE: X<0.3m, Y<0.3m, Z<0.2m
% Attitude RMSE: Roll<0.5°, Pitch<0.5°, Yaw<1.0°
% 10/10 PASS
```

### ロールバック手順

```bash
# 失敗時はgit stashで変更を退避
git stash

# または特定のcommitに戻る
git checkout <last_working_commit>
```

---

## 完了基準

1. ✅ 全Markdownファイルが整理され、最新状態を反映
2. ✅ デバッグ・ログ機能が完全削除
3. ✅ クラス重複が解消（各クラス1定義のみ）
4. ✅ ファイル数が30%以上削減
5. ✅ `build_mex()` 成功
6. ✅ `run_batch_10sets()` 10/10 PASS
7. ✅ コード行数が20%以上削減
