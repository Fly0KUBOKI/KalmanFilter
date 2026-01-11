# Phase 3: 冗長コード・重複の廃止

**目標**: 重複クラス定義、重複ファイルを統合・削除  
**所要時間**: 2時間  
**リスク**: 高（依存関係の確認が必須）  
**前提条件**: Phase 2 完了

---

## 1. 問題の概要

### 1.1 重複クラス定義

| クラス | 定義場所 | 状態 |
|-------|---------|------|
| NoiseEstimator | robust_statistics.hpp | ✅ 正式版 |
| NoiseEstimator | validation.hpp | ❌ 重複（削除対象） |
| OutlierDetector | outlier_detector.hpp | ✅ 正式版 |
| OutlierDetector | validation.hpp | ❌ 重複（削除対象） |
| DivergenceGuard | robust_statistics.hpp | ✅ 正式版 |
| DivergenceGuard | validation.hpp | ❌ 重複（削除対象） |

### 1.2 重複ディレクトリ

```
MEX/
├── Impl/                    ← 正式版
│   ├── mex_eskf_common.hpp
│   ├── mex_run_eskf_impl.hpp
│   ├── mex_run_eskf_sensor_updates.hpp
│   ├── mex_run_eskf.cpp
│   └── ...
└── Inc/                     ← 完全重複（削除対象）
    ├── mex_eskf_common.hpp
    ├── mex_run_eskf_impl.hpp
    ├── mex_run_eskf_sensor_updates.hpp
    └── ...
```

---

## 2. validation.hpp の修正

### 2.1 現状

`Lib/Common/inc/validation.hpp` (254行) に以下の重複定義がある:
- OutlierDetector (行30-100)
- NoiseEstimator (行101-180)
- DivergenceGuard (行181-254)

### 2.2 修正方針

1. 重複クラス定義を削除
2. 正式版をincludeに変更
3. 本来のバリデーション機能のみ残す

### 2.3 修正後の validation.hpp

```cpp
#pragma once

#ifndef COMMON_VALIDATION_HPP
#define COMMON_VALIDATION_HPP

#include "../../Matrix/fixed_matrix.hpp"
#include "Sensor/outlier_detector.hpp"
#include "Sensor/robust_statistics.hpp"
#include <cmath>
#include <cfloat>

namespace common {
namespace validation {

using cm = cmath_fx::FixedMatrix;

// 正式版クラスのエイリアス（後方互換性のため）
using OutlierDetector = common::sensor::OutlierDetector;
using NoiseEstimator = common::sensor::NoiseEstimator;
using DivergenceGuard = common::sensor::DivergenceGuard;

// ========== 共分散バリデーション ==========
// 共分散行列の対称性・正定値性を確認
template<typename T>
bool validate_covariance(const cmath_fx::Matrix<15,15,T>& P, T tolerance = 1e-6) {
    // 対称性チェック
    for(int i = 0; i < 15; ++i) {
        for(int j = i+1; j < 15; ++j) {
            if(std::fabs(P(i,j) - P(j,i)) > tolerance) {
                return false;
            }
        }
    }
    // 対角成分が正か
    for(int i = 0; i < 15; ++i) {
        if(P(i,i) <= 0) {
            return false;
        }
    }
    return true;
}

// ========== 状態バリデーション ==========
// 四元数の正規化チェック
template<typename T>
bool validate_quaternion(const T q[4], T tolerance = 1e-4) {
    T norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    return std::fabs(norm - 1.0) < tolerance;
}

// 数値の有限性チェック
template<typename T>
bool is_finite(const T* arr, int n) {
    for(int i = 0; i < n; ++i) {
        if(!std::isfinite(arr[i])) return false;
    }
    return true;
}

} // namespace validation
} // namespace common

#endif
```

### 2.4 削除される行数

- 削除: 約200行（重複クラス定義）
- 追加: 約50行（バリデーション関数）
- 結果: 254行 → 約60行

---

## 3. MEX/Inc/ ディレクトリの削除

### 3.1 削除前確認

```bash
# MEX/Inc/ をインクルードしている箇所を検索
grep -r "MEX/Inc/" kalman/cpp --include="*.hpp" --include="*.cpp"

# mex_run_eskf.cpp のincludeパスを確認
head -30 kalman\cpp\MEX\Impl\mex_run_eskf.cpp
```

**確認事項**:
- `mex_run_eskf.cpp` が `Inc/` と `Impl/` どちらを参照しているか
- 両方参照している場合、`Impl/` に統一

### 3.2 mex_run_eskf.cpp のinclude修正（必要に応じて）

```cpp
// 変更前（もし Inc/ を参照している場合）
#include "../Inc/mex_eskf_common.hpp"
#include "../Inc/mex_run_eskf_impl.hpp"

// 変更後（Impl/ に統一）
#include "mex_eskf_common.hpp"
#include "mex_run_eskf_impl.hpp"
```

### 3.3 削除実行

```bash
# フォルダ全体を削除
rmdir /s /q kalman\cpp\MEX\Inc
```

### 3.4 削除されるファイル

| ファイル | 行数 |
|---------|------|
| mex_eskf_common.hpp | 約200行 |
| mex_eskf_init.hpp | 約150行 |
| mex_run_eskf_impl.hpp | 約300行 |
| mex_run_eskf_sensor_updates.hpp | 約200行 |
| mex_type_conversion.hpp | 約150行 |
| sensor_preprocessor.hpp | 約100行 |
| **合計** | **約1,100行** |

---

## 4. eskf_runner.hpp の重複コード確認

### 4.1 確認対象

`Lib/ESKF/inc/eskf_runner.hpp` に以下の可能性:
- 他ファイルと重複するユーティリティ関数
- インライン化すべき小関数

### 4.2 確認コマンド

```bash
# eskf_runner.hpp の関数一覧
grep -n "inline\|void\|bool\|float\|double" kalman\cpp\Lib\ESKF\inc\eskf_runner.hpp | head -50
```

### 4.3 統合候補

以下の関数が複数ファイルに存在する場合、正式版1箇所に統合:
- `normalize_quaternion()` → `cquat::normalize_quat<T>()` に統一
- `symmetrize_covariance()` → `filter_mgmt.hpp` に統一
- `clamp()` → `common::math` に統一

---

## 5. 未使用ファイルの削除

### 5.1 確認対象ディレクトリ

| ディレクトリ | 状態 | 対応 |
|-------------|------|------|
| Lib/KF/ | 使用状況不明 | 確認後削除可能 |
| Lib/EKF/ | 使用状況不明 | 確認後削除可能 |
| Lib/UKF/ | 未完成 | 確認後削除可能 |

### 5.2 使用確認コマンド

```bash
# KF/ の使用確認
grep -r "KF/" kalman/cpp --include="*.hpp" --include="*.cpp" | grep -v "KF/inc"
grep -r "kf_operations" kalman/cpp --include="*.hpp" --include="*.cpp"

# EKF/ の使用確認
grep -r "EKF/" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -r "ekf_core" kalman/cpp --include="*.hpp" --include="*.cpp"

# UKF/ の使用確認
grep -r "UKF/" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -r "ukf_core" kalman/cpp --include="*.hpp" --include="*.cpp"
```

### 5.3 判断基準

- **使用されている**: 残す
- **ESKF/MEUKFからのみ参照**: kf_operations.hpp のみ残し、他は削除
- **未使用**: 削除

---

## 6. 詳細な実施手順

### Step 1: 依存関係の確認

```bash
# validation.hpp の使用箇所
grep -rn "validation.hpp" kalman/cpp --include="*.hpp" --include="*.cpp"

# 各重複クラスの使用箇所
grep -rn "OutlierDetector" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -rn "NoiseEstimator" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -rn "DivergenceGuard" kalman/cpp --include="*.hpp" --include="*.cpp"
```

### Step 2: validation.hpp の修正

1. バックアップ作成
2. 重複クラス定義を削除
3. include文を追加
4. using宣言で後方互換性を確保

### Step 3: MEX/Inc/ 削除

1. include パス確認
2. mex_run_eskf.cpp の修正（必要な場合）
3. Inc/ ディレクトリ削除

### Step 4: ビルド確認

```matlab
cd kalman/cpp/build
clear mex
build_mex()
```

### Step 5: 回帰テスト

```matlab
clear mex
cd ../..
run_batch_10sets()
```

---

## 7. 修正前後の比較

### 7.1 validation.hpp

```diff
 #pragma once
 
 #ifndef COMMON_VALIDATION_HPP
 #define COMMON_VALIDATION_HPP
 
 #include "../../Matrix/fixed_matrix.hpp"
+#include "Sensor/outlier_detector.hpp"
+#include "Sensor/robust_statistics.hpp"
 #include <cmath>
 #include <cfloat>
 
 namespace common {
 namespace validation {
 
 using cm = cmath_fx::FixedMatrix;
 
-// ========== 外れ値検出器 ==========
-class OutlierDetector {
-    // 約70行の定義を削除
-};
+// 正式版クラスのエイリアス
+using OutlierDetector = common::sensor::OutlierDetector;
+using NoiseEstimator = common::sensor::NoiseEstimator;
+using DivergenceGuard = common::sensor::DivergenceGuard;
-
-// ========== ノイズ推定器 ==========
-class NoiseEstimator {
-    // 約80行の定義を削除
-};
-
-// ========== 発散防止 ==========
-class DivergenceGuard {
-    // 約70行の定義を削除
-};
 
+// バリデーション関数のみ残す
+template<typename T>
+bool validate_covariance(const cmath_fx::Matrix<15,15,T>& P, T tolerance = 1e-6);
+
+template<typename T>
+bool validate_quaternion(const T q[4], T tolerance = 1e-4);
+
+template<typename T>
+bool is_finite(const T* arr, int n);
 
 } // namespace validation
 } // namespace common
 
 #endif
```

---

## 8. 完了確認チェックリスト

- [ ] validation.hpp から重複クラス定義を削除
- [ ] validation.hpp にincludeとusingエイリアスを追加
- [ ] MEX/Inc/ ディレクトリを削除
- [ ] mex_run_eskf.cpp のincludeパスを確認・修正
- [ ] 未使用のKF/EKF/UKFファイルを確認
- [ ] `build_mex()` 成功
- [ ] `run_batch_10sets()` 10/10 PASS
- [ ] Git commit 完了

---

## 9. 削減効果

| 項目 | Before | After | 削減 |
|-----|--------|-------|------|
| validation.hpp | 254行 | 60行 | 76% |
| MEX/Inc/ | 1,100行 | 0行 | 100% |
| **合計** | **1,354行** | **60行** | **96%** |

---

## 10. 次のPhaseへの移行条件

- [x] 全重複クラス定義が統合済み
- [x] 重複ディレクトリ（MEX/Inc/）が削除済み
- [x] ビルド成功
- [x] 回帰テスト合格

**次のPhase**: Phase 4 - クラス設計の最適化
