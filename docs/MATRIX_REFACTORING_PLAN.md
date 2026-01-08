# 行列計算の統一・改善計画

## 🎯 目標

1. **Matrixライブラリ**: 基本的な行列演算を完全に統一
2. **KFライブラリ**: カルマンフィルタ固有の演算を集約
3. **コードの簡潔性**: 直感的で保守性の高い実装に改善
4. **重複削減**: 約340行のコード削減

---

## 📋 全体スケジュール

| Phase | 内容 | 期間 | リスク |
|-------|------|------|--------|
| Phase 1 | Matrix層の強化・統一 | 2-3日 | 低 |
| Phase 2 | KF層の整理・集約 | 2-3日 | 中 |
| Phase 3 | 既存コードの移行 | 3-4日 | 高 |
| Phase 4 | ユーティリティ再編成 | 1-2日 | 低 |
| Phase 5 | 検証・最適化 | 2-3日 | 中 |

**総所要期間**: 10-15日（テスト含む）

---

## 🏗️ Phase 1: Matrix層の強化・統一

### 目標
基本的な行列演算を `Matrix/fixed_matrix.hpp` に完全統一し、他の実装を削除可能にする。

### 1.1 Cholesky分解の統一

#### 新規実装: `Matrix/matrix_decomposition.hpp`
```cpp
#pragma once
#include "fixed_matrix.hpp"

namespace cmath_fx {
namespace decomp {

// ========================================
// Generic Cholesky Decomposition
// ========================================

// Standard Cholesky (任意サイズ、正定値必須)
template <int N, typename T = float>
bool cholesky(const Matrix<N, N, T>& A, Matrix<N, N, T>& L) {
    L = Matrix<N, N, T>::Zero();
    
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j <= i; ++j) {
            T sum = static_cast<T>(0);
            for (int k = 0; k < j; ++k) {
                sum += L(i, k) * L(j, k);
            }
            
            if (i == j) {
                T val = A(i, i) - sum;
                if (val <= static_cast<T>(1e-12)) return false;
                L(i, i) = std::sqrt(val);
            } else {
                if (std::abs(L(j, j)) < static_cast<T>(1e-12)) return false;
                L(i, j) = (A(i, j) - sum) / L(j, j);
            }
        }
    }
    return true;
}

// Robust Cholesky（多段フォールバック）
template <int N, typename T = float>
bool cholesky_robust(Matrix<N, N, T>& A, Matrix<N, N, T>& L) {
    // 1. Symmetrize
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            T avg = (A(i, j) + A(j, i)) * static_cast<T>(0.5);
            A(i, j) = avg;
            A(j, i) = avg;
        }
    }
    
    // 2. Check minimum eigenvalue (approximated by min diagonal)
    T min_diag = A(0, 0);
    for (int i = 1; i < N; ++i) {
        if (A(i, i) < min_diag) min_diag = A(i, i);
    }
    
    // 3. Regularize if not positive definite
    if (min_diag <= static_cast<T>(0)) {
        T reg = std::abs(min_diag) + static_cast<T>(1e-6);
        for (int i = 0; i < N; ++i) A(i, i) += reg;
    }
    
    // 4. Try standard Cholesky
    if (cholesky(A, L)) return true;
    
    // 5. Stronger regularization
    for (int i = 0; i < N; ++i) A(i, i) += static_cast<T>(1e-4);
    if (cholesky(A, L)) return true;
    
    // 6. Fallback: diagonal approximation
    L = Matrix<N, N, T>::Zero();
    for (int i = 0; i < N; ++i) {
        L(i, i) = std::sqrt(std::max(static_cast<T>(0), A(i, i)));
    }
    return true;  // Always succeeds
}

// 3x3専用最適化版（コンパイル時特殊化）
template <typename T>
bool cholesky_3x3_optimized(const Matrix<3, 3, T>& A, Matrix<3, 3, T>& L) {
    // 展開されたループ（高速化）
    L = Matrix<3, 3, T>::Zero();
    
    // L(0,0)
    if (A(0, 0) <= static_cast<T>(1e-12)) return false;
    L(0, 0) = std::sqrt(A(0, 0));
    
    // L(1,0), L(1,1)
    L(1, 0) = A(1, 0) / L(0, 0);
    T val11 = A(1, 1) - L(1, 0) * L(1, 0);
    if (val11 <= static_cast<T>(1e-12)) return false;
    L(1, 1) = std::sqrt(val11);
    
    // L(2,0), L(2,1), L(2,2)
    L(2, 0) = A(2, 0) / L(0, 0);
    L(2, 1) = (A(2, 1) - L(2, 0) * L(1, 0)) / L(1, 1);
    T val22 = A(2, 2) - L(2, 0) * L(2, 0) - L(2, 1) * L(2, 1);
    if (val22 <= static_cast<T>(1e-12)) return false;
    L(2, 2) = std::sqrt(val22);
    
    return true;
}

} // namespace decomp
} // namespace cmath_fx
```

#### 削除対象
- `ukf_utils::cholesky3x3()` → `decomp::cholesky_3x3_optimized()` に置換
- `ukf_utils::cholesky3x3_robust()` → `decomp::cholesky_robust<3>()` に置換
- `meukf::cholesky3x3()` → 削除、上記関数を使用
- `meukf::cholesky3x3_robust()` → 削除
- `MathUtils::safe_cholesky()` → `decomp::cholesky_robust()` に置換

---

### 1.2 3x3逆行列の統一

#### 新規実装: `Matrix/matrix_inverse.hpp`
```cpp
#pragma once
#include "fixed_matrix.hpp"

namespace cmath_fx {
namespace inv {

// 解析的逆行列（3x3専用、高速）
template <typename T>
bool inverse_3x3_analytic(const Matrix<3, 3, T>& A, Matrix<3, 3, T>& A_inv) {
    // 行列式を計算
    T det = A(0,0) * (A(1,1)*A(2,2) - A(2,1)*A(1,2))
          - A(0,1) * (A(1,0)*A(2,2) - A(2,0)*A(1,2))
          + A(0,2) * (A(1,0)*A(2,1) - A(2,0)*A(1,1));
    
    if (std::abs(det) < static_cast<T>(1e-10)) {
        return false;
    }
    
    T inv_det = static_cast<T>(1.0) / det;
    
    // 随伴行列を計算
    A_inv(0,0) = (A(1,1)*A(2,2) - A(2,1)*A(1,2)) * inv_det;
    A_inv(0,1) = (A(0,2)*A(2,1) - A(0,1)*A(2,2)) * inv_det;
    A_inv(0,2) = (A(0,1)*A(1,2) - A(0,2)*A(1,1)) * inv_det;
    A_inv(1,0) = (A(1,2)*A(2,0) - A(1,0)*A(2,2)) * inv_det;
    A_inv(1,1) = (A(0,0)*A(2,2) - A(0,2)*A(2,0)) * inv_det;
    A_inv(1,2) = (A(1,0)*A(0,2) - A(0,0)*A(1,2)) * inv_det;
    A_inv(2,0) = (A(1,0)*A(2,1) - A(2,0)*A(1,1)) * inv_det;
    A_inv(2,1) = (A(2,0)*A(0,1) - A(0,0)*A(2,1)) * inv_det;
    A_inv(2,2) = (A(0,0)*A(1,1) - A(1,0)*A(0,1)) * inv_det;
    
    return true;
}

// 汎用逆行列（Gauss-Jordan、任意サイズ）
template <int N, typename T>
bool inverse_gauss_jordan(const Matrix<N, N, T>& A, Matrix<N, N, T>& inv) {
    // 既存のMatrix::inverse()のロジックをここに移動
    // （省略: fixed_matrix.hppのL96-143と同一）
    return A.inverse(inv);
}

// 自動選択（サイズに応じて最適なアルゴリズム）
template <int N, typename T>
bool inverse(const Matrix<N, N, T>& A, Matrix<N, N, T>& inv) {
    if constexpr (N == 3) {
        return inverse_3x3_analytic(A, inv);
    } else {
        return inverse_gauss_jordan(A, inv);
    }
}

} // namespace inv
} // namespace cmath_fx
```

#### 削除対象
- `MathUtils::invert3x3()` → `inv::inverse_3x3_analytic()` に置換

---

### 1.3 対称化処理の統一

#### 新規実装: `Matrix/matrix_utils.hpp`
```cpp
#pragma once
#include "fixed_matrix.hpp"

namespace cmath_fx {
namespace utils {

// 行列の対称化（テンプレート版）
template <int N, typename T>
void symmetrize(Matrix<N, N, T>& M) {
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            T avg = (M(i, j) + M(j, i)) * static_cast<T>(0.5);
            M(i, j) = avg;
            M(j, i) = avg;
        }
    }
}

// 演算子版（非破壊的）
template <int N, typename T>
Matrix<N, N, T> symmetrize_copy(const Matrix<N, N, T>& M) {
    Matrix<N, N, T> result = M;
    symmetrize(result);
    return result;
}

// 正定値化（対角成分の正則化）
template <int N, typename T>
void ensure_positive_definite(Matrix<N, N, T>& M, T min_eigenvalue = static_cast<T>(1e-8)) {
    // 1. Symmetrize
    symmetrize(M);
    
    // 2. Check minimum diagonal
    T min_diag = M(0, 0);
    for (int i = 1; i < N; ++i) {
        if (M(i, i) < min_diag) min_diag = M(i, i);
    }
    
    // 3. Regularize if needed
    if (min_diag < min_eigenvalue) {
        T reg = min_eigenvalue - min_diag;
        for (int i = 0; i < N; ++i) {
            M(i, i) += reg;
        }
        symmetrize(M);  // Re-symmetrize
    }
}

} // namespace utils
} // namespace cmath_fx
```

#### 置換対象
- `P = (P + P.transpose()) * 0.5f;` → `utils::symmetrize(P);`
- `MathUtils::enforce_symmetry()` → `utils::symmetrize_copy()`
- `ukf_utils::ensure_positive_definite_3x3()` → `utils::ensure_positive_definite<3>()`

---

### 1.4 ファイル構成（新Matrix層）

```
Lib/Matrix/
├── fixed_matrix.hpp           # 基本行列クラス（既存）
├── matrix_decomposition.hpp   # NEW: Cholesky, LU, QR など
├── matrix_inverse.hpp          # NEW: 逆行列計算
├── matrix_utils.hpp            # NEW: 対称化、正定値化など
└── README.md                   # NEW: Matrix層のAPI仕様書
```

---

## 🔧 Phase 2: KF層の整理・集約

### 目標
カルマンフィルタ固有の演算を `KF/` に完全集約し、他ライブラリからの依存を削除。

### 2.1 KF共通演算の標準化

#### 新規実装: `KF/inc/kf_operations.hpp`
```cpp
#pragma once

#include "../../Matrix/fixed_matrix.hpp"
#include "../../Matrix/matrix_decomposition.hpp"
#include "../../Matrix/matrix_inverse.hpp"
#include "../../Matrix/matrix_utils.hpp"

namespace kf {

// ========================================
// Kalman Filter Standard Operations
// ========================================

// Innovation と Innovation Covariance の計算
template <int M, int N, typename T>
struct InnovationResult {
    cmath_fx::Vector<M, T> y;        // Innovation
    cmath_fx::Matrix<M, M, T> S;      // Innovation covariance
};

template <int M, int N, typename T>
InnovationResult<M, N, T> compute_innovation(
    const cmath_fx::Vector<M, T>& z,          // Measurement
    const cmath_fx::Vector<M, T>& h,          // Predicted measurement
    const cmath_fx::Matrix<M, N, T>& H,       // Observation matrix
    const cmath_fx::Matrix<N, N, T>& P,       // State covariance
    const cmath_fx::Matrix<M, M, T>& R        // Measurement noise
) {
    InnovationResult<M, N, T> result;
    
    // y = z - h
    result.y = z - h;
    
    // S = H * P * H' + R
    result.S = H * P * H.transpose() + R;
    
    // Symmetrize
    cmath_fx::utils::symmetrize(result.S);
    
    return result;
}

// Kalman Gain の計算
template <int N, int M, typename T>
cmath_fx::Matrix<N, M, T> compute_kalman_gain(
    const cmath_fx::Matrix<N, N, T>& P,       // State covariance
    const cmath_fx::Matrix<M, N, T>& H,       // Observation matrix
    const cmath_fx::Matrix<M, M, T>& S        // Innovation covariance
) {
    cmath_fx::Matrix<M, M, T> S_inv;
    if (!cmath_fx::inv::inverse(S, S_inv)) {
        // Singular matrix - return zero gain
        return cmath_fx::Matrix<N, M, T>::Zero();
    }
    
    // K = P * H' * S^-1
    return P * H.transpose() * S_inv;
}

// 状態更新 (Joseph form)
template <int N, int M, typename T>
struct UpdateResult {
    cmath_fx::Vector<N, T> x;        // Updated state
    cmath_fx::Matrix<N, N, T> P;      // Updated covariance
};

template <int N, int M, typename T>
UpdateResult<N, M, T> update_state_joseph(
    const cmath_fx::Vector<N, T>& x_pred,         // Predicted state
    const cmath_fx::Matrix<N, N, T>& P_pred,      // Predicted covariance
    const cmath_fx::Matrix<N, M, T>& K,           // Kalman gain
    const cmath_fx::Matrix<M, N, T>& H,           // Observation matrix
    const cmath_fx::Vector<M, T>& y,              // Innovation
    const cmath_fx::Matrix<M, M, T>& R            // Measurement noise
) {
    UpdateResult<N, M, T> result;
    
    // x = x_pred + K * y
    result.x = x_pred + K * y;
    
    // P = (I - K*H) * P_pred * (I - K*H)' + K*R*K' (Joseph form)
    auto I = cmath_fx::Matrix<N, N, T>::Identity();
    auto I_KH = I - K * H;
    result.P = I_KH * P_pred * I_KH.transpose() + K * R * K.transpose();
    
    // Symmetrize
    cmath_fx::utils::symmetrize(result.P);
    
    return result;
}

// 簡易版状態更新（Joseph formなし）
template <int N, int M, typename T>
UpdateResult<N, M, T> update_state_simple(
    const cmath_fx::Vector<N, T>& x_pred,
    const cmath_fx::Matrix<N, N, T>& P_pred,
    const cmath_fx::Matrix<N, M, T>& K,
    const cmath_fx::Matrix<M, N, T>& H,
    const cmath_fx::Vector<M, T>& y
) {
    UpdateResult<N, M, T> result;
    
    // x = x_pred + K * y
    result.x = x_pred + K * y;
    
    // P = (I - K*H) * P_pred
    auto I = cmath_fx::Matrix<N, N, T>::Identity();
    result.P = (I - K * H) * P_pred;
    
    // Symmetrize
    cmath_fx::utils::symmetrize(result.P);
    
    return result;
}

// Mahalanobis距離の計算
template <int M, typename T>
T mahalanobis_distance_squared(
    const cmath_fx::Vector<M, T>& innovation,
    const cmath_fx::Matrix<M, M, T>& S
) {
    cmath_fx::Matrix<M, M, T> S_inv;
    if (!cmath_fx::inv::inverse(S, S_inv)) {
        // Fallback: use max diagonal variance
        T max_var = S(0, 0);
        for (int i = 1; i < M; ++i) {
            if (S(i, i) > max_var) max_var = S(i, i);
        }
        if (max_var < static_cast<T>(1e-9)) max_var = static_cast<T>(1.0);
        
        T norm_sq = static_cast<T>(0);
        for (int i = 0; i < M; ++i) {
            norm_sq += innovation(i, 0) * innovation(i, 0);
        }
        return norm_sq / max_var;
    }
    
    // d^2 = y' * S^-1 * y
    auto tmp = S_inv * innovation;
    T dist_sq = static_cast<T>(0);
    for (int i = 0; i < M; ++i) {
        dist_sq += innovation(i, 0) * tmp(i, 0);
    }
    return dist_sq;
}

} // namespace kf
```

---

### 2.2 既存コードの置換

#### `kalman_filter_core.hpp` のリファクタリング
```cpp
// BEFORE
template <int N, int M, typename T>
static cmath_fx::Matrix<N, M, T> compute_kalman_gain(...) {
    cmath_fx::Matrix<M, M, T> S_inv;
    if (!S.inverse(S_inv)) {
        return cmath_fx::Matrix<N, M, T>::Zero();
    }
    return (P * H.transpose()) * S_inv;
}

// AFTER
template <int N, int M, typename T>
static cmath_fx::Matrix<N, M, T> compute_kalman_gain(...) {
    return kf::compute_kalman_gain<N, M, T>(P, H, S);
}
```

#### `MathUtils` からの分離
- `compute_innovation_and_S()` → `kf::compute_innovation()` に移動
- `mahalanobis_distance_squared()` → `kf::mahalanobis_distance_squared()` に移動

---

## 🔄 Phase 3: 既存コードの移行

### 3.1 `kf_core.hpp` のリファクタリング

#### BEFORE（直接ループ実装）
```cpp
// Predict: P = F*P*F' + Q
cm FP; FP.resize(n_, n_);
for (int i = 0; i < n_; i++) {
    for (int j = 0; j < n_; j++) {
        float sum = 0.0f;
        for (int k = 0; k < n_; k++) {
            sum += F(i, k) * P_(k, j);
        }
        FP(i, j) = sum;
    }
}
cm P_new; P_new.resize(n_, n_);
for (int i = 0; i < n_; i++) {
    for (int j = 0; j < n_; j++) {
        float sum = Q_(i, j);
        for (int k = 0; k < n_; k++) {
            sum += FP(i, k) * F(j, k);
        }
        P_new(i, j) = sum;
    }
}
P_ = P_new;
```

#### AFTER（演算子オーバーロード活用）
```cpp
// Predict: P = F*P*F' + Q
P_ = F * P_ * F.transpose() + Q_;
cmath_fx::utils::symmetrize(P_);
```

**削減行数**: 約20行 → 2行

---

### 3.2 `meukf_core.cpp` の独自実装削除

#### 削除する static 関数
```cpp
// Line 252-285: cholesky3x3() → cmath_fx::decomp::cholesky_3x3_optimized()
// Line 288-327: cholesky3x3_robust() → cmath_fx::decomp::cholesky_robust<3>()
// Line 331-360: ensure_positive_definite() → cmath_fx::utils::ensure_positive_definite<3>()
```

#### 置換例
```cpp
// BEFORE
Matrix3x3 L;
if (!cholesky3x3_robust(P_att_copy, L)) {
    // fallback
}

// AFTER
using namespace cmath_fx;
Matrix3x3 L;
if (!decomp::cholesky_robust<3>(P_att_copy, L)) {
    // fallback
}
```

**削減行数**: 約130行

---

### 3.3 `ukf_utils.hpp` の重複削除

#### 削除対象
- `cholesky3x3()` → L23-51（29行）
- `cholesky3x3_robust()` → L54-90（37行）
- `ensure_positive_definite_3x3()` → L93-122（30行）

**削減行数**: 約96行

#### 置換先
```cpp
#include "../../Matrix/matrix_decomposition.hpp"
using namespace cmath_fx::decomp;
```

---

## 🗂️ Phase 4: ユーティリティ再編成

### 目標
`math_utils.hpp` を分割し、責務を明確化。

### 4.1 新ファイル構成

```
Lib/Common/inc/Math/
├── math_utils.hpp           # 角度処理、数値安定化（残存）
├── matrix_operations.hpp    # NEW: 行列演算（Matrixライブラリへ移動予定）
├── statistics.hpp           # 既存: 統計関数（median, MAD, robust_statistics）
├── geometry.hpp             # NEW: 座標変換（LLA⟷ENU）
└── numerical.hpp            # NEW: 補間、安全演算
```

### 4.2 分割内容

#### `math_utils.hpp`（簡素化後）
- 角度処理: `wrap_to_pi`, `wrap_to_180`, `angle_difference`
- 数値安定化: `safe_divide`, `safe_sqrt`, `safe_asin`, `safe_acos`

#### `geometry.hpp`（新規）
- 座標変換: `lla_to_enu`, `enu_to_lla`
- 回転行列: `euler_to_rotm`, `rotm_to_euler`（必要に応じて）

#### `numerical.hpp`（新規）
- 補間: `linear_interpolate`
- その他数値計算

#### 削除対象
- `enforce_symmetry()` → Matrix層へ移動
- `compute_innovation_and_S()` → KF層へ移動
- `invert3x3()` → Matrix層へ移動
- `safe_cholesky()` → Matrix層へ移動
- `mahalanobis_distance_*()` → KF層へ移動

---

## ✅ Phase 5: 検証・最適化

### 5.1 単体テスト

#### 新規テストファイル: `tests/test_matrix_operations.cpp`
```cpp
#include "../Lib/Matrix/matrix_decomposition.hpp"
#include "../Lib/Matrix/matrix_inverse.hpp"
#include "../Lib/Matrix/matrix_utils.hpp"
#include <cassert>
#include <iostream>

void test_cholesky() {
    using namespace cmath_fx;
    
    // 3x3 positive definite matrix
    Matrix<3, 3, float> A;
    A(0,0) = 4.0f; A(0,1) = 2.0f; A(0,2) = 1.0f;
    A(1,0) = 2.0f; A(1,1) = 5.0f; A(1,2) = 3.0f;
    A(2,0) = 1.0f; A(2,1) = 3.0f; A(2,2) = 6.0f;
    
    Matrix<3, 3, float> L;
    bool ok = decomp::cholesky<3>(A, L);
    assert(ok);
    
    // Verify: A = L * L'
    auto LL_t = L * L.transpose();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            assert(std::abs(LL_t(i,j) - A(i,j)) < 1e-5f);
        }
    }
    
    std::cout << "test_cholesky: PASSED\n";
}

void test_inverse_3x3() {
    using namespace cmath_fx;
    
    Matrix<3, 3, float> A;
    A(0,0) = 4.0f; A(0,1) = 7.0f; A(0,2) = 2.0f;
    A(1,0) = 3.0f; A(1,1) = 6.0f; A(1,2) = 1.0f;
    A(2,0) = 2.0f; A(2,1) = 5.0f; A(2,2) = 3.0f;
    
    Matrix<3, 3, float> A_inv;
    bool ok = inv::inverse_3x3_analytic(A, A_inv);
    assert(ok);
    
    // Verify: A * A_inv = I
    auto I_test = A * A_inv;
    auto I_expected = Matrix<3, 3, float>::Identity();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            assert(std::abs(I_test(i,j) - I_expected(i,j)) < 1e-4f);
        }
    }
    
    std::cout << "test_inverse_3x3: PASSED\n";
}

int main() {
    test_cholesky();
    test_inverse_3x3();
    // ... 他のテスト
    return 0;
}
```

---

### 5.2 回帰テスト

既存のMATLABテストを実行して数値差がないことを確認:
```matlab
% Phase 3完了後
run_batch_10sets();

% Results/batch_10sets_results.mat を旧版と比較
% - RMSE差分 < 1e-6
% - max_error差分 < 1e-5
```

---

### 5.3 パフォーマンス測定

#### ベンチマーク対象
1. Cholesky分解（汎用 vs 3x3最適化）
2. 逆行列（Gauss-Jordan vs 解析的）
3. 行列積（演算子 vs 直接ループ）

#### 期待値
- 3x3最適化版: 20-30% 高速化
- 解析的逆行列: 40-50% 高速化
- 演算子オーバーロード: パフォーマンス同等（可読性向上が主目的）

---

## 📊 期待される効果

### コード削減量
| 削除内容 | 行数 |
|---------|------|
| Cholesky重複実装 | 約160行 |
| 3x3逆行列重複 | 約30行 |
| 対称化処理重複 | 約30行 |
| Innovation計算重複 | 約60行 |
| 正定値化重複 | 約60行 |
| **合計** | **約340行** |

### 可読性向上
```cpp
// BEFORE（KF/kf_core.hpp L58-85）
cm P_new; P_new.resize(n_, n_);
for (int i = 0; i < n_; i++) {
    for (int j = 0; j < n_; j++) {
        float sum = Q_(i, j);
        for (int k = 0; k < n_; k++) {
            sum += FP(i, k) * F(j, k);
        }
        P_new(i, j) = sum;
    }
}
P_ = P_new;

for (int i = 0; i < n_; i++) {
    for (int j = i+1; j < n_; j++) {
        float avg = (P_(i,j) + P_(j,i)) * 0.5f;
        P_(i,j) = avg;
        P_(j,i) = avg;
    }
}

// AFTER
P_ = F * P_ * F.transpose() + Q_;
cmath_fx::utils::symmetrize(P_);
```

**可読性**: 20行 → 2行（90%削減）

---

## 🚨 リスク管理

### Phase 3のリスク軽減策

#### リスク1: meukf_core.cpp の巨大さ（1346行）
**対策**:
- 段階的移行（1関数ずつ置換）
- 各置換後に `run_simulation(42, true)` で検証
- 差分が0.1%以内であることを確認

#### リスク2: 型変換の不整合
**対策**:
- float/double混在に注意（Phase 3で部分対応済み）
- テンプレート関数は明示的な型指定を推奨
- コンパイル時警告を有効化

#### リスク3: MEX互換性
**対策**:
- FixedMatrixクラスは維持
- テンプレート⟷ランタイム変換関数を追加

---

## 🔄 移行手順（Phase 3詳細）

### ステップ1: Matrix層の新ファイル作成
1. `matrix_decomposition.hpp` 作成
2. `matrix_inverse.hpp` 作成
3. `matrix_utils.hpp` 作成
4. 単体テスト実行

### ステップ2: KF層の新ファイル作成
1. `kf_operations.hpp` 作成
2. `kalman_filter_core.hpp` をリファクタリング
3. 単体テスト実行

### ステップ3: ukf_utils.hpp 置換
1. インクルード追加: `#include "../../Matrix/matrix_decomposition.hpp"`
2. 重複関数を削除
3. 呼び出し箇所を置換
4. `run_simulation` でテスト

### ステップ4: meukf_core.cpp 置換
1. static関数3つを削除
2. 呼び出し箇所を新APIに置換（約10箇所）
3. `run_batch_10sets()` で回帰テスト

### ステップ5: kf_core.hpp リファクタリング
1. predict() 関数を演算子版に置換
2. update() 関数を演算子版に置換
3. 全テストスイート実行

### ステップ6: math_utils.hpp 分割
1. geometry.hpp に座標変換を移動
2. numerical.hpp に補間を移動
3. 行列関連を削除（既にMatrix層へ移動済み）

### ステップ7: 最終検証
1. `run_batch_10sets()` で10seed統計確認
2. パフォーマンスベンチマーク実行
3. ドキュメント更新

---

## 📚 ドキュメント更新

### 新規作成
- `Lib/Matrix/README.md`: Matrix APIリファレンス
- `Lib/KF/README.md`: KF演算APIリファレンス

### 更新対象
- `docs/CPP_ARCHITECTURE.md`: 新しいライブラリ構成を反映
- `kalman/cpp/markdown/LIB_FUNCTION_REFERENCE.md`: 関数一覧を更新
- `.github/copilot-instructions.md`: 新しい使用パターンを追記

---

## ✅ 完了条件

### Phase 1
- [ ] `matrix_decomposition.hpp` 実装完了
- [ ] `matrix_inverse.hpp` 実装完了
- [ ] `matrix_utils.hpp` 実装完了
- [ ] 単体テスト全パス

### Phase 2
- [ ] `kf_operations.hpp` 実装完了
- [ ] `kalman_filter_core.hpp` リファクタリング完了
- [ ] Innovation/Gain計算の移行完了

### Phase 3
- [ ] `ukf_utils.hpp` 重複削除完了
- [ ] `meukf_core.cpp` 独自実装削除完了
- [ ] `kf_core.hpp` リファクタリング完了
- [ ] 回帰テスト全パス（数値差 < 1e-5）

### Phase 4
- [ ] `math_utils.hpp` 分割完了
- [ ] `geometry.hpp` 作成完了
- [ ] `numerical.hpp` 作成完了

### Phase 5
- [ ] 単体テスト全パス
- [ ] 回帰テスト（10seed）全パス
- [ ] パフォーマンスベンチマーク実施
- [ ] ドキュメント更新完了

## 進捗状況 (2026-01-06)

- **Phase 1 完了**: `matrix_decomposition.hpp`, `matrix_inverse.hpp`, `matrix_utils.hpp` を追加し、主要な重複を委譲しました。
- **KF 層**: `Lib/KF/inc/kf_operations.hpp` を追加し、複数の Joseph 形式更新・共分散処理を委譲しました（部分完了）。
- **コンパイル & テスト**: 主要ソースのコンパイル確認と `tests/test_matrix_operations.cpp` による単体検証を実行、テストは通過しました。その後テスト用ファイルは削除しました。
- **回帰テスト**: MATLAB 回帰 `run_batch_10sets()` を実行し、直近ログで 10/10 PASS を確認しました（ログ: `kalman/Results/log/`）。
- **現在の状態**: 残りの重複箇所を小分けで自動置換中。差し替え毎にコンパイル確認を行っています。
- **次のステップ**: 残り置換の継続 → コンパイル確認 → 最終回帰 → ドキュメント更新 → PR 作成。

（必要ならこの進捗を別ファイル `docs/PROGRESS.md` として分離します。希望を教えてください）

---

## 🎉 期待される成果

1. **保守性向上**: 重複削減により修正箇所が1/5に
2. **可読性向上**: 直接ループ → 演算子で直感的に
3. **バグ低減**: 統一APIにより不整合を防止
4. **パフォーマンス**: 3x3最適化で20-30%高速化
5. **テスト容易性**: 単体テスト可能なモジュール設計

---

**次ステップ**: Phase 1から順次実装を開始してください。各Phaseごとにテストを実行し、問題がないことを確認してから次へ進めてください。
