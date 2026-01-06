#pragma once

#include "fixed_matrix.hpp"
#include <cmath>
#include <algorithm>

namespace cmath_fx {
namespace decomp {

// Generic Cholesky Decomposition (NxN)
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

// Robust Cholesky with multi-stage fallback
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

    // 2. Check minimum diagonal as proxy for eigenvalue
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
    if (cholesky<N, T>(A, L)) return true;

    // 5. Stronger regularization
    for (int i = 0; i < N; ++i) A(i, i) += static_cast<T>(1e-4);
    if (cholesky<N, T>(A, L)) return true;

    // 6. Fallback: diagonal sqrt approximation
    L = Matrix<N, N, T>::Zero();
    for (int i = 0; i < N; ++i) {
        L(i, i) = std::sqrt(std::max(static_cast<T>(0), A(i, i)));
    }
    return true; // succeed with approximation
}

// 3x3 optimized Cholesky
template <typename T>
bool cholesky_3x3_optimized(const Matrix<3, 3, T>& A, Matrix<3, 3, T>& L) {
    L = Matrix<3, 3, T>::Zero();

    if (A(0, 0) <= static_cast<T>(1e-12)) return false;
    L(0, 0) = std::sqrt(A(0, 0));

    L(1, 0) = A(1, 0) / L(0, 0);
    T val11 = A(1, 1) - L(1, 0) * L(1, 0);
    if (val11 <= static_cast<T>(1e-12)) return false;
    L(1, 1) = std::sqrt(val11);

    L(2, 0) = A(2, 0) / L(0, 0);
    L(2, 1) = (A(2, 1) - L(2, 0) * L(1, 0)) / L(1, 1);
    T val22 = A(2, 2) - L(2, 0) * L(2, 0) - L(2, 1) * L(2, 1);
    if (val22 <= static_cast<T>(1e-12)) return false;
    L(2, 2) = std::sqrt(val22);

    return true;
}

} // namespace decomp
} // namespace cmath_fx
