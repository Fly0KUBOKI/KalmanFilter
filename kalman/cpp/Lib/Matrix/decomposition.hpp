#pragma once

#include "matrix.hpp"
#include <algorithm>

namespace lib {
namespace matrix {

// コレスキー分解 (下三角行列)
// 成功時: true, L に結果
// 失敗時: false (正定値でない)
template<Index N, typename T = Scalar>
bool cholesky(const Mat<N, N, T>& A, Mat<N, N, T>& L) {
    // Lをゼロ初期化
    L = Mat<N, N, T>::Zero();
    
    for (Index i = 0; i < N; ++i) {
        for (Index j = 0; j <= i; ++j) {
            T sum = static_cast<T>(0);
            
            if (i == j) {
                // 対角要素
                for (Index k = 0; k < j; ++k) {
                    sum += L(j, k) * L(j, k);
                }
                T diag = A(j, j) - sum;
                if (diag <= static_cast<T>(0)) {
                    return false;  // 正定値でない
                }
                L(j, j) = std::sqrt(diag);
            } else {
                // 非対角要素
                for (Index k = 0; k < j; ++k) {
                    sum += L(i, k) * L(j, k);
                }
                if (std::abs(L(j, j)) < static_cast<T>(1e-12f)) {
                    return false;  // ゼロ除算回避
                }
                L(i, j) = (A(i, j) - sum) / L(j, j);
            }
        }
    }
    return true;
}

// 堅牢コレスキー分解 (多段フォールバック)
template<Index N, typename T = Scalar>
bool cholesky_robust(Mat<N, N, T>& A, Mat<N, N, T>& L, T reg = static_cast<T>(1e-6f)) {
    // 1. 対称化
    for (Index i = 0; i < N; ++i) {
        for (Index j = i + 1; j < N; ++j) {
            T avg = (A(i, j) + A(j, i)) * static_cast<T>(0.5f);
            A(i, j) = avg;
            A(j, i) = avg;
        }
    }
    
    // 2. 最小対角要素チェック
    T min_diag = A(0, 0);
    for (Index i = 1; i < N; ++i) {
        if (A(i, i) < min_diag) {
            min_diag = A(i, i);
        }
    }
    
    // 3. 正定値でない場合は正則化
    if (min_diag <= static_cast<T>(0)) {
        T regularization = std::abs(min_diag) + reg;
        for (Index i = 0; i < N; ++i) {
            A(i, i) += regularization;
        }
    }
    
    // 4. 通常のコレスキー分解
    if (cholesky(A, L)) {
        return true;
    }
    
    // 5. より強い正則化で再試行
    T strong_reg = static_cast<T>(1e-4f);
    for (Index i = 0; i < N; ++i) {
        A(i, i) += strong_reg;
    }
    if (cholesky(A, L)) {
        return true;
    }
    
    // 6. 最終手段: 対角近似
    L = Mat<N, N, T>::Zero();
    for (Index i = 0; i < N; ++i) {
        L(i, i) = std::sqrt(std::max(static_cast<T>(0), A(i, i)));
    }
    return true;
}

// 対称化
template<Index N, typename T = Scalar>
void symmetrize(Mat<N, N, T>& A) {
    for (Index i = 0; i < N; ++i) {
        for (Index j = i + 1; j < N; ++j) {
            T avg = (A(i, j) + A(j, i)) * static_cast<T>(0.5f);
            A(i, j) = avg;
            A(j, i) = avg;
        }
    }
}

// 正定値化
template<Index N, typename T = Scalar>
void make_positive_definite(Mat<N, N, T>& A, T min_eig = static_cast<T>(1e-9f)) {
    // 対称化
    symmetrize(A);
    
    // 最小対角要素チェック
    T min_diag = A(0, 0);
    for (Index i = 1; i < N; ++i) {
        if (A(i, i) < min_diag) {
            min_diag = A(i, i);
        }
    }
    
    // 正定値でない場合は正則化
    if (min_diag <= static_cast<T>(0)) {
        T reg = std::abs(min_diag) + min_eig;
        for (Index i = 0; i < N; ++i) {
            A(i, i) += reg;
        }
    }
}

} // namespace matrix
} // namespace lib




