#pragma once

#include "fixed_matrix.hpp"

namespace cmath_fx {
namespace utils {

// Symmetrize matrix in-place (template)
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

// Non-destructive copy version
template <int N, typename T>
Matrix<N, N, T> symmetrize_copy(const Matrix<N, N, T>& M) {
    Matrix<N, N, T> res = M;
    symmetrize<N, T>(res);
    return res;
}

// Symmetrize runtime FixedMatrix in-place
inline void symmetrize(FixedMatrix& M) {
    int n = M.rows;
    int m = M.cols;
    int N = (n < m) ? n : m;
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            float avg = 0.5f * (M(i, j) + M(j, i));
            M(i, j) = avg;
            M(j, i) = avg;
        }
    }
}

// Ensure positive definite by regularizing diagonal if needed
template <int N, typename T>
void ensure_positive_definite(Matrix<N, N, T>& M, T min_eigenvalue = static_cast<T>(1e-8)) {
    symmetrize<N, T>(M);

    T min_diag = M(0, 0);
    for (int i = 1; i < N; ++i) {
        if (M(i, i) < min_diag) min_diag = M(i, i);
    }

    if (min_diag < min_eigenvalue) {
        T reg = min_eigenvalue - min_diag;
        for (int i = 0; i < N; ++i) M(i, i) += reg;
        symmetrize<N, T>(M);
    }
}

} // namespace utils
} // namespace cmath_fx
