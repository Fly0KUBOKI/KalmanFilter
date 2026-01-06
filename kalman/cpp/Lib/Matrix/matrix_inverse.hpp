#pragma once

#include "fixed_matrix.hpp"
#include <cmath>

namespace cmath_fx {
namespace inv {

// Analytic inverse for 3x3 matrices
template <typename T>
bool inverse_3x3_analytic(const Matrix<3, 3, T>& A, Matrix<3, 3, T>& A_inv) {
    T det = A(0,0) * (A(1,1)*A(2,2) - A(2,1)*A(1,2))
          - A(0,1) * (A(1,0)*A(2,2) - A(2,0)*A(1,2))
          + A(0,2) * (A(1,0)*A(2,1) - A(2,0)*A(1,1));

    if (std::abs(det) < static_cast<T>(1e-10)) return false;
    T inv_det = static_cast<T>(1.0) / det;

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

// Generic inverse: use Matrix::inverse for arbitrary size
template <int N, typename T>
bool inverse(const Matrix<N, N, T>& A, Matrix<N, N, T>& inv) {
    if constexpr (N == 3) {
        return inverse_3x3_analytic<T>(A, inv);
    } else {
        return A.inverse(inv);
    }
}

} // namespace inv
} // namespace cmath_fx
