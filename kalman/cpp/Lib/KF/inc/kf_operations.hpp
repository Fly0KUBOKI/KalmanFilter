#pragma once

#include "../../Matrix/fixed_matrix.hpp"
#include "../../Matrix/matrix_utils.hpp"
#include "../../Matrix/matrix_inverse.hpp"

namespace kf {
namespace ops {

// Compute innovation covariance S = H * P * H' + R (templated square MxM)
template <int N, int M, typename T = float>
inline void compute_innovation_cov(const cmath_fx::Matrix<N, N, T>& P,
                                   const cmath_fx::Matrix<M, N, T>& H,
                                   const cmath_fx::Matrix<M, M, T>& R,
                                   cmath_fx::Matrix<M, M, T>& S) {
    S = H * P * H.transpose() + R;
    cmath_fx::utils::symmetrize<M, T>(S);
}

// Compute Kalman gain K = P * H' * S^{-1}. Leaves K zero on singular S.
template <int N, int M, typename T = float>
inline void compute_kalman_gain(const cmath_fx::Matrix<N, N, T>& P,
                                const cmath_fx::Matrix<M, N, T>& H,
                                const cmath_fx::Matrix<M, M, T>& R,
                                cmath_fx::Matrix<N, M, T>& K) {
    cmath_fx::Matrix<M, M, T> S;
    compute_innovation_cov<N, M, T>(P, H, R, S);
    cmath_fx::Matrix<M, M, T> S_inv;
    if (!cmath_fx::inv::inverse<M, T>(S, S_inv)) {
        K = cmath_fx::Matrix<N, M, T>::Zero();
        return;
    }
    K = P * H.transpose() * S_inv;
}

// Joseph form covariance update: P = (I - K H) P (I - K H)' + K R K'
template <int N, int M, typename T = float>
inline void joseph_form_update(const cmath_fx::Matrix<N, N, T>& P_pred,
                               const cmath_fx::Matrix<N, M, T>& K,
                               const cmath_fx::Matrix<M, N, T>& H,
                               const cmath_fx::Matrix<M, M, T>& R,
                               cmath_fx::Matrix<N, N, T>& P_upd) {
    auto I = cmath_fx::Matrix<N, N, T>::Identity();
    auto KH = K * H;
    auto IKH = I - KH;
    P_upd = IKH * P_pred * IKH.transpose() + K * R * K.transpose();
    cmath_fx::utils::symmetrize<N, T>(P_upd);
}

// Runtime-size FixedMatrix variants
inline void joseph_form_update(const cmath_fx::FixedMatrix& P_pred,
                               const cmath_fx::FixedMatrix& K,
                               const cmath_fx::FixedMatrix& H,
                               const cmath_fx::FixedMatrix& R,
                               cmath_fx::FixedMatrix& P_upd) {
    int n = P_pred.rows;
    cmath_fx::FixedMatrix I(n, n);
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) I(i, j) = (i == j) ? 1.0f : 0.0f;
    cmath_fx::FixedMatrix KH = K * H;
    cmath_fx::FixedMatrix IKH(n, n);
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) IKH(i, j) = I(i, j) - KH(i, j);
    cmath_fx::FixedMatrix temp1 = IKH * P_pred;
    cmath_fx::FixedMatrix term1 = temp1 * IKH.transpose();
    cmath_fx::FixedMatrix term2 = K * R * K.transpose();
    P_upd = term1 + term2;
    cmath_fx::utils::symmetrize(P_upd);
}

} // namespace ops
} // namespace kf
