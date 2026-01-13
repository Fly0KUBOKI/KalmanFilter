#pragma once
#ifndef LIB_KF_INC_KF_OPERATIONS_HPP
#define LIB_KF_INC_KF_OPERATIONS_HPP

#include "../../Matrix/fixed_matrix.hpp"

namespace kf {

// Innovation と Innovation Covariance の計算
template <int M, int N, typename T>
struct InnovationResult {
    cmath_fx::Vector<M, T> y;        // Innovation
    cmath_fx::Matrix<M, M, T> S;     // Innovation covariance
};

template <int M, int N, typename T>
InnovationResult<M, N, T> compute_innovation(
    const cmath_fx::Vector<M, T>& z,
    const cmath_fx::Vector<M, T>& h,
    const cmath_fx::Matrix<M, N, T>& H,
    const cmath_fx::Matrix<N, N, T>& P,
    const cmath_fx::Matrix<M, M, T>& R
) {
    InnovationResult<M, N, T> result;
    result.y = z - h;
    result.S = H * P * H.transpose() + R;
    cmath_fx::utils::symmetrize(result.S);
    return result;
}

// Kalman Gain の計算
template <int N, int M, typename T>
cmath_fx::Matrix<N, M, T> compute_kalman_gain(
    const cmath_fx::Matrix<N, N, T>& P,
    const cmath_fx::Matrix<M, N, T>& H,
    const cmath_fx::Matrix<M, M, T>& S
) {
    cmath_fx::Matrix<M, M, T> S_inv;
    if (!cmath_fx::inv::inverse(S, S_inv)) {
        return cmath_fx::Matrix<N, M, T>::Zero();
    }
    return P * H.transpose() * S_inv;
}

// 状態更新 (Joseph form)
template <int N, int M, typename T>
struct UpdateResult {
    cmath_fx::Vector<N, T> x;        // Updated state
    cmath_fx::Matrix<N, N, T> P;     // Updated covariance
};

template <int N, int M, typename T>
UpdateResult<N, M, T> update_state_joseph(
    const cmath_fx::Vector<N, T>& x_pred,
    const cmath_fx::Matrix<N, N, T>& P_pred,
    const cmath_fx::Matrix<N, M, T>& K,
    const cmath_fx::Matrix<M, N, T>& H,
    const cmath_fx::Vector<M, T>& y,
    const cmath_fx::Matrix<M, M, T>& R
) {
    UpdateResult<N, M, T> result;
    result.x = x_pred + K * y;
    cmath_fx::Matrix<N, N, T> I = cmath_fx::Matrix<N, N, T>::Identity();
    cmath_fx::Matrix<N, N, T> I_KH = I - K * H;
    result.P = I_KH * P_pred * I_KH.transpose() + K * R * K.transpose();
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
    result.x = x_pred + K * y;
    cmath_fx::Matrix<N, N, T> I = cmath_fx::Matrix<N, N, T>::Identity();
    result.P = (I - K * H) * P_pred;
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
        T max_var = S(0, 0);
        for (int i = 1; i < M; ++i) if (S(i, i) > max_var) max_var = S(i, i);
        if (max_var < static_cast<T>(1e-9)) max_var = static_cast<T>(1.0);
        T norm_sq = static_cast<T>(0);
        for (int i = 0; i < M; ++i) norm_sq += innovation(i, 0) * innovation(i, 0);
        return norm_sq / max_var;
    }
    cmath_fx::Vector<M, T> tmp = S_inv * innovation;
    T dist_sq = static_cast<T>(0);
    for (int i = 0; i < M; ++i) dist_sq += innovation(i, 0) * tmp(i, 0);
    return dist_sq;
}

} // namespace kf
#pragma once

#include "../../Matrix/fixed_matrix.hpp"

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
    cmath_fx::Matrix<N, N, T> I = cmath_fx::Matrix<N, N, T>::Identity();
    cmath_fx::Matrix<N, N, T> KH = K * H;
    cmath_fx::Matrix<N, N, T> IKH = I - KH;
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
// Runtime-size helpers
namespace ops {
inline float mahalanobis_distance_squared(const cmath_fx::FixedMatrix& innovation,
                                          const cmath_fx::FixedMatrix& S) {
    int m = S.rows;
    if (m == 0 || innovation.rows != m) return 0.0f;
    cmath_fx::FixedMatrix S_inv; S_inv.resize(m, m);
    if (!S.inverse(S_inv)) {
        float max_var = S(0,0);
        for (int i = 1; i < m; ++i) if (S(i,i) > max_var) max_var = S(i,i);
        if (max_var < 1e-9f) max_var = 1.0f;
        float norm_sq = 0.0f;
        for (int i = 0; i < m; ++i) norm_sq += innovation(i,0) * innovation(i,0);
        return norm_sq / max_var;
    }
    cmath_fx::FixedMatrix tmp = S_inv * innovation;
    float dist_sq = 0.0f;
    for (int i = 0; i < m; ++i) dist_sq += innovation(i,0) * tmp(i,0);
    return dist_sq;
}

} // namespace ops

} // namespace kf

#endif // LIB_KF_INC_KF_OPERATIONS_HPP
