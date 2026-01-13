#pragma once
#ifndef LIB_KF_INC_KALMAN_FILTER_CORE_HPP
#define LIB_KF_INC_KALMAN_FILTER_CORE_HPP

// Implementation: このヘッダー内に実装含む（テンプレート実装）

#include "../../Matrix/fixed_matrix.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Common/inc/Math/statistics.hpp"
#include "../../Common/inc/Math/geometry.hpp"
#include "../../Common/inc/Math/numerical.hpp"
#include "../../KF/inc/kf_operations.hpp"

namespace kf {

class KalmanFilterCore {
public:
    // カルマンゲイン計算: K = P * H^T * S^-1
    // P: NxN, H: MxN, S: MxM -> K: NxM
    template <int N, int M, typename T>
    static cmath_fx::Matrix<N, M, T> compute_kalman_gain(
        const cmath_fx::Matrix<N, N, T>& P,
        const cmath_fx::Matrix<M, N, T>& H,
        const cmath_fx::Matrix<M, M, T>& S
    ) {
        return ::kf::compute_kalman_gain<N, M, T>(P, H, S);
    }

    // Innovation と S の計算
    // y = z - h, S = H*P*H' + R
    template <int N, int M, typename T>
    static void compute_innovation_and_S(
        const cmath_fx::Vector<M, T>& z,
        const cmath_fx::Vector<M, T>& h,
        const cmath_fx::Matrix<M, N, T>& H,
        const cmath_fx::Matrix<N, N, T>& P_pred,
        const cmath_fx::Matrix<M, M, T>& R,
        cmath_fx::Vector<M, T>& y,
        cmath_fx::Matrix<M, M, T>& S,
        cmath_fx::Matrix<M, M, T>& R_out
    ) {
        // Use KF common implementation
        kf::InnovationResult<M, N, T> res = ::kf::compute_innovation<M, N, T>(z, h, H, P_pred, R);
        y = res.y;
        S = res.S;
        R_out = R;
    }

    // 状態と共分散の更新 (Joseph form)
    template <int N, int M, typename T>
    static void update_state_covariance(
        const cmath_fx::Vector<N, T>& x_pred,
        const cmath_fx::Matrix<N, N, T>& P_pred,
        const cmath_fx::Matrix<N, M, T>& K,
        const cmath_fx::Matrix<M, N, T>& H,
        const cmath_fx::Vector<M, T>& y,
        const cmath_fx::Matrix<M, M, T>& R,
        cmath_fx::Vector<N, T>& x_upd,
        cmath_fx::Matrix<N, N, T>& P_upd
    ) {
        // Delegate to KF operation
        kf::UpdateResult<N, M, T> res = ::kf::update_state_joseph<N, M, T>(x_pred, P_pred, K, H, y, R);
        x_upd = res.x;
        P_upd = res.P;
    }

    // 歪対称行列
    template <typename T>
    static cmath_fx::Matrix<3, 3, T> skew_symmetric(const cmath_fx::Vector<3, T>& v) {
        cmath_fx::Matrix<3, 3, T> S;
        S(0,0) = 0;      S(0,1) = -v(2,0); S(0,2) = v(1,0);
        S(1,0) = v(2,0); S(1,1) = 0;       S(1,2) = -v(0,0);
        S(2,0) = -v(1,0); S(2,1) = v(0,0);  S(2,2) = 0;
        return S;
    }
};

} // namespace kf

#endif // LIB_KF_INC_KALMAN_FILTER_CORE_HPP
