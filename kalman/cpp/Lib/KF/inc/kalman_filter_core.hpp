#pragma once

// Implementation: このヘッダー内に実装含む（テンプレート実装）

#include "../../Matrix/fixed_matrix.hpp"

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
        cmath_fx::Matrix<M, M, T> S_inv;
        if (!S.inverse(S_inv)) {
            // 逆行列計算失敗時はゼロ行列を返す
            return cmath_fx::Matrix<N, M, T>::Zero();
        }
        
        // K = P * H^T * S^-1
        // P: NxN, H^T: NxM -> PHt: NxM
        // PHt: NxM, S_inv: MxM -> K: NxM
        return (P * H.transpose()) * S_inv;
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
        // Innovation: y = z - h
        y = z - h;
        
        // S = H*P*H' + R
        S = H * P_pred * H.transpose() + R;
        
        // 対称化
        S = (S + S.transpose()) * T(0.5);
        
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
        // x_upd = x_pred + K * y
        x_upd = x_pred + K * y;
        
        // P_upd = (I - K*H) * P_pred * (I - K*H)' + K*R*K' (Joseph form)
        auto I = cmath_fx::Matrix<N, N, T>::Identity();
        auto I_KH = I - K * H;
        P_upd = I_KH * P_pred * I_KH.transpose() + K * R * K.transpose();
        
        // 対称化
        P_upd = (P_upd + P_upd.transpose()) * T(0.5);
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
