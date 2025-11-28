#pragma once

#include "../Common/Math/fixed_matrix.hpp"

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
