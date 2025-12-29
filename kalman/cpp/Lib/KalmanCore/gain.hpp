#pragma once

#include "../Matrix/matrix.hpp"
#include "../Matrix/decomposition.hpp"

namespace lib {
namespace kalman {

using namespace matrix;

// カルマンゲイン計算: K = P * H^T * S^-1
// P: NxN, H: MxN, S: MxM -> K: NxM
template<Index N, Index M, typename T = Scalar>
Status compute_gain(
    const Mat<N, N, T>& P,
    const Mat<M, N, T>& H,
    const Mat<M, M, T>& S,
    Mat<N, M, T>& K
) {
    // S の逆行列計算
    Mat<M, M, T> S_inv;
    if (!S.inverse(S_inv)) {
        // 逆行列計算失敗時はゼロ行列を返す
        K = Mat<N, M, T>::Zero();
        return STATUS_SINGULAR;
    }
    
    // K = P * H^T * S^-1
    Mat<N, M, T> PHt = P * H.transpose();
    K = PHt * S_inv;
    
    return STATUS_OK;
}

// イノベーション共分散: S = H * P * H^T + R
template<Index N, Index M, typename T = Scalar>
void compute_innovation_cov(
    const Mat<N, N, T>& P,
    const Mat<M, N, T>& H,
    const Mat<M, M, T>& R,
    Mat<M, M, T>& S
) {
    // S = H * P * H^T + R
    Mat<M, N, T> HP = H * P;
    S = HP * H.transpose() + R;
    
    // 対称化
    symmetrize(S);
}

// 歪対称行列 (3x3)
template<typename T = Scalar>
Mat3 skew_symmetric(const Vec3& v) {
    Mat3 S;
    S(0, 0) = static_cast<T>(0);
    S(0, 1) = -v(2, 0);
    S(0, 2) = v(1, 0);
    S(1, 0) = v(2, 0);
    S(1, 1) = static_cast<T>(0);
    S(1, 2) = -v(0, 0);
    S(2, 0) = -v(1, 0);
    S(2, 1) = v(0, 0);
    S(2, 2) = static_cast<T>(0);
    return S;
}

} // namespace kalman
} // namespace lib


