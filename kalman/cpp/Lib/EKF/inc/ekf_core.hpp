#pragma once

// Implementation: このヘッダー内に実装含む（テンプレート実装）

#include "../../Matrix/fixed_matrix.hpp"
#include "../../KF/kalman_filter_core.hpp"
#include "../../Common/inc/Math/math_utils.hpp"

namespace ekf {

template<int N, int M, typename T = float>
class EKFCore {
public:
    using VectorN = cmath_fx::Vector<N, T>;
    using VectorM = cmath_fx::Vector<M, T>;
    using MatrixNN = cmath_fx::Matrix<N, N, T>;
    using MatrixMM = cmath_fx::Matrix<M, M, T>;
    using MatrixMN = cmath_fx::Matrix<M, N, T>;
    using MatrixNM = cmath_fx::Matrix<N, M, T>;
    
    // EKF予測ステップ
    // f_func: 状態遷移関数 x_next = f(x)
    template<typename FFunc>
    static void predict(
        VectorN& x,
        MatrixNN& P,
        FFunc f_func,
        const MatrixNN& Q
    ) {
        // 状態予測
        x = f_func(x);
        
        // 共分散予測 P = P + Q
        P = P + Q;
        
        // 対称化
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                P(i, j) = 0.5 * (P(i, j) + P(j, i));
            }
        }
    }
    
    // EKF予測ステップ（ヤコビアン付き）
    // f_func: 状態遷移関数 x_next = f(x)
    // F: ヤコビアン行列 ∂f/∂x
    template<typename FFunc>
    static void predict_with_jacobian(
        VectorN& x,
        MatrixNN& P,
        FFunc f_func,
        const MatrixNN& F,
        const MatrixNN& Q
    ) {
        // 状態予測
        x = f_func(x);
        
        // 共分散予測 P = F*P*F' + Q
        auto FP = F * P;
        P = FP * F.transpose() + Q;
        
        // 対称化
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                P(i, j) = 0.5 * (P(i, j) + P(j, i));
            }
        }
    }
    
    // EKF観測更新
    // h_func: 観測関数 z_pred = h(x)
    // H: 観測ヤコビアン ∂h/∂x
    template<typename HFunc>
    static void update(
        VectorN& x,
        MatrixNN& P,
        const VectorM& z,
        HFunc h_func,
        const MatrixMN& H,
        const MatrixMM& R,
        MatrixNM* K_out = nullptr,
        MatrixMM* S_out = nullptr,
        VectorM* y_out = nullptr
    ) {
        // 予測観測値
        VectorM z_pred = h_func(x);
        
        // イノベーションとその共分散 S を統一関数で計算
        VectorM y = z - z_pred;
        // convert to common::math::cm
        common::math::cm z_cm; z_cm.resize(M,1);
        common::math::cm h_cm; h_cm.resize(M,1);
        common::math::cm H_cm; H_cm.resize(M,N);
        common::math::cm P_cm; P_cm.resize(N,N);
        common::math::cm R_cm; R_cm.resize(M,M);
        for(int i=0;i<M;i++) { z_cm(i,0) = z(i,0); h_cm(i,0) = z_pred(i,0); }
        for(int i=0;i<M;i++) for(int j=0;j<N;j++) H_cm(i,j) = H(i,j);
        for(int i=0;i<N;i++) for(int j=0;j<N;j++) P_cm(i,j) = P(i,j);
        for(int i=0;i<M;i++) for(int j=0;j<M;j++) R_cm(i,j) = R(i,j);

        common::math::cm y_cm; y_cm.resize(M,1);
        common::math::cm S_cm; S_cm.resize(M,M);
        common::math::cm R_out; R_out.resize(M,M);
        common::math::MathUtils::compute_innovation_and_S(z_cm, h_cm, H_cm, P_cm, R_cm, y_cm, S_cm, R_out);

        // copy back S and y
        MatrixMM S; 
        for(int i=0;i<M;i++) for(int j=0;j<M;j++) S(i,j) = S_cm(i,j);
        for(int i=0;i<M;i++) y(i,0) = y_cm(i,0);
        
        // カルマンゲイン K = P*H'*inv(S)
        MatrixNM K = kf::KalmanFilterCore::compute_kalman_gain<N, M, T>(P, H, S);
        
        // 状態更新 x = x + K*y
        VectorN Ky = K * y;
        x = x + Ky;
        
        // 共分散更新（Joseph形式）
        MatrixNN I = MatrixNN::Identity();
        auto KH = K * H;
        MatrixNN IKH = I - KH;
        
        auto term1 = IKH * P * IKH.transpose();
        auto KR = K * R;
        auto term2 = KR * K.transpose();
        
        P = term1 + term2;
        
        // 対称化
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                P(i, j) = 0.5 * (P(i, j) + P(j, i));
            }
        }
        
        // 出力パラメータ
        if (K_out) *K_out = K;
        if (S_out) *S_out = S;
        if (y_out) *y_out = y;
    }
    
    // 簡易版観測更新（ヤコビアンなし、線形近似）
    template<typename HFunc>
    static void update_simple(
        VectorN& x,
        MatrixNN& P,
        const VectorM& z,
        HFunc h_func,
        const MatrixMM& R
    ) {
        // 数値微分でヤコビアンを計算
        MatrixMN H;
        compute_observation_jacobian(x, h_func, H);
        
        // 通常の更新
        update(x, P, z, h_func, H, R);
    }
    
private:
    // 観測ヤコビアンの数値微分計算
    template<typename HFunc>
    static void compute_observation_jacobian(
        const VectorN& x,
        HFunc h_func,
        MatrixMN& H
    ) {
        constexpr T eps = 1e-6;
        
        VectorM h0 = h_func(x);
        
        for (int j = 0; j < N; ++j) {
            VectorN x_pert = x;
            x_pert(j, 0) += eps;
            
            VectorM h_pert = h_func(x_pert);
            
            for (int i = 0; i < M; ++i) {
                H(i, j) = (h_pert(i, 0) - h0(i, 0)) / eps;
            }
        }
    }
};

} // namespace ekf
