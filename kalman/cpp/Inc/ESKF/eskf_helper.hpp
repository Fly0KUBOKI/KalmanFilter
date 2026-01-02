#pragma once

#include "../Lib/Matrix/fixed_matrix.hpp"
#include "../Lib/Quaternion/quaternion_functions.hpp"

namespace eskf {

template<typename T = float>
class ESKFHelper {
public:
    using Vector3 = cmath_fx::Vector<3, T>;
    using Vector4 = cmath_fx::Vector<4, T>;
    using Vector15 = cmath_fx::Vector<15, T>;
    using Matrix15 = cmath_fx::Matrix<15, 15, T>;
    
    // ノミナル状態構造体
    struct NominalState {
        Vector3 p;   // 位置
        Vector3 v;   // 速度
        Vector4 q;   // 姿勢 (quaternion)
        Vector3 ba;  // 加速度バイアス
        Vector3 bg;  // 角速度バイアス
    };
    
    // 誤差状態をノミナル状態に注入
    static void inject_error_state(NominalState& nominal, const Vector15& dx) {
        // 位置更新
        nominal.p(0,0) += dx(0,0);
        nominal.p(1,0) += dx(1,0);
        nominal.p(2,0) += dx(2,0);
        
        // 速度更新
        nominal.v(0,0) += dx(3,0);
        nominal.v(1,0) += dx(4,0);
        nominal.v(2,0) += dx(5,0);
        
        // 姿勢更新（小角度クォータニオン積）
        Vector4 dq;
        cquat::from_small_angle(dx(6,0), dx(7,0), dx(8,0), dq);
        Vector4 q_new;
        cquat::multiply_quat(dq, nominal.q, q_new);
        cquat::normalize_quat(q_new);
        nominal.q = q_new;
        
        // バイアス更新
        nominal.ba(0,0) += dx(9,0);
        nominal.ba(1,0) += dx(10,0);
        nominal.ba(2,0) += dx(11,0);
        
        nominal.bg(0,0) += dx(12,0);
        nominal.bg(1,0) += dx(13,0);
        nominal.bg(2,0) += dx(14,0);
    }
    
    // 制約付き誤差状態注入
    static void inject_with_constraints(
        NominalState& nominal,
        const Vector15& dx,
        T max_velocity = 100.0f,
        T max_accel_bias = 10.0f,
        T max_gyro_bias = 1.0f
    ) {
        // 基本注入
        inject_error_state(nominal, dx);
        
        // 速度制限
        T v_norm = std::sqrt(
            nominal.v(0,0)*nominal.v(0,0) +
            nominal.v(1,0)*nominal.v(1,0) +
            nominal.v(2,0)*nominal.v(2,0)
        );
        if (v_norm > max_velocity) {
            T scale = max_velocity / v_norm;
            nominal.v(0,0) *= scale;
            nominal.v(1,0) *= scale;
            nominal.v(2,0) *= scale;
        }
        
        // 加速度バイアス制限
        T ba_norm = std::sqrt(
            nominal.ba(0,0)*nominal.ba(0,0) +
            nominal.ba(1,0)*nominal.ba(1,0) +
            nominal.ba(2,0)*nominal.ba(2,0)
        );
        if (ba_norm > max_accel_bias) {
            T scale = max_accel_bias / ba_norm;
            nominal.ba(0,0) *= scale;
            nominal.ba(1,0) *= scale;
            nominal.ba(2,0) *= scale;
        }
        
        // 角速度バイアス制限
        T bg_norm = std::sqrt(
            nominal.bg(0,0)*nominal.bg(0,0) +
            nominal.bg(1,0)*nominal.bg(1,0) +
            nominal.bg(2,0)*nominal.bg(2,0)
        );
        if (bg_norm > max_gyro_bias) {
            T scale = max_gyro_bias / bg_norm;
            nominal.bg(0,0) *= scale;
            nominal.bg(1,0) *= scale;
            nominal.bg(2,0) *= scale;
        }
    }
    
    // Joseph形式の共分散更新
    template<int M>
    static void joseph_form_covariance_update(
        Matrix15& P,
        const cmath_fx::Matrix<15, M, T>& K,
        const cmath_fx::Matrix<M, 15, T>& H,
        const cmath_fx::Matrix<M, M, T>& R
    ) {
        Matrix15 I;
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                I(i, j) = (i == j) ? 1.0 : 0.0;
            }
        }
        
        // IKH = I - K * H
        auto KH = K * H;
        Matrix15 IKH = I - KH;
        
        // P = (I - K*H) * P * (I - K*H)' + K * R * K'
        auto IKH_P = IKH * P;
        auto IKH_P_IKHt = IKH_P * IKH.transpose();
        
        auto KR = K * R;
        auto KRKt = KR * K.transpose();
        
        P = IKH_P_IKHt + KRKt;
        
        // 対称化
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P(i, j) = 0.5 * (P(i, j) + P(j, i));
            }
        }
    }
    
    // 共分散正則化
    static void regularize_covariance(Matrix15& P, T eps = 1e-9f) {
        // 対称化
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P(i, j) = 0.5 * (P(i, j) + P(j, i));
            }
        }
        
        // 対角成分に最小値保証
        for (int i = 0; i < 15; ++i) {
            if (P(i, i) < eps) {
                P(i, i) = eps;
            }
        }
    }
};

} // namespace eskf
