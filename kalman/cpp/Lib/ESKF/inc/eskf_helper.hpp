#pragma once

#include "../../Matrix/fixed_matrix.hpp"
#include "../../Quaternion/quaternion_functions.hpp"

namespace eskf {

template<typename T = float>
class ESKFHelper {
public:
    using Vector3 = cmath_fx::Vector<3, T>;
    using Vector4 = cmath_fx::Vector<4, T>;
    using Vector15 = cmath_fx::Vector<15, T>;
    using Matrix15 = cmath_fx::Matrix<15, 15, T>;
    
    struct NominalState { Vector3 p; Vector3 v; Vector4 q; Vector3 ba; Vector3 bg; };
    
    static void inject_error_state(NominalState& nominal, const Vector15& dx) {
        nominal.p(0,0) += dx(0,0);
        nominal.p(1,0) += dx(1,0);
        nominal.p(2,0) += dx(2,0);
        nominal.v(0,0) += dx(3,0);
        nominal.v(1,0) += dx(4,0);
        nominal.v(2,0) += dx(5,0);
        Vector4 dq; cquat::from_small_angle(dx(6,0), dx(7,0), dx(8,0), dq);
        Vector4 q_new; cquat::multiply_quat(dq, nominal.q, q_new); cquat::normalize_quat(q_new); nominal.q = q_new;
        nominal.ba(0,0) += dx(9,0); nominal.ba(1,0) += dx(10,0); nominal.ba(2,0) += dx(11,0);
        nominal.bg(0,0) += dx(12,0); nominal.bg(1,0) += dx(13,0); nominal.bg(2,0) += dx(14,0);
    }
    
    static void inject_with_constraints(NominalState& nominal, const Vector15& dx, T max_velocity = 100.0f, T max_accel_bias = 10.0f, T max_gyro_bias = 1.0f) {
        inject_error_state(nominal, dx);
        T v_norm = std::sqrt(nominal.v(0,0)*nominal.v(0,0) + nominal.v(1,0)*nominal.v(1,0) + nominal.v(2,0)*nominal.v(2,0));
        if (v_norm > max_velocity) { T scale = max_velocity / v_norm; nominal.v(0,0) *= scale; nominal.v(1,0) *= scale; nominal.v(2,0) *= scale; }
        T ba_norm = std::sqrt(nominal.ba(0,0)*nominal.ba(0,0) + nominal.ba(1,0)*nominal.ba(1,0) + nominal.ba(2,0)*nominal.ba(2,0));
        if (ba_norm > max_accel_bias) { T scale = max_accel_bias / ba_norm; nominal.ba(0,0) *= scale; nominal.ba(1,0) *= scale; nominal.ba(2,0) *= scale; }
        T bg_norm = std::sqrt(nominal.bg(0,0)*nominal.bg(0,0) + nominal.bg(1,0)*nominal.bg(1,0) + nominal.bg(2,0)*nominal.bg(2,0));
        if (bg_norm > max_gyro_bias) { T scale = max_gyro_bias / bg_norm; nominal.bg(0,0) *= scale; nominal.bg(1,0) *= scale; nominal.bg(2,0) *= scale; }
    }
    
    template<int M>
    static void joseph_form_covariance_update(Matrix15& P, const cmath_fx::Matrix<15, M, T>& K, const cmath_fx::Matrix<M, 15, T>& H, const cmath_fx::Matrix<M, M, T>& R) {
        Matrix15 I; for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) I(i,j) = (i==j)?1.0:0.0;
        auto KH = K * H; Matrix15 IKH = I - KH; auto IKH_P = IKH * P; auto IKH_P_IKHt = IKH_P * IKH.transpose(); auto KR = K * R; auto KRKt = KR * K.transpose(); P = IKH_P_IKHt + KRKt;
        for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = 0.5 * (P(i,j) + P(j,i));
    }
    
    static void regularize_covariance(Matrix15& P, T eps = 1e-9f) {
        for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = 0.5 * (P(i,j) + P(j,i));
        for (int i = 0; i < 15; ++i) if (P(i,i) < eps) P(i,i) = eps;
    }
};

} // namespace eskf
