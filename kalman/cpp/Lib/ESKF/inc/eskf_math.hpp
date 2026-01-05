#pragma once

// Implementation: Src/ESKF/eskf_math.cpp

#include "../../Matrix/fixed_matrix.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include "../../Common/inc/Math/math_utils.hpp"

namespace eskf_math {

using Scalar = float;
using Vector3 = cmath_fx::Vector<3, Scalar>;
using Vector4 = cmath_fx::Vector<4, Scalar>;
using Vector15 = cmath_fx::Vector<15, Scalar>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, Scalar>;
using Matrix15x15 = cmath_fx::Matrix<15, 15, Scalar>;

class ESKFMath {
public:
    static void quaternion_integration(
        const Vector4& q_in,
        const Vector3& w,
        Scalar dt,
        Vector4& q_out
    );

    static void accel_to_quaternion(
        const Vector3& a_meas,
        Scalar scale_factor,
        Vector4& q_out
    );

    struct PVIntegrationInput { Vector3 p; Vector3 v; Vector3 a_world; Vector3 g; Scalar dt; Vector3 prev_a; Vector3 prev_v; bool use_ab2; Scalar max_accel; Scalar max_velocity; };
    struct PVIntegrationOutput { Vector3 p_new; Vector3 v_new; Vector3 a_out; Vector3 v_out; };
    static void pv_integration(const PVIntegrationInput& input, PVIntegrationOutput& output);

    static void compute_F_matrix(const Vector4& q, const Vector3& a_meas, const Vector3& ba, const Vector3& w_meas, const Vector3& bg, Scalar dt, Matrix15x15& F);
    static void covariance_prediction(const Matrix15x15& P, const Matrix15x15& F, const Matrix15x15& Q, Matrix15x15& P_new);
    static void inject_error_state(const Vector3& p_in, const Vector3& v_in, const Vector4& q_in, const Vector3& ba_in, const Vector3& bg_in, const Vector15& dx, Vector3& p_out, Vector3& v_out, Vector4& q_out, Vector3& ba_out, Vector3& bg_out);
    static void mag_observation_prediction(const Vector4& q, const Vector3& m_world, Vector3& m_body_expected);
    static void gps_to_local(const Vector3& gps_pos, const Vector3& origin_pos, Vector3& local_pos);
    static Scalar pressure_to_altitude(Scalar pressure);

    template<int N, int M>
    static void kalman_update(const cmath_fx::Vector<N, Scalar>& x_in, const cmath_fx::Matrix<N, N, Scalar>& P_in, const cmath_fx::Vector<M, Scalar>& y, const cmath_fx::Matrix<M, N, Scalar>& H, const cmath_fx::Matrix<M, M, Scalar>& R, cmath_fx::Vector<N, Scalar>& x_out, cmath_fx::Matrix<N, N, Scalar>& P_out, cmath_fx::Matrix<N, M, Scalar>& K_out, cmath_fx::Matrix<M, M, Scalar>& S_out);
};

template<int N, int M>
void ESKFMath::kalman_update(const cmath_fx::Vector<N, Scalar>& x_in, const cmath_fx::Matrix<N, N, Scalar>& P_in, const cmath_fx::Vector<M, Scalar>& y, const cmath_fx::Matrix<M, N, Scalar>& H, const cmath_fx::Matrix<M, M, Scalar>& R, cmath_fx::Vector<N, Scalar>& x_out, cmath_fx::Matrix<N, N, Scalar>& P_out, cmath_fx::Matrix<N, M, Scalar>& K_out, cmath_fx::Matrix<M, M, Scalar>& S_out) {
    // Compute S using unified utility (ensures symmetry/robustness)
    {
        common::math::cm z_cm; z_cm.resize(M,1);
        common::math::cm h_cm; h_cm.resize(M,1);
        common::math::cm H_cm; H_cm.resize(M,N);
        common::math::cm P_cm; P_cm.resize(N,N);
        common::math::cm R_cm; R_cm.resize(M,M);
        // z_cm: use current innovation y as z placeholder, h_cm = 0 -> y = z - h = y
        for (int i = 0; i < M; ++i) z_cm(i,0) = y(i,0);
        for (int i = 0; i < M; ++i) h_cm(i,0) = 0.0f;
        for (int i = 0; i < M; ++i) for (int j = 0; j < N; ++j) H_cm(i,j) = H(i,j);
        for (int i = 0; i < N; ++i) for (int j = 0; j < N; ++j) P_cm(i,j) = P_in(i,j);
        for (int i = 0; i < M; ++i) for (int j = 0; j < M; ++j) R_cm(i,j) = R(i,j);

        common::math::cm y_cm; y_cm.resize(M,1);
        common::math::cm S_cm; S_cm.resize(M,M);
        common::math::cm R_out; R_out.resize(M,M);
        common::math::MathUtils::compute_innovation_and_S(z_cm, h_cm, H_cm, P_cm, R_cm, y_cm, S_cm, R_out);

        // copy back
        for (int i = 0; i < M; ++i) for (int j = 0; j < M; ++j) S_out(i,j) = S_cm(i,j);
        for (int i = 0; i < M; ++i) y(i,0) = y_cm(i,0);
    }
    cmath_fx::Matrix<M, M, Scalar> S_inv;
    if (!S_out.inverse(S_inv)) {
        K_out = cmath_fx::Matrix<N, M, Scalar>::Zero();
        x_out = x_in;
        P_out = P_in;
        return;
    }
    K_out = P_in * H.transpose() * S_inv;
    x_out = x_in + K_out * y;
    auto I = cmath_fx::Matrix<N, N, Scalar>::Identity();
    auto I_KH = I - K_out * H;
    P_out = I_KH * P_in * I_KH.transpose() + K_out * R * K_out.transpose();
    P_out = (P_out + P_out.transpose()) * static_cast<Scalar>(0.5);
}

} // namespace eskf_math
