#pragma once
#ifndef LIB_ESKF_INC_ESKF_MATH_HPP
#define LIB_ESKF_INC_ESKF_MATH_HPP

// Implementation: Src/ESKF/eskf_math.cpp

#include "../../Matrix/fixed_matrix.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include "../../Sensor/sensor_processing.hpp"
#include "../../Sensor/coordinate_transform.hpp"
#include "../../Matrix/Math/statistics.hpp"
#include "../../KF/inc/kalman_filter_core.hpp"
#include <cmath>

namespace eskf_math {

using Scalar = float;
using Vector3 = cmath_fx::Vector<3, Scalar>;
using Vector4 = cmath_fx::Vector<4, Scalar>;
using Vector15 = cmath_fx::Vector<15, Scalar>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, Scalar>;
using Matrix15x15 = cmath_fx::Matrix<15, 15, Scalar>;

class ESKFMath {
public:
    // DEPRECATED: Use cquat::quaternion_integration() directly
    static void quaternion_integration(
        const Vector4& q_in,
        const Vector3& w,
        Scalar dt,
        Vector4& q_out
    );

    // DEPRECATED: Use sensor::processing::accel_to_quaternion() directly
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
    
    // DEPRECATED: Use sensor::processing::mag_observation_prediction() directly
    static void mag_observation_prediction(const Vector4& q, const Vector3& m_world, Vector3& m_body_expected);
    
    // DEPRECATED: Use sensor::coord::gps_to_local() directly
    static void gps_to_local(const Vector3& gps_pos, const Vector3& origin_pos, Vector3& local_pos);
    
    // DEPRECATED: Use sensor::coord::pressure_to_altitude() directly
    static Scalar pressure_to_altitude(Scalar pressure);

    template<int N, int M>
    static void kalman_update(const cmath_fx::Vector<N, Scalar>& x_in, const cmath_fx::Matrix<N, N, Scalar>& P_in, const cmath_fx::Vector<M, Scalar>& y, const cmath_fx::Matrix<M, N, Scalar>& H, const cmath_fx::Matrix<M, M, Scalar>& R, cmath_fx::Vector<N, Scalar>& x_out, cmath_fx::Matrix<N, N, Scalar>& P_out, cmath_fx::Matrix<N, M, Scalar>& K_out, cmath_fx::Matrix<M, M, Scalar>& S_out);
};

template<int N, int M>
void ESKFMath::kalman_update(const cmath_fx::Vector<N, Scalar>& x_in, const cmath_fx::Matrix<N, N, Scalar>& P_in, const cmath_fx::Vector<M, Scalar>& y, const cmath_fx::Matrix<M, N, Scalar>& H, const cmath_fx::Matrix<M, M, Scalar>& R, cmath_fx::Vector<N, Scalar>& x_out, cmath_fx::Matrix<N, N, Scalar>& P_out, cmath_fx::Matrix<N, M, Scalar>& K_out, cmath_fx::Matrix<M, M, Scalar>& S_out) {
    // Use kf operations (compile-time sizes) for innovation, gain and Joseph update
    kf::InnovationResult<M, N, Scalar> res = ::kf::compute_innovation<M, N, Scalar>(y, cmath_fx::Vector<M, Scalar>::Zero(), H, P_in, R);
    S_out = res.S;
    // Compute Kalman gain
    K_out = ::kf::compute_kalman_gain<N, M, Scalar>(P_in, H, S_out);
    // Update state and covariance via Joseph form
    kf::UpdateResult<N, M, Scalar> upd = ::kf::update_state_joseph<N, M, Scalar>(x_in, P_in, K_out, H, res.y, R);
    x_out = upd.x;
    P_out = upd.P;
}

} // namespace eskf_math

#endif // LIB_ESKF_INC_ESKF_MATH_HPP
