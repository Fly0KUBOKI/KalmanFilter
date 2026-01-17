// eskf_math.cpp
// DEPRECATED WRAPPERS: These delegate to unified modules (Sensor, Quaternion, Matrix)
// Callers should use sensor::processing::*, sensor::coord::*, cquat::*, cmath_fx::utils::*

#include "../inc/eskf_math.hpp"
#include "../inc/eskf_includes.hpp"
// filter_mgmt.hpp is provided by eskf_includes.hpp

namespace eskf_math {

void ESKFMath::quaternion_integration(const Vector4& q_in, const Vector3& w, Scalar dt, Vector4& q_out) {
    // Delegate to unified quaternion module
    cquat::quaternion_integration(q_in, w, dt, q_out);
}

void ESKFMath::accel_to_quaternion(const Vector3& a_meas, Scalar scale_factor, Vector4& q_out) {
    // Delegate to unified sensor processing module
    sensor::processing::accel_to_quaternion(a_meas, scale_factor, q_out);
}

void ESKFMath::pv_integration(const PVIntegrationInput& input, PVIntegrationOutput& output) {
    if (input.use_ab2 && (input.prev_a(0,0) != static_cast<Scalar>(0.0) || input.prev_a(1,0) != static_cast<Scalar>(0.0) || input.prev_a(2,0) != static_cast<Scalar>(0.0))) {
        for (int i=0;i<3;++i) { Scalar a_avg = static_cast<Scalar>(1.5) * input.a_world(i,0) - static_cast<Scalar>(0.5) * input.prev_a(i,0); output.v_new(i,0)=input.v(i,0)+a_avg*input.dt; output.p_new(i,0)=input.p(i,0)+input.v(i,0)*input.dt + static_cast<Scalar>(0.5)*a_avg*input.dt*input.dt; }
    } else { for (int i=0;i<3;++i) { output.v_new(i,0)=input.v(i,0)+input.a_world(i,0)*input.dt; output.p_new(i,0)=input.p(i,0)+input.v(i,0)*input.dt + static_cast<Scalar>(0.5)*input.a_world(i,0)*input.dt*input.dt; } }
    for (int i=0;i<3;++i) if (std::abs(output.v_new(i,0)) > input.max_velocity) output.v_new(i,0) = (output.v_new(i,0) > static_cast<Scalar>(0.0)) ? input.max_velocity : -input.max_velocity;
    output.a_out = input.a_world; output.v_out = output.v_new;
}

void ESKFMath::compute_F_matrix(const Vector4& q, const Vector3& a_meas, const Vector3& ba, const Vector3& w_meas, const Vector3& bg, Scalar dt, Matrix15x15& F) { F = Matrix15x15(); for (int i=0;i<15;++i) F(i,i) = static_cast<Scalar>(1.0) + dt * static_cast<Scalar>(0.01); }

void ESKFMath::covariance_prediction(const Matrix15x15& P, const Matrix15x15& F, const Matrix15x15& Q, Matrix15x15& P_new) {
    Matrix15x15 F_P = F * P;
    Matrix15x15 F_P_Ft = F_P * F.transpose();
    P_new = F_P_Ft + Q;
    // Ensure symmetry via central matrix utility
    {
        cmath_fx::Matrix<15,15,float> Pmat = P_new;
        cmath_fx::utils::symmetrize<15, float>(Pmat);
        for (int i=0;i<15;++i) for (int j=0;j<15;++j) P_new(i,j) = static_cast<Scalar>(Pmat(i,j));
    }
}

void ESKFMath::inject_error_state(const Vector3& p_in, const Vector3& v_in, const Vector4& q_in, const Vector3& ba_in, const Vector3& bg_in, const Vector15& dx, Vector3& p_out, Vector3& v_out, Vector4& q_out, Vector3& ba_out, Vector3& bg_out) { for (int i=0;i<3;++i){ p_out(i,0)=p_in(i,0)+dx(i,0); v_out(i,0)=v_in(i,0)+dx(i+3,0); ba_out(i,0)=ba_in(i,0)+dx(i+9,0); bg_out(i,0)=bg_in(i,0)+dx(i+12,0); } Vector4 dq; dq(0,0)=static_cast<Scalar>(1.0); dq(1,0)=dx(6,0)*static_cast<Scalar>(0.5); dq(2,0)=dx(7,0)*static_cast<Scalar>(0.5); dq(3,0)=dx(8,0)*static_cast<Scalar>(0.5); cquat::normalize_quat(dq); cquat::multiply_quat(q_in, dq, q_out); cquat::normalize_quat(q_out); }

void ESKFMath::mag_observation_prediction(const Vector4& q, const Vector3& m_world, Vector3& m_body_expected) {
    // Delegate to unified sensor processing module
    sensor::processing::mag_observation_prediction(q, m_world, m_body_expected);
}

void ESKFMath::gps_to_local(const Vector3& gps_pos, const Vector3& origin_pos, Vector3& local_pos) {
    // Delegate to unified coordinate transform module
    sensor::coord::gps_to_local(gps_pos, origin_pos, local_pos);
}

Scalar ESKFMath::pressure_to_altitude(Scalar pressure) {
    // Delegate to unified coordinate transform module
    return sensor::coord::pressure_to_altitude(pressure);
}

} // namespace eskf_math
