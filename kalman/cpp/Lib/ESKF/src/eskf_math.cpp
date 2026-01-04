// eskf_math.cpp

#include "../inc/eskf_math.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include <cmath>

namespace eskf_math {

void ESKFMath::quaternion_integration(const Vector4& q_in, const Vector3& w, Scalar dt, Vector4& q_out) {
    Vector3 w_half = w; for (int i=0;i<3;++i) w_half(i,0) *= static_cast<Scalar>(0.5) * dt;
    Vector4 dq; dq(0,0) = static_cast<Scalar>(-0.5) * (q_in(1,0)*w_half(0,0) + q_in(2,0)*w_half(1,0) + q_in(3,0)*w_half(2,0)); dq(1,0) = static_cast<Scalar>(0.5)*(q_in(0,0)*w_half(0,0)+q_in(2,0)*w_half(2,0)-q_in(3,0)*w_half(1,0)); dq(2,0)=static_cast<Scalar>(0.5)*(q_in(0,0)*w_half(1,0)-q_in(1,0)*w_half(2,0)+q_in(3,0)*w_half(0,0)); dq(3,0)=static_cast<Scalar>(0.5)*(q_in(0,0)*w_half(2,0)+q_in(1,0)*w_half(1,0)-q_in(2,0)*w_half(0,0));
    q_out(0,0)=q_in(0,0)+dq(0,0); q_out(1,0)=q_in(1,0)+dq(1,0); q_out(2,0)=q_in(2,0)+dq(2,0); q_out(3,0)=q_in(3,0)+dq(3,0); cquat::normalize_quat(q_out);
}

void ESKFMath::accel_to_quaternion(const Vector3& a_meas, Scalar scale_factor, Vector4& q_out) {
    Scalar a_norm = 0.0f; for (int i=0;i<3;++i) a_norm += a_meas(i,0)*a_meas(i,0); a_norm = std::sqrt(a_norm);
    if (a_norm < static_cast<Scalar>(1e-6)) { q_out(0,0)=static_cast<Scalar>(1.0); q_out(1,0)=0; q_out(2,0)=0; q_out(3,0)=0; return; }
    Vector3 a_norm_vec = a_meas; for (int i=0;i<3;++i){ a_norm_vec(i,0) /= a_norm; a_norm_vec(i,0) *= scale_factor; }
    Scalar roll = std::atan2(a_norm_vec(1,0), a_norm_vec(2,0)); Scalar pitch = std::asin(-a_norm_vec(0,0));
    Scalar cr = std::cos(roll * static_cast<Scalar>(0.5)); Scalar sr = std::sin(roll * static_cast<Scalar>(0.5)); Scalar cp = std::cos(pitch * static_cast<Scalar>(0.5)); Scalar sp = std::sin(pitch * static_cast<Scalar>(0.5));
    q_out(0,0) = cr * cp; q_out(1,0) = sr * cp; q_out(2,0) = cr * sp; q_out(3,0) = -sr * sp; cquat::normalize_quat(q_out);
}

void ESKFMath::pv_integration(const PVIntegrationInput& input, PVIntegrationOutput& output) {
    if (input.use_ab2 && (input.prev_a(0,0) != static_cast<Scalar>(0.0) || input.prev_a(1,0) != static_cast<Scalar>(0.0) || input.prev_a(2,0) != static_cast<Scalar>(0.0))) {
        for (int i=0;i<3;++i) { Scalar a_avg = static_cast<Scalar>(1.5) * input.a_world(i,0) - static_cast<Scalar>(0.5) * input.prev_a(i,0); output.v_new(i,0)=input.v(i,0)+a_avg*input.dt; output.p_new(i,0)=input.p(i,0)+input.v(i,0)*input.dt + static_cast<Scalar>(0.5)*a_avg*input.dt*input.dt; }
    } else { for (int i=0;i<3;++i) { output.v_new(i,0)=input.v(i,0)+input.a_world(i,0)*input.dt; output.p_new(i,0)=input.p(i,0)+input.v(i,0)*input.dt + static_cast<Scalar>(0.5)*input.a_world(i,0)*input.dt*input.dt; } }
    for (int i=0;i<3;++i) if (std::abs(output.v_new(i,0)) > input.max_velocity) output.v_new(i,0) = (output.v_new(i,0) > static_cast<Scalar>(0.0)) ? input.max_velocity : -input.max_velocity;
    output.a_out = input.a_world; output.v_out = output.v_new;
}

void ESKFMath::compute_F_matrix(const Vector4& q, const Vector3& a_meas, const Vector3& ba, const Vector3& w_meas, const Vector3& bg, Scalar dt, Matrix15x15& F) { F = Matrix15x15(); for (int i=0;i<15;++i) F(i,i) = static_cast<Scalar>(1.0) + dt * static_cast<Scalar>(0.01); }

void ESKFMath::covariance_prediction(const Matrix15x15& P, const Matrix15x15& F, const Matrix15x15& Q, Matrix15x15& P_new) { Matrix15x15 F_P = F * P; Matrix15x15 F_P_Ft = F_P * F.transpose(); P_new = F_P_Ft + Q; for (int i=0;i<15;++i) for (int j=i+1;j<15;++j){ Scalar v = static_cast<Scalar>(0.5)*(P_new(i,j) + P_new(j,i)); P_new(i,j)=v; P_new(j,i)=v; } }

void ESKFMath::inject_error_state(const Vector3& p_in, const Vector3& v_in, const Vector4& q_in, const Vector3& ba_in, const Vector3& bg_in, const Vector15& dx, Vector3& p_out, Vector3& v_out, Vector4& q_out, Vector3& ba_out, Vector3& bg_out) { for (int i=0;i<3;++i){ p_out(i,0)=p_in(i,0)+dx(i,0); v_out(i,0)=v_in(i,0)+dx(i+3,0); ba_out(i,0)=ba_in(i,0)+dx(i+9,0); bg_out(i,0)=bg_in(i,0)+dx(i+12,0); } Vector4 dq; dq(0,0)=static_cast<Scalar>(1.0); dq(1,0)=dx(6,0)*static_cast<Scalar>(0.5); dq(2,0)=dx(7,0)*static_cast<Scalar>(0.5); dq(3,0)=dx(8,0)*static_cast<Scalar>(0.5); cquat::normalize_quat(dq); cquat::multiply_quat(q_in, dq, q_out); cquat::normalize_quat(q_out); }

void ESKFMath::mag_observation_prediction(const Vector4& q, const Vector3& m_world, Vector3& m_body_expected) { Matrix3x3 R; cquat::quat_to_rotm(q, R); for (int i=0;i<3;++i){ m_body_expected(i,0)=static_cast<Scalar>(0.0); for (int j=0;j<3;++j) m_body_expected(i,0) += R(i,j) * m_world(j,0); } }

void ESKFMath::gps_to_local(const Vector3& gps_pos, const Vector3& origin_pos, Vector3& local_pos) { for (int i=0;i<3;++i) local_pos(i,0) = gps_pos(i,0) - origin_pos(i,0); }

Scalar ESKFMath::pressure_to_altitude(Scalar pressure) { const Scalar p0 = static_cast<Scalar>(101325.0); const Scalar L = static_cast<Scalar>(0.0065); const Scalar T0 = static_cast<Scalar>(288.15); const Scalar g = static_cast<Scalar>(9.80665); const Scalar M = static_cast<Scalar>(0.0289644); const Scalar R = static_cast<Scalar>(8.31447); if (pressure <= static_cast<Scalar>(0.0)) return static_cast<Scalar>(0.0); Scalar h = (T0 / L) * (static_cast<Scalar>(1.0) - std::pow(pressure / p0, (R * L) / (g * M))); return h; }

} // namespace eskf_math
