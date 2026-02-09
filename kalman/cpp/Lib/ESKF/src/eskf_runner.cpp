#include "../inc/eskf_runner.hpp"
#include "../inc/eskf_includes.hpp"

namespace eskf {

HybridFilterRunner::HybridFilterRunner() {}

void HybridFilterRunner::predict(FilterState* s, const float* a_meas, const float* w_meas) {
    using namespace cmath_fx;
    using namespace common::sensor;
    using PredictParams = PredictPostprocessParams;

    // Local float representations (FilterState now uses float for core fields)
    Vector<3,float> p_f, v_f, ba_f, bg_f, a_meas_f, w_meas_f, g_f;
    Vector<4,float> q_f;
    Matrix<15,15,float> P_f, Qnom_f, Qadapt_f, Pnew_f;
    Vector<3,float> gyro_thr_f, accel_thr_f;

    // Copy from state (no casts required)
    for (int i = 0; i < 3; ++i) {
        p_f(i,0) = s->p[i];
        v_f(i,0) = s->v[i];
        ba_f(i,0) = s->ba[i];
        bg_f(i,0) = s->bg[i];
        a_meas_f(i,0) = a_meas ? a_meas[i] : 0.0f;
        w_meas_f(i,0) = w_meas ? w_meas[i] : 0.0f;
        g_f(i,0) = s->g[i];
        gyro_thr_f(i,0) = 0.0f;
        accel_thr_f(i,0) = 0.0f;
    }

    for (int i = 0; i < 4; ++i) q_f(i,0) = s->q[i];
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) { P_f(i,j) = s->P[i + j*15]; Qnom_f(i,j) = s->Q_nominal[i + j*15]; }

    float dt_f = s->dt;
    Qadapt_f = Qnom_f;
    if (s->adaptive_q_enabled) {
        HybridFilterCore::compute_adaptive_Q(Qnom_f, a_meas_f, w_meas_f, Qadapt_f);
    }

    HybridFilterCore::integrate_nominal(p_f, v_f, q_f, ba_f, bg_f, a_meas_f, w_meas_f, dt_f, g_f, gyro_thr_f, accel_thr_f);
    HybridFilterCore::predict_covariance(P_f, q_f, a_meas_f, ba_f, w_meas_f, bg_f, Qadapt_f, dt_f, Pnew_f);

    // Symmetrize and copy back
    cmath_fx::utils::symmetrize<15, float>(Pnew_f);
    P_f = Pnew_f;

    // copy body rates (float)
    if (w_meas) {
        for (int i = 0; i < 3; ++i) s->w_body[i] = w_meas[i];
    } else {
        for (int i = 0; i < 3; ++i) s->w_body[i] = 0.0f;
    }

    Vector<3,float> a_for_vel_f;
    for (int i = 0; i < 3; ++i) a_for_vel_f(i,0) = a_meas ? a_meas[i] : 0.0f;

    bool enable_accel_z = s->enable_accel_z_integration;
    float accel_z_threshold = s->accel_z_threshold;
    float accel_z_damping = s->accel_z_damping;
    float velocity_damping = s->velocity_damping;

    if (enable_accel_z) apply_accel_z_integration(v_f, q_f, a_for_vel_f, dt_f, g_f, accel_z_threshold, accel_z_damping);

    PredictParams params; params.enable_accel_z_integration = false; params.accel_z_threshold = accel_z_threshold; params.accel_z_damping = accel_z_damping; params.velocity_damping = velocity_damping;
    predict_postprocess(v_f, q_f, P_f, a_for_vel_f, dt_f, g_f, params);

    // regularize_covariance is stateless — use a lightweight local DivergenceGuard
    DivergenceGuard div_guard;
    using cm = cmath_fx::FixedMatrix;
    cm P_fixed(15,15);
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P_fixed(i,j) = P_f(i,j);
    div_guard.regularize_covariance(P_fixed);
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P_f(i,j) = P_fixed(i,j);

    common::covariance::ensure_positive_definite(P_f);
    apply_velocity_clipping(v_f, P_f, 3.0f);
    common::covariance::symmetrize(P_f);

    // Write results back to state (no casts)
    for (int i = 0; i < 3; ++i) { s->p[i] = p_f(i,0); s->v[i] = v_f(i,0); s->ba[i] = ba_f(i,0); s->bg[i] = bg_f(i,0); }
    for (int i = 0; i < 4; ++i) s->q[i] = q_f(i,0);
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) s->P[i + j*15] = P_f(i,j);
}

void HybridFilterRunner::apply_accel_z_integration(cmath_fx::Vector<3, float>& v, const cmath_fx::Vector<4, float>& q, const cmath_fx::Vector<3, float>& a_for_vel, float dt, const cmath_fx::Vector<3, float>& g, float accel_z_threshold, float accel_z_damping) {
    using namespace cmath_fx; using namespace cquat;
    Vector<4, float> q_norm = q; cquat::normalize_quat(q_norm); float R_row[9]; quat_to_rotm_array(q_norm, R_row); Matrix<3,3,float> R; for (int i=0;i<3;++i) for (int j=0;j<3;++j) R(i,j) = R_row[j*3 + i]; Vector<3,float> Ra; for (int i=0;i<3;++i) { Ra(i,0)=0.0f; for (int j=0;j<3;++j) Ra(i,0) += R(i,j) * a_for_vel(j,0); } Vector<3,float> a_ned; a_ned(0,0)=Ra(0,0); a_ned(1,0)=Ra(1,0); a_ned(2,0)=Ra(2,0) - g(2,0); float az_excess = a_ned(2,0); if (std::abs(az_excess) > accel_z_threshold) v(2,0) = v(2,0) * (1.0f - accel_z_damping) + az_excess * dt;
}

void HybridFilterRunner::apply_velocity_clipping(cmath_fx::Vector<3, float>& v, cmath_fx::Matrix<15, 15, float>& P, float max_vel) {
    float vnorm = 0.0f; for (int i=0;i<3;++i) vnorm += v(i,0)*v(i,0); vnorm = std::sqrt(vnorm); if (vnorm > max_vel) { float scale = max_vel / vnorm; for (int i=0;i<3;++i) v(i,0) *= scale; }
}

void HybridFilterRunner::regularize_covariance(cmath_fx::Matrix<15, 15, float>& P) {
    // Delegate to common normalization and central symmetrization helper
    common::covariance::ensure_positive_definite(P);
    common::covariance::symmetrize(P);
}

} // namespace eskf
