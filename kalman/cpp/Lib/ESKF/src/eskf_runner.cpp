#include "../inc/eskf_runner.hpp"
#include "../inc/eskf_core.hpp"
#include "../inc/eskf_postprocess.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include "../../Common/inc/Sensor/sensor_filter.hpp"
#include "../../Common/inc/filter_mgmt.hpp"
#include <cmath>
#include <cstring>
#include <vector>

namespace eskf {

ESKFRunner::ESKFRunner() {}

void ESKFRunner::predict(ESKFState* s, const double* a_meas, const double* w_meas) {
    using namespace cmath_fx;
    using namespace common::sensor;
    using PredictParams = PredictPostprocessParams;
    Vector<3,float> p_f, v_f, ba_f, bg_f, a_meas_f, w_meas_f, g_f; Vector<4,float> q_f; Matrix<15,15,float> P_f, Qnom_f, Qadapt_f, Pnew_f; Vector<3,float> gyro_thr_f, accel_thr_f;
    for (int i=0;i<3;++i){ p_f(i,0)=static_cast<float>(s->p[i]); v_f(i,0)=static_cast<float>(s->v[i]); ba_f(i,0)=static_cast<float>(s->ba[i]); bg_f(i,0)=static_cast<float>(s->bg[i]); a_meas_f(i,0)=static_cast<float>(a_meas[i]); w_meas_f(i,0)=static_cast<float>(w_meas[i]); g_f(i,0)=static_cast<float>(s->g[i]); gyro_thr_f(i,0)=0.0f; accel_thr_f(i,0)=0.0f; }
    for (int i=0;i<4;++i) q_f(i,0)=static_cast<float>(s->q[i]); for (int i=0;i<15;++i) for (int j=0;j<15;++j){ P_f(i,j)=static_cast<float>(s->P[i + j*15]); Qnom_f(i,j)=static_cast<float>(s->Q_nominal[i + j*15]); }
    float dt_f = static_cast<float>(s->dt);
    Qadapt_f = Qnom_f; if (s->adaptive_q_enabled) { ESKFCore::compute_adaptive_Q(Qnom_f, a_meas_f, w_meas_f, Qadapt_f); }
    ESKFCore::integrate_nominal(p_f, v_f, q_f, ba_f, bg_f, a_meas_f, w_meas_f, dt_f, g_f, gyro_thr_f, accel_thr_f);
    ESKFCore::predict_covariance(P_f, q_f, a_meas_f, ba_f, w_meas_f, bg_f, Qadapt_f, dt_f, Pnew_f);
    // Symmetrize using common helper
    {
        cmath_fx::Matrix<15,15,float> Ptmp;
        for (int i=0;i<15;++i) for (int j=0;j<15;++j) Ptmp(i,j) = Pnew_f(i,j);
        common::filter::symmetrize_covariance(Ptmp);
        for (int i=0;i<15;++i) for (int j=0;j<15;++j) Pnew_f(i,j) = Ptmp(i,j);
    }
    P_f = Pnew_f; memcpy(s->w_body, w_meas, 3*sizeof(double)); Vector<3,float> a_for_vel_f; for (int i=0;i<3;++i) a_for_vel_f(i,0)=static_cast<float>(a_meas[i]); bool enable_accel_z = s->enable_accel_z_integration; float accel_z_threshold = static_cast<float>(s->accel_z_threshold); float accel_z_damping = static_cast<float>(s->accel_z_damping); float velocity_damping = static_cast<float>(s->velocity_damping);
    if (enable_accel_z) apply_accel_z_integration(v_f, q_f, a_for_vel_f, dt_f, g_f, accel_z_threshold, accel_z_damping);
    PredictParams params; params.enable_accel_z_integration = false; params.accel_z_threshold = accel_z_threshold; params.accel_z_damping = accel_z_damping; params.velocity_damping = velocity_damping; predict_postprocess(v_f, q_f, P_f, a_for_vel_f, dt_f, g_f, params);
    static SensorFilterLib filter_lib; using cm = cmath_fx::FixedMatrix; cm P_fixed(15,15); for (int i=0;i<15;++i) for (int j=0;j<15;++j) P_fixed(i,j) = P_f(i,j); filter_lib.divergence_guard.regularize_covariance(P_fixed); for (int i=0;i<15;++i) for (int j=0;j<15;++j) P_f(i,j) = P_fixed(i,j);
    regularize_covariance(P_f); apply_velocity_clipping(v_f, P_f, 3.0f);
    {
        cmath_fx::Matrix<15,15,float> Ptmp;
        for (int i=0;i<15;++i) for (int j=0;j<15;++j) Ptmp(i,j) = P_f(i,j);
        common::filter::symmetrize_covariance(Ptmp);
        for (int i=0;i<15;++i) for (int j=0;j<15;++j) P_f(i,j) = Ptmp(i,j);
    }
    for (int i=0;i<3;++i){ s->p[i]=static_cast<double>(p_f(i,0)); s->v[i]=static_cast<double>(v_f(i,0)); s->ba[i]=static_cast<double>(ba_f(i,0)); s->bg[i]=static_cast<double>(bg_f(i,0)); }
    for (int i=0;i<4;++i) s->q[i]=static_cast<double>(q_f(i,0)); for (int i=0;i<15;++i) for (int j=0;j<15;++j) s->P[i + j*15] = static_cast<double>(P_f(i,j));
}

void ESKFRunner::apply_accel_z_integration(cmath_fx::Vector<3, float>& v, const cmath_fx::Vector<4, float>& q, const cmath_fx::Vector<3, float>& a_for_vel, float dt, const cmath_fx::Vector<3, float>& g, float accel_z_threshold, float accel_z_damping) {
    using namespace cmath_fx; using namespace cquat;
    Vector<4, float> q_norm = q; cquat::normalize_quat(q_norm); float R_row[9]; quat_to_rotm_array(q_norm, R_row); Matrix<3,3,float> R; for (int i=0;i<3;++i) for (int j=0;j<3;++j) R(i,j) = R_row[j*3 + i]; Vector<3,float> Ra; for (int i=0;i<3;++i) { Ra(i,0)=0.0f; for (int j=0;j<3;++j) Ra(i,0) += R(i,j) * a_for_vel(j,0); } Vector<3,float> a_ned; a_ned(0,0)=Ra(0,0); a_ned(1,0)=Ra(1,0); a_ned(2,0)=Ra(2,0) - g(2,0); float az_excess = a_ned(2,0); if (std::abs(az_excess) > accel_z_threshold) v(2,0) = v(2,0) * (1.0f - accel_z_damping) + az_excess * dt;
}

void ESKFRunner::apply_velocity_clipping(cmath_fx::Vector<3, float>& v, cmath_fx::Matrix<15, 15, float>& P, float max_vel) {
    float vnorm = 0.0f; for (int i=0;i<3;++i) vnorm += v(i,0)*v(i,0); vnorm = std::sqrt(vnorm); if (vnorm > max_vel) { float scale = max_vel / vnorm; for (int i=0;i<3;++i) v(i,0) *= scale; }
}

void ESKFRunner::regularize_covariance(cmath_fx::Matrix<15, 15, float>& P) {
    // Delegate to common normalization and symmetrization helpers to ensure consistent behavior
    cmath_fx::Matrix<15,15,float> Ptmp;
    for (int i=0;i<15;++i) for (int j=0;j<15;++j) Ptmp(i,j) = P(i,j);
    common::filter::normalize_covariance(Ptmp);
    common::filter::symmetrize_covariance(Ptmp);
    for (int i=0;i<15;++i) for (int j=0;j<15;++j) P(i,j) = Ptmp(i,j);
}

} // namespace eskf
