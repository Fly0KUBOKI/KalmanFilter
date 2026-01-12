#include "../inc/eskf_sensor_updates.hpp"
#include "../inc/eskf_state.hpp"
#include <cmath>
#include "../../Core/portable_math.hpp"

namespace eskf {

using namespace cmath_fx;
using namespace common::sensor;
using cm = cmath_fx::FixedMatrix;

#include <cmath>
static bool is_nan_any(const float* v, int n) {
    for (int i = 0; i < n; ++i) {
        if (std::isnan(v[i])) return true;
    }
    return false;
}

static void copy_vec(float* dst, const float* src, int n) {
    for (int i = 0; i < n; ++i) dst[i] = src[i];
}

SensorUpdateResult update_accel_sensor(ESKFState* s, const Vector<3, float>& a_meas, SensorFilterLib& filter_lib) {
    SensorUpdateResult result; result.should_skip = true; result.updated = false;
    Vector<3, float> prev_accel_f; for (int i = 0; i < 3; ++i) prev_accel_f(i, 0) = s->prev_accel[i];
    PreprocessResult pre = preprocess_accel(a_meas, prev_accel_f, s->buffer_tolerance);
    float meas_arr[3] = {a_meas(0, 0), a_meas(1, 0), a_meas(2, 0)};
    if (!pre.no_change && !is_nan_any(meas_arr, 3) && !pre.is_outlier) {
        float w_norm = 0.0f; for (int i = 0; i < 3; ++i) { float w = static_cast<float>(s->w_body[i]); w_norm += w*w; } w_norm = common::math::portable_sqrt(w_norm);
        if (w_norm <= 1.5f) {
            result.should_skip = false; copy_vec(s->prev_accel, meas_arr, 3);
            Vector<4, float> q_f; for (int i = 0; i < 4; ++i) q_f(i,0) = s->q[i];
            ESKFCore::update_accel(q_f, pre.output, 1.0f);
            for (int i = 0; i < 4; ++i) s->q[i] = q_f(i,0); result.updated = true;
        }
    }
    return result;
}

SensorUpdateResult update_mag_sensor(ESKFState* s, const Vector<3, float>& m_meas, SensorFilterLib& filter_lib) {
    SensorUpdateResult result; result.should_skip = true; result.updated = false;
    Vector<3, float> prev_mag_f; for (int i = 0; i < 3; ++i) prev_mag_f(i, 0) = s->prev_mag[i];
    PreprocessResult pre = preprocess_mag(m_meas, prev_mag_f, s->buffer_tolerance);
    float meas_arr[3] = {m_meas(0, 0), m_meas(1, 0), m_meas(2, 0)};
    if (!pre.no_change && !is_nan_any(meas_arr, 3) && !pre.is_outlier) {
        result.should_skip = false; copy_vec(s->prev_mag, meas_arr, 3);
        Vector<3, float> p_f, v_f, ba_f, bg_f; Vector<4, float> q_f; Matrix<15,15,float> P_f;
        for (int i=0;i<3;++i){ p_f(i,0)=s->p[i]; v_f(i,0)=s->v[i]; ba_f(i,0)=s->ba[i]; bg_f(i,0)=s->bg[i]; }
        for (int i=0;i<4;++i) q_f(i,0)=s->q[i]; for (int i=0;i<15;++i) for (int j=0;j<15;++j) P_f(i,j)=s->P[i + j*15];
        cm R_mat = filter_lib.noise_estimator.get_R_matrix("mag"); Matrix<3,3,float> R_mag; float mag_min=0.01f; for(int i=0;i<3;++i) for(int j=0;j<3;++j) R_mag(i,j) = (i==j)? std::fmax(static_cast<float>(R_mat(i,0))*1.5f, mag_min) : 0.0f;
        Vector<3, float> m_world; m_world(0,0)=50.0f; m_world(1,0)=0.0f; m_world(2,0)=0.0f;
        Matrix<15,3,float> K_out; Vector<15,float> dx_out; ESKFCore::update_mag(q_f, P_f, pre.output, m_world, R_mag, K_out, dx_out);
        UpdatePostprocessResult post_result = update_state_from_dx(dx_out, p_f, v_f, q_f, ba_f, bg_f, P_f);
        if (!post_result.should_skip) {
            for (int i=0;i<3;++i){ s->p[i]=post_result.p(i,0); s->v[i]=post_result.v(i,0); s->ba[i]=post_result.ba(i,0); s->bg[i]=post_result.bg(i,0); }
            for (int i=0;i<4;++i) s->q[i]=post_result.q(i,0); for (int i=0;i<15;++i) for (int j=0;j<15;++j) s->P[i + j*15] = post_result.P(i,j); result.updated = true;
        } else { result.should_skip = true; }
    }
    return result;
}

SensorUpdateResult update_baro_sensor(ESKFState* s, double pressure, SensorFilterLib& filter_lib) {
    SensorUpdateResult result; result.should_skip = true; result.updated = false; float prev_baro = s->prev_baro;
    if (std::fabs(pressure - prev_baro) > s->buffer_tolerance) {
        result.should_skip = false; s->prev_baro = pressure; double alt_baro = preprocess_baro(pressure);
        Vector<3,float> p_f, gps_origin_f; Vector<4,float> q_f; Matrix<15,15,float> P_f; for (int i=0;i<3;++i){ p_f(i,0)=s->p[i]; gps_origin_f(i,0)=static_cast<float>(s->gps_origin[i]); }
        for (int i=0;i<4;++i) q_f(i,0)=s->q[i]; for (int i=0;i<15;++i) for (int j=0;j<15;++j) P_f(i,j)=s->P[i + j*15];
        cm R_mat = filter_lib.noise_estimator.get_R_matrix("baro"); float R_baro = static_cast<float>(R_mat(0,0)) * static_cast<float>(s->baro_weight);
        Matrix<15,1,float> K_out; Vector<15,float> dx_out; float alt_f = static_cast<float>(alt_baro); ESKFCore::update_baro(p_f, P_f, alt_f, gps_origin_f, R_baro, K_out, dx_out);
        Vector<3,float> v_f, ba_f, bg_f; for (int i=0;i<3;++i){ v_f(i,0)=s->v[i]; ba_f(i,0)=s->ba[i]; bg_f(i,0)=s->bg[i]; }
        UpdatePostprocessResult post_result = update_state_from_dx(dx_out, p_f, v_f, q_f, ba_f, bg_f, P_f);
        if (!post_result.should_skip) { for (int i=0;i<3;++i){ s->p[i]=post_result.p(i,0); s->v[i]=post_result.v(i,0); s->ba[i]=post_result.ba(i,0); s->bg[i]=post_result.bg(i,0); } for (int i=0;i<4;++i) s->q[i]=post_result.q(i,0); for (int i=0;i<15;++i) for (int j=0;j<15;++j) s->P[i + j*15] = post_result.P(i,j); result.updated = true; } else { result.should_skip = true; }
    }
    return result;
}

SensorUpdateResult update_gps_sensor(ESKFState* s, double lat, double lon, double alt, SensorFilterLib& filter_lib) {
    SensorUpdateResult result; result.should_skip = true; result.updated = false;
    Vector<3,float> p_f, v_f, ba_f, bg_f, gps_origin_f; Vector<4,float> q_f; Matrix<15,15,float> P_f; for (int i=0;i<3;++i){ p_f(i,0)=s->p[i]; v_f(i,0)=s->v[i]; ba_f(i,0)=s->ba[i]; bg_f(i,0)=s->bg[i]; gps_origin_f(i,0)=static_cast<float>(s->gps_origin[i]); }
    for (int i=0;i<4;++i) q_f(i,0)=s->q[i]; for (int i=0;i<15;++i) for (int j=0;j<15;++j) P_f(i,j)=s->P[i + j*15];
    PreprocessResult pre = preprocess_gps(lat, lon, alt, gps_origin_f, s->buffer_tolerance);
    if (pre.no_change || pre.is_outlier) return result; result.should_skip = false;
    cm R_mat = filter_lib.noise_estimator.get_R_matrix("gps"); Matrix<3,3,float> R_gps; for (int i=0;i<3;++i) for (int j=0;j<3;++j) R_gps(i,j) = (i==j)? static_cast<float>(R_mat(i,0)) : 0.0f;
    Matrix<15,3,float> K_out; Vector<15,float> dx_out; ESKFCore::update_gps(p_f, v_f, P_f, pre.output, gps_origin_f, R_gps, K_out, dx_out);
    UpdatePostprocessResult post_result = update_state_from_dx(dx_out, p_f, v_f, q_f, ba_f, bg_f, P_f);
    if (!post_result.should_skip) { for (int i=0;i<3;++i){ s->p[i]=post_result.p(i,0); s->v[i]=post_result.v(i,0); s->ba[i]=post_result.ba(i,0); s->bg[i]=post_result.bg(i,0); } for (int i=0;i<4;++i) s->q[i]=post_result.q(i,0); for (int i=0;i<15;++i) for (int j=0;j<15;++j) s->P[i + j*15] = post_result.P(i,j); s->prev_gps_lat = lat; s->prev_gps_lon = lon; s->prev_gps_alt = alt; result.updated = true; } else { result.should_skip = true; }
    return result;
}

} // namespace eskf
