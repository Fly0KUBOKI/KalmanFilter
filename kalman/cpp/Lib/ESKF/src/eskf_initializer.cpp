#include "../inc/eskf_initializer.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Common/inc/Math/statistics.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Common/inc/Math/statistics.hpp"
#include "../../Common/inc/Math/geometry.hpp"
#include "../../Common/inc/Math/numerical.hpp"
#include "../../Common/inc/Math/math_utils.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include "../../Common/inc/Math/portable_math.hpp"

namespace eskf {

using namespace common::math;
using namespace cmath_fx;
using namespace cquat;

static void copy_vec_double(double* dst, const double* src, int n) { memcpy(dst, src, n * sizeof(double)); }
static void copy_vec_float(float* dst, const double* src, int n) { for (int i=0;i<n;++i) dst[i] = static_cast<float>(src[i]); }

ESKFState* initialize_eskf_state(const ESKFInitializationData& data) {
    ESKFState* s = new ESKFState(); memset(s, 0, sizeof(ESKFState)); s->valid = true; s->dt = data.dt;
    const float GRAVITY = 9.80665f; const float DEG2RAD = 0.017453292f;
    int N_static = data.n_static; if (N_static > data.n_samples) N_static = data.n_samples;
    float p_f[3] = {0.0f,0.0f,0.0f}; float v_f[3] = {0.0f,0.0f,0.0f}; float g_f[3] = {0.0f,0.0f,GRAVITY}; double q[4] = {1,0,0,0}; float ba_f[3]={0.0f,0.0f,0.0f}; float bg_f[3]={0.0f,0.0f,0.0f};
    float sigma_a = 0.1f; float sigma_g = DEG2RAD * 0.1f; float sigma_mag = 10.0f; float sigma_press = 1.0f; float sigma_gps = 1.0f; float gyro_noise_threshold = DEG2RAD * 0.1f; float gps_origin_f[3] = {0.0f,0.0f,0.0f};
    if (data.accel_x && data.accel_y && data.accel_z && N_static > 10) {
        double accel_mean_x_d, accel_mean_y_d, accel_mean_z_d; compute_mean_3d(data.accel_x, data.accel_y, data.accel_z, N_static, &accel_mean_x_d, &accel_mean_y_d, &accel_mean_z_d);
        float accel_mean_x = static_cast<float>(accel_mean_x_d), accel_mean_y = static_cast<float>(accel_mean_y_d), accel_mean_z = static_cast<float>(accel_mean_z_d);
        double sigma_a_d = compute_std_3d(data.accel_x, data.accel_y, data.accel_z, N_static, accel_mean_x_d, accel_mean_y_d, accel_mean_z_d);
        sigma_a = static_cast<float>(sigma_a_d); if (sigma_a < 0.01f) sigma_a = 0.01f;
        float phi = static_cast<float>(common::math::portable_atan2(-accel_mean_y, -accel_mean_z)); float theta = static_cast<float>(common::math::portable_atan2(accel_mean_x, common::math::portable_sqrt(accel_mean_y*accel_mean_y + accel_mean_z*accel_mean_z)));
        if (data.gyro_x && data.gyro_y && data.gyro_z) {
            double gyro_mean_x_d, gyro_mean_y_d, gyro_mean_z_d; compute_mean_3d(data.gyro_x, data.gyro_y, data.gyro_z, N_static, &gyro_mean_x_d, &gyro_mean_y_d, &gyro_mean_z_d);
            double sigma_g_deg_d = compute_std_3d(data.gyro_x, data.gyro_y, data.gyro_z, N_static, gyro_mean_x_d, gyro_mean_y_d, gyro_mean_z_d);
            sigma_g = static_cast<float>(DEG2RAD * sigma_g_deg_d); if (sigma_g < 0.001f) sigma_g = 0.001f;
            double std_wx = compute_std(data.gyro_x, N_static, gyro_mean_x_d); double std_wy = compute_std(data.gyro_y, N_static, gyro_mean_y_d); double std_wz = compute_std(data.gyro_z, N_static, gyro_mean_z_d); double max_std = std_wx; if (std_wy > max_std) max_std = std_wy; if (std_wz > max_std) max_std = std_wz; gyro_noise_threshold = static_cast<float>(2.0 * DEG2RAD * max_std);
        }
        float psi = 0.0f; if (data.mag_x && data.mag_y && data.mag_z) {
            double mag_mean_x_d, mag_mean_y_d, mag_mean_z_d; compute_mean_3d(data.mag_x, data.mag_y, data.mag_z, N_static, &mag_mean_x_d, &mag_mean_y_d, &mag_mean_z_d);
            float mag_mean_x = static_cast<float>(mag_mean_x_d), mag_mean_y = static_cast<float>(mag_mean_y_d), mag_mean_z = static_cast<float>(mag_mean_z_d);
            float sigma_mag_f = static_cast<float>(compute_std_3d(data.mag_x, data.mag_y, data.mag_z, N_static, mag_mean_x_d, mag_mean_y_d, mag_mean_z_d)); sigma_mag = (sigma_mag_f < 0.1f) ? 0.1f : sigma_mag_f;
            Vector<4, float> quat_rp; from_euler_deg(static_cast<float>(phi * 180.0f / common::math::MathUtils::PI), static_cast<float>(theta * 180.0f / common::math::MathUtils::PI), 0.0f, quat_rp); cquat::normalize_quat(quat_rp); float R_rp[9]; quat_to_rotm_array(quat_rp, R_rp);
            float m_level_x = R_rp[0]*mag_mean_x + R_rp[3]*mag_mean_y + R_rp[6]*mag_mean_z; float m_level_y = R_rp[1]*mag_mean_x + R_rp[4]*mag_mean_y + R_rp[7]*mag_mean_z; psi = -common::math::portable_atan2(m_level_y, m_level_x);
        }
        Vector<4,float> quat_final; from_euler_deg(static_cast<float>(phi * 180.0 / common::math::MathUtils::PI), static_cast<float>(theta * 180.0 / common::math::MathUtils::PI), static_cast<float>(psi * 180.0 / common::math::MathUtils::PI), quat_final); cquat::normalize_quat(quat_final); q[0]=static_cast<float>(quat_final(0,0)); q[1]=static_cast<float>(quat_final(1,0)); q[2]=static_cast<float>(quat_final(2,0)); q[3]=static_cast<float>(quat_final(3,0));
            if (data.pressure) { std::vector<float> alt_baro(N_static); float alt_mean=0.0f; for (int i=0;i<N_static;++i){ alt_baro[i]=common::math::pressure_to_altitude_simple(static_cast<float>(data.pressure[i])); alt_mean += alt_baro[i]; } alt_mean /= static_cast<float>(N_static); float sum_sq=0.0f; for (int i=0;i<N_static;++i){ float diff = alt_baro[i] - alt_mean; sum_sq += diff * diff; } sigma_press = common::math::portable_sqrt(sum_sq / static_cast<float>(N_static - 1)); if (sigma_press < 0.1f) sigma_press = 0.1f; }
        if (data.gps_lat && data.gps_lon && data.gps_alt) { double lat_sum=0, lon_sum=0, alt_sum=0; int valid_count=0; for (int i=0;i<N_static;++i){ if (!std::isnan(data.gps_lat[i])){ lat_sum += data.gps_lat[i]; lon_sum += data.gps_lon[i]; alt_sum += data.gps_alt[i]; valid_count++; } } if (valid_count>0){ double gps0_lat = lat_sum/valid_count; double gps0_lon = lon_sum/valid_count; double gps0_alt = alt_sum/valid_count; gps_origin_f[0] = static_cast<float>(gps0_lat); gps_origin_f[1] = static_cast<float>(gps0_lon); gps_origin_f[2] = static_cast<float>(gps0_alt); double cos_lat0 = cos(gps0_lat * DEG2RAD); std::vector<float> x_m(valid_count), y_m(valid_count), z_m(valid_count); int idx=0; for (int i=0;i<N_static;++i){ if (!std::isnan(data.gps_lat[i])){ y_m[idx] = static_cast<float>((data.gps_lat[i] - gps0_lat) / 9.0e-6); x_m[idx] = static_cast<float>((data.gps_lon[i] - gps0_lon) / (9.0e-6 / cos_lat0)); z_m[idx] = static_cast<float>(data.gps_alt[i] - gps0_alt); idx++; } } float mean_x=0.0f, mean_y=0.0f, mean_z=0.0f; for (int i=0;i<valid_count;++i){ mean_x += x_m[i]; mean_y += y_m[i]; mean_z += z_m[i]; } mean_x /= static_cast<float>(valid_count); mean_y /= static_cast<float>(valid_count); mean_z /= static_cast<float>(valid_count); float std_x = static_cast<float>(compute_std(x_m.data(), valid_count, mean_x)); float std_y = static_cast<float>(compute_std(y_m.data(), valid_count, mean_y)); float std_z = static_cast<float>(compute_std(z_m.data(), valid_count, mean_z)); sigma_gps = (std_x + std_y + std_z) / 3.0f; if (sigma_gps < 0.1f) sigma_gps = 0.1f; } }
    }
    float Q_f[15*15] = {0.0f}; for (int i=3;i<6;++i) Q_f[i*15 + i] = 0.003f * 0.003f; for (int i=6;i<9;++i) Q_f[i*15 + i] = 0.003f * 0.003f; for (int i=9;i<12;++i) Q_f[i*15 + i] = static_cast<float>(sigma_a * sigma_a * 1e-3); for (int i=12;i<15;++i) Q_f[i*15 + i] = static_cast<float>(sigma_g * sigma_g * 1e-3);
    float P_f[15*15] = {0.0f}; for (int i=0;i<15;++i) P_f[i*15 + i] = 0.01f; for (int i=0;i<3;++i) P_f[i*15 + i] = 5.0f; for (int i=3;i<6;++i) P_f[i*15 + i] = 0.5f; for (int i=9;i<12;++i) P_f[i*15 + i] = 0.5f; for (int i=12;i<15;++i) P_f[i*15 + i] = 0.1f;
    // write back into ESKFState (ESKFState now stores float for core fields)
    for (int i=0;i<3;++i) s->p[i] = static_cast<float>(p_f[i]);
    for (int i=0;i<3;++i) s->v[i] = static_cast<float>(v_f[i]);
    for (int i=0;i<3;++i) s->ba[i] = static_cast<float>(ba_f[i]);
    for (int i=0;i<3;++i) s->bg[i] = static_cast<float>(bg_f[i]);
    for (int i=0;i<15*15;++i) s->P[i] = static_cast<float>(P_f[i]);
    for (int i=0;i<15*15;++i) s->Q_nominal[i] = static_cast<float>(Q_f[i]);
    for (int i=0;i<3;++i) s->g[i] = static_cast<float>(g_f[i]);
    for (int i=0;i<3;++i) s->gps_origin[i] = gps_origin_f[i];
    s->gyro_noise_threshold = static_cast<float>(gyro_noise_threshold);
    double zeros3_d[3] = {0,0,0};
    // prev_accel/gyro/mag are float arrays: initialize to zero
    for (int i=0;i<3;++i) s->prev_accel[i] = 0.0f;
    for (int i=0;i<3;++i) s->prev_gyro[i] = 0.0f;
    for (int i=0;i<3;++i) s->prev_mag[i] = 0.0f;
    s->prev_gps_lat = 0.0f; s->prev_gps_lon = 0.0f; s->prev_gps_alt = 0.0f;
    s->prev_baro = 0.0f;
    s->buffer_tolerance = 1e-9f;
    s->zupt_threshold_accel = 1.0f;
    s->zupt_threshold_gyro = static_cast<float>(DEG2RAD * 3.0);
    s->zupt_min_duration = 10;
    s->zupt_counter = 0;
    s->is_stationary = false;
    s->adaptive_q_enabled = true;
    s->velocity_damping = 0.0f;
    s->enable_accel_z_integration = true;
    s->accel_z_threshold = 0.5f;
    s->accel_z_damping = 0.1f;
    s->baro_weight = 0.2f;
    for (int i=0;i<3;++i) s->w_body[i] = 0.0f;
    s->last_reset_step = 0;
    return s;
}

} // namespace eskf

