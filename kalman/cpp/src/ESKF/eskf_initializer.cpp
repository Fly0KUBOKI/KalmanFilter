#include "../../Inc/ESKF/eskf_initializer.hpp"
#include "../../Inc/ESKF/eskf_state.hpp"
#include "../../Inc/Common/Math/quaternion_lib.hpp"
#include "../../Inc/Common/Math/statistics.hpp"
#include <mex.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace eskf {

using namespace common::math;
using Quat = quat_lib::Quaternion<double>;

// Helper functions for MATLAB struct access
static const mxArray* get_field(const mxArray* s, const char* name) {
    if (!mxIsStruct(s)) return nullptr;
    return mxGetField(s, 0, name);
}

static const mxArray* get_field_any(const mxArray* s, const char* name1, const char* name2) {
    const mxArray* f = get_field(s, name1);
    if (f) return f;
    return get_field(s, name2);
}

static double* get_data(const mxArray* arr) {
    if (!arr) return nullptr;
    return mxGetPr(arr);
}

static int get_length(const mxArray* arr) {
    if (!arr) return 0;
    return static_cast<int>(mxGetNumberOfElements(arr));
}

static void copy_vec(double* dst, const double* src, int n) {
    memcpy(dst, src, n * sizeof(double));
}

ESKFState* initialize_eskf_from_matlab(const mxArray* obs, double static_time, double dt) {
    ESKFState* s = new ESKFState();
    memset(s, 0, sizeof(ESKFState));
    s->valid = true;
    s->dt = dt;
    
    const double GRAVITY = 9.80665;
    const double DEG2RAD = 0.017453292519943295;
    
    // 静止サンプル数の計算
    int N_static = static_cast<int>(floor(static_time / dt));
    
    // 基本状態の初期化
    double p[3] = {0, 0, 0};
    double v[3] = {0, 0, 0};
    double g[3] = {0, 0, GRAVITY};
    double q[4] = {1, 0, 0, 0};
    double ba[3] = {0, 0, 0};
    double bg[3] = {0, 0, 0};
    
    // ノイズパラメータのデフォルト値
    double sigma_a = 0.1;
    double sigma_g = DEG2RAD * 0.1;
    double sigma_mag = 10.0;
    double sigma_press = 1.0;
    double sigma_gps = 1.0;
    double gyro_noise_threshold = DEG2RAD * 0.1;
    
    // GPS原点
    double gps_origin[3] = {0, 0, 0};
    
    // 静止データがある場合の処理
    const mxArray* ax_arr = get_field_any(obs, "ax", "accel_x");
    const mxArray* ay_arr = get_field_any(obs, "ay", "accel_y");
    const mxArray* az_arr = get_field_any(obs, "az", "accel_z");
    
    int n_samples = ax_arr ? get_length(ax_arr) : 0;
    if (N_static > n_samples) N_static = n_samples;
    
    if (ax_arr && ay_arr && az_arr && N_static > 10) {
        double* ax = get_data(ax_arr);
        double* ay = get_data(ay_arr);
        double* az = get_data(az_arr);
        
        // 加速度平均と標準偏差
        double accel_mean_x, accel_mean_y, accel_mean_z;
        compute_mean_3d(ax, ay, az, N_static, &accel_mean_x, &accel_mean_y, &accel_mean_z);
        sigma_a = compute_std_3d(ax, ay, az, N_static, accel_mean_x, accel_mean_y, accel_mean_z);
        if (sigma_a < 0.01) sigma_a = 0.01;
        
        // 初期姿勢計算（Roll/Pitch）
        double phi = atan2(-accel_mean_y, -accel_mean_z);
        double theta = atan2(accel_mean_x, sqrt(accel_mean_y*accel_mean_y + accel_mean_z*accel_mean_z));
        
        // ジャイロデータ
        const mxArray* wx_arr = get_field_any(obs, "wx", "gyro_x");
        const mxArray* wy_arr = get_field_any(obs, "wy", "gyro_y");
        const mxArray* wz_arr = get_field_any(obs, "wz", "gyro_z");
        
        if (wx_arr && wy_arr && wz_arr) {
            double* wx = get_data(wx_arr);
            double* wy = get_data(wy_arr);
            double* wz = get_data(wz_arr);
            
            double gyro_mean_x, gyro_mean_y, gyro_mean_z;
            compute_mean_3d(wx, wy, wz, N_static, &gyro_mean_x, &gyro_mean_y, &gyro_mean_z);
            double sigma_g_deg = compute_std_3d(wx, wy, wz, N_static, gyro_mean_x, gyro_mean_y, gyro_mean_z);
            sigma_g = DEG2RAD * sigma_g_deg;
            if (sigma_g < 0.001) sigma_g = 0.001;
            
            // gyro_noise_threshold の計算
            double std_wx = compute_std(wx, N_static, gyro_mean_x);
            double std_wy = compute_std(wy, N_static, gyro_mean_y);
            double std_wz = compute_std(wz, N_static, gyro_mean_z);
            double max_std = std_wx;
            if (std_wy > max_std) max_std = std_wy;
            if (std_wz > max_std) max_std = std_wz;
            gyro_noise_threshold = 2.0 * DEG2RAD * max_std;
        }
        
        // 磁気データからYaw計算
        const mxArray* mx_arr = get_field_any(obs, "mx", "mag_x");
        const mxArray* my_arr = get_field_any(obs, "my", "mag_y");
        const mxArray* mz_arr = get_field_any(obs, "mz", "mag_z");
        
        double psi = 0.0;
        if (mx_arr && my_arr && mz_arr) {
            double* mx = get_data(mx_arr);
            double* my = get_data(my_arr);
            double* mz = get_data(mz_arr);
            
            double mag_mean_x, mag_mean_y, mag_mean_z;
            compute_mean_3d(mx, my, mz, N_static, &mag_mean_x, &mag_mean_y, &mag_mean_z);
            sigma_mag = compute_std_3d(mx, my, mz, N_static, mag_mean_x, mag_mean_y, mag_mean_z);
            if (sigma_mag < 0.1) sigma_mag = 0.1;
            
            // Roll/Pitchのみのクォータニオン
            Quat quat_rp = Quat::from_euler(phi * 180.0 / M_PI, theta * 180.0 / M_PI, 0.0);
            quat_rp.normalize();
            
            // 回転行列
            double R_rp[9];
            quat_rp.to_rotation_matrix(R_rp);
            
            // 磁気を水平面に射影
            double m_level_x = R_rp[0]*mag_mean_x + R_rp[3]*mag_mean_y + R_rp[6]*mag_mean_z;
            double m_level_y = R_rp[1]*mag_mean_x + R_rp[4]*mag_mean_y + R_rp[7]*mag_mean_z;
            
            psi = -atan2(m_level_y, m_level_x);
        }
        
        // 最終クォータニオン
        Quat quat_final = Quat::from_euler(phi * 180.0 / M_PI, theta * 180.0 / M_PI, psi * 180.0 / M_PI);
        quat_final.normalize();
        q[0] = quat_final.w;
        q[1] = quat_final.x;
        q[2] = quat_final.y;
        q[3] = quat_final.z;
        
        // 気圧データ
        const mxArray* pressure_arr = get_field_any(obs, "pressure", "baro");
        if (pressure_arr) {
            double* pressure = get_data(pressure_arr);
            
            // 気圧高度計算
            std::vector<double> alt_baro(N_static);
            double alt_mean = 0.0;
            for (int i = 0; i < N_static; ++i) {
                alt_baro[i] = 44330.0 * (1.0 - pow(pressure[i] / 101325.0, 0.1903));
                alt_mean += alt_baro[i];
            }
            alt_mean /= N_static;
            
            double sum_sq = 0.0;
            for (int i = 0; i < N_static; ++i) {
                double diff = alt_baro[i] - alt_mean;
                sum_sq += diff * diff;
            }
            sigma_press = sqrt(sum_sq / (N_static - 1));
            if (sigma_press < 0.1) sigma_press = 0.1;
        }
        
        // GPSデータ
        const mxArray* lat_arr = get_field_any(obs, "lat", "gps_lat");
        const mxArray* lon_arr = get_field_any(obs, "lon", "gps_lon");
        const mxArray* alt_arr = get_field_any(obs, "alt", "gps_alt");
        
        if (lat_arr && lon_arr && alt_arr) {
            double* lat = get_data(lat_arr);
            double* lon = get_data(lon_arr);
            double* alt = get_data(alt_arr);
            
            // GPS原点計算（NaNを除外）
            double lat_sum = 0.0, lon_sum = 0.0, alt_sum = 0.0;
            int valid_count = 0;
            for (int i = 0; i < N_static; ++i) {
                if (!mxIsNaN(lat[i])) {
                    lat_sum += lat[i];
                    lon_sum += lon[i];
                    alt_sum += alt[i];
                    valid_count++;
                }
            }
            
            if (valid_count > 0) {
                gps_origin[0] = lat_sum / valid_count;
                gps_origin[1] = lon_sum / valid_count;
                gps_origin[2] = alt_sum / valid_count;
                
                // GPS標準偏差計算
                double cos_lat0 = cos(gps_origin[0] * DEG2RAD);
                std::vector<double> x_m(valid_count), y_m(valid_count), z_m(valid_count);
                int idx = 0;
                for (int i = 0; i < N_static; ++i) {
                    if (!mxIsNaN(lat[i])) {
                        y_m[idx] = (lat[i] - gps_origin[0]) / 9.0e-6;
                        x_m[idx] = (lon[i] - gps_origin[1]) / (9.0e-6 / cos_lat0);
                        z_m[idx] = alt[i] - gps_origin[2];
                        idx++;
                    }
                }
                
                double mean_x = 0, mean_y = 0, mean_z = 0;
                for (int i = 0; i < valid_count; ++i) {
                    mean_x += x_m[i];
                    mean_y += y_m[i];
                    mean_z += z_m[i];
                }
                mean_x /= valid_count;
                mean_y /= valid_count;
                mean_z /= valid_count;
                
                double std_x = compute_std(x_m.data(), valid_count, mean_x);
                double std_y = compute_std(y_m.data(), valid_count, mean_y);
                double std_z = compute_std(z_m.data(), valid_count, mean_z);
                sigma_gps = (std_x + std_y + std_z) / 3.0;
                if (sigma_gps < 0.1) sigma_gps = 0.1;
            }
        }
    }
    
    // Q行列の初期化
    double Q[15*15] = {0};
    for (int i = 3; i < 6; ++i) Q[i*15 + i] = 0.003 * 0.003;  // 速度
    for (int i = 6; i < 9; ++i) Q[i*15 + i] = 0.003 * 0.003;  // 姿勢
    for (int i = 9; i < 12; ++i) Q[i*15 + i] = sigma_a * sigma_a * 1e-3;  // 加速度バイアス
    for (int i = 12; i < 15; ++i) Q[i*15 + i] = sigma_g * sigma_g * 1e-3;  // ジャイロバイアス
    
    // P行列の初期化
    double P[15*15] = {0};
    for (int i = 0; i < 15; ++i) P[i*15 + i] = 0.01;
    for (int i = 0; i < 3; ++i) P[i*15 + i] = 5.0;  // 位置
    for (int i = 3; i < 6; ++i) P[i*15 + i] = 0.5;  // 速度
    for (int i = 9; i < 12; ++i) P[i*15 + i] = 0.5;  // 加速度バイアス
    for (int i = 12; i < 15; ++i) P[i*15 + i] = 0.1;  // ジャイロバイアス
    
    // 状態をコピー
    copy_vec(s->p, p, 3);
    copy_vec(s->v, v, 3);
    copy_vec(s->q, q, 4);
    copy_vec(s->ba, ba, 3);
    copy_vec(s->bg, bg, 3);
    memcpy(s->P, P, 15*15*sizeof(double));
    memcpy(s->Q_nominal, Q, 15*15*sizeof(double));
    copy_vec(s->g, g, 3);
    copy_vec(s->gps_origin, gps_origin, 3);
    s->gyro_noise_threshold = gyro_noise_threshold;
    
    // 前回値の初期化
    double zeros3[3] = {0, 0, 0};
    copy_vec(s->prev_accel, zeros3, 3);
    copy_vec(s->prev_gyro, zeros3, 3);
    copy_vec(s->prev_mag, zeros3, 3);
    s->prev_gps_lat = 0;
    s->prev_gps_lon = 0;
    s->prev_gps_alt = 0;
    s->prev_baro = 0;
    s->buffer_tolerance = 1e-9;
    
    // その他のパラメータ
    s->zupt_threshold_accel = 1.0;
    s->zupt_threshold_gyro = DEG2RAD * 3.0;
    s->zupt_min_duration = 10;
    s->zupt_counter = 0;
    s->is_stationary = false;
    s->adaptive_q_enabled = true;
    s->velocity_damping = 0.0;
    s->enable_accel_z_integration = true;
    s->accel_z_threshold = 0.5;
    s->accel_z_damping = 0.1;
    s->baro_weight = 0.2;
    copy_vec(s->w_body, zeros3, 3);
    s->last_reset_step = 0;
    
    return s;
}

} // namespace eskf

