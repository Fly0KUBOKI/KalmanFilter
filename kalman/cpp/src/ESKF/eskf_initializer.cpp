#include "../../Inc/ESKF/eskf_initializer.hpp"
#include "../../Lib/Quaternion/quaternion_lib.hpp"
#include "../../Inc/Common/Math/statistics.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace eskf {

using namespace common::math;
using Quat = quat_lib::Quaternion<double>;

static void copy_vec(double* dst, const double* src, int n) {
    memcpy(dst, src, n * sizeof(double));
}

ESKFState* initialize_eskf_state(const ESKFInitializationData& data) {
    ESKFState* s = new ESKFState();
    memset(s, 0, sizeof(ESKFState));
    s->valid = true;
    s->dt = data.dt;
    
    const double GRAVITY = 9.80665;
    const double DEG2RAD = 0.017453292519943295;
    
    // Calculate number of static samples
    int N_static = data.n_static;
    if (N_static > data.n_samples) N_static = data.n_samples;
    
    // Initialize basic state
    double p[3] = {0, 0, 0};
    double v[3] = {0, 0, 0};
    double g[3] = {0, 0, GRAVITY};
    double q[4] = {1, 0, 0, 0};
    double ba[3] = {0, 0, 0};
    double bg[3] = {0, 0, 0};
    
    // Default noise parameters
    double sigma_a = 0.1;
    double sigma_g = DEG2RAD * 0.1;
    double sigma_mag = 10.0;
    double sigma_press = 1.0;
    double sigma_gps = 1.0;
    double gyro_noise_threshold = DEG2RAD * 0.1;
    
    // GPS origin
    double gps_origin[3] = {0, 0, 0};
    
    // Process static data if available
    if (data.accel_x && data.accel_y && data.accel_z && N_static > 10) {
        // Acceleration mean and standard deviation
        double accel_mean_x, accel_mean_y, accel_mean_z;
        compute_mean_3d(data.accel_x, data.accel_y, data.accel_z, N_static, 
                        &accel_mean_x, &accel_mean_y, &accel_mean_z);
        sigma_a = compute_std_3d(data.accel_x, data.accel_y, data.accel_z, N_static, 
                                 accel_mean_x, accel_mean_y, accel_mean_z);
        if (sigma_a < 0.01) sigma_a = 0.01;
        
        // Initial attitude calculation (Roll/Pitch)
        double phi = atan2(-accel_mean_y, -accel_mean_z);
        double theta = atan2(accel_mean_x, sqrt(accel_mean_y*accel_mean_y + accel_mean_z*accel_mean_z));
        
        // Gyro data
        if (data.gyro_x && data.gyro_y && data.gyro_z) {
            double gyro_mean_x, gyro_mean_y, gyro_mean_z;
            compute_mean_3d(data.gyro_x, data.gyro_y, data.gyro_z, N_static, 
                            &gyro_mean_x, &gyro_mean_y, &gyro_mean_z);
            double sigma_g_deg = compute_std_3d(data.gyro_x, data.gyro_y, data.gyro_z, N_static, 
                                                 gyro_mean_x, gyro_mean_y, gyro_mean_z);
            sigma_g = DEG2RAD * sigma_g_deg;
            if (sigma_g < 0.001) sigma_g = 0.001;
            
            // Calculate gyro_noise_threshold
            double std_wx = compute_std(data.gyro_x, N_static, gyro_mean_x);
            double std_wy = compute_std(data.gyro_y, N_static, gyro_mean_y);
            double std_wz = compute_std(data.gyro_z, N_static, gyro_mean_z);
            double max_std = std_wx;
            if (std_wy > max_std) max_std = std_wy;
            if (std_wz > max_std) max_std = std_wz;
            gyro_noise_threshold = 2.0 * DEG2RAD * max_std;
        }
        
        // Magnetometer data for Yaw calculation
        double psi = 0.0;
        if (data.mag_x && data.mag_y && data.mag_z) {
            double mag_mean_x, mag_mean_y, mag_mean_z;
            compute_mean_3d(data.mag_x, data.mag_y, data.mag_z, N_static, 
                            &mag_mean_x, &mag_mean_y, &mag_mean_z);
            sigma_mag = compute_std_3d(data.mag_x, data.mag_y, data.mag_z, N_static, 
                                       mag_mean_x, mag_mean_y, mag_mean_z);
            if (sigma_mag < 0.1) sigma_mag = 0.1;
            
            // Roll/Pitch only quaternion
            Quat quat_rp = Quat::from_euler(phi * 180.0 / M_PI, theta * 180.0 / M_PI, 0.0);
            quat_rp.normalize();
            
            // Rotation matrix
            double R_rp[9];
            quat_rp.to_rotation_matrix(R_rp);
            
            // Project magnetometer to horizontal plane
            double m_level_x = R_rp[0]*mag_mean_x + R_rp[3]*mag_mean_y + R_rp[6]*mag_mean_z;
            double m_level_y = R_rp[1]*mag_mean_x + R_rp[4]*mag_mean_y + R_rp[7]*mag_mean_z;
            
            psi = -atan2(m_level_y, m_level_x);
        }
        
        // Final quaternion
        Quat quat_final = Quat::from_euler(phi * 180.0 / M_PI, theta * 180.0 / M_PI, psi * 180.0 / M_PI);
        quat_final.normalize();
        q[0] = quat_final.w;
        q[1] = quat_final.x;
        q[2] = quat_final.y;
        q[3] = quat_final.z;
        
        // Barometric pressure data
        if (data.pressure) {
            // Barometric altitude calculation
            std::vector<double> alt_baro(N_static);
            double alt_mean = 0.0;
            for (int i = 0; i < N_static; ++i) {
                alt_baro[i] = 44330.0 * (1.0 - pow(data.pressure[i] / 101325.0, 0.1903));
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
        
        // GPS data
        if (data.gps_lat && data.gps_lon && data.gps_alt) {
            // GPS origin calculation (exclude NaN)
            double lat_sum = 0.0, lon_sum = 0.0, alt_sum = 0.0;
            int valid_count = 0;
            for (int i = 0; i < N_static; ++i) {
                if (!std::isnan(data.gps_lat[i])) {
                    lat_sum += data.gps_lat[i];
                    lon_sum += data.gps_lon[i];
                    alt_sum += data.gps_alt[i];
                    valid_count++;
                }
            }
            
            if (valid_count > 0) {
                gps_origin[0] = lat_sum / valid_count;
                gps_origin[1] = lon_sum / valid_count;
                gps_origin[2] = alt_sum / valid_count;
                
                // GPS standard deviation calculation
                double cos_lat0 = cos(gps_origin[0] * DEG2RAD);
                std::vector<double> x_m(valid_count), y_m(valid_count), z_m(valid_count);
                int idx = 0;
                for (int i = 0; i < N_static; ++i) {
                    if (!std::isnan(data.gps_lat[i])) {
                        y_m[idx] = (data.gps_lat[i] - gps_origin[0]) / 9.0e-6;
                        x_m[idx] = (data.gps_lon[i] - gps_origin[1]) / (9.0e-6 / cos_lat0);
                        z_m[idx] = data.gps_alt[i] - gps_origin[2];
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
    
    // Q matrix initialization
    double Q[15*15] = {0};
    for (int i = 3; i < 6; ++i) Q[i*15 + i] = 0.003 * 0.003;  // velocity
    for (int i = 6; i < 9; ++i) Q[i*15 + i] = 0.003 * 0.003;  // attitude
    for (int i = 9; i < 12; ++i) Q[i*15 + i] = sigma_a * sigma_a * 1e-3;  // accel bias
    for (int i = 12; i < 15; ++i) Q[i*15 + i] = sigma_g * sigma_g * 1e-3;  // gyro bias
    
    // P matrix initialization
    double P[15*15] = {0};
    for (int i = 0; i < 15; ++i) P[i*15 + i] = 0.01;
    for (int i = 0; i < 3; ++i) P[i*15 + i] = 5.0;  // position
    for (int i = 3; i < 6; ++i) P[i*15 + i] = 0.5;  // velocity
    for (int i = 9; i < 12; ++i) P[i*15 + i] = 0.5;  // accel bias
    for (int i = 12; i < 15; ++i) P[i*15 + i] = 0.1;  // gyro bias
    
    // Copy state
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
    
    // Initialize previous values
    double zeros3[3] = {0, 0, 0};
    copy_vec(s->prev_accel, zeros3, 3);
    copy_vec(s->prev_gyro, zeros3, 3);
    copy_vec(s->prev_mag, zeros3, 3);
    s->prev_gps_lat = 0;
    s->prev_gps_lon = 0;
    s->prev_gps_alt = 0;
    s->prev_baro = 0;
    s->buffer_tolerance = 1e-9;
    
    // Other parameters
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

