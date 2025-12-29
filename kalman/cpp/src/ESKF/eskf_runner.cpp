#include "../Inc/ESKF/eskf_runner.hpp"
#include "../Inc/Common/filter_management.hpp"
#include "../Inc/Common/Math/statistics.hpp"
#include "../Inc/Common/Math/quaternion_lib.hpp"
#include <cstring>
#include <vector>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace eskf {

// Static instance of SensorFilterLib
SensorFilterLib ESKFRunner::filter_lib_;

// Predict step
void ESKFRunner::predict(ESKFState* s, const double* a_meas, const double* w_meas) {
    using PredictParams = eskf::PredictPostprocessParams;
    
    // Convert double to float for ESKFCore (uses float internally)
    Vector<3, float> p_f, v_f, ba_f, bg_f, a_meas_f, w_meas_f, g_f;
    Vector<4, float> q_f;
    Matrix<15, 15, float> P_f, Qnom_f, Qadapt_f, Pnew_f;
    Vector<3, float> gyro_thr_f, accel_thr_f;
    
    // Convert inputs
    for (int i = 0; i < 3; ++i) {
        p_f(i, 0) = static_cast<float>(s->p[i]);
        v_f(i, 0) = static_cast<float>(s->v[i]);
        ba_f(i, 0) = static_cast<float>(s->ba[i]);
        bg_f(i, 0) = static_cast<float>(s->bg[i]);
        a_meas_f(i, 0) = static_cast<float>(a_meas[i]);
        w_meas_f(i, 0) = static_cast<float>(w_meas[i]);
        g_f(i, 0) = static_cast<float>(s->g[i]);
        gyro_thr_f(i, 0) = 0.0f;  // Default thresholds
        accel_thr_f(i, 0) = 0.0f;
    }
    for (int i = 0; i < 4; ++i) {
        q_f(i, 0) = static_cast<float>(s->q[i]);
    }
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_f(i, j) = static_cast<float>(s->P[i + j*15]);
            Qnom_f(i, j) = static_cast<float>(s->Q_nominal[i + j*15]);
        }
    }
    
    float dt_f = static_cast<float>(s->dt);
    
    // Adaptive Q scaling
    Qadapt_f = Qnom_f;
    if (s->adaptive_q_enabled) {
        ESKFCore::compute_adaptive_Q(Qnom_f, a_meas_f, w_meas_f, Qadapt_f);
    }
    
    // Integrate nominal state
    ESKFCore::integrate_nominal(p_f, v_f, q_f, ba_f, bg_f, a_meas_f, w_meas_f, dt_f, g_f, gyro_thr_f, accel_thr_f);
    
    // Predict covariance
    ESKFCore::predict_covariance(P_f, q_f, a_meas_f, ba_f, w_meas_f, bg_f, Qadapt_f, dt_f, Pnew_f);
    
    // Ensure symmetry
    for (int i = 0; i < 15; ++i) {
        for (int j = i + 1; j < 15; ++j) {
            float v = 0.5f * (Pnew_f(i, j) + Pnew_f(j, i));
            Pnew_f(i, j) = v;
            Pnew_f(j, i) = v;
        }
    }
    
    // Copy Pnew_f to P_f for postprocessing
    P_f = Pnew_f;
    
    copy_vec(s->w_body, w_meas, 3);
    
    // Predict postprocess
    Vector<3, float> a_for_vel_f;
    for (int i = 0; i < 3; ++i) {
        a_for_vel_f(i, 0) = static_cast<float>(a_meas[i]);
    }
    bool enable_accel_z = s->enable_accel_z_integration;
    float accel_z_threshold = static_cast<float>(s->accel_z_threshold);
    float accel_z_damping = static_cast<float>(s->accel_z_damping);
    float velocity_damping = static_cast<float>(s->velocity_damping);
    
    // 1. accel_z_integration - Direct C++ implementation using quaternion_lib.hpp
    if (enable_accel_z) {
        // Convert Vector<4, float> to Quaternion<float>
        QuatF quat(q_f(0, 0), q_f(1, 0), q_f(2, 0), q_f(3, 0));
        quat.normalize();
        
        // Get rotation matrix (row-major order from quaternion_lib)
        float R_row[9];
        quat.to_rotation_matrix(R_row);
        
        // Convert row-major to column-major with transpose
        Matrix<3, 3, float> R;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                // Transpose: row-major R_row to column-major R
                R(i, j) = R_row[j * 3 + i];
            }
        }
        
        // Calculate R * a_for_vel (matrix-vector multiplication)
        Vector<3, float> Ra;
        for (int i = 0; i < 3; ++i) {
            Ra(i, 0) = 0.0f;
            for (int j = 0; j < 3; ++j) {
                Ra(i, 0) += R(i, j) * a_for_vel_f(j, 0);
            }
        }
        
        // Calculate (R * a_for_vel) - [0; 0; g(3)]
        Vector<3, float> a_ned;
        a_ned(0, 0) = Ra(0, 0);
        a_ned(1, 0) = Ra(1, 0);
        a_ned(2, 0) = Ra(2, 0) - g_f(2, 0);
        
        float az_excess = a_ned(2, 0);
        if (std::abs(az_excess) > accel_z_threshold) {
            v_f(2, 0) = v_f(2, 0) * (1.0f - accel_z_damping) + az_excess * dt_f;
        }
    }
    
    // 2-6. 後処理（velocity_damping, P normalization, velocity clipping）
    PredictParams params;
    params.enable_accel_z_integration = false;  // 既に上で処理済み
    params.accel_z_threshold = accel_z_threshold;
    params.accel_z_damping = accel_z_damping;
    params.velocity_damping = velocity_damping;
    eskf::predict_postprocess(v_f, q_f, P_f, a_for_vel_f, dt_f, g_f, params);
    
    // 3. divergence_guard.regularize_covariance
    // Convert Matrix<15,15,float> to FixedMatrix (cm)
    cm P_fixed(15, 15);
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_fixed(i, j) = P_f(i, j);
        }
    }
    filter_lib_.divergence_guard.regularize_covariance(P_fixed);
    // Convert back to Matrix<15,15,float>
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_f(i, j) = P_fixed(i, j);
        }
    }
    
    // 5. divergence_guard.check_and_clip_velocity
    // vel_indices = [4, 5, 6] (1-based in MATLAB, but we use 0-based internally: 3, 4, 5)
    std::vector<int> vel_indices = {3, 4, 5};
    bool was_clipped = false;
    
    // Define max variances consistent with MATLAB limits
    std::vector<float> max_var(15);
    for(int i=0; i<15; ++i) max_var[i] = 1e6f; // default large
    // position 0:2 -> 100^2
    for(int i=0; i<3; ++i) max_var[i] = 100.0f*100.0f;
    // velocity 3:5 -> 20^2
    for(int i=3; i<6; ++i) max_var[i] = 20.0f*20.0f;
    // attitude 6:8 -> (deg2rad(45))^2
    float d45 = 45.0f * 3.14159265f / 180.0f;
    for(int i=6; i<9; ++i) max_var[i] = d45*d45;
    // accel bias 9:11
    for(int i=9; i<12; ++i) max_var[i] = 0.1f;
    // gyro bias 12:14
    for(int i=12; i<15; ++i) max_var[i] = 0.01f;
    
    // Clip covariance per-velocity-index
    for(size_t kk=0; kk<vel_indices.size(); ++kk) {
        int idx = vel_indices[kk];
        if (idx < 0 || idx >= 15) continue;
        float Pii = P_f(idx, idx);
        if (Pii > max_var[idx]) {
            float factor = sqrtf(max_var[idx] / Pii);
            for(int j=0; j<15; ++j) P_f(idx, j) *= factor;
            for(int i=0; i<15; ++i) P_f(i, idx) *= factor;
            P_f(idx, idx) = max_var[idx];
            was_clipped = true;
        }
    }
    
    // Clip velocity magnitude to max_vel (3.0 m/s)
    float max_vel = 3.0f;
    float vnorm = 0.0f;
    for(int i=0; i<3; ++i) vnorm += v_f(i, 0) * v_f(i, 0);
    vnorm = sqrtf(vnorm);
    if (vnorm > max_vel) {
        float scale = max_vel / vnorm;
        for(int i=0; i<3; ++i) v_f(i, 0) *= scale;
        was_clipped = true;
    }
    
    // Ensure symmetry of P
    for(int i=0; i<15; ++i) {
        for(int j=i+1; j<15; ++j) {
            float v = 0.5f * (P_f(i, j) + P_f(j, i));
            P_f(i, j) = v;
            P_f(j, i) = v;
        }
    }
    
    // Convert back to double (all state variables)
    for (int i = 0; i < 3; ++i) {
        s->p[i] = static_cast<double>(p_f(i, 0));
        s->v[i] = static_cast<double>(v_f(i, 0));
        s->ba[i] = static_cast<double>(ba_f(i, 0));
        s->bg[i] = static_cast<double>(bg_f(i, 0));
    }
    for (int i = 0; i < 4; ++i) {
        s->q[i] = static_cast<double>(q_f(i, 0));
    }
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            s->P[i + j*15] = static_cast<double>(P_f(i, j));
        }
    }
}

// Check and reset if divergence detected
void ESKFRunner::check_and_reset(ESKFState* s, int k) {
    // Check for divergence
    // Convert P matrix to Matrix type
    Matrix<15, 15, float> P_float;
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_float(i, j) = static_cast<float>(s->P[i + j*15]);
        }
    }
    
    Vector<3, float> p_float, v_float, ba_float, bg_float;
    Vector<4, float> q_float;
    for (int i = 0; i < 3; ++i) {
        p_float(i, 0) = static_cast<float>(s->p[i]);
        v_float(i, 0) = static_cast<float>(s->v[i]);
        ba_float(i, 0) = static_cast<float>(s->ba[i]);
        bg_float(i, 0) = static_cast<float>(s->bg[i]);
    }
    for (int i = 0; i < 4; ++i) {
        q_float(i, 0) = static_cast<float>(s->q[i]);
    }
    
    bool need_reset = check_state_divergence(p_float, v_float, q_float, ba_float, bg_float, P_float);
    
    if (need_reset) {
        s->last_reset_step = k;
        
        // Reset state using filter_management directly
        using namespace common::filter;
        Vector<3, float> v_float, ba_float, bg_float;
        Vector<4, float> q_float;
        Matrix<15, 15, float> P_float;
        
        // Reset P matrix using setIdentityScaled (reset_scale = 0.01)
        float reset_scale = 0.01f;
        setIdentityScaled(P_float, reset_scale);
        
        // Convert current state to float type
        for (int i = 0; i < 3; ++i) {
            v_float(i, 0) = static_cast<float>(s->v[i]);
            ba_float(i, 0) = static_cast<float>(s->ba[i]);
            bg_float(i, 0) = static_cast<float>(s->bg[i]);
        }
        for (int i = 0; i < 4; ++i) {
            q_float(i, 0) = static_cast<float>(s->q[i]);
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P_float(i, j) = static_cast<float>(s->P[i + j*15]);
            }
        }
        
        // Reset processing
        reset_state_on_divergence(v_float, ba_float, bg_float, q_float, P_float);
        
        // Convert results back to double type
        for (int i = 0; i < 3; ++i) {
            s->v[i] = static_cast<double>(v_float(i, 0));
            s->ba[i] = static_cast<double>(ba_float(i, 0));
            s->bg[i] = static_cast<double>(bg_float(i, 0));
        }
        for (int i = 0; i < 4; ++i) {
            s->q[i] = static_cast<double>(q_float(i, 0));
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                s->P[i + j*15] = static_cast<double>(P_float(i, j));
            }
        }
    }
}

// ZUPT check and update
void ESKFRunner::zupt_check_and_update(ESKFState* s, const double* a_meas, const double* w_meas) {
    // ZUPT check
    Vector<3, float> a_float, w_float;
    for (int i = 0; i < 3; ++i) {
        a_float(i, 0) = static_cast<float>(a_meas[i]);
        w_float(i, 0) = static_cast<float>(w_meas[i]);
    }
    
    bool stationary = check_zupt_condition(a_float, w_float, 
                                           static_cast<float>(s->zupt_threshold_accel),
                                           static_cast<float>(s->zupt_threshold_gyro));
    
    if (stationary) {
        s->zupt_counter++;
    } else {
        s->zupt_counter = 0;
    }
    
    s->is_stationary = (s->zupt_counter >= s->zupt_min_duration);
    
    if (s->is_stationary) {
        // ZUPT update using ESKFCore directly
        Vector<3, float> v_in, v_out;
        Matrix<15, 15, float> P_in, P_out;
        
        // Convert double to float
        for (int i = 0; i < 3; ++i) {
            v_in(i, 0) = static_cast<float>(s->v[i]);
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P_in(i, j) = static_cast<float>(s->P[i + j*15]);
            }
        }
        
        // Call ESKFCore::update_zupt
        ESKFCore::update_zupt(v_in, P_in, v_out, P_out);
        
        // Convert back to double
        for (int i = 0; i < 3; ++i) {
            s->v[i] = static_cast<double>(v_out(i, 0));
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                s->P[i + j*15] = static_cast<double>(P_out(i, j));
            }
        }
    }
}

// Initialize ESKF state from static sensor data
void ESKFRunner::initialize(ESKFState* s, const InitParams& params) {
    using namespace common::math;
    using Quat = quat_lib::Quaternion<double>;
    
    // Constants
    const double GRAVITY = 9.80665;
    const double DEG2RAD = 0.017453292519943295;
    
    // Clear state
    memset(s, 0, sizeof(ESKFState));
    s->valid = true;
    s->dt = params.dt;
    
    // Basic state initialization
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
    if (params.has_accel && params.N_static > 10) {
        // Accelerometer mean and std dev
        double accel_mean_x, accel_mean_y, accel_mean_z;
        compute_mean_3d(params.ax, params.ay, params.az, params.N_static, 
                       &accel_mean_x, &accel_mean_y, &accel_mean_z);
        sigma_a = compute_std_3d(params.ax, params.ay, params.az, params.N_static, 
                                accel_mean_x, accel_mean_y, accel_mean_z);
        if (sigma_a < 0.01) sigma_a = 0.01;
        
        // Initial attitude calculation (Roll/Pitch)
        double phi = atan2(-accel_mean_y, -accel_mean_z);
        double theta = atan2(accel_mean_x, sqrt(accel_mean_y*accel_mean_y + accel_mean_z*accel_mean_z));
        
        if (params.has_gyro) {
            double gyro_mean_x, gyro_mean_y, gyro_mean_z;
            compute_mean_3d(params.wx, params.wy, params.wz, params.N_static, 
                           &gyro_mean_x, &gyro_mean_y, &gyro_mean_z);
            double sigma_g_deg = compute_std_3d(params.wx, params.wy, params.wz, params.N_static, 
                                                gyro_mean_x, gyro_mean_y, gyro_mean_z);
            sigma_g = DEG2RAD * sigma_g_deg;
            if (sigma_g < 0.001) sigma_g = 0.001;
            
            // gyro_noise_threshold calculation
            double std_wx = compute_std(params.wx, params.N_static, gyro_mean_x);
            double std_wy = compute_std(params.wy, params.N_static, gyro_mean_y);
            double std_wz = compute_std(params.wz, params.N_static, gyro_mean_z);
            double max_std = std_wx;
            if (std_wy > max_std) max_std = std_wy;
            if (std_wz > max_std) max_std = std_wz;
            gyro_noise_threshold = 2.0 * DEG2RAD * max_std;
        }
        
        // Magnetometer data for Yaw calculation
        double psi = 0.0;
        if (params.has_mag) {
            double mag_mean_x, mag_mean_y, mag_mean_z;
            compute_mean_3d(params.mx, params.my, params.mz, params.N_static, 
                           &mag_mean_x, &mag_mean_y, &mag_mean_z);
            sigma_mag = compute_std_3d(params.mx, params.my, params.mz, params.N_static, 
                                      mag_mean_x, mag_mean_y, mag_mean_z);
            if (sigma_mag < 0.1) sigma_mag = 0.1;
            
            // Roll/Pitch only quaternion
            Quat quat_rp = Quat::from_euler(phi * 180.0 / M_PI, theta * 180.0 / M_PI, 0.0);
            double q_rp[4] = {quat_rp.w, quat_rp.x, quat_rp.y, quat_rp.z};
            
            // Rotation matrix
            double R_row[9];
            quat_rp.to_rotation_matrix(R_row);
            double R_rp[9];
            R_rp[0] = R_row[0]; R_rp[3] = R_row[1]; R_rp[6] = R_row[2];
            R_rp[1] = R_row[3]; R_rp[4] = R_row[4]; R_rp[7] = R_row[5];
            R_rp[2] = R_row[6]; R_rp[5] = R_row[7]; R_rp[8] = R_row[8];
            
            // Project magnetometer to horizontal plane
            double m_level_x = R_rp[0]*mag_mean_x + R_rp[3]*mag_mean_y + R_rp[6]*mag_mean_z;
            double m_level_y = R_rp[1]*mag_mean_x + R_rp[4]*mag_mean_y + R_rp[7]*mag_mean_z;
            
            psi = -atan2(m_level_y, m_level_x);
        }
        
        // Final quaternion
        Quat quat_final = Quat::from_euler(phi * 180.0 / M_PI, theta * 180.0 / M_PI, psi * 180.0 / M_PI);
        q[0] = quat_final.w;
        q[1] = quat_final.x;
        q[2] = quat_final.y;
        q[3] = quat_final.z;
    }
    
    // Barometer data
    if (params.has_baro && params.pressure) {
        std::vector<double> alt_baro(params.N_static);
        double alt_mean = 0.0;
        for (int i = 0; i < params.N_static; ++i) {
            alt_baro[i] = 44330.0 * (1.0 - pow(params.pressure[i] / 101325.0, 0.1903));
            alt_mean += alt_baro[i];
        }
        alt_mean /= params.N_static;
        
        double sum_sq = 0.0;
        for (int i = 0; i < params.N_static; ++i) {
            double diff = alt_baro[i] - alt_mean;
            sum_sq += diff * diff;
        }
        sigma_press = sqrt(sum_sq / (params.N_static - 1));
        if (sigma_press < 0.1) sigma_press = 0.1;
    }
    
    // GPS data
    if (params.has_gps && params.lat && params.lon && params.alt) {
        // GPS origin calculation (excluding NaN)
        double lat_sum = 0.0, lon_sum = 0.0, alt_sum = 0.0;
        int valid_count = 0;
        for (int i = 0; i < params.N_static; ++i) {
            if (!std::isnan(params.lat[i])) {
                lat_sum += params.lat[i];
                lon_sum += params.lon[i];
                alt_sum += params.alt[i];
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
            for (int i = 0; i < params.N_static; ++i) {
                if (!std::isnan(params.lat[i])) {
                    y_m[idx] = (params.lat[i] - gps_origin[0]) / 9.0e-6;
                    x_m[idx] = (params.lon[i] - gps_origin[1]) / (9.0e-6 / cos_lat0);
                    z_m[idx] = params.alt[i] - gps_origin[2];
                    idx++;
                }
            }
            
            double mean_x = compute_mean(x_m.data(), valid_count);
            double mean_y = compute_mean(y_m.data(), valid_count);
            double mean_z = compute_mean(z_m.data(), valid_count);
            
            double std_x = compute_std(x_m.data(), valid_count, mean_x);
            double std_y = compute_std(y_m.data(), valid_count, mean_y);
            double std_z = compute_std(z_m.data(), valid_count, mean_z);
            sigma_gps = (std_x + std_y + std_z) / 3.0;
            if (sigma_gps < 0.1) sigma_gps = 0.1;
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
    
    // Copy to ESKFState
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
    
    // prev_* initialization
    double zeros3[3] = {0, 0, 0};
    copy_vec(s->prev_accel, zeros3, 3);
    copy_vec(s->prev_gyro, zeros3, 3);
    copy_vec(s->prev_mag, zeros3, 3);
    s->prev_gps_lat = 0;
    s->prev_gps_lon = 0;
    s->prev_gps_alt = 0;
    s->prev_baro = 0;
    s->buffer_tolerance = 1e-9;
    
    // ZUPT related
    s->zupt_threshold_accel = 1.0;
    s->zupt_threshold_gyro = DEG2RAD * 3.0;
    s->zupt_min_duration = 10;
    s->zupt_counter = 0;
    s->is_stationary = false;
    
    // Other parameters
    s->adaptive_q_enabled = true;
    s->velocity_damping = 0.0;
    s->enable_accel_z_integration = true;
    s->accel_z_threshold = 0.5;
    s->accel_z_damping = 0.1;
    s->baro_weight = 0.2;
    copy_vec(s->w_body, zeros3, 3);
    s->last_reset_step = 0;
}

// Sensor update and GPS update will be implemented later with callback support
// For now, these remain in mex_run_eskf.cpp due to mexCallMATLAB dependency

} // namespace eskf

