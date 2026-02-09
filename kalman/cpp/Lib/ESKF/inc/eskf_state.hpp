#pragma once
#ifndef LIB_ESKF_INC_ESKF_STATE_HPP
#define LIB_ESKF_INC_ESKF_STATE_HPP


namespace eskf {

struct FilterState {
    float p[3], v[3], q[4], ba[3], bg[3];
    float P[15*15];
    float Q_nominal[15*15];
    float g[3];
    float mag_ref[3];  // Magnetic reference vector in world frame (computed during static initialization)
    float q_init[4];   // Initial quaternion (roll/pitch/yaw estimated at static initialization). Used to compute relative yaw.
    float dt;
    double gps_origin[3];
    float prev_accel[3], prev_gyro[3], prev_mag[3];
    double prev_gps_lat, prev_gps_lon, prev_gps_alt;  // GPS座標はdoubleで保持
    float prev_baro;
    float buffer_tolerance;
    float w_body[3];
    float velocity_damping;
    float baro_weight;
    float zupt_threshold_accel, zupt_threshold_gyro;
    int zupt_min_duration;
    int zupt_counter;
    bool is_stationary;
    bool adaptive_q_enabled;
    bool enable_accel_z_integration;
    float accel_z_threshold, accel_z_damping;
    float gyro_noise_threshold;
    int last_reset_step;
    bool valid;
};

} // namespace eskf

#endif // ESKF_ESKF_STATE_HPP
