#pragma once

#ifndef ESKF_STATE_HPP
#define ESKF_STATE_HPP

// ESKF State Structure
// Contains all state variables and parameters for the ESKF filter

struct ESKFState {
    double p[3], v[3], q[4], ba[3], bg[3];
    double P[15*15];
    double Q_nominal[15*15];
    double g[3];
    double dt;
    double gps_origin[3];
    double prev_accel[3], prev_gyro[3], prev_mag[3];
    double prev_gps_lat, prev_gps_lon, prev_gps_alt;
    double prev_baro;
    double buffer_tolerance;
    double w_body[3];
    double velocity_damping;
    double baro_weight;
    double zupt_threshold_accel, zupt_threshold_gyro;
    int zupt_min_duration;
    int zupt_counter;
    bool is_stationary;
    bool adaptive_q_enabled;
    bool enable_accel_z_integration;
    double accel_z_threshold, accel_z_damping;
    double gyro_noise_threshold;
    int last_reset_step;
    bool valid;
};

#endif // ESKF_STATE_HPP


