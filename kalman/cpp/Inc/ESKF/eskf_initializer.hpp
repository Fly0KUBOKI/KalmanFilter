#pragma once

#ifndef ESKF_ESKF_INITIALIZER_HPP
#define ESKF_ESKF_INITIALIZER_HPP

#include "Common/Math/fixed_matrix.hpp"
#include "Common/Math/quaternion_lib.hpp"
#include <vector>

namespace eskf {

// ESKF初期化パラメータ
struct InitializationParams {
    // ノイズパラメータ
    double sigma_a = 0.1;
    double sigma_g = 0.0017453292519943295;  // DEG2RAD * 0.1
    double sigma_mag = 10.0;
    double sigma_press = 1.0;
    double sigma_gps = 1.0;
    double gyro_noise_threshold = 0.0017453292519943295;  // DEG2RAD * 0.1
    
    // GPS原点
    double gps_origin[3] = {0, 0, 0};
    
    // 初期状態
    double p[3] = {0, 0, 0};
    double v[3] = {0, 0, 0};
    double q[4] = {1, 0, 0, 0};
    double ba[3] = {0, 0, 0};
    double bg[3] = {0, 0, 0};
    double g[3] = {0, 0, 9.80665};
    
    // Q行列とP行列
    cmath_fx::Matrix<15, 15, float> Q;
    cmath_fx::Matrix<15, 15, float> P;
};

// ESKF初期化結果
struct InitializationResult {
    InitializationParams params;
    
    // 前回値
    double prev_accel[3] = {0, 0, 0};
    double prev_gyro[3] = {0, 0, 0};
    double prev_mag[3] = {0, 0, 0};
    double prev_gps_lat = 0;
    double prev_gps_lon = 0;
    double prev_gps_alt = 0;
    double prev_baro = 0;
};

// ESKF初期化（静止データからノイズ推定と初期姿勢計算）
// obs_data: 観測データ（構造体として渡す）
// static_time: 静止時間（秒）
// dt: サンプリング時間（秒）
// 戻り値: 初期化結果
InitializationResult initialize_eskf(
    const std::vector<double>& ax_data,
    const std::vector<double>& ay_data,
    const std::vector<double>& az_data,
    const std::vector<double>& wx_data,
    const std::vector<double>& wy_data,
    const std::vector<double>& wz_data,
    const std::vector<double>& mx_data,
    const std::vector<double>& my_data,
    const std::vector<double>& mz_data,
    const std::vector<double>& pressure_data,
    const std::vector<double>& lat_data,
    const std::vector<double>& lon_data,
    const std::vector<double>& alt_data,
    double static_time,
    double dt
);

} // namespace eskf

#endif // ESKF_ESKF_INITIALIZER_HPP


