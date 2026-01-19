#pragma once
#ifndef LIB_SENSOR_SENSOR_PREPROCESSOR_HPP
#define LIB_SENSOR_SENSOR_PREPROCESSOR_HPP

#include "../Matrix/fixed_matrix.hpp"
#include "coordinate_transform.hpp"
#include <cmath>
#include <cstdlib>

namespace sensor {
namespace preprocess {

struct Result {
    cmath_fx::Vector<3, float> output;
    bool is_outlier;
    bool no_change;
};

// 完全一致ベースの変化検出（センサーデータ生成側で同じ値を複製するため）
inline bool exact_match_vec3(const float* a, const float* b) {
    return (a[0] == b[0]) && (a[1] == b[1]) && (a[2] == b[2]);
}

inline bool exact_match_scalar(float a, float b) {
    return (a == b);
}

inline bool exact_match_gps(double a, double b) {
    return (a == b);
}

// Inline implementations (header-only)
inline Result accel(
    const cmath_fx::Vector<3, float>& a_meas,
    const cmath_fx::Vector<3, float>& prev_a,
    double buffer_tolerance = 1e-9
) {
    Result result;
    result.is_outlier = false;
    result.no_change = false;
    
    // 完全一致チェック（センサー更新頻度による同一値検出）
    float a_arr[3] = {a_meas(0,0), a_meas(1,0), a_meas(2,0)};
    float prev_arr[3] = {prev_a(0,0), prev_a(1,0), prev_a(2,0)};
    if (exact_match_vec3(a_arr, prev_arr)) {
        result.no_change = true;
        for (int i = 0; i < 3; ++i) {
            result.output(i, 0) = a_meas(i, 0);
        }
        return result;
    }
    
    // 変化あり → 出力設定 & 外れ値チェック
    for (int i = 0; i < 3; ++i) {
        result.output(i, 0) = a_meas(i, 0);
    }
    
    double a_norm = std::sqrt(
        static_cast<double>(a_meas(0, 0)) * a_meas(0, 0) +
        static_cast<double>(a_meas(1, 0)) * a_meas(1, 0) +
        static_cast<double>(a_meas(2, 0)) * a_meas(2, 0)
    );
    if (a_norm < 0.1 || std::fabs(a_norm - 9.81) > 3.0) {
        result.is_outlier = true;
    }
    return result;
}

inline Result gyro(
    const cmath_fx::Vector<3, float>& g_meas,
    const cmath_fx::Vector<3, float>& prev_g,
    double buffer_tolerance = 1e-9
) {
    Result result;
    result.is_outlier = false;
    result.no_change = false;
    
    // 完全一致チェック
    float g_arr[3] = {g_meas(0,0), g_meas(1,0), g_meas(2,0)};
    float prev_arr[3] = {prev_g(0,0), prev_g(1,0), prev_g(2,0)};
    if (exact_match_vec3(g_arr, prev_arr)) {
        result.no_change = true;
        for (int i = 0; i < 3; ++i) {
            result.output(i, 0) = g_meas(i, 0);
        }
        return result;
    }
    
    // 変化あり
    for (int i = 0; i < 3; ++i) {
        result.output(i, 0) = g_meas(i, 0);
    }
    return result;
}

inline Result mag(
    const cmath_fx::Vector<3, float>& m_meas,
    const cmath_fx::Vector<3, float>& prev_m,
    double buffer_tolerance = 1e-9
) {
    Result result;
    result.is_outlier = false;
    result.no_change = false;
    
    // 完全一致チェック
    float m_arr[3] = {m_meas(0,0), m_meas(1,0), m_meas(2,0)};
    float prev_arr[3] = {prev_m(0,0), prev_m(1,0), prev_m(2,0)};
    if (exact_match_vec3(m_arr, prev_arr)) {
        result.no_change = true;
        for (int i = 0; i < 3; ++i) {
            result.output(i, 0) = m_meas(i, 0);
        }
        return result;
    }
    
    // 変化あり
    for (int i = 0; i < 3; ++i) {
        result.output(i, 0) = m_meas(i, 0);
    }
    return result;
}

inline double baro(double pressure, double prev_pressure, bool* no_change) {
    // 完全一致チェック
    if (exact_match_scalar(static_cast<float>(pressure), static_cast<float>(prev_pressure))) {
        if (no_change) *no_change = true;
        return coord::pressure_to_altitude_d(prev_pressure);  // 前回の値を返す
    }
    if (no_change) *no_change = false;
    return coord::pressure_to_altitude_d(pressure);
}

inline Result gps(
    double lat, double lon, double alt,
    const cmath_fx::Vector<3, float>& origin,
    double prev_lat, double prev_lon, double prev_alt,
    double buffer_tolerance = 1e-9
) {
    Result result;
    result.is_outlier = false;
    result.no_change = false;
    
    // 完全一致チェック（GPS座標のみ）
    if (exact_match_gps(lat, prev_lat) && 
        exact_match_gps(lon, prev_lon) && 
        exact_match_gps(alt, prev_alt)) {
        result.no_change = true;
        // 前回のENU座標を維持（再計算不要）
        result.output(0, 0) = 0.0f;  // ダミー値（呼び出し側で無視される）
        result.output(1, 0) = 0.0f;
        result.output(2, 0) = 0.0f;
        return result;
    }
    
    // 変化あり → GPS→ENU変換
    double lat0 = static_cast<double>(origin(0, 0));
    double lon0 = static_cast<double>(origin(1, 0));
    double alt0 = static_cast<double>(origin(2, 0));
    double dlat = lat - lat0;
    double dlon = lon - lon0;
    double dalt = alt - alt0;
    // GPS座標をメートル単位に変換（元の実装と同じ簡易変換）
    // 注意: 元の実装はEN座標順が [North, East, Up] だった
    double y_m = dlat / 9.0e-6;
    double lat0rad = lat0 * 3.14159265358979323846 / 180.0;
    double x_m = dlon / (9.0e-6 / std::cos(lat0rad));
    double z_m = -dalt;  // 元の実装では-daltだった
    
    result.output(0, 0) = static_cast<float>(y_m);   // North
    result.output(1, 0) = static_cast<float>(x_m);   // East  
    result.output(2, 0) = static_cast<float>(z_m);   // Up (負の高度)
    return result;
}

} // namespace preprocess
} // namespace sensor

#endif // LIB_SENSOR_SENSOR_PREPROCESSOR_HPP

