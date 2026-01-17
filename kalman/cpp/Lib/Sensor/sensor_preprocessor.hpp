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

// Inline implementations (header-only)
inline Result accel(
    const cmath_fx::Vector<3, float>& a_meas,
    const cmath_fx::Vector<3, float>& prev_a,
    double buffer_tolerance = 1e-9
) {
    Result result;
    result.is_outlier = false;
    result.no_change = false;
    double delta = 0.0;
    for (int i = 0; i < 3; ++i) {
        double d = static_cast<double>(a_meas(i, 0)) - static_cast<double>(prev_a(i, 0));
        delta += d * d;
    }
    delta = std::sqrt(delta);
    for (int i = 0; i < 3; ++i) {
        result.output(i, 0) = a_meas(i, 0);
    }
    if (delta <= buffer_tolerance) {
        result.no_change = true;
        return result;
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

inline Result mag(
    const cmath_fx::Vector<3, float>& m_meas,
    const cmath_fx::Vector<3, float>& prev_m,
    double buffer_tolerance = 1e-9
) {
    Result result;
    result.is_outlier = false;
    result.no_change = false;
    double delta = 0.0;
    for (int i = 0; i < 3; ++i) {
        double d = static_cast<double>(m_meas(i, 0)) - static_cast<double>(prev_m(i, 0));
        delta += d * d;
    }
    delta = std::sqrt(delta);
    for (int i = 0; i < 3; ++i) {
        result.output(i, 0) = m_meas(i, 0);
    }
    if (delta <= buffer_tolerance) {
        result.no_change = true;
        return result;
    }
    return result;
}

inline double baro(double pressure) {
    return coord::pressure_to_altitude_d(pressure);
}

inline Result gps(
    double lat, double lon, double alt,
    const cmath_fx::Vector<3, float>& origin,
    double buffer_tolerance = 1e-9
) {
    Result result;
    result.is_outlier = false;
    result.no_change = false;
    double lat0 = static_cast<double>(origin(0, 0));
    double lon0 = static_cast<double>(origin(1, 0));
    double alt0 = static_cast<double>(origin(2, 0));
    double dlat = lat - lat0;
    double dlon = lon - lon0;
    double dalt = alt - alt0;
    if (std::fabs(dlat) <= buffer_tolerance &&
        std::fabs(dlon) <= buffer_tolerance &&
        std::fabs(dalt) <= buffer_tolerance) {
        result.output(0, 0) = 0.0f;
        result.output(1, 0) = 0.0f;
        result.output(2, 0) = 0.0f;
        result.no_change = true;
        return result;
    }
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

