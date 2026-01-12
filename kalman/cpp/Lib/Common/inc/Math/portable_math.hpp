#pragma once
#ifndef LIB_COMMON_INC_MATH_PORTABLE_MATH_HPP
#define LIB_COMMON_INC_MATH_PORTABLE_MATH_HPP

#include <cmath>
#include <cstdlib>
#include <cfloat>
#include <cstring>

namespace common {
namespace math {

// Portable math constants and functions
constexpr float EPS = 1e-10f;
constexpr float EPSILON = 1e-10f;
constexpr float PI = 3.14159265358979323846f;

inline float portable_sqrt(float x) {
    return std::sqrt(x);
}

inline double portable_sqrt(double x) {
    return std::sqrt(x);
}

inline float portable_fabs(float x) {
    return std::fabs(x);
}

inline double portable_fabs(double x) {
    return std::fabs(x);
}

inline float portable_atan2(float y, float x) {
    return std::atan2(y, x);
}

inline double portable_atan2(double y, double x) {
    return std::atan2(y, x);
}

inline bool is_finite(float x) {
    return std::isfinite(x);
}

inline bool is_nan(float x) {
    return std::isnan(x);
}

inline float pressure_to_altitude(float pressure_pa) {
    // Barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
    // where P0 = 101325 Pa (standard sea-level pressure)
    const float P0 = 101325.0f;
    const float exponent = 1.0f / 5.255f;
    if (pressure_pa <= 0.0f) return 0.0f;
    float ratio = pressure_pa / P0;
    float altitude = 44330.0f * (1.0f - std::pow(ratio, exponent));
    return altitude;
}

inline float pressure_to_altitude_simple(float pressure_pa) {
    // Simplified barometric formula
    const float P0 = 101325.0f;
    const float exponent = 1.0f / 5.255f;
    if (pressure_pa <= 0.0f) return 0.0f;
    float ratio = pressure_pa / P0;
    float altitude = 44330.0f * (1.0f - std::pow(ratio, exponent));
    return altitude;
}

} // namespace math
} // namespace common

#endif // LIB_COMMON_INC_MATH_PORTABLE_MATH_HPP
