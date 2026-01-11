#pragma once
#include <cmath>
#include <cfloat>

namespace common {
namespace math {

inline float pressure_to_altitude(float pressure) {
    const float p0 = 101325.0f;
    const float T0 = 288.15f;
    const float L = 0.0065f;
    const float g = 9.80665f;
    const float M = 0.0289644f;
    const float R = 8.31447f;

    if (pressure <= 0.0f) return 0.0f;

    const float alpha = (R * L) / (g * M);
    const float p_ratio = pressure / p0;

    float log_p_ratio = std::log(p_ratio);
    float power_term = std::exp(alpha * log_p_ratio);

    float altitude = (T0 / L) * (1.0f - power_term);
    return altitude;
}

inline float pressure_to_altitude_simple(float pressure) {
    const float p0 = 101325.0f;
    const float p_ratio = pressure / p0;
    if (p_ratio <= 0.0f) return 0.0f;
    float log_p = std::log(p_ratio);
    float exp_term = std::exp(0.1903f * log_p);
    return 44330.0f * (1.0f - exp_term);
}

template<typename T>
inline T portable_sqrt(T x) {
    if (x <= static_cast<T>(0)) return static_cast<T>(0);
    return std::sqrt(x);
}

template<typename T>
inline T portable_atan2(T y, T x) {
    return std::atan2(y, x);
}

template<typename T>
inline T portable_pow(T base, T exp) {
    return std::pow(base, exp);
}

} // namespace math
} // namespace common
