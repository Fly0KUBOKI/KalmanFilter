#pragma once
#ifndef LIB_MATRIX_MATH_STATISTICS_HPP
#define LIB_MATRIX_MATH_STATISTICS_HPP


#include <cmath>
#include <cstddef>

namespace common {
namespace math {

// Mathematical constants
// Provide `PI` here so other modules depending on the old `portable_math.hpp`
// can continue to reference `common::math::PI` without needing the legacy header.
constexpr double PI = 3.14159265358979323846;

template <typename T>
inline T pi() { return static_cast<T>(PI); }

template<typename T>
inline void compute_mean_3d(const T* ax, const T* ay, const T* az, std::size_t n, T* mean_x, T* mean_y, T* mean_z) {
    if (n == 0) { *mean_x = *mean_y = *mean_z = T(0); return; }
    T mx = T(0), my = T(0), mz = T(0);
    for (std::size_t i = 0; i < n; ++i) { mx += ax[i]; my += ay[i]; mz += az[i]; }
    T inv_n = T(1) / static_cast<T>(n);
    *mean_x = mx * inv_n; *mean_y = my * inv_n; *mean_z = mz * inv_n;
}

template<typename T>
inline T compute_std(const T* data, std::size_t n, T mean) {
    if (n < 2) return T(0);
    T sum_sq = T(0);
    for (std::size_t i = 0; i < n; ++i) {
        T diff = data[i] - mean;
        sum_sq += diff * diff;
    }
    return std::sqrt(sum_sq / static_cast<T>(n - 1));
}

template<typename T>
inline T compute_std_3d(const T* ax, const T* ay, const T* az, std::size_t n, T mean_x, T mean_y, T mean_z) {
    T std_x = compute_std(ax, n, mean_x);
    T std_y = compute_std(ay, n, mean_y);
    T std_z = compute_std(az, n, mean_z);
    return (std_x + std_y + std_z) / T(3);
}

} // namespace math
} // namespace common

#endif // COMMON_MATH_STATISTICS_HPP

