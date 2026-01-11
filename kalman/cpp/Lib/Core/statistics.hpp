#pragma once

#include <algorithm>
#include "../Matrix/fixed_matrix.hpp"
#include "portable_math.hpp"

namespace common {
namespace math {

using cm = cmath_fx::FixedMatrix;

// Median (FixedMatrix column vector)
inline float median(const cm& data) {
    if (data.rows == 0) return 0.0f;
    const int MAX_SIZE = 20;
    int n = data.rows;
    if (n > MAX_SIZE) n = MAX_SIZE;
    float sorted[MAX_SIZE];
    for (int i = 0; i < n; ++i) sorted[i] = data(i,0);
    std::sort(sorted, sorted + n);
    if (n % 2 == 0) return 0.5f * (sorted[n/2-1] + sorted[n/2]);
    return sorted[n/2];
}

inline float mad(const cm& data) {
    if (data.rows == 0) return 0.0f;
    float med = median(data);
    cm abs_dev; abs_dev.resize(data.rows,1);
    for (int i=0;i<data.rows;++i) abs_dev(i,0) = fabsf(data(i,0) - med);
    return median(abs_dev);
}

inline void robust_statistics(const cm& data, float& mean_val, float& std_val,
                              float outlier_threshold = 3.0f) {
    if (data.rows == 0) { mean_val = 0.0f; std_val = 0.0f; return; }
    float mu = median(data);
    float sigma = 1.4826f * mad(data);
    int n_inliers = 0; float sum = 0.0f; float sum_sq = 0.0f;
    for (int i=0;i<data.rows;++i) {
        float z = fabsf(data(i,0) - mu) / (sigma + 1e-9f);
        if (z < outlier_threshold) { sum += data(i,0); sum_sq += data(i,0)*data(i,0); n_inliers++; }
    }
    if (n_inliers > 0) { mean_val = sum / n_inliers; std_val = common::math::portable_sqrt(sum_sq / n_inliers - mean_val * mean_val); }
    else { mean_val = mu; std_val = sigma; }
}

// Templated helpers (C-style arrays) used by ESKF initializer
template<typename T>
inline T compute_mean(const T* data, std::size_t n) {
    if (n == 0) return T(0);
    T sum = T(0);
    for (std::size_t i = 0; i < n; ++i) sum += data[i];
    return sum / static_cast<T>(n);
}

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
    return common::math::portable_sqrt(sum_sq / static_cast<T>(n - 1));
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
