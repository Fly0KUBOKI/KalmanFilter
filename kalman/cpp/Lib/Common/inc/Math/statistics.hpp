#pragma once

#ifndef COMMON_MATH_STATISTICS_HPP
#define COMMON_MATH_STATISTICS_HPP

#include <cstddef>
#include <cmath>

namespace common {
namespace math {

// 平均値計算
// data: データ配列
// n: データ数
// 戻り値: 平均値
template<typename T>
T compute_mean(const T* data, std::size_t n) {
    if (n == 0) return T(0);
    T sum = T(0);
    for (std::size_t i = 0; i < n; ++i) {
        sum += data[i];
    }
    return sum / static_cast<T>(n);
}

// 3次元データの平均値計算
// ax, ay, az: 各軸のデータ配列
// n: データ数
// mean_x, mean_y, mean_z: 平均値 [出力]
template<typename T>
void compute_mean_3d(const T* ax, const T* ay, const T* az, std::size_t n, T* mean_x, T* mean_y, T* mean_z) {
    *mean_x = *mean_y = *mean_z = T(0);
    if (n == 0) return;
    for (std::size_t i = 0; i < n; ++i) {
        *mean_x += ax[i];
        *mean_y += ay[i];
        *mean_z += az[i];
    }
    T inv_n = T(1) / static_cast<T>(n);
    *mean_x *= inv_n;
    *mean_y *= inv_n;
    *mean_z *= inv_n;
}

// 標準偏差計算
// data: データ配列
// n: データ数
// mean: 平均値
// 戻り値: 標準偏差
template<typename T>
T compute_std(const T* data, std::size_t n, T mean) {
    if (n < 2) return T(0);
    T sum_sq = T(0);
    for (std::size_t i = 0; i < n; ++i) {
        T diff = data[i] - mean;
        sum_sq += diff * diff;
    }
    return std::sqrt(sum_sq / static_cast<T>(n - 1));
}

// 3次元データの標準偏差計算（平均値の平均）
// ax, ay, az: 各軸のデータ配列
// n: データ数
// mean_x, mean_y, mean_z: 各軸の平均値
// 戻り値: 標準偏差の平均
template<typename T>
T compute_std_3d(const T* ax, const T* ay, const T* az, std::size_t n, T mean_x, T mean_y, T mean_z) {
    T std_x = compute_std(ax, n, mean_x);
    T std_y = compute_std(ay, n, mean_y);
    T std_z = compute_std(az, n, mean_z);
    return (std_x + std_y + std_z) / T(3);
}

} // namespace math
} // namespace common

#endif // COMMON_MATH_STATISTICS_HPP

