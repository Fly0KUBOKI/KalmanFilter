#pragma once

#ifndef CORE_MATH_VECTOR_UTILS_HPP
#define CORE_MATH_VECTOR_UTILS_HPP

#include "../Matrix/fixed_matrix.hpp"
#include <cmath>
#include "portable_math.hpp"
#include <cstring>

namespace common {
namespace math {

// 3次元ベクトルのノルム計算
template<typename T>
T norm3(const T* v) {
    return common::math::portable_sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// 配列にNaNが含まれているかチェック
template<typename T>
bool is_nan_any(const T* v, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        if (std::isnan(v[i])) return true;
    }
    return false;
}

// ベクトルコピー（memcpyラッパー）
template<typename T>
void copy_vec(T* dst, const T* src, std::size_t n) {
    std::memcpy(dst, src, n * sizeof(T));
}

// ベクトルのノルムでクリップ
// v: ベクトル [入出力]
// max_norm: 最大ノルム
// 戻り値: クリップされたかどうか
template<int R, typename T>
bool clip_vector_norm(cmath_fx::Vector<R, T>& v, T max_norm) {
    T v_norm = T(0);
    for (int i = 0; i < R; ++i) {
        v_norm += v(i, 0) * v(i, 0);
    }
    v_norm = common::math::portable_sqrt(v_norm);
    if (v_norm > max_norm) {
        T scale = max_norm / v_norm;
        for (int i = 0; i < R; ++i) {
            v(i, 0) *= scale;
        }
        return true;
    }
    return false;
}

} // namespace math
} // namespace common

#endif // CORE_MATH_VECTOR_UTILS_HPP
