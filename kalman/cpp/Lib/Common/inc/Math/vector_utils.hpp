#pragma once
#ifndef LIB_COMMON_INC_MATH_VECTOR_UTILS_HPP
#define LIB_COMMON_INC_MATH_VECTOR_UTILS_HPP


#include "portable_math.hpp"
#include "../../Matrix/fixed_matrix.hpp"

namespace common {
namespace math {

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

#endif // COMMON_MATH_VECTOR_UTILS_HPP
