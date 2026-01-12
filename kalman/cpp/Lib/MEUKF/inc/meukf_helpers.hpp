#pragma once
#ifndef LIB_MEUKF_INC_MEUKF_HELPERS_HPP
#define LIB_MEUKF_INC_MEUKF_HELPERS_HPP
#include "meukf_core.hpp"
#include <cstdlib>
#include <cmath>
#include "../../Common/inc/Math/portable_math.hpp"

namespace meukf {

inline int get_debug_level() {
    const char* s = std::getenv("MEUKF_DEBUG_LEVEL");
    if (!s) return 0;
    return std::atoi(s);
}

inline Vector3 make_vector3(double x, double y, double z) {
    Vector3 v;
    v(0,0) = static_cast<float>(x);
    v(1,0) = static_cast<float>(y);
    v(2,0) = static_cast<float>(z);
    return v;
}

inline Vector2 make_vector2(float x, float y) {
    Vector2 v;
    v(0,0) = x; v(1,0) = y;
    return v;
}

inline Vector4 make_vector4(double w, double x, double y, double z) {
    Vector4 v;
    v(0,0) = static_cast<float>(w);
    v(1,0) = static_cast<float>(x);
    v(2,0) = static_cast<float>(y);
    v(3,0) = static_cast<float>(z);
    return v;
}

inline double vector3_norm(const Vector3& v) {
    double s = (double)v(0,0) * v(0,0) + (double)v(1,0) * v(1,0) + (double)v(2,0) * v(2,0);
    return common::math::portable_sqrt(static_cast<float>(s));
}

} // namespace meukf

#endif // LIB_MEUKF_INC_MEUKF_HELPERS_HPP
