#pragma once

#include <cmath>

namespace common {
namespace math {

inline float safe_divide(float numerator, float denominator, float default_value = 0.0f) {
    const float EPS = 1.0e-9f;
    if (fabsf(denominator) < EPS) return default_value;
    return numerator / denominator;
}

inline float safe_sqrt(float x) {
    return sqrtf(fmaxf(x, 0.0f));
}

inline float safe_asin(float x) {
    x = fmaxf(fminf(x, 1.0f), -1.0f);
    return asinf(x);
}

inline float safe_acos(float x) {
    x = fmaxf(fminf(x, 1.0f), -1.0f);
    return acosf(x);
}

inline float linear_interpolate(float x, float x1, float y1, float x2, float y2) {
    const float EPS = 1.0e-9f;
    if (fabsf(x2 - x1) < EPS) return y1;
    float t = (x - x1) / (x2 - x1);
    return y1 + t * (y2 - y1);
}

} // namespace math
} // namespace common
