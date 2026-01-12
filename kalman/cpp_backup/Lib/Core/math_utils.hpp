#pragma once

#include "../Matrix/fixed_matrix.hpp"
#include "portable_math.hpp"
#include "../KF/inc/kf_operations.hpp"

namespace common {
namespace math {

using cm = cmath_fx::FixedMatrix;

inline constexpr float EPS = 1e-6f;
inline constexpr float PI = 3.14159265358979323846f;

inline float wrap_to_pi(float angle) {
    angle = fmodf(angle + PI, 2.0f * PI);
    if (angle < 0.0f) angle += 2.0f * PI;
    return angle - PI;
}

inline float wrap_to_180(float angle) {
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle - 180.0f;
}

inline float angle_difference(float a1, float a2) { return wrap_to_pi(a2 - a1); }

inline cm normalize_vector(const cm& v) {
    cm result = v; float n = 0.0f;
    for (int i = 0; i < v.rows; ++i) n += v(i,0) * v(i,0);
    n = common::math::portable_sqrt(n);
    if (n < EPS) { for (int i = 0; i < result.rows; ++i) result(i,0)=0.0f; }
    else { for (int i = 0; i < result.rows; ++i) result(i,0)=v(i,0)/n; }
    return result;
}

inline cm clip_vector(const cm& v, float max_norm, bool& clipped) {
    cm result = v; float n = 0.0f; for (int i=0;i<v.rows;++i) n += v(i,0)*v(i,0); n = common::math::portable_sqrt(n);
    if (n > max_norm) { float s = max_norm / n; for (int i=0;i<result.rows;++i) result(i,0)=v(i,0)*s; clipped=true; }
    else { clipped=false; }
    return result;
}

// Inverse normal CDF (Acklam's approximation)
inline double inv_normal_cdf(double p) {
    if (p <= 0.0) return -INFINITY;
    if (p >= 1.0) return INFINITY;
    static const double a1 = -3.969683028665376e+01;
    static const double a2 = 2.209460984245205e+02;
    static const double a3 = -2.759285104469687e+02;
    static const double a4 = 1.383577518672690e+02;
    static const double a5 = -3.066479806614716e+01;
    static const double a6 = 2.506628277459239e+00;

    static const double b1 = -5.447609879822406e+01;
    static const double b2 = 1.615858368580409e+02;
    static const double b3 = -1.556989798598866e+02;
    static const double b4 = 6.680131188771972e+01;
    static const double b5 = -1.328068155288572e+01;

    static const double c1 = -7.784894002430293e-03;
    static const double c2 = -3.223964580411365e-01;
    static const double c3 = -2.400758277161838e+00;
    static const double c4 = -2.549732539343734e+00;
    static const double c5 = 4.374664141464968e+00;
    static const double c6 = 2.938163982698783e+00;

    static const double d1 = 7.784695709041462e-03;
    static const double d2 = 3.224671290700398e-01;
    static const double d3 = 2.445134137142996e+00;
    static const double d4 = 3.754408661907416e+00;

    const double p_low = 0.02425;
    const double p_high = 1.0 - p_low;

    double q, r;
    if (p < p_low) {
        q = common::math::portable_sqrt(-2.0 * std::log(p));
        return (((((c1*q + c2)*q + c3)*q + c4)*q + c5)*q + c6) /
               ((((d1*q + d2)*q + d3)*q + d4)*q + 1.0);
    } else if (p <= p_high) {
        q = p - 0.5;
        r = q * q;
        return (((((a1*r + a2)*r + a3)*r + a4)*r + a5)*r + a6)*q /
               (((((b1*r + b2)*r + b3)*r + b4)*r + b5)*r + 1.0);
    } else {
        q = common::math::portable_sqrt(-2.0 * std::log(1.0 - p));
        return -(((((c1*q + c2)*q + c3)*q + c4)*q + c5)*q + c6) /
                ((((d1*q + d2)*q + d3)*q + d4)*q + 1.0);
    }
}

// Approximate chi-square quantile (inverse CDF) using Wilson-Hilferty transform
inline double chi2_quantile(int k, double p) {
    if (k <= 0) return 0.0;
    double z = inv_normal_cdf(p);
    double term = 1.0 - 2.0/(9.0*k) + z * common::math::portable_sqrt(2.0/(9.0*k));
    return k * term * term * term;
}

// Chi-square outlier test using Mahalanobis distance
inline bool is_outlier_chi_square(const cm& innovation, const cm& S, int dof, double alpha = 0.05) {
    float dist_sq = kf::ops::mahalanobis_distance_squared(innovation, S);
    double crit = chi2_quantile(dof, 1.0 - alpha);
    return dist_sq > crit;
}

} // namespace math
} // namespace common
