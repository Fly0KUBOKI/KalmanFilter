#pragma once

#ifndef CORE_MATH_MAHALANOBIS_HPP
#define CORE_MATH_MAHALANOBIS_HPP

#include "../KF/inc/kf_operations.hpp"
#include "../Matrix/fixed_matrix.hpp"

namespace common {
namespace math {

using cm = cmath_fx::FixedMatrix;

// Templated fixed-size variant -> forward to kf::mahalanobis_distance_squared (templated)
template <int M, typename T>
inline T mahalanobis_distance_squared(const cmath_fx::Vector<M, T>& innovation, const cmath_fx::Matrix<M, M, T>& S) {
    return kf::mahalanobis_distance_squared<M, T>(innovation, S);
}

} // namespace math
} // namespace common

#endif // CORE_MATH_MAHALANOBIS_HPP
