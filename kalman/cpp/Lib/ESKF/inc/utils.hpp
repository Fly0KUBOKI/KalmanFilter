#pragma once
#ifndef LIB_ESKF_INC_UTILS_HPP
#define LIB_ESKF_INC_UTILS_HPP

#include "interface.hpp"
#include "filter_mgmt.hpp"
#include <cmath>
#include "../../Quaternion/quaternion_functions.hpp"
// Matrix utilities consolidated into fixed_matrix.hpp

namespace kalman {

// NOTE: DEPRECATED wrapper functions have been removed.
// Use the unified modules directly:
//  - symmetrizeCov() → cmath_fx::utils::symmetrize<15, float>()
//  - normalizeQuat() → cquat::normalize_quat()

} // namespace kalman

#endif // LIB_COMMON_INC_UTILS_HPP
