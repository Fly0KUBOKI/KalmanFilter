#pragma once
#ifndef LIB_KF_INC_KF_INCLUDES_HPP
#define LIB_KF_INC_KF_INCLUDES_HPP

// =================================================================
// KF - Unified Include Header
// =================================================================
// This header consolidates all common includes used in KF
// header files to reduce redundancy and maintain consistency.
//
// The KF module is header-only and provides generic Kalman Filter
// implementations via template classes and functions.
// =================================================================

// === Core KF Types and Operations ===
#include "kalman_filter_core.hpp"

// === Matrix and Linear Algebra ===
#include "../../Matrix/fixed_matrix.hpp"

// === Standard C++ Library ===
#include <cmath>

#endif  // LIB_KF_INC_KF_INCLUDES_HPP
