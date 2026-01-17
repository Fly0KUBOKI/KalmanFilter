#pragma once
#ifndef LIB_MEUKF_INC_MEUKF_INCLUDES_HPP
#define LIB_MEUKF_INC_MEUKF_INCLUDES_HPP

// =================================================================
// MEUKF - Unified Include Header
// =================================================================
// This header consolidates all common includes used in MEUKF
// implementation files to reduce redundancy and maintain consistency.
//
// Include this header in all MEUKF .cpp files instead of individual
// includes scattered throughout each implementation file.
// =================================================================

// === Core MEUKF Types and Core Functions ===
#include "unified_types.hpp"      // FilterInput, FilterOutput, FilterState definitions
#include "meukf_core.hpp"
#include "meukf_types.hpp"

// === Matrix and Linear Algebra ===
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Matrix/Math/statistics.hpp"

// === Quaternion Operations ===
#include "../../Quaternion/quaternion_functions.hpp"

// === Sensor Processing ===
#include "../../Sensor/sensor_processing.hpp"
#include "../../Sensor/sensor_filters.hpp"

// === UKF Components ===
#include "../../UKF/inc/ukf_update.hpp"
#include "../../UKF/inc/ukf_sigma_points.hpp"

// === Standard C++ Library ===
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cstddef>

#endif  // LIB_MEUKF_INC_MEUKF_INCLUDES_HPP
