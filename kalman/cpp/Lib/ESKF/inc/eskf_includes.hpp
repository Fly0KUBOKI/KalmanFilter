#pragma once
#ifndef LIB_ESKF_INC_ESKF_INCLUDES_HPP
#define LIB_ESKF_INC_ESKF_INCLUDES_HPP

// =================================================================
// ESKF - Unified Include Header
// =================================================================
// This header consolidates all common includes used in ESKF
// implementation files to reduce redundancy and maintain consistency.
//
// Include this header in all ESKF .cpp files instead of individual
// includes scattered throughout each implementation file.
// =================================================================

// === Core ESKF Components ===
#include "eskf_core.hpp"
#include "eskf_postprocess.hpp"
#include "filter_mgmt.hpp"

// === Kalman Filter Components ===
#include "../../KF/inc/kalman_filter_core.hpp"

// === Matrix and Linear Algebra ===
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Matrix/Math/statistics.hpp"

// === Quaternion Operations ===
#include "../../Quaternion/quaternion_functions.hpp"

// === Sensor Processing ===
#include "../../Sensor/sensor_filters.hpp"
#include "../../Sensor/sensor_processing.hpp"
#include "../../Sensor/coordinate_transform.hpp"
#include "../../Sensor/sensor_all.hpp"

// === Standard C++ Library ===
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#endif  // LIB_ESKF_INC_ESKF_INCLUDES_HPP
