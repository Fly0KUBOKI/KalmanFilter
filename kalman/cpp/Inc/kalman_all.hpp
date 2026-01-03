// kalman_all.hpp
// Master header that aggregates core KalmanFilter public headers.
#pragma once

#ifndef KALMAN_ALL_HPP
#define KALMAN_ALL_HPP

// Core utilities
#include "Lib/Matrix/fixed_matrix.hpp"
#include "Lib/Quaternion/quaternion_functions.hpp"

// Common interfaces and utilities
#include "Lib/Common/inc/interface.hpp"
#include "Lib/Common/inc/utils.hpp"
#include "Lib/Common/inc/filter_mgmt.hpp"
#include "Lib/Common/inc/Math/math_utils.hpp"
#include "Lib/Common/inc/Sensor/sensor_filter.hpp"

// Standalone API is optional; define KALMAN_NO_STANDALONE to exclude
#ifndef KALMAN_NO_STANDALONE
#include "Lib/Common/inc/standalone.hpp"
#endif

// Filter implementations (public headers)
#include "Lib/KF/inc/kf_core.hpp"
#include "Lib/EKF/inc/ekf_core.hpp"
#include "Lib/EKF/inc/ekf_linear_update.hpp"
#include "Lib/UKF/inc/ukf_core.hpp"
#include "Lib/UKF/inc/ukf_sigma_points.hpp"
#include "Lib/UKF/inc/ukf_update.hpp"
#include "Lib/ESKF/inc/filter.hpp"
#include "Lib/MEUKF/inc/meukf_core.hpp"

// Version info
#define KALMAN_VERSION "2.0.0"
#define KALMAN_BUILD_DATE __DATE__

namespace kalman {
  inline const char* version() { return KALMAN_VERSION; }
}

#endif // KALMAN_ALL_HPP
