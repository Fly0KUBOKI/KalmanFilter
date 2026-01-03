// kalman_all.hpp
// Master header that aggregates core KalmanFilter public headers.
#pragma once

#ifndef KALMAN_ALL_HPP
#define KALMAN_ALL_HPP

// Core utilities
#include "Lib/Matrix/fixed_matrix.hpp"
#include "Lib/Quaternion/quaternion_functions.hpp"

// Common interfaces and standalone API
#include "Lib/Common/inc/interface.hpp"
#include "Lib/Common/inc/standalone.hpp"

// Filter implementations (thin adapters)
#include "Lib/ESKF/inc/filter.hpp"

// Version info
#define KALMAN_VERSION "2.0.0"
#define KALMAN_BUILD_DATE __DATE__

namespace kalman {
  inline const char* version() { return KALMAN_VERSION; }
}

#endif // KALMAN_ALL_HPP
