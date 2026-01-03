// kalman_all.hpp
// Master header placeholder for incremental refactor (Phase2).
// Minimal content to avoid impacting existing build; expand later.
#pragma once

#ifndef KALMAN_ALL_HPP
#define KALMAN_ALL_HPP

#include "matrix.hpp"
#include "quaternion.hpp"

// Version info
#define KALMAN_VERSION "2.0.0"
#define KALMAN_BUILD_DATE __DATE__

namespace kalman {
  inline const char* version() { return KALMAN_VERSION; }
}

#endif // KALMAN_ALL_HPP
