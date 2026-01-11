#pragma once

#include "interface.hpp"

namespace kalman {

typedef enum {
  FILTER_KF = 0,
  FILTER_EKF = 1,
  FILTER_UKF = 2,
  FILTER_ESKF = 3,
  FILTER_MEUKF = 4
} FilterType;

// Standalone global API (simple C++ entry points)
uint8_t filter_init(void);
uint8_t filter_update(const SensorData& obs);
uint8_t filter_getState(State& out);
uint8_t filter_reset(void);
uint8_t filter_setType(FilterType t);

} // namespace kalman
