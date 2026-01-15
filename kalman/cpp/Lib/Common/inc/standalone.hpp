#pragma once
#ifndef LIB_COMMON_INC_STANDALONE_HPP
#define LIB_COMMON_INC_STANDALONE_HPP

#include "interface.hpp"

namespace kalman {

typedef enum {
  FILTER_KF = 0,
  FILTER_EKF = 1,
  FILTER_UKF = 2,
  FILTER_ESKF = 3,
  FILTER_MEUKF = 4
} FilterType;

// Opaque handle for new multi-instance API
typedef void* FilterHandle;

// NOTE: legacy global API removed. Use handle-based API below.

// New handle-based API (supports multiple instances)
FilterHandle filter_create(FilterType type);
void filter_destroy(FilterHandle h);
uint8_t filter_init(FilterHandle h, const SensorData* init_data, uint32_t init_samples, float dt);
uint8_t filter_set_params(FilterHandle h, const Params& params);
uint8_t filter_set_gps_origin(FilterHandle h, double lat, double lon, double alt);
uint8_t filter_update(FilterHandle h, const SensorData& obs);
uint8_t filter_get_state(FilterHandle h, State& out);
uint8_t filter_reset(FilterHandle h);
uint8_t filter_is_initialized(FilterHandle h);
const char* filter_get_version(void);

} // namespace kalman

#endif // LIB_COMMON_INC_STANDALONE_HPP
