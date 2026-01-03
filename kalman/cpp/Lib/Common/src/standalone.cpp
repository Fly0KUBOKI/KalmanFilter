#include "Lib/Common/inc/standalone.hpp"
#include "Lib/ESKF/inc/filter.hpp"
#include <cstring>

namespace kalman {

static FilterType g_type = FILTER_ESKF;
static Filter* g_filter = nullptr;
static bool g_initialized = false;

uint8_t filter_setType(FilterType t) {
  g_type = t;
  return 0;
}

uint8_t filter_init(void) {
  if (g_filter) { delete g_filter; g_filter = nullptr; g_initialized = false; }
  // For now default to ESKF; future work: instantiate selected type
  if (g_type == FILTER_ESKF) {
    g_filter = new ESKFFilter();
  } else {
    g_filter = new ESKFFilter();
  }
  if (!g_filter) return 1;
  SensorData zero_obs; std::memset(&zero_obs, 0, sizeof(zero_obs));
  float static_time = 5.0f;
  uint8_t r = g_filter->init(zero_obs, static_time);
  g_initialized = (r == 0);
  return g_initialized ? 0 : 1;
}

uint8_t filter_update(const SensorData& obs) {
  if (!g_initialized || !g_filter) return 1;
  return g_filter->update(obs);
}

uint8_t filter_getState(State& out) {
  if (!g_initialized || !g_filter) return 1;
  return g_filter->getState(out);
}

uint8_t filter_reset(void) {
  if (!g_filter) return 0;
  uint8_t r = g_filter->reset();
  delete g_filter; g_filter = nullptr; g_initialized = false;
  return r;
}

} // namespace kalman
