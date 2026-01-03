#include "../inc/filter.hpp"
#include <cstring>

namespace kalman {

ESKFFilter::ESKFFilter() {
  std::memset(&state_, 0, sizeof(state_));
  std::memset(&params_, 0, sizeof(params_));
  state_.q[0] = 1.0f; // identity quaternion
}

ESKFFilter::~ESKFFilter() {}

uint8_t ESKFFilter::init(const SensorData& /*obs*/, float /*static_time*/) {
  // Minimal init: set identity quaternion
  state_.q[0] = 1.0f;
  return 0;
}

uint8_t ESKFFilter::update(const SensorData& /*obs*/) {
  // Stubbed update
  return 0;
}

uint8_t ESKFFilter::getState(State& out) {
  out = state_;
  return 0;
}

uint8_t ESKFFilter::setParams(const Params& p) {
  params_ = p;
  return 0;
}

uint8_t ESKFFilter::reset() {
  std::memset(&state_, 0, sizeof(state_));
  state_.q[0] = 1.0f;
  return 0;
}

} // namespace kalman
