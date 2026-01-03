#pragma once

#include "../../Common/inc/interface.hpp"

namespace kalman {

class ESKFFilter : public Filter {
public:
  ESKFFilter();
  ~ESKFFilter() override;

  uint8_t init(const SensorData& obs, float static_time) override;
  uint8_t update(const SensorData& obs) override;
  uint8_t getState(State& out) override;
  uint8_t setParams(const Params& p) override;
  uint8_t reset() override;

private:
  State state_;
  Params params_;
};

} // namespace kalman
#pragma once

#include "Lib/Common/inc/interface.hpp"
#include <cstring>

namespace kalman {

class ESKFFilter : public Filter {
public:
  ESKFFilter() { std::memset(&state_, 0, sizeof(state_)); state_.q[0] = 1.0f; }
  ~ESKFFilter() override {}

  uint8_t init(const SensorData& obs, float static_time) override {
    (void)obs; (void)static_time;
    std::memset(&state_, 0, sizeof(state_)); state_.q[0] = 1.0f; return 0;
  }

  uint8_t update(const SensorData& obs) override {
    (void)obs; return 0; // stub: real implementation lives in Lib/ESKF/src
  }

  uint8_t getState(State& out) override { out = state_; return 0; }

  uint8_t setParams(const Params& p) override { (void)p; return 0; }

  uint8_t reset() override { std::memset(&state_, 0, sizeof(state_)); state_.q[0] = 1.0f; return 0; }

private:
  State state_;
};

} // namespace kalman
