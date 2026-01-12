#pragma once
#ifndef LIB_ESKF_INC_FILTER_HPP
#define LIB_ESKF_INC_FILTER_HPP

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

#endif // LIB_ESKF_INC_FILTER_HPP
