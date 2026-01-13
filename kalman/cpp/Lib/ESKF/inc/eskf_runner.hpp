#pragma once
#ifndef LIB_ESKF_INC_ESKF_RUNNER_HPP
#define LIB_ESKF_INC_ESKF_RUNNER_HPP


#include "eskf_state.hpp"
#include "eskf_core.hpp"
#include "eskf_postprocess.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Sensor/sensor_filter.hpp"
#include <cstddef>

namespace eskf {

class HybridFilterRunner {
public:
    HybridFilterRunner();
    ~HybridFilterRunner() = default;
    static void predict(FilterState* s, const float* a_meas, const float* w_meas);
private:
    static void apply_accel_z_integration(cmath_fx::Vector<3, float>& v, const cmath_fx::Vector<4, float>& q, const cmath_fx::Vector<3, float>& a_for_vel, float dt, const cmath_fx::Vector<3, float>& g, float accel_z_threshold, float accel_z_damping);
    static void apply_velocity_clipping(cmath_fx::Vector<3, float>& v, cmath_fx::Matrix<15, 15, float>& P, float max_vel);
    static void regularize_covariance(cmath_fx::Matrix<15, 15, float>& P);
};

} // namespace eskf

#endif // ESKF_ESKF_RUNNER_HPP
