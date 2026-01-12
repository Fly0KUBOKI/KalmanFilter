#pragma once

#ifndef ESKF_ESKF_POSTPROCESS_HPP
#define ESKF_ESKF_POSTPROCESS_HPP

#include "../../Matrix/fixed_matrix.hpp"
#include <cstddef>

namespace eskf {

struct PredictPostprocessParams {
    bool enable_accel_z_integration = false;
    float accel_z_threshold = 0.5f;
    float accel_z_damping = 0.1f;
    float velocity_damping = 0.0f;
};

struct PredictPostprocessResult {
    cmath_fx::Vector<3, float> v;
    cmath_fx::Matrix<15, 15, float> P;
};

void predict_postprocess(
    cmath_fx::Vector<3, float>& v,
    const cmath_fx::Vector<4, float>& q,
    cmath_fx::Matrix<15, 15, float>& P,
    const cmath_fx::Vector<3, float>& a_for_vel,
    float dt,
    const cmath_fx::Vector<3, float>& g,
    const PredictPostprocessParams& params
);

struct UpdatePostprocessResult {
    cmath_fx::Vector<3, float> p;
    cmath_fx::Vector<3, float> v;
    cmath_fx::Vector<4, float> q;
    cmath_fx::Vector<3, float> ba;
    cmath_fx::Vector<3, float> bg;
    cmath_fx::Matrix<15, 15, float> P;
    bool should_skip = false;
};

UpdatePostprocessResult update_state_from_dx(
    const cmath_fx::Vector<15, float>& dx,
    const cmath_fx::Vector<3, float>& state_p,
    const cmath_fx::Vector<3, float>& state_v,
    const cmath_fx::Vector<4, float>& state_q,
    const cmath_fx::Vector<3, float>& state_ba,
    const cmath_fx::Vector<3, float>& state_bg,
    const cmath_fx::Matrix<15, 15, float>& new_state_P
);


} // namespace eskf

#endif // ESKF_ESKF_POSTPROCESS_HPP
