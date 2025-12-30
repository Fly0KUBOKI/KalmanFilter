#pragma once

#ifndef ESKF_ESKF_INITIALIZER_HPP
#define ESKF_ESKF_INITIALIZER_HPP

#include "eskf_state.hpp"
#include <mex.h>

namespace eskf {

// Forward declaration
struct ESKFState;

// Initialize ESKF state from MATLAB observation data
// obs: MATLAB struct array containing sensor data
// static_time: static initialization time (seconds)
// dt: sampling time (seconds)
// Returns: initialized ESKFState (caller must manage memory)
ESKFState* initialize_eskf_from_matlab(const mxArray* obs, double static_time, double dt);

} // namespace eskf

#endif // ESKF_ESKF_INITIALIZER_HPP
