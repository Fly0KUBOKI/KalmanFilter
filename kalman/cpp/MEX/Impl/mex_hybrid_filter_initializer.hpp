#pragma once
#ifndef MEX_IMPL_MEX_ESKF_INITIALIZER_HPP
#define MEX_IMPL_MEX_ESKF_INITIALIZER_HPP


#include "../../Lib/ESKF/inc/eskf_state.hpp"
#include <mex.h>

namespace eskf {

// Forward declaration
struct FilterState;

// Initialize ESKF state from MATLAB observation data
// obs: MATLAB struct array containing sensor data
// static_time: static initialization time (seconds)
// Returns: initialized FilterState (caller must manage memory)
// Note: dt is calculated dynamically from timestamps during step execution
FilterState* initialize_eskf_from_matlab(const mxArray* obs, double static_time);

} // namespace eskf

#endif // MEX_MEX_ESKF_INITIALIZER_HPP_IMPL

