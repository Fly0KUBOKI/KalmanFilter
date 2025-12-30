#pragma once

#ifndef MEX_MEX_ESKF_COMMON_HPP
#define MEX_MEX_ESKF_COMMON_HPP

// Common includes and definitions for mex_run_eskf.cpp

#include <mex.h>
#include <cmath>
#include <cstring>
#include <string>
#include <map>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../Common/Math/fixed_matrix.hpp"
#include "../Common/Math/vector_utils.hpp"
#include "../Common/Math/quaternion_lib.hpp"
#include "../Common/Math/statistics.hpp"
#include "../ESKF/eskf_core.hpp"
#include "../ESKF/eskf_postprocess.hpp"
#include "../ESKF/eskf_state.hpp"
#include "../ESKF/eskf_runner.hpp"
#include "../ESKF/eskf_initializer.hpp"
#include "../Common/filter_management.hpp"
#include "../Common/Sensor/sensor_filter.hpp"
#include "../Common/Sensor/sensor_preprocessor.hpp"
#include "../ESKF/eskf_sensor_updates.hpp"
#include "mex_type_conversion.hpp"
#include "mex_helpers.hpp"

using namespace common::math;
using namespace common::filter;
using namespace common::sensor;
using Quat = quat_lib::Quaternion<double>;
using QuatF = quat_lib::Quaternion<float>;
using namespace cmath_fx;
using namespace eskf;
using namespace mex_conv;
using namespace mex_helpers;
using cm = cmath_fx::FixedMatrix;

// Simple inline functions instead of macros
inline void getAccel(const mxArray* obs, mwIndex idx, double* out) {
    mex_helpers::getVec3(obs, "ax", "ay", "az", idx, out);
}
inline void getGyro(const mxArray* obs, mwIndex idx, double* out) {
    mex_helpers::getVec3(obs, "wx", "wy", "wz", idx, out);
}
inline void getMag(const mxArray* obs, mwIndex idx, double* out) {
    mex_helpers::getVec3(obs, "mx", "my", "mz", idx, out);
}

// Forward declaration for global variables
namespace mex_run_eskf_impl {
    extern SensorFilterLib g_filter_lib;
}

#endif // MEX_MEX_ESKF_COMMON_HPP

