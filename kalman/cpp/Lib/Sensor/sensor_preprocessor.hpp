#pragma once
#ifndef LIB_SENSOR_SENSOR_PREPROCESSOR_HPP
#define LIB_SENSOR_SENSOR_PREPROCESSOR_HPP


#include "../Matrix/fixed_matrix.hpp"
#include <cmath>

namespace common {
namespace sensor {

struct PreprocessResult {
    cmath_fx::Vector<3, float> output;
    bool is_outlier;
    bool no_change;
};

PreprocessResult preprocess_accel(
    const cmath_fx::Vector<3, float>& a_meas,
    const cmath_fx::Vector<3, float>& prev_a,
    double buffer_tolerance = 1e-9
);

PreprocessResult preprocess_mag(
    const cmath_fx::Vector<3, float>& m_meas,
    const cmath_fx::Vector<3, float>& prev_m,
    double buffer_tolerance = 1e-9
);

double preprocess_baro(double pressure);

PreprocessResult preprocess_gps(
    double lat, double lon, double alt,
    const cmath_fx::Vector<3, float>& origin,
    double buffer_tolerance = 1e-9
);

} // namespace sensor
} // namespace common

#endif // COMMON_SENSOR_SENSOR_PREPROCESSOR_HPP

