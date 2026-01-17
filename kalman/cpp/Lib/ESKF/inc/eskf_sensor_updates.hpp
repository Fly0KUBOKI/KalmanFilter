#pragma once
#ifndef LIB_ESKF_INC_ESKF_SENSOR_UPDATES_HPP
#define LIB_ESKF_INC_ESKF_SENSOR_UPDATES_HPP


#include "eskf_state.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Sensor/sensor_filters.hpp"
#include "../../Sensor/sensor_preprocessor.hpp"
#include "eskf_core.hpp"
#include "eskf_postprocess.hpp"

namespace eskf {

struct FilterState;

struct SensorUpdateResult { bool should_skip; bool updated; };

SensorUpdateResult update_accel_sensor(FilterState* s, const cmath_fx::Vector<3, float>& a_meas, sensor::filter::SensorFilterLib& filter_lib);
SensorUpdateResult update_mag_sensor(FilterState* s, const cmath_fx::Vector<3, float>& m_meas, sensor::filter::SensorFilterLib& filter_lib);
SensorUpdateResult update_baro_sensor(FilterState* s, double pressure, sensor::filter::SensorFilterLib& filter_lib);
SensorUpdateResult update_gps_sensor(FilterState* s, double lat, double lon, double alt, sensor::filter::SensorFilterLib& filter_lib);

} // namespace eskf

#endif // ESKF_ESKF_SENSOR_UPDATES_HPP
