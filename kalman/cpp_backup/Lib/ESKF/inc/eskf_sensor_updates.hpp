#pragma once

#ifndef ESKF_ESKF_SENSOR_UPDATES_HPP
#define ESKF_ESKF_SENSOR_UPDATES_HPP

#include "eskf_state.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Sensor/filters.hpp"
#include "../../Common/inc/Sensor/sensor_preprocessor.hpp"
#include "eskf_core.hpp"
#include "eskf_postprocess.hpp"

namespace eskf {

struct ESKFState;

struct SensorUpdateResult { bool should_skip; bool updated; };

SensorUpdateResult update_accel_sensor(ESKFState* s, const cmath_fx::Vector<3, float>& a_meas, common::sensor::SensorFilterLib& filter_lib);
SensorUpdateResult update_mag_sensor(ESKFState* s, const cmath_fx::Vector<3, float>& m_meas, common::sensor::SensorFilterLib& filter_lib);
SensorUpdateResult update_baro_sensor(ESKFState* s, double pressure, common::sensor::SensorFilterLib& filter_lib);
SensorUpdateResult update_gps_sensor(ESKFState* s, double lat, double lon, double alt, common::sensor::SensorFilterLib& filter_lib);

} // namespace eskf

#endif // ESKF_ESKF_SENSOR_UPDATES_HPP
