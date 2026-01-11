#pragma once

#ifndef MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP_IMPL
#define MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP_IMPL

#include "mex_eskf_common.hpp"
#include <cstring>

namespace mex_run_eskf_impl {

// Implementations: lightweight wrappers that call into Lib/ESKF sensor update functions
// Provides: call_sensor_update(...) and call_gps_update(...)

inline void call_sensor_update(ESKFState* s, const char* sensor_type, const double* data, int len, int k) {
	if (!s || !sensor_type) return;

	if (std::strcmp(sensor_type, "accel") == 0 && len >= 3) {
		cmath_fx::Vector<3, float> a;
		for (int i = 0; i < 3; ++i) a(i,0) = static_cast<float>(data[i]);
		eskf::update_accel_sensor(s, a, mex_run_eskf_impl::g_filter_lib);
		return;
	}

	if (std::strcmp(sensor_type, "mag") == 0 && len >= 3) {
		cmath_fx::Vector<3, float> m;
		for (int i = 0; i < 3; ++i) m(i,0) = static_cast<float>(data[i]);
		eskf::update_mag_sensor(s, m, mex_run_eskf_impl::g_filter_lib);
		return;
	}

	if (std::strcmp(sensor_type, "baro") == 0 && len >= 1) {
		double pressure = data[0];
		eskf::update_baro_sensor(s, pressure, mex_run_eskf_impl::g_filter_lib);
		return;
	}

	// Unknown sensor: no-op
}

inline void call_gps_update(ESKFState* s, double lat, double lon, double alt, int k) {
	if (!s) return;
	eskf::update_gps_sensor(s, lat, lon, alt, mex_run_eskf_impl::g_filter_lib);
}

} // namespace mex_run_eskf_impl

#endif // MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP_IMPL

