#pragma once

// Canonical mex_helpers implementation (moved from MEX/Inc)
#ifndef MEX_MEX_HELPERS_HPP
#define MEX_MEX_HELPERS_HPP

#include <mex.h>
#include <string>
#include <cstring>
#include "../../Lib/Quaternion/quaternion_functions.hpp"
#include "../../Lib/Matrix/fixed_matrix.hpp"

namespace mex_helpers {

inline std::string getCmd(const mxArray* a) {
	char buf[256] = {0};
	if (!mxIsChar(a)) return "";
	mxGetString(a, buf, sizeof(buf));
	return std::string(buf);
}

inline double get_value_from_single(const mxArray* arr, mwIndex i, const char* name) {
	if (!arr) return 0.0;
	if (mxGetClassID(arr) != mxSINGLE_CLASS) {
		mexErrMsgIdAndTxt("mex_helpers:type_error",
			"Expected single (float) array for field '%s', but got %s.",
			name, mxGetClassName(arr));
		return 0.0;
	}
	const float* pf = reinterpret_cast<const float*>(mxGetData(arr));
	return pf[i];
}

inline void getVec3(const mxArray* s, const char* xname, const char* yname, const char* zname, mwIndex idx, double* out) {
	mxArray* fx = mxGetField((mxArray*)s, 0, xname);
	mxArray* fy = mxGetField((mxArray*)s, 0, yname);
	mxArray* fz = mxGetField((mxArray*)s, 0, zname);
	out[0] = get_value_from_single(fx, idx, xname);
	out[1] = get_value_from_single(fy, idx, yname);
	out[2] = get_value_from_single(fz, idx, zname);
}

inline void copy_vec(double* dst, const double* src, int n) {
	memcpy(dst, src, n * sizeof(double));
}

inline bool is_nan_any(const double* v, int n) {
	for (int i = 0; i < n; ++i) if (mxIsNaN(v[i])) return true;
	return false;
}

inline const mxArray* get_field(const mxArray* s, const char* name) {
	if (!mxIsStruct(s)) return nullptr;
	return mxGetField(const_cast<mxArray*>(s), 0, name);
}

inline const mxArray* get_field_any(const mxArray* s, const char* name1, const char* name2) {
	const mxArray* f = get_field(s, name1);
	if (f) return f;
	return get_field(s, name2);
}

inline double* get_data(const mxArray* arr) {
	if (!arr) return nullptr;
	return mxGetPr(const_cast<mxArray*>(arr));
}

inline int get_length(const mxArray* arr) {
	if (!arr) return 0;
	return static_cast<int>(mxGetNumberOfElements(arr));
}

inline void quat_to_euler(const double* q_in, double* euler) {
	cmath_fx::Vector<4, float> q;
	q(0,0) = static_cast<float>(q_in[0]);
	q(1,0) = static_cast<float>(q_in[1]);
	q(2,0) = static_cast<float>(q_in[2]);
	q(3,0) = static_cast<float>(q_in[3]);
	cquat::normalize_quat(q);
	float roll_deg, pitch_deg, yaw_deg;
	cquat::to_euler_deg(q, roll_deg, pitch_deg, yaw_deg);
	const double DEG2RAD = 3.14159265358979323846 / 180.0;
	euler[0] = roll_deg * DEG2RAD;
	euler[1] = pitch_deg * DEG2RAD;
	euler[2] = yaw_deg * DEG2RAD;
}

} // namespace mex_helpers

#endif // MEX_MEX_HELPERS_HPP

