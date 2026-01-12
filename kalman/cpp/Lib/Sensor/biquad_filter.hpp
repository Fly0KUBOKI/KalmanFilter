#pragma once
// Wrapper to new location
#pragma once

#ifndef LIB_SENSOR_BIQUAD_FILTER_HPP
#define LIB_SENSOR_BIQUAD_FILTER_HPP

#include "../Matrix/fixed_matrix.hpp"
#include <cmath>

namespace common {
namespace sensor {

using cm = cmath_fx::FixedMatrix;

class BiquadLowpassFilter {
private:
	cm x1_, x2_, y1_, y2_;
	float b0_, b1_, b2_, a1_, a2_;
	bool initialized_;
public:
	BiquadLowpassFilter() : initialized_(false) { b0_=b1_=b2_=a1_=a2_=0.0f; }
	void configure(float dt, float cutoff_freq) {
		float omega = 2.0f * 3.14159265358979323846f * cutoff_freq;
		float K = tanf(omega * dt / 2.0f);
		float norm = 1.0f / (1.0f + K / 0.7071f + K * K);
		b0_ = K * K * norm; b1_ = 2.0f * b0_; b2_ = b0_;
		a1_ = 2.0f * (K * K - 1.0f) * norm;
		a2_ = (1.0f - K / 0.7071f + K * K) * norm;
		initialized_ = false;
	}
	cm filter(const cm& input) {
		if (!initialized_) { x1_ = input; x2_ = input; y1_ = input; y2_ = input; initialized_ = true; return input; }
		cm result; result.resize(input.rows, input.cols);
		for (int i = 0; i < input.rows; ++i) for (int j = 0; j < input.cols; ++j) result(i,j) = b0_ * input(i,j) + b1_ * x1_(i,j) + b2_ * x2_(i,j) - a1_ * y1_(i,j) - a2_ * y2_(i,j);
		x2_ = x1_; x1_ = input; y2_ = y1_; y1_ = result;
		return result;
	}
	void reset() { initialized_ = false; }
};

} // namespace sensor
} // namespace common

#endif // LIB_SENSOR_BIQUAD_FILTER_HPP
