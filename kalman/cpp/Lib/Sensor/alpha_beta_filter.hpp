#pragma once

#ifndef LIB_SENSOR_ALPHA_BETA_FILTER_HPP
#define LIB_SENSOR_ALPHA_BETA_FILTER_HPP

#include "../Matrix/fixed_matrix.hpp"
#include <cmath>

namespace common {
namespace sensor {

using cm = cmath_fx::FixedMatrix;

class AlphaBetaFilter {
private:
	cm position_;
	cm velocity_;
	float alpha_, beta_;
	bool initialized_;
public:
	AlphaBetaFilter(float alpha = 0.5f, float beta = 0.1f) : alpha_(alpha), beta_(beta), initialized_(false) {}
	void filter(const cm& measurement, float dt, cm& pos_out, cm& vel_out) {
		if (!initialized_) {
			position_ = measurement;
			velocity_.resize(measurement.rows, measurement.cols);
			for (int i = 0; i < velocity_.rows; ++i) for (int j = 0; j < velocity_.cols; ++j) velocity_(i,j) = 0.0f;
			initialized_ = true; pos_out = position_; vel_out = velocity_; return;
		}
		cm pos_pred; pos_pred.resize(position_.rows, position_.cols);
		for (int i = 0; i < position_.rows; ++i) for (int j = 0; j < position_.cols; ++j) pos_pred(i,j) = position_(i,j) + velocity_(i,j) * dt;
		cm residual; residual.resize(measurement.rows, measurement.cols);
		for (int i = 0; i < residual.rows; ++i) for (int j = 0; j < residual.cols; ++j) residual(i,j) = measurement(i,j) - pos_pred(i,j);
		pos_out.resize(position_.rows, position_.cols); vel_out.resize(velocity_.rows, velocity_.cols);
		for (int i = 0; i < position_.rows; ++i) for (int j = 0; j < position_.cols; ++j) { pos_out(i,j) = pos_pred(i,j) + alpha_ * residual(i,j); vel_out(i,j) = velocity_(i,j) + beta_ * residual(i,j) / dt; }
		position_ = pos_out; velocity_ = vel_out;
	}
	void reset() { initialized_ = false; }
	void reset_zero() { position_.resize(0,0); velocity_.resize(0,0); initialized_ = false; }
	void set_parameters(float alpha, float beta) { alpha_ = alpha; beta_ = beta; }
};

} // namespace sensor
} // namespace common

#endif // LIB_SENSOR_ALPHA_BETA_FILTER_HPP
