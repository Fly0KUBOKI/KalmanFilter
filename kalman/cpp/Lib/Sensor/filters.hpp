#pragma once

#ifndef LIB_SENSOR_FILTERS_HPP
#define LIB_SENSOR_FILTERS_HPP

#include "../Matrix/fixed_matrix.hpp"
#include "../KF/inc/kf_operations.hpp"
#include "../Matrix/Math/statistics.hpp"
#include "../Matrix/fixed_matrix.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cfloat>

#include "../Sensor/ema_filter.hpp"
#include "../Sensor/biquad_filter.hpp"
#include "../Sensor/alpha_beta_filter.hpp"
#include "../Sensor/outlier_detector.hpp"
#include "../Sensor/robust_statistics.hpp"

namespace common {
namespace sensor {

using cm = cmath_fx::FixedMatrix;

class SensorFilterLib {
public:
	EMAFilter accel_filter;
	EMAFilter mag_filter;
	AlphaBetaFilter gps_filter;
	EMAFilter baro_filter;

	OutlierDetector accel_outlier;
	OutlierDetector mag_outlier;
	float accel_threshold_sigma_;
	float accel_min_std_;
	float gravity_range_min_;
	float gravity_range_max_;

	NoiseEstimator noise_estimator;
	DivergenceGuard divergence_guard;

	SensorFilterLib()
		: noise_estimator(10), accel_threshold_sigma_(3.0f), accel_min_std_(0.1f),
		  gravity_range_min_(8.5f), gravity_range_max_(10.5f) {
		accel_filter.set_alpha(0.3f);
		mag_filter.set_alpha(0.2f);
		baro_filter.set_alpha(0.4f);
		gps_filter.set_parameters(0.5f, 0.1f);
	}

	void reset_all() {
		accel_filter.reset();
		mag_filter.reset();
		baro_filter.reset();
		gps_filter = AlphaBetaFilter();
		accel_outlier.reset();
		mag_outlier.reset();
	}

	void reset_all_zero() {
		accel_filter.reset_zero();
		mag_filter.reset_zero();
		baro_filter.reset_zero();
		gps_filter = AlphaBetaFilter();
		accel_outlier.reset();
		mag_outlier.reset();
	}

	void set_accel_config(float ema_alpha, int history_size, float threshold_sigma, float min_std) {
		accel_filter.set_alpha(ema_alpha);
		accel_threshold_sigma_ = threshold_sigma;
		accel_min_std_ = min_std;
		(void)history_size;
	}

	cm filter_accel(const cm& a_meas, const cm& a_expected, bool& is_outlier) {
		float a_norm_sq = 0.0f;
		for (int i = 0; i < 3; ++i) a_norm_sq += a_meas(i,0) * a_meas(i,0);
		float a_norm = std::sqrt(a_norm_sq);

		if (a_norm < gravity_range_min_ || a_norm > gravity_range_max_) {
			is_outlier = true;
			return accel_filter.get_value();
		}

		::common::math::cm innov; innov.resize(3,1);
		for (int i = 0; i < 3; ++i) innov(i,0) = a_meas(i,0) - a_expected(i,0);
		::common::math::cm Rmat = noise_estimator.get_R_matrix("accel");
		is_outlier = accel_outlier.detect_mahalanobis_static(innov, Rmat, accel_threshold_sigma_);

		if (is_outlier) return accel_filter.get_value();
		return accel_filter.filter(a_meas);
	}

	cm filter_mag(const cm& m_meas, const cm& m_expected, bool& is_outlier) {
		::common::math::cm innov; innov.resize(3,1);
		for (int i = 0; i < 3; ++i) innov(i,0) = m_meas(i,0) - m_expected(i,0);
		::common::math::cm Rmat = noise_estimator.get_R_matrix("mag");
		is_outlier = mag_outlier.detect_mahalanobis_static(innov, Rmat, 3.0f);
		return mag_filter.filter(m_meas);
	}

	void filter_gps(const cm& gps_pos, float dt, cm& pos_out, cm& vel_out) {
		gps_filter.filter(gps_pos, dt, pos_out, vel_out);
	}

	float filter_baro(float pressure) {
		cm p_in; p_in.resize(1,1); p_in(0,0) = pressure;
		cm p_out = baro_filter.filter(p_in);
		return p_out(0,0);
	}
};

} // namespace sensor
} // namespace common

#endif // LIB_SENSOR_FILTERS_HPP

