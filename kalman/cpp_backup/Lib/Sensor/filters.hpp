#pragma once

#ifndef LIB_SENSOR_FILTERS_HPP
#define LIB_SENSOR_FILTERS_HPP

#include <cmath>
#include <cstring>

namespace common {
namespace sensor {

// ========== EMAフィルタ ==========
template<int N>
class EMAFilter {
private:
    float m_alpha;
    float m_state[N];
    bool m_initialized;
    
public:
    explicit EMAFilter(float alpha = 0.1f) 
        : m_alpha(alpha), m_initialized(false) {
        std::memset(m_state, 0, sizeof(m_state));
    }
    
    void filter(const float in[N], float out[N]) {
        if(!m_initialized) {
            std::memcpy(m_state, in, sizeof(m_state));
            m_initialized = true;
        }
        for(int i = 0; i < N; ++i) {
            m_state[i] = m_alpha * in[i] + (1.0f - m_alpha) * m_state[i];
            out[i] = m_state[i];
        }
    }
    
    void reset() {
        m_initialized = false;
        std::memset(m_state, 0, sizeof(m_state));
    }
};

// ========== Biquadローパスフィルタ ==========
template<int N>
class BiquadLowpassFilter {
private:
    float m_b0, m_b1, m_b2, m_a1, m_a2;
    float m_x1[N], m_x2[N], m_y1[N], m_y2[N];
    
public:
    BiquadLowpassFilter(float cutoff_hz, float sample_rate, float Q = 0.707f) {
        float omega = 2.0f * 3.14159265f * cutoff_hz / sample_rate;
        float alpha = std::sin(omega) / (2.0f * Q);
        float cos_omega = std::cos(omega);
        
        float a0 = 1.0f + alpha;
        m_b0 = (1.0f - cos_omega) / 2.0f / a0;
        m_b1 = (1.0f - cos_omega) / a0;
        m_b2 = m_b0;
        m_a1 = -2.0f * cos_omega / a0;
        m_a2 = (1.0f - alpha) / a0;
        
        std::memset(m_x1, 0, sizeof(m_x1));
        std::memset(m_x2, 0, sizeof(m_x2));
        std::memset(m_y1, 0, sizeof(m_y1));
        std::memset(m_y2, 0, sizeof(m_y2));
    }
    
    void filter(const float in[N], float out[N]) {
        for(int i = 0; i < N; ++i) {
            float y = m_b0*in[i] + m_b1*m_x1[i] + m_b2*m_x2[i] 
                    - m_a1*m_y1[i] - m_a2*m_y2[i];
            m_x2[i] = m_x1[i];
            m_x1[i] = in[i];
            m_y2[i] = m_y1[i];
            m_y1[i] = y;
            out[i] = y;
        }
    }
};

// ========== Alpha-Betaフィルタ ==========
template<int N>
class AlphaBetaFilter {
private:
    float m_alpha, m_beta;
    float m_position[N];
    float m_velocity[N];
    bool m_initialized;
    
public:
    AlphaBetaFilter(float alpha = 0.5f, float beta = 0.1f)
        : m_alpha(alpha), m_beta(beta), m_initialized(false) {
        std::memset(m_position, 0, sizeof(m_position));
        std::memset(m_velocity, 0, sizeof(m_velocity));
    }
    
    void filter(const float in[N], float out[N], float dt) {
        if(!m_initialized) {
            std::memcpy(m_position, in, sizeof(m_position));
            m_initialized = true;
        }
        for(int i = 0; i < N; ++i) {
            float predicted = m_position[i] + m_velocity[i] * dt;
            float residual = in[i] - predicted;
            m_position[i] = predicted + m_alpha * residual;
            m_velocity[i] += m_beta * residual / dt;
            out[i] = m_position[i];
        }
    }
};

} // namespace sensor
} // namespace common

#endif
#pragma once

#ifndef LIB_SENSOR_FILTERS_HPP
#define LIB_SENSOR_FILTERS_HPP

// Aggregator + SensorFilterLib implementation migrated from Common/inc/Sensor

#include "../Matrix/fixed_matrix.hpp"
#include "../KF/inc/kf_operations.hpp"
#include "../Core/statistics.hpp"
#include "../Core/geometry.hpp"
#include "../Core/numerical.hpp"
#include "../Core/portable_math.hpp"
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
	// configurable parameters for accel outlier detection
	float accel_threshold_sigma_;
	float accel_min_std_;
	// gravity norm validation range (matches MATLAB SensorAccelFilter defaults)
	float gravity_range_min_;
	float gravity_range_max_;
    
	NoiseEstimator noise_estimator;
	DivergenceGuard divergence_guard;
    
	SensorFilterLib() : noise_estimator(10), accel_threshold_sigma_(3.0f), accel_min_std_(0.1f),
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
		gps_filter.reset();
		accel_outlier.reset();
		mag_outlier.reset();
	}
    
	void reset_all_zero() {
		accel_filter.reset_zero();
		mag_filter.reset_zero();
		baro_filter.reset_zero();
		gps_filter.reset_zero();
		accel_outlier.reset();
		mag_outlier.reset();
	}

	void set_accel_config(float ema_alpha, int history_size, float threshold_sigma, float min_std) {
		accel_filter.set_alpha(ema_alpha);
		accel_threshold_sigma_ = threshold_sigma;
		accel_min_std_ = min_std;
		(void)history_size; // history_size not used (fixed MAX_HISTORY)
	}
    
	cm filter_accel(const cm& a_meas, const cm& a_expected, bool& is_outlier) {
		float a_norm_sq = 0.0f;
		for (int i = 0; i < 3; ++i) {
			a_norm_sq += a_meas(i,0) * a_meas(i,0);
		}
		float a_norm = common::math::portable_sqrt(a_norm_sq);
        
		if (a_norm < gravity_range_min_ || a_norm > gravity_range_max_) {
			is_outlier = true;
			return accel_filter.get_value();
		}
        
			::common::math::cm innov; innov.resize(3,1);
			for (int i = 0; i < 3; ++i) innov(i,0) = a_meas(i,0) - a_expected(i,0);

			::common::math::cm Rmat = noise_estimator.get_R_matrix("accel");
			is_outlier = accel_outlier.detect_mahalanobis_static(innov, Rmat, accel_threshold_sigma_);
        
		if (is_outlier) {
			return accel_filter.get_value();
		} else {
			return accel_filter.filter(a_meas);
		}
	}
    
	cm filter_mag(const cm& m_meas, const cm& m_expected, bool& is_outlier) {
		float residual_norm = 0.0f;
		for (int i = 0; i < 3; ++i) {
			float diff = m_meas(i,0) - m_expected(i,0);
			residual_norm += diff * diff;
		}
		residual_norm = common::math::portable_sqrt(residual_norm);
        
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
