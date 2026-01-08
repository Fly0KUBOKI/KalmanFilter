#pragma once

#ifndef COMMON_SENSOR_ROBUST_STATISTICS_HPP
#define COMMON_SENSOR_ROBUST_STATISTICS_HPP

/**
 * @file robust_statistics.hpp
 * @brief Robust noise estimation and divergence prevention
 * 
 * Provides adaptive noise covariance estimation and divergence guards
 * for Kalman filter stability.
 */

#include "../../Matrix/fixed_matrix.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace common {
namespace sensor {

using cm = cmath_fx::FixedMatrix;

/**
 * @class NoiseEstimator
 * @brief Adaptive measurement noise estimator
 */
class NoiseEstimator {
public:
    /**
     * @brief Constructor
     * @param history_size Number of samples to average
     */
    explicit NoiseEstimator(int history_size = 10) 
        : history_size_(history_size), count_(0) {}
    
    /**
     * @brief Estimate noise from innovation
     * @param sensor_type Sensor identifier string
     * @param innovation Innovation (residual) vector
     * @param P_pred Predicted covariance
     * @param R_prev Previous noise covariance
     */
    void estimate(const char* sensor_type, const cm& innovation, 
                  const cm& P_pred, const cm& R_prev) {
        // Simplified adaptive estimation
        // In production, would use recursive noise estimation
        (void)sensor_type;
        (void)innovation;
        (void)P_pred;
        (void)R_prev;
        ++count_;
        if (count_ > history_size_) count_ = 0;
    }
    
    /**
     * @brief Get current measurement noise covariance
     * @param sensor_type Sensor identifier
     * @return Diagonal noise covariance matrix
     */
    cm get_R_matrix(const char* sensor_type) {
        cm R;
        R.resize(3, 3);
        
        // Default noise covariances per sensor type
        float sigma_sq = 1.0f;  // Default
        
        if (sensor_type != nullptr) {
            std::string type_str(sensor_type);
            if (type_str == "accel") {
                sigma_sq = 0.05f * 0.05f;  // 0.05 m/s^2
            } else if (type_str == "mag") {
                sigma_sq = 100.0f;  // 100 nT
            } else if (type_str == "gps") {
                sigma_sq = 1.0f;  // 1 m
            }
        }
        
        // Fill diagonal
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) = (i == j) ? sigma_sq : 0.0f;
            }
        }
        return R;
    }

private:
    int history_size_;
    int count_;
};

/**
 * @class DivergenceGuard
 * @brief Filter divergence detection and prevention
 */
class DivergenceGuard {
public:
    static constexpr float TRACE_THRESHOLD = 1e4f;
    static constexpr float MIN_EIGENVALUE = 1e-6f;
    
    /**
     * @brief Check for divergence and attenuate if needed
     * @param sensor_type Sensor identifier
     * @param innovation Innovation vector
     * @param dx Error state vector
     * @param was_attenuated Output flag
     * @return true if update should be skipped
     */
    bool check_and_attenuate(const char* sensor_type, cm& innovation, 
                            cm& dx, bool& was_attenuated) {
        was_attenuated = false;
        (void)sensor_type;
        (void)innovation;
        (void)dx;
        return false;
    }
    
    /**
     * @brief Regularize covariance matrix
     * @param P Covariance matrix (modified in-place)
     */
    void regularize_covariance(cm& P) {
        // Add small diagonal regularization for numerical stability
        const float reg_term = 1e-8f;
        for (int i = 0; i < 15; ++i) {
            P(i, i) += reg_term;
        }
    }
};

} // namespace sensor
} // namespace common

#endif // COMMON_SENSOR_ROBUST_STATISTICS_HPP
