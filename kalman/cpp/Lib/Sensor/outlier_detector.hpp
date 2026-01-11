#pragma once

#ifndef LIB_SENSOR_OUTLIER_DETECTOR_HPP
#define LIB_SENSOR_OUTLIER_DETECTOR_HPP

#include "../Matrix/fixed_matrix.hpp"
#include "../KF/inc/kf_operations.hpp"
#include "../Common/inc/Math/math_utils.hpp"
#include "../Common/inc/Math/portable_math.hpp"
#include <cmath>

namespace common {
namespace sensor {

using cm = cmath_fx::FixedMatrix;

class OutlierDetector {
private:
    static const int MAX_HISTORY = 20;
    float history_[MAX_HISTORY];
    int count_;
public:
    OutlierDetector() : count_(0) {}
    bool detect(float residual_norm, float threshold_sigma = 3.0f, float min_std = 0.1f) {
        float noise_std;
        if (count_ == 0) {
            noise_std = fmaxf(residual_norm, min_std);
        } else if (count_ < 5) {
            float sum = 0.0f, sum_sq = 0.0f;
            for (int i = 0; i < count_; ++i) { sum += history_[i]; sum_sq += history_[i] * history_[i]; }
            float mean = sum / count_;
            float variance = sum_sq / count_ - mean * mean;
            noise_std = common::math::portable_sqrt(fmaxf(variance, 0.0f));
            noise_std = fmaxf(noise_std, fmaxf(residual_norm / 3.0f, min_std));
        } else {
            float sum = 0.0f, sum_sq = 0.0f;
            for (int i = 0; i < count_; ++i) { sum += history_[i]; sum_sq += history_[i] * history_[i]; }
            float mean = sum / count_;
            noise_std = common::math::portable_sqrt(sum_sq / count_ - mean * mean);
            noise_std = fmaxf(noise_std, fmaxf(residual_norm / 3.0f, min_std));
        }
        float threshold = threshold_sigma * noise_std;
        bool is_outlier = (residual_norm > threshold);
        if (!is_outlier) {
            if (count_ < MAX_HISTORY) history_[count_++] = residual_norm;
            else { for (int i = 0; i < MAX_HISTORY - 1; ++i) history_[i] = history_[i+1]; history_[MAX_HISTORY-1] = residual_norm; }
        }
        return is_outlier;
    }
    static bool detect_mahalanobis_static(const ::common::math::cm& innovation, const ::common::math::cm& S, float threshold_sigma = 3.0f) {
        float dist_sq = ::kf::ops::mahalanobis_distance_squared(innovation, S);
        float dist = common::math::portable_sqrt(dist_sq);
        return dist > threshold_sigma;
    }
    void reset() { count_ = 0; }
};

} // namespace sensor
} // namespace common

#endif // LIB_SENSOR_OUTLIER_DETECTOR_HPP
