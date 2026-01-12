#pragma once

#ifndef LIB_SENSOR_EMA_FILTER_HPP
#define LIB_SENSOR_EMA_FILTER_HPP

#include "../Matrix/fixed_matrix.hpp"
#include <cmath>

namespace common {
namespace sensor {

using cm = cmath_fx::FixedMatrix;

class EMAFilter {
private:
    cm filtered_;
    float alpha_;
    bool initialized_;
public:
    EMAFilter(float alpha = 0.3f) : alpha_(alpha), initialized_(false) {}
    cm filter(const cm& input) {
        if (!initialized_) {
            filtered_ = input;
            initialized_ = true;
            return filtered_;
        }
        cm result;
        result.resize(input.rows, input.cols);
        for (int i = 0; i < input.rows; ++i) {
            for (int j = 0; j < input.cols; ++j) {
                result(i,j) = alpha_ * input(i,j) + (1.0f - alpha_) * filtered_(i,j);
            }
        }
        filtered_ = result;
        return result;
    }
    cm get_value() const { return filtered_; }
    void reset() { initialized_ = false; }
    void reset_zero() { for(int i=0;i<filtered_.rows*filtered_.cols;++i) filtered_.data[i]=0.0f; initialized_ = true; }
    void set_alpha(float alpha) { alpha_ = fmaxf(0.0f, fminf(1.0f, alpha)); }
};

} // namespace sensor
} // namespace common

#endif // LIB_SENSOR_EMA_FILTER_HPP
