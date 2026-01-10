#include "../inc/Sensor/sensor_filter_base.hpp"

// Out-of-class definitions for static const float members
const float common::sensor::NoiseEstimator::R_ABS_MIN = 1e-12f;
const float common::sensor::NoiseEstimator::R_ABS_MAX = 1e6f;
const float common::sensor::NoiseEstimator::OUTLIER_FACTOR = 20.0f;
