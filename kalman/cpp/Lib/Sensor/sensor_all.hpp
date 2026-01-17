#pragma once
#ifndef LIB_SENSOR_SENSOR_ALL_HPP
#define LIB_SENSOR_SENSOR_ALL_HPP

/**
 * @file sensor_all.hpp
 * @brief Sensor module unified aggregator header
 * 
 * Complete sensor processing library organized by function:
 * 
 * 1. Coordinate Transforms (sensor::coord::)
 *    - GPS LLA to/from ENU conversion
 *    - Pressure to altitude conversion
 * 
 * 2. Sensor Signal Processing (sensor::processing::)
 *    - Accelerometer to quaternion estimation
 *    - Magnetometer observation prediction
 *    - Body to/from World coordinate transforms
 * 
 * 3. Sensor Preprocessing (sensor::preprocess::)
 *    - Raw sensor buffering and change detection
 *    - Simple outlier flagging
 * 
 * 4. Sensor Filtering (sensor::filter::)
 *    - EMA/Alpha-Beta/Biquad filters
 *    - Outlier detection and rejection
 *    - Adaptive noise estimation
 *    - Divergence guard and covariance regularization
 */

#include "coordinate_transform.hpp"
#include "sensor_processing.hpp"
#include "sensor_preprocessor.hpp"
#include "sensor_filters.hpp"

#endif // LIB_SENSOR_SENSOR_ALL_HPP
