#pragma once

#ifndef ESKF_ESKF_INITIALIZER_HPP
#define ESKF_ESKF_INITIALIZER_HPP

#include "eskf_state.hpp"

namespace eskf {

// Initialization data structure for ESKF
// All pointers are optional (can be nullptr if data is not available)
struct ESKFInitializationData {
    // Sensor data arrays (optional, nullptr if not available)
    const double* accel_x;   // Acceleration X
    const double* accel_y;   // Acceleration Y
    const double* accel_z;   // Acceleration Z
    const double* gyro_x;    // Gyroscope X
    const double* gyro_y;    // Gyroscope Y
    const double* gyro_z;    // Gyroscope Z
    const double* mag_x;     // Magnetometer X
    const double* mag_y;     // Magnetometer Y
    const double* mag_z;     // Magnetometer Z
    const double* pressure;  // Barometric pressure
    const double* gps_lat;   // GPS latitude
    const double* gps_lon;   // GPS longitude
    const double* gps_alt;   // GPS altitude
    
    // Data parameters
    int n_samples;           // Total number of samples
    int n_static;            // Number of static samples to use for initialization
    
    // Timing parameters
    double static_time;      // Static initialization time (seconds)
    double dt;               // Sampling time (seconds)
};

// Initialize ESKF state from sensor data
// This is a pure C++ function with no MATLAB dependencies
// data: initialization data structure
// Returns: initialized ESKFState (caller must manage memory with delete)
ESKFState* initialize_eskf_state(const ESKFInitializationData& data);

} // namespace eskf

#endif // ESKF_ESKF_INITIALIZER_HPP
