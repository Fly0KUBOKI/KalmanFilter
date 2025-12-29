#pragma once

#ifndef ESKF_RUNNER_HPP
#define ESKF_RUNNER_HPP

#include "eskf_state.hpp"
#include "../Common/Math/fixed_matrix.hpp"
#include "../Common/Math/quaternion_lib.hpp"
#include "../Common/Math/vector_utils.hpp"
#include "../Common/filter_management.hpp"
#include "../Common/Sensor/sensor_filter.hpp"
#include "../Common/Sensor/sensor_preprocessor.hpp"
#include "../ESKF/eskf_postprocess.hpp"
#include "../ESKF/eskf_core.hpp"
#include <cstring>
#include <vector>
#include <cmath>

namespace eskf {

using namespace common::math;
using namespace common::filter;
using namespace common::sensor;
using QuatF = quat_lib::Quaternion<float>;
using namespace cmath_fx;

// Forward declaration for callback function type
// This allows ESKFRunner to call MEUKF update without direct MATLAB dependency
struct MEUKFUpdateParams {
    const double* state_p;
    const double* state_v;
    const double* state_q;
    const double* state_ba;
    const double* state_bg;
    const double* state_P;
    const double* sensor_data;
    const double* mex_params;
    double* new_p;
    double* new_v;
    double* new_q;
    double* new_ba;
    double* new_bg;
    double* new_P;
    double* dbg_innov;
    double* dbg_H;
    double* dbg_dx;
    int innov_len;
    int H_rows;
    int H_cols;
};

// Callback function type for MEUKF update
// Returns true if update was successful
typedef bool (*MEUKFUpdateCallback)(const MEUKFUpdateParams& params);

// ESKF Runner class
// Contains all ESKF core logic, independent of MATLAB MEX interface
class ESKFRunner {
private:
    // Static instance of SensorFilterLib for reuse
    static SensorFilterLib filter_lib_;
    
    // Helper function: copy vector
    static void copy_vec(double* dst, const double* src, int n) {
        memcpy(dst, src, n * sizeof(double));
    }
    
    // Helper function: check if any value is NaN
    static bool is_nan_any(const double* v, int n) {
        for (int i = 0; i < n; ++i) {
            if (std::isnan(v[i])) return true;
        }
        return false;
    }
    
    // Helper function: compute 3D vector norm
    static double norm3(const double* v) {
        return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    }

public:
    // Initialize ESKF state from static sensor data
    // Input: static sensor data arrays and their lengths
    // Output: initialized ESKFState
    struct InitParams {
        const double* ax, *ay, *az;  // Accelerometer data
        const double* wx, *wy, *wz;  // Gyro data
        const double* mx, *my, *mz;  // Magnetometer data
        const double* pressure;      // Barometer data
        const double* lat, *lon, *alt; // GPS data
        int N_static;                // Number of static samples
        double dt;                    // Time step
        bool has_accel, has_gyro, has_mag, has_baro, has_gps;
    };
    
    static void initialize(ESKFState* s, const InitParams& params);
    
    // Predict step
    static void predict(ESKFState* s, const double* a_meas, const double* w_meas);
    
    // Sensor update step (requires MEUKF callback)
    static void sensor_update(ESKFState* s, const char* type, const double* meas, int meas_len, 
                              double sample, MEUKFUpdateCallback meukf_callback);
    
    // GPS update step (requires MEUKF callback)
    static void gps_update(ESKFState* s, double lat, double lon, double alt, 
                          double sample, MEUKFUpdateCallback meukf_callback);
    
    // Check and reset if divergence detected
    static void check_and_reset(ESKFState* s, int k);
    
    // ZUPT check and update
    static void zupt_check_and_update(ESKFState* s, const double* a_meas, const double* w_meas);
    
    // Step function (combines predict, sensor updates, reset check)
    static void step(ESKFState* s, const double* a_meas, const double* w_meas, 
                     const double* mag_meas, const double* baro_meas, bool has_baro,
                     double gps_lat, double gps_lon, double gps_alt, bool has_gps,
                     int k, MEUKFUpdateCallback meukf_callback);
};

} // namespace eskf

#endif // ESKF_RUNNER_HPP

