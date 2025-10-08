/*
 * kalman.hpp
 *
 *  Created on: Aug 21, 2025
 *      Author: takut
 */

#ifndef INC_KALMAN_FILTER_HPP_
#define INC_KALMAN_FILTER_HPP_

#include <cstdint>
#include <cmath>
#include <cstring>
#include <cstddef>

// EWMA alpha for noise estimation
#define NOISE_ALPHA 0.02f

// Maximum sizes for the filter
#define STATE_SIZE_MAX    10    // maximum state vector size
#define OBS_SIZE_MAX      10    // maximum observation vector size

// Maximum temporary buffer size (state x state)
#define TEMP_BUFFER_MAX (STATE_SIZE_MAX * STATE_SIZE_MAX)

class KalmanFilter {
public:
    void Init(uint8_t state_size, uint8_t obs_size);
    void Update();
    // Estimate measurement/prediction noise from incoming measurements (EWMA)
    void EstimateNoise(float meas);
    void GetData(float* out_states);
    // Allow user to set noise levels from MATLAB
    void SetProcessNoise(float q);
    void SetMeasurementNoise(float r);
    // Set diagonal of prediction covariance P to a constant value (for initialization)
    void SetPredictionCovarianceDiag(float val);

    // Input parameters
    float observation[OBS_SIZE_MAX]={};                // observation values
    float prediction[STATE_SIZE_MAX]={};               // predicted state values


    float system_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX]={};    // F
    float observation_matrix[OBS_SIZE_MAX * STATE_SIZE_MAX]={}; // H

    // Set full process/measurement noise matrices (row-major float arrays)
    void SetQMatrix(const float* Q);
    void SetRMatrix(const float* R);

    // Minimal measurement assemble types (kept small on purpose)
    struct Meas {
    bool has_gps = false; float gps[2] = {0.0f, 0.0f};
    bool has_vel = false; float vel[2] = {0.0f, 0.0f};
    };

    struct MeasTag {
        char name[16];
        uint8_t start; // 0-based start index in z
        uint8_t length;
    };

    // AssembleMeasurements: minimal implementation (gps, vel) that
    // fills out_z (len z_len), out_h (z_len), out_H (row-major z_len x state_size),
    // out_R (row-major z_len x z_len), tags (preallocated), and sets z_len.
    void AssembleMeasurements(const Meas& meas, const float* x_pred, float* out_z, float* out_h,
                              float* out_H, float* out_R, MeasTag* tags, uint8_t& z_len, uint8_t state_size);


private:
    uint8_t STATE_SIZE = 0;
    uint8_t OBS_SIZE = 0;

    // Stored variables
    float observation_noise = 0.0f;                         // observation noise (R scalar)
    float prediction_noise = 0.0f;                          // process noise (Q scalar)
    float output[STATE_SIZE_MAX]={};                        // output state vector
    float prediction_noise_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX] = {};      // Q matrix
    float observation_noise_matrix[OBS_SIZE_MAX * OBS_SIZE_MAX] = {};         // R matrix
    float prediction_covariance[STATE_SIZE_MAX * STATE_SIZE_MAX] = {};        // P matrix
    float observation_covariance[OBS_SIZE_MAX * OBS_SIZE_MAX] = {};          // S/O matrix
    float kalman_gain[STATE_SIZE_MAX * OBS_SIZE_MAX] = {};                   // K matrix
    float identity_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX] = {};             // Identity matrix

    // General temporary buffers (allocated to maximum sizes in header)
    float temp_buffer_a[TEMP_BUFFER_MAX] = {};
    float temp_buffer_b[TEMP_BUFFER_MAX] = {};

    // Internal state for online noise estimation (moved from static locals to per-instance)
    float noise_meas_mean = 0.0f;
    float noise_meas_var = 1.0f;
    float noise_proc_mean = 0.0f;
    float noise_proc_var = 0.1f;
    float noise_prev_pred0 = 0.0f;

    // Flags indicating whether full Q/R matrices have been provided via setters
    bool qMatrixSet = false;
    bool rMatrixSet = false;


};

#endif /* INC_KALMAN_FILTER_HPP_ */
