/*
 * kalman.cpp
 *
 *  Created on: Aug 12, 2025
 *      Author: takut
 */

#include "../inc/kalman_filter.hpp"



// Lightweight Kalman filter implementation for altitude estimation.
// Uses only raw arrays and simple loops. Keeps public API (Init/Update/GetData/EstimateNoise).

void KalmanFilter::Init(uint8_t state_size, uint8_t obs_size) {

    STATE_SIZE = state_size;
    OBS_SIZE = obs_size;

    // Clear arrays (only up to max sizes defined in header)
    for (uint8_t i = 0; i < (STATE_SIZE_MAX * STATE_SIZE_MAX); ++i) {
        prediction_covariance[i] = 0.0f;
        prediction_noise_matrix[i] = 0.0f;
        identity_matrix[i] = 0.0f;
        system_matrix[i] = 0.0f;
    }
    for (uint8_t i = 0; i < (OBS_SIZE_MAX * OBS_SIZE_MAX); ++i) {
        observation_noise_matrix[i] = 0.0f;
        observation_covariance[i] = 0.0f;
    }
    for (uint8_t i = 0; i < (STATE_SIZE_MAX * OBS_SIZE_MAX); ++i) {
        kalman_gain[i] = 0.0f;
    }
    for (uint8_t i = 0; i < STATE_SIZE_MAX; ++i) {
        prediction[i] = 0.0f;
        output[i] = 0.0f;
    }
    for (uint8_t i = 0; i < OBS_SIZE_MAX; ++i) {
        observation[i] = 0.0f;
    }

    // Initialize P and I diagonal to 1.0 for active state size
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        prediction_covariance[i * STATE_SIZE + i] = 1.0f;
        identity_matrix[i * STATE_SIZE + i] = 1.0f;
    }
}


void KalmanFilter::Update() {
    // Update Q and R diagonal entries
    // If full Q/R matrices are not set by caller, fill diagonals from scalar members
    if (!qMatrixSet) {
        for (uint8_t i = 0; i < STATE_SIZE; ++i) {
            // zero full matrix first to avoid residuals
            for (uint8_t j = 0; j < STATE_SIZE; ++j) prediction_noise_matrix[i * STATE_SIZE + j] = 0.0f;
            prediction_noise_matrix[i * STATE_SIZE + i] = prediction_noise;
        }
    }
    if (!rMatrixSet) {
        for (uint8_t i = 0; i < OBS_SIZE; ++i) {
            for (uint8_t j = 0; j < OBS_SIZE; ++j) observation_noise_matrix[i * OBS_SIZE + j] = 0.0f;
            observation_noise_matrix[i * OBS_SIZE + i] = observation_noise;
        }
    }

    const uint8_t s = STATE_SIZE;
    const uint8_t o = OBS_SIZE;

    // Temporary buffers (fixed max size)
    float tmpA[STATE_SIZE_MAX * STATE_SIZE_MAX];
    float tmpB[STATE_SIZE_MAX * STATE_SIZE_MAX];
    for (uint8_t i = 0; i < (STATE_SIZE_MAX * STATE_SIZE_MAX); ++i) { tmpA[i] = 0.0f; tmpB[i] = 0.0f; }

    // ---------- P = F * P * F^T + Q ----------
    // tmpA = F * P
    for (uint8_t i = 0; i < s; ++i) {
        for (uint8_t j = 0; j < s; ++j) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < s; ++k) {
                sum += system_matrix[i * s + k] * prediction_covariance[k * s + j];
            }
            tmpA[i * s + j] = sum;
        }
    }
    // tmpB = tmpA * F^T -> newP
    for (uint8_t i = 0; i < s; ++i) {
        for (uint8_t j = 0; j < s; ++j) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < s; ++k) {
                sum += tmpA[i * s + k] * system_matrix[j * s + k];
            }
            tmpB[i * s + j] = sum + prediction_noise_matrix[i * s + j];
        }
    }
    // copy back to P
    for (uint8_t i = 0; i < (s * s); ++i) prediction_covariance[i] = tmpB[i];

    // ---------- SO = H * P * H^T + R ----------
    // Since OBS_SIZE is 1 in this project, handle scalar path efficiently
    float so = 0.0f;
    if (o == 1) {
        // SO = H * P * H^T + R (scalar)
        for (uint8_t i = 0; i < s; ++i) {
            for (uint8_t j = 0; j < s; ++j) {
                so += observation_matrix[0 * s + i] * prediction_covariance[i * s + j] * observation_matrix[0 * s + j];
            }
        }
        so += observation_noise_matrix[0];
        observation_covariance[0] = so;
        // Simple NaN/Inf guard and regularization for scalar case
        if (!isfinite(so) || fabsf(so) < 1e-8f) {
            const float EPS_REG = 1e-6f;
            so = (isfinite(so) ? so : 0.0f) + EPS_REG;
            observation_covariance[0] = so;
        }
    } else {
        // general (rare) path: compute SO matrix
        for (uint8_t i = 0; i < (o * o); ++i) observation_covariance[i] = 0.0f;
        for (uint8_t ii = 0; ii < o; ++ii) {
            for (uint8_t jj = 0; jj < o; ++jj) {
                float sum = 0.0f;
                for (uint8_t m = 0; m < s; ++m) {
                    for (uint8_t n = 0; n < s; ++n) {
                        sum += observation_matrix[ii * s + m] * prediction_covariance[m * s + n] * observation_matrix[jj * s + n];
                    }
                }
                observation_covariance[ii * o + jj] = sum + observation_noise_matrix[ii * o + jj];
            }
        }
        // Simple regularization for full S: ensure diagonal entries are not too small and finite
        for (uint8_t i = 0; i < o; ++i) {
            float diag = observation_covariance[i * o + i];
            if (!isfinite(diag) || fabsf(diag) < 1e-8f) {
                observation_covariance[i * o + i] = (isfinite(diag) ? diag : 0.0f) + 1e-6f;
            }
        }
    }

    // ---------- K = P * H^T * SO^-1 ----------
    // For obs=1, use scalar inverse
    if (o == 1) {
        const float EPS = 1e-6f;
        float inv_so = (fabsf(so) > EPS) ? (1.0f / so) : (1.0f / EPS);
        // temp = P * H^T -> state x 1
        float temp_ph[STATE_SIZE_MAX];
        for (uint8_t i = 0; i < s; ++i) {
            float sum = 0.0f;
            for (uint8_t j = 0; j < s; ++j) {
                sum += prediction_covariance[i * s + j] * observation_matrix[0 * s + j];
            }
            temp_ph[i] = sum;
        }
        // K = temp_ph * inv_so
        for (uint8_t i = 0; i < s; ++i) {
            kalman_gain[i * o + 0] = temp_ph[i] * inv_so;
        }
    } else {
        // o>1: K = P H^T S^{-1} using Cholesky
        // PHt = P * H^T (s x o)
        float PHt[STATE_SIZE_MAX * OBS_SIZE_MAX];
        for (uint8_t i = 0; i < s; ++i) {
            for (uint8_t j = 0; j < o; ++j) {
                float sum = 0.0f;
                for (uint8_t k = 0; k < s; ++k) sum += prediction_covariance[i * s + k] * observation_matrix[j * s + k];
                PHt[i * o + j] = sum;
            }
        }
        // Cholesky decomposition of S: S = L L^T
        float L[OBS_SIZE_MAX * OBS_SIZE_MAX];
        for (uint8_t i = 0; i < o * o; ++i) L[i] = 0.0f;
        for (uint8_t i = 0; i < o; ++i) {
            for (uint8_t j = 0; j <= i; ++j) {
                float sum = observation_covariance[i * o + j];
                for (uint8_t k = 0; k < j; ++k) sum -= L[i * o + k] * L[j * o + k];
                L[i * o + j] = (i == j) ? sqrtf(sum > 1e-10f ? sum : 1e-10f) : sum / L[j * o + j];
            }
        }
        // Solve L Y = PHt^T for Y (o x s), then L^T K^T = Y
        for (uint8_t col = 0; col < s; ++col) {
            float y[OBS_SIZE_MAX];
            for (uint8_t i = 0; i < o; ++i) {
                float sum = PHt[col * o + i];
                for (uint8_t j = 0; j < i; ++j) sum -= L[i * o + j] * y[j];
                y[i] = sum / L[i * o + i];
            }
            for (int i = o - 1; i >= 0; --i) {
                float sum = y[i];
                for (uint8_t j = i + 1; j < o; ++j) sum -= L[j * o + i] * kalman_gain[col * o + j];
                kalman_gain[col * o + i] = sum / L[i * o + i];
            }
        }
    }

    // ---------- output = prediction + K * (observation - H * prediction) ----------
    // compute H * prediction (obs vector)
    float hpred[OBS_SIZE_MAX];
    for (uint8_t ii = 0; ii < o; ++ii) {
        float sum = 0.0f;
        for (uint8_t j = 0; j < s; ++j) sum += observation_matrix[ii * s + j] * prediction[j];
        hpred[ii] = sum;
    }
    // residual z - H*pred
    float resid[OBS_SIZE_MAX];
    for (uint8_t ii = 0; ii < o; ++ii) resid[ii] = observation[ii] - hpred[ii];
    // delta = K * resid
    float delta[STATE_SIZE_MAX];
    for (uint8_t i = 0; i < s; ++i) {
        float sum = 0.0f;
        for (uint8_t ii = 0; ii < o; ++ii) sum += kalman_gain[i * o + ii] * resid[ii];
        delta[i] = sum;
    }
    // OUT = PRE + delta -> store in output[]
    for (uint8_t i = 0; i < s; ++i) {
        output[i] = prediction[i] + delta[i];
    }

    // ---------- P = (I - K * H) * P ----------
    // Compute (I - K*H)
    float ImKH[STATE_SIZE_MAX * STATE_SIZE_MAX];
    for (uint8_t i = 0; i < s; ++i) {
        for (uint8_t j = 0; j < s; ++j) {
            float kh = 0.0f;
            for (uint8_t ii = 0; ii < o; ++ii) {
                kh += kalman_gain[i * o + ii] * observation_matrix[ii * s + j];
            }
            ImKH[i * s + j] = identity_matrix[i * s + j] - kh;
        }
    }
    // newP = ImKH * P
    for (uint8_t i = 0; i < s; ++i) {
        for (uint8_t j = 0; j < s; ++j) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < s; ++k) sum += ImKH[i * s + k] * prediction_covariance[k * s + j];
            tmpA[i * s + j] = sum;
        }
    }
    for (uint8_t i = 0; i < (s * s); ++i) {
        prediction_covariance[i] = tmpA[i];
    }
}

void KalmanFilter::GetData(float* out_states) {
    for (int i = 0; i < STATE_SIZE; i++) {
        out_states[i] = output[i];
    }
}


// Simple online noise estimation moved from Altitude; keeps EWMA of measurement and
// prediction variances and updates observation_noise and prediction_noise members.
void KalmanFilter::EstimateNoise(float meas) {
    const float EPS_NOISE = 1e-6f;
    // measurement mean/variance (EWMA) using instance members
    noise_meas_mean = (1.0f - NOISE_ALPHA) * noise_meas_mean + NOISE_ALPHA * meas;
    float v = meas - noise_meas_mean;
    noise_meas_var = (1.0f - NOISE_ALPHA) * noise_meas_var + NOISE_ALPHA * v * v;

    // prediction error proxy: use prediction[0] (prediction vector first element)
    float pred0 = prediction[0];
    float pred_err = pred0 - noise_prev_pred0;
    noise_proc_mean = (1.0f - NOISE_ALPHA) * noise_proc_mean + NOISE_ALPHA * pred_err;
    float pv = pred_err - noise_proc_mean;
    noise_proc_var = (1.0f - NOISE_ALPHA) * noise_proc_var + NOISE_ALPHA * pv * pv;

    // clamp and reflect into Kalman matrices
    if (noise_meas_var > EPS_NOISE) {
        observation_noise = noise_meas_var;
    } else {
        observation_noise = EPS_NOISE;
    }
    if (noise_proc_var > EPS_NOISE) {
        prediction_noise = noise_proc_var;
    } else {
        prediction_noise = EPS_NOISE;
    }

    noise_prev_pred0 = pred0;
}

// Set full Q matrix (row-major). Size is STATE_SIZE x STATE_SIZE.
void KalmanFilter::SetQMatrix(const float* Q) {
    if (Q == nullptr) return;
    for (uint8_t i = 0; i < STATE_SIZE; ++i) {
        for (uint8_t j = 0; j < STATE_SIZE; ++j) {
            prediction_noise_matrix[i * STATE_SIZE + j] = Q[i * STATE_SIZE + j];
        }
    }
    qMatrixSet = true;
}

// Set full R matrix (row-major). Size is OBS_SIZE x OBS_SIZE.
void KalmanFilter::SetRMatrix(const float* R) {
    if (R == nullptr) return;
    for (uint8_t i = 0; i < OBS_SIZE; ++i) {
        for (uint8_t j = 0; j < OBS_SIZE; ++j) {
            observation_noise_matrix[i * OBS_SIZE + j] = R[i * OBS_SIZE + j];
        }
    }
    rMatrixSet = true;
}



