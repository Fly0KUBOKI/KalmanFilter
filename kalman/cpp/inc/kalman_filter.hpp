/*
 * kalman.hpp
 *
 *  Created on: Aug 21, 2025
 *      Author: takut
 */

#ifndef INC_KALMAN_FILTER_HPP_
#define INC_KALMAN_FILTER_HPP_

#include <cstdint>
#include "math.h"

// EWMA alpha for noise estimation
#define NOISE_ALPHA 0.02f

// フィルタの最大サイズ定義
#define STATE_SIZE_MAX    10    // 状態ベクトルの最大サイズ
#define OBS_SIZE_MAX      10    // 観測ベクトルの最大サイズ

// テンポラリバッファの最大サイズ（state x state が最大想定）
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

    // 入力パラメータ
    float observation[OBS_SIZE_MAX]={};                // 観測値
    float prediction[STATE_SIZE_MAX]={};               // 予測値


    float system_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX]={};    // F
    float observation_matrix[OBS_SIZE_MAX * STATE_SIZE_MAX]={}; // H



private:
    uint8_t STATE_SIZE = 0;
    uint8_t OBS_SIZE = 0;

    // 保存する変数
    float observation_noise = 0.0f;                         // 観測ノイズ
    float prediction_noise = 0.0f;                          // 予測ノイズ
    float output[STATE_SIZE_MAX]={};                        // 出力値
    float prediction_noise_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX] = {};      // Q
    float observation_noise_matrix[OBS_SIZE_MAX * OBS_SIZE_MAX] = {};         // R
    float prediction_covariance[STATE_SIZE_MAX * STATE_SIZE_MAX] = {};        // P
    float observation_covariance[OBS_SIZE_MAX * OBS_SIZE_MAX] = {};          // SO
    float kalman_gain[STATE_SIZE_MAX * OBS_SIZE_MAX] = {};                   // K
    float identity_matrix[STATE_SIZE_MAX * STATE_SIZE_MAX] = {};             // I

    // 汎用テンポラリバッファ（ヘッダでは最大サイズで確保）
    float temp_buffer_a[TEMP_BUFFER_MAX] = {};
    float temp_buffer_b[TEMP_BUFFER_MAX] = {};

    #if KALMAN_USE_SMOOTHER
    float smooth_output[STATE_SIZE_MAX]={};            // 平滑化後の出力
    float smooth_buffer[STATE_SIZE_MAX][SMOOTHER_WINDOW_SIZE] = {};       // 直近N回分のバッファ（移動平均用）
    int smooth_index = 0;

    // スムーザー関数
    void Smoother();
    #endif

};



#endif /* INC_KALMAN_FILTER_HPP_ */
