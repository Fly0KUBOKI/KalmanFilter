#pragma once

#ifndef COMMON_SENSOR_SENSOR_FILTER_HPP
#define COMMON_SENSOR_SENSOR_FILTER_HPP

#include "../../../Matrix/fixed_matrix.hpp"
#include "../../../KF/inc/kf_operations.hpp"
#include "../Math/statistics.hpp"
#include "../Math/geometry.hpp"
#include "../Math/numerical.hpp"
#include "../Math/portable_math.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cfloat>
#include <cstdio>
#include <string>
// mexPrintf を使ったログ出力（MEX ビルド時）
#ifdef MATLAB_MEX_FILE
# include "mex.h"
#else
# include <stdio.h>
# define mexPrintf printf
#endif

#include "ema_filter.hpp"
#include "biquad_filter.hpp"
#include "alpha_beta_filter.hpp"
#include "robust_statistics.hpp"
#include "outlier_detector.hpp"

namespace common {
namespace sensor {

// グローバルのログカウンタ（各ログ呼び出しごとにインクリメントされます）
static std::atomic<uint64_t> g_log_counter{0};

// ログ出力のグローバル制御フラグ (false = ログ無効)
static std::atomic<bool> g_enable_sensor_logging{false};

// センサーログ用の安全な printf ラッパー
inline void sensor_log_enable(bool en) { g_enable_sensor_logging.store(en); }

inline void sensor_log(const char* fmt, ...) {
    if(!g_enable_sensor_logging.load()) return;
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    mexPrintf("%s", buf);
}


using cm = cmath_fx::FixedMatrix;

// EMA/Biquad/AlphaBeta/Robust stats are provided by modular headers included above.

// AlphaBetaFilter is provided by modular header included above

// OutlierDetector is included above

// Noise/Divergence classes provided by modular header included above

// ========== センサーフィルタ統合クラス ==========
class SensorFilterLib {
public:
    EMAFilter accel_filter;
    EMAFilter mag_filter;
    AlphaBetaFilter gps_filter;
    EMAFilter baro_filter;
    
    OutlierDetector accel_outlier;
    OutlierDetector mag_outlier;
    // configurable parameters for accel outlier detection
    float accel_threshold_sigma_;
    float accel_min_std_;
    // gravity norm validation range (matches MATLAB SensorAccelFilter defaults)
    float gravity_range_min_;
    float gravity_range_max_;
    
    NoiseEstimator noise_estimator;
    DivergenceGuard divergence_guard;
    
    SensorFilterLib() : noise_estimator(10), accel_threshold_sigma_(3.0f), accel_min_std_(0.1f),
                        gravity_range_min_(8.5f), gravity_range_max_(10.5f) {
        accel_filter.set_alpha(0.3f);
        mag_filter.set_alpha(0.2f);
        baro_filter.set_alpha(0.4f);
        gps_filter.set_parameters(0.5f, 0.1f);
    }
    
    void reset_all() {
        accel_filter.reset();
        mag_filter.reset();
        baro_filter.reset();
        gps_filter.reset();
        accel_outlier.reset();
        mag_outlier.reset();
    }
    
    void reset_all_zero() {
        accel_filter.reset_zero();
        mag_filter.reset_zero();
        baro_filter.reset_zero();
        gps_filter.reset_zero();
        accel_outlier.reset();
        mag_outlier.reset();
    }

    void set_accel_config(float ema_alpha, int history_size, float threshold_sigma, float min_std) {
        accel_filter.set_alpha(ema_alpha);
        accel_threshold_sigma_ = threshold_sigma;
        accel_min_std_ = min_std;
        (void)history_size; // history_size not used (fixed MAX_HISTORY)
    }
    
    // 加速度フィルタ（外れ値検出付き）
    // 
    // 【重要】重力ノルム検証について
    // この関数はMATLAB側のSensorAccelFilter.mと完全にパリティを保つ必要がある。
    // MATLAB側では gravity_range = [8.5, 10.5] の範囲外の加速度は外れ値として棄却される。
    // この検証が欠落すると、異常な加速度データがMEUKFに渡され、姿勢推定（Roll/Pitch）が
    // 大きく劣化する（RMSE 0.27deg → 1.6deg程度）。
    // 
    // 2025/12/22: コメントアウトを解除し、MATLAB側と同一のロジックを有効化
    cm filter_accel(const cm& a_meas, const cm& a_expected, bool& is_outlier) {
        // === 重力ノルム検証 (MATLAB SensorAccelFilter.m と同一) ===
        // 加速度ノルムが [gravity_range_min_, gravity_range_max_] 範囲外なら外れ値として棄却
        float a_norm_sq = 0.0f;
        for (int i = 0; i < 3; ++i) {
            a_norm_sq += a_meas(i,0) * a_meas(i,0);
        }
        float a_norm = common::math::portable_sqrt(a_norm_sq);
        
        if (a_norm < gravity_range_min_ || a_norm > gravity_range_max_) {
            is_outlier = true;
            // 外れ値の場合は前回のフィルタ済み値を返す
            return accel_filter.get_value();
        }
        
            // イノベーション（3x1）を作成し、統一 Mahalanobis 判定へ委譲
            ::common::math::cm innov; innov.resize(3,1);
            for (int i = 0; i < 3; ++i) innov(i,0) = a_meas(i,0) - a_expected(i,0);

            // センサーのノイズ推定から R を取得して S として用いる（簡易モデル）
            ::common::math::cm Rmat = noise_estimator.get_R_matrix("accel");
            // Rmat は diag 形式で返ってくる想定
            is_outlier = accel_outlier.detect_mahalanobis_static(innov, Rmat, accel_threshold_sigma_);
        
        if (is_outlier) {
            // 外れ値の場合は前回値を返す
            return accel_filter.get_value();
        } else {
            return accel_filter.filter(a_meas);
        }
    }
    
    cm filter_mag(const cm& m_meas, const cm& m_expected, bool& is_outlier) {
        float residual_norm = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float diff = m_meas(i,0) - m_expected(i,0);
            residual_norm += diff * diff;
        }
        residual_norm = common::math::portable_sqrt(residual_norm);
        
            // イノベーション（3x1）を作成し、統一 Mahalanobis 判定へ委譲
            ::common::math::cm innov; innov.resize(3,1);
            for (int i = 0; i < 3; ++i) innov(i,0) = m_meas(i,0) - m_expected(i,0);
            ::common::math::cm Rmat = noise_estimator.get_R_matrix("mag");
            is_outlier = mag_outlier.detect_mahalanobis_static(innov, Rmat, 3.0f);
        
        return mag_filter.filter(m_meas);
    }
    
    void filter_gps(const cm& gps_pos, float dt, cm& pos_out, cm& vel_out) {
        gps_filter.filter(gps_pos, dt, pos_out, vel_out);
    }
    
    float filter_baro(float pressure) {
        cm p_in; p_in.resize(1,1); p_in(0,0) = pressure;
        cm p_out = baro_filter.filter(p_in);
        return p_out(0,0);
    }
};

} // namespace sensor
} // namespace common

#endif // COMMON_SENSOR_SENSOR_FILTER_HPP

