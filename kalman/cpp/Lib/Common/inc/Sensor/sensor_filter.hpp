#pragma once

#ifndef COMMON_SENSOR_SENSOR_FILTER_HPP
#define COMMON_SENSOR_SENSOR_FILTER_HPP

#include "../../../Matrix/fixed_matrix.hpp"
#include "../Math/statistics.hpp"
#include "../Math/geometry.hpp"
#include "../Math/numerical.hpp"
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

// ========== EMAフィルタ ==========
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
    
    cm get_value() const {
        return filtered_;
    }
    
    void reset() {
        initialized_ = false;
    }
    
    void reset_zero() {
        for(int i=0; i<filtered_.rows*filtered_.cols; ++i) filtered_.data[i] = 0.0f;
        initialized_ = true;
    }
    
    void set_alpha(float alpha) {
        alpha_ = fmaxf(0.0f, fminf(1.0f, alpha));
    }
};

// ========== Biquad ローパスフィルタ ==========
class BiquadLowpassFilter {
private:
    cm x1_, x2_;  // 入力履歴
    cm y1_, y2_;  // 出力履歴
    float b0_, b1_, b2_;  // 分子係数
    float a1_, a2_;       // 分母係数
    bool initialized_;
    
public:
    BiquadLowpassFilter() : initialized_(false) {
        b0_ = b1_ = b2_ = 0.0f;
        a1_ = a2_ = 0.0f;
    }
    
    void configure(float dt, float cutoff_freq) {
        // Biquad係数計算
        float omega = 2.0f * 3.14159265358979323846f * cutoff_freq;
        float K = tanf(omega * dt / 2.0f);
        float norm = 1.0f / (1.0f + K / 0.7071f + K * K);
        
        b0_ = K * K * norm;
        b1_ = 2.0f * b0_;
        b2_ = b0_;
        a1_ = 2.0f * (K * K - 1.0f) * norm;
        a2_ = (1.0f - K / 0.7071f + K * K) * norm;
        
        initialized_ = false;
    }
    
    cm filter(const cm& input) {
        if (!initialized_) {
            x1_ = input;
            x2_ = input;
            y1_ = input;
            y2_ = input;
            initialized_ = true;
            return input;
        }
        
        cm result;
        result.resize(input.rows, input.cols);
        
        for (int i = 0; i < input.rows; ++i) {
            for (int j = 0; j < input.cols; ++j) {
                result(i,j) = b0_ * input(i,j) + b1_ * x1_(i,j) + b2_ * x2_(i,j)
                            - a1_ * y1_(i,j) - a2_ * y2_(i,j);
            }
        }
        
        x2_ = x1_;
        x1_ = input;
        y2_ = y1_;
        y1_ = result;
        
        return result;
    }
    
    void reset() {
        initialized_ = false;
    }
};

// ========== Alpha-Betaフィルタ ==========
class AlphaBetaFilter {
private:
    cm position_;
    cm velocity_;
    float alpha_;
    float beta_;
    bool initialized_;
    
public:
    AlphaBetaFilter(float alpha = 0.5f, float beta = 0.1f) 
        : alpha_(alpha), beta_(beta), initialized_(false) {}
    
    void filter(const cm& measurement, float dt, cm& pos_out, cm& vel_out) {
        if (!initialized_) {
            position_ = measurement;
            velocity_.resize(measurement.rows, measurement.cols);
            for (int i = 0; i < velocity_.rows; ++i) {
                for (int j = 0; j < velocity_.cols; ++j) {
                    velocity_(i,j) = 0.0f;
                }
            }
            initialized_ = true;
            pos_out = position_;
            vel_out = velocity_;
            return;
        }
        
        // 予測
        cm pos_pred;
        pos_pred.resize(position_.rows, position_.cols);
        for (int i = 0; i < position_.rows; ++i) {
            for (int j = 0; j < position_.cols; ++j) {
                pos_pred(i,j) = position_(i,j) + velocity_(i,j) * dt;
            }
        }
        
        // 残差
        cm residual;
        residual.resize(measurement.rows, measurement.cols);
        for (int i = 0; i < residual.rows; ++i) {
            for (int j = 0; j < residual.cols; ++j) {
                residual(i,j) = measurement(i,j) - pos_pred(i,j);
            }
        }
        
        // 更新
        pos_out.resize(position_.rows, position_.cols);
        vel_out.resize(velocity_.rows, velocity_.cols);
        for (int i = 0; i < position_.rows; ++i) {
            for (int j = 0; j < position_.cols; ++j) {
                pos_out(i,j) = pos_pred(i,j) + alpha_ * residual(i,j);
                vel_out(i,j) = velocity_(i,j) + beta_ * residual(i,j) / dt;
            }
        }
        
        position_ = pos_out;
        velocity_ = vel_out;
    }
    
    void reset() {
        initialized_ = false;
    }
    
    void reset_zero() {
        position_.resize(0, 0);
        velocity_.resize(0, 0);
        initialized_ = false;
    }
    
    void set_parameters(float alpha, float beta) {
        alpha_ = alpha;
        beta_ = beta;
    }
};

// ========== 外れ値検出器 ==========
// 
// 【重要】MATLAB SensorAccelFilter.m とのパリティについて
// MATLAB側では履歴が空の場合、noise_estimate = residual_norm とし、
// 最初のデータは外れ値と判定されない（閾値が 3 * residual_norm になるため）。
// C++側でもこの動作を再現する必要がある。
// 
// 2025/12/22: 履歴が空の場合の動作をMATLAB側と一致させる修正
class OutlierDetector {
private:
    static constexpr int MAX_HISTORY = 20;
    float history_[MAX_HISTORY];
    int count_;
    
public:
    OutlierDetector() : count_(0) {}
    
    bool detect(float residual_norm, float threshold_sigma = 3.0f, float min_std = 0.1f) {
        float noise_std;
        
        if (count_ == 0) {
            // 履歴が空の場合: MATLAB側と同様に residual_norm を基準にする
            // これにより最初のデータは外れ値と判定されない
            noise_std = fmaxf(residual_norm, min_std);
        } else if (count_ < 5) {
            // 履歴が少ない場合: 現在の履歴の標準偏差を使用（MATLAB互換）
            float sum = 0.0f, sum_sq = 0.0f;
            for (int i = 0; i < count_; ++i) {
                sum += history_[i];
                sum_sq += history_[i] * history_[i];
            }
            float mean = sum / count_;
            float variance = sum_sq / count_ - mean * mean;
            noise_std = sqrtf(fmaxf(variance, 0.0f));
            noise_std = fmaxf(noise_std, fmaxf(residual_norm / 3.0f, min_std));
        } else {
            // 通常ケース: 履歴から標準偏差を計算
            float sum = 0.0f, sum_sq = 0.0f;
            for (int i = 0; i < count_; ++i) {
                sum += history_[i];
                sum_sq += history_[i] * history_[i];
            }
            float mean = sum / count_;
            noise_std = sqrtf(sum_sq / count_ - mean * mean);
            // MATLAB parity: noise_estimate = max(noise_std, residual_norm / 3.0)
            // This prevents noise_std from being too small when residuals are consistent
            noise_std = fmaxf(noise_std, fmaxf(residual_norm / 3.0f, min_std));
        }
        
        float threshold = threshold_sigma * noise_std;
        bool is_outlier = (residual_norm > threshold);

        // MATLAB parity: Only add to history if NOT an outlier
        // This prevents outliers from corrupting the noise estimate
        if (!is_outlier) {
            if (count_ < MAX_HISTORY) {
                history_[count_++] = residual_norm;
            } else {
                for (int i = 0; i < MAX_HISTORY - 1; ++i) {
                    history_[i] = history_[i+1];
                }
                history_[MAX_HISTORY-1] = residual_norm;
            }
        }

        return is_outlier;
    }

    // Mahalanobis-based静的判定: innovation とその共分散 S を与えて閾値判定
    // このメソッドは履歴を使わず、MathUtils の実装へ委譲します。
    static bool detect_mahalanobis_static(const common::math::cm& innovation, const common::math::cm& S, float threshold_sigma = 3.0f) {
        float dist_sq = kf::ops::mahalanobis_distance_squared(innovation, S);
        float dist = sqrtf(dist_sq);
        return dist > threshold_sigma;
    }
    
    void reset() {
        count_ = 0;
    }
};

// ========== ノイズ推定器 ==========
class NoiseEstimator {
private:
    static constexpr int MAX_WARMUP = 10;
    static constexpr float R_ABS_MIN = 1e-12f;
    static constexpr float R_ABS_MAX = 1e6f;
    static constexpr float OUTLIER_FACTOR = 20.0f;
    
    cm R_accel_, R_gyro_, R_mag_, R_gps_;
    float R_baro_;
    int count_accel_, count_gyro_, count_mag_, count_gps_, count_baro_;
    cm sum_accel_, sum_gyro_, sum_mag_, sum_gps_;
    float sum_baro_;
    float alpha_;
    int warmup_samples_;
    
public:
    NoiseEstimator(int warmup = 10) 
        : warmup_samples_(warmup), alpha_(0.01f),
          count_accel_(0), count_gyro_(0), count_mag_(0), count_gps_(0), count_baro_(0),
          R_baro_(1.0f), sum_baro_(0.0f) {
        
        // デフォルトノイズ値
        R_accel_.resize(3, 1); for(int i=0; i<3; i++) R_accel_(i,0) = 0.01f;
        R_gyro_.resize(3, 1); for(int i=0; i<3; i++) R_gyro_(i,0) = 1.74e-4f; // deg2rad(0.1)^2
        R_mag_.resize(3, 1); for(int i=0; i<3; i++) R_mag_(i,0) = 25.0f;
        R_gps_.resize(3, 1); R_gps_(0,0)=9.0f; R_gps_(1,0)=9.0f; R_gps_(2,0)=25.0f;
        
        sum_accel_.resize(3, 1); for(int i=0; i<3; i++) sum_accel_(i,0) = 0.0f;
        sum_gyro_.resize(3, 1); for(int i=0; i<3; i++) sum_gyro_(i,0) = 0.0f;
        sum_mag_.resize(3, 1); for(int i=0; i<3; i++) sum_mag_(i,0) = 0.0f;
        sum_gps_.resize(3, 1); for(int i=0; i<3; i++) sum_gps_(i,0) = 0.0f;
    }
    
    void estimate(const char* sensor_type, const cm& innovation, const cm& H, const cm& P_pred) {
        // イノベーション共分散成分計算
        cm HPHT; HPHT.resize(innovation.rows, innovation.rows);
        cm tmp; tmp.resize(H.rows, P_pred.cols);
        for(int i=0; i<H.rows; i++) {
            for(int j=0; j<P_pred.cols; j++) {
                float sum = 0.0f;
                for(int k=0; k<H.cols; k++) sum += H(i,k) * P_pred(k,j);
                tmp(i,j) = sum;
            }
        }
        for(int i=0; i<tmp.rows; i++) {
            for(int j=0; j<tmp.rows; j++) {
                float sum = 0.0f;
                for(int k=0; k<H.cols; k++) sum += tmp(i,k) * H(j,k);
                HPHT(i,j) = sum;
            }
        }
        
        cm innov_var; innov_var.resize(innovation.rows, 1);
        for(int i=0; i<innovation.rows; i++) {
            float innov_sq = innovation(i,0) * innovation(i,0);
            innov_var(i,0) = fmaxf(innov_sq - HPHT(i,i), R_ABS_MIN);
        }
        
        // センサータイプごとの更新
        if(strcmp(sensor_type, "accel") == 0) update_noise(R_accel_, count_accel_, sum_accel_, innov_var);
        else if(strcmp(sensor_type, "gyro") == 0) update_noise(R_gyro_, count_gyro_, sum_gyro_, innov_var);
        else if(strcmp(sensor_type, "mag") == 0) update_noise(R_mag_, count_mag_, sum_mag_, innov_var);
        else if(strcmp(sensor_type, "gps") == 0) update_noise(R_gps_, count_gps_, sum_gps_, innov_var);
        else if(strcmp(sensor_type, "baro") == 0) {
            float var = innov_var(0,0);
            count_baro_++;
            if(count_baro_ <= warmup_samples_) {
                sum_baro_ += var;
                R_baro_ = sum_baro_ / count_baro_;
            } else {
                float max_allowed = R_baro_ * OUTLIER_FACTOR;
                var = fminf(var, max_allowed);
                R_baro_ = (1.0f - alpha_) * R_baro_ + alpha_ * var;
            }
            R_baro_ = fminf(fmaxf(R_baro_, R_ABS_MIN), R_ABS_MAX);
        }
    }
    
    cm get_R_matrix(const char* sensor_type) {
        cm R;
        if(strcmp(sensor_type, "accel") == 0) {
            R.resize(3, 3);
            for(int i=0; i<3; i++) for(int j=0; j<3; j++) R(i,j) = (i==j) ? R_accel_(i,0) : 0.0f;
        } else if(strcmp(sensor_type, "gyro") == 0) {
            R.resize(3, 3);
            for(int i=0; i<3; i++) for(int j=0; j<3; j++) R(i,j) = (i==j) ? R_gyro_(i,0) : 0.0f;
        } else if(strcmp(sensor_type, "mag") == 0) {
            R.resize(3, 3);
            float mag_min = 0.01f;
            for(int i=0; i<3; i++) for(int j=0; j<3; j++) R(i,j) = (i==j) ? fmaxf(R_mag_(i,0), mag_min) : 0.0f;
        } else if(strcmp(sensor_type, "gps") == 0) {
            R.resize(3, 3);
            for(int i=0; i<3; i++) for(int j=0; j<3; j++) R(i,j) = (i==j) ? R_gps_(i,0) : 0.0f;
        } else if(strcmp(sensor_type, "baro") == 0) {
            R.resize(1, 1);
            R(0,0) = R_baro_;
        }
        // デバッグ出力（MEX 実行時に MATLAB コンソールへ）
        debug_print_R(sensor_type, R);
        return R;
    }

    // デバッグ用: R を出力するヘルパ
    void debug_print_R(const char* sensor_type, const cm& R) {
        // 出力は Results/noise_debug.txt に追記されます（MEX 実行時にコンソール出力も残す）
        try {
            std::ofstream nd("Results/noise_debug.txt", std::ios::app);
            if(!nd) {
                // fallback to mexPrintf
            } else {
                // timestamp and sample index
                uint64_t sample_idx = ++g_log_counter;
                auto now = std::chrono::system_clock::now();
                double ts = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
                if(strcmp(sensor_type, "gps") == 0 && R.rows >= 3) {
                    nd << "MARKER=NOISE_LOG sensor=gps sample=" << sample_idx << " time=" << ts << " R=" << R(0,0) << "," << R(1,1) << "," << R(2,2) << std::endl;
                } else if(strcmp(sensor_type, "baro") == 0 && R.rows >= 1) {
                    nd << "MARKER=NOISE_LOG sensor=baro sample=" << sample_idx << " time=" << ts << " R=" << R(0,0) << std::endl;
                } else if(R.rows > 0) {
                    nd << "MARKER=NOISE_LOG sensor=" << sensor_type << " sample=" << sample_idx << " time=" << ts << " R_diag=";
                    for(int i=0;i<R.rows;i++) {
                        nd << R(i,i);
                        if(i < R.rows-1) nd << ",";
                    }
                    nd << std::endl;
                }
            }
        } catch(...) {
            // ignore file errors
        }

        // 保険として MATLAB コンソールにも出す
        uint64_t sample_idx = ++g_log_counter;
        auto now = std::chrono::system_clock::now();
        double ts = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
        if(strcmp(sensor_type, "gps") == 0 && R.rows >= 3) {
            sensor_log("MARKER=NOISE_LOG sensor=gps sample=%llu time=%.9g R=%.9g,%.9g,%.9g\n", (unsigned long long)sample_idx, ts, R(0,0), R(1,1), R(2,2));
        } else if(strcmp(sensor_type, "baro") == 0 && R.rows >= 1) {
            sensor_log("MARKER=NOISE_LOG sensor=baro sample=%llu time=%.9g R=%.9g\n", (unsigned long long)sample_idx, ts, R(0,0));
        } else if(R.rows > 0) {
            // build CSV-style R_diag string
            std::string s = "MARKER=NOISE_LOG sensor=" + std::string(sensor_type) + " sample=" + std::to_string((unsigned long long)sample_idx) + " time=" + std::to_string(ts) + " R_diag=";
            for(int i=0;i<R.rows;i++) {
                s += std::to_string(R(i,i));
                if(i < R.rows-1) s += ",";
            }
            s += "\n";
            sensor_log("%s", s.c_str());
        }
    }
    
private:
    void update_noise(cm& R, int& count, cm& sum, const cm& innov_var) {
        count++;
        if(count <= warmup_samples_) {
            for(int i=0; i<R.rows; i++) sum(i,0) += innov_var(i,0);
            for(int i=0; i<R.rows; i++) R(i,0) = sum(i,0) / count;
        } else {
            for(int i=0; i<R.rows; i++) {
                float max_allowed = R(i,0) * OUTLIER_FACTOR;
                float var = fminf(innov_var(i,0), max_allowed);
                R(i,0) = (1.0f - alpha_) * R(i,0) + alpha_ * var;
            }
        }
        for(int i=0; i<R.rows; i++) {
            R(i,0) = fminf(fmaxf(R(i,0), R_ABS_MIN), R_ABS_MAX);
        }
    }
};

// ========== 発散防止 ==========
class DivergenceGuard {
private:
    static constexpr int MAX_SENSORS = 5;
    static constexpr int MAX_INNOV_HISTORY = 20;
    
    struct SensorHistory {
        cm prev_innovation;
        int update_count;
        bool valid;
        SensorHistory() : update_count(0), valid(false) {}
    };
    
    SensorHistory history_[MAX_SENSORS];
    
    float max_allowed_innov_;
    float innov_change_ratio_threshold_;
    float attenuation_factor_;
    float max_innov_cap_fraction_;
    float min_eigenvalue_factor_;
    float jitter_base_;
    
    int sensor_index(const char* name) {
        if(strcmp(name, "accel") == 0) return 0;
        if(strcmp(name, "gyro") == 0) return 1;
        if(strcmp(name, "mag") == 0) return 2;
        if(strcmp(name, "gps") == 0) return 3;
        if(strcmp(name, "baro") == 0) return 4;
        return -1;
    }
    
public:
    DivergenceGuard() 
        : max_allowed_innov_(50.0f), innov_change_ratio_threshold_(2.0f),
          attenuation_factor_(0.5f), max_innov_cap_fraction_(1.0f),
          min_eigenvalue_factor_(1e-8f), jitter_base_(1e-6f) {}
    
    bool check_and_attenuate(const char* sensor_name, cm& innovation, cm& dx, bool& was_attenuated) {
        was_attenuated = false;
        int idx = sensor_index(sensor_name);
        if(idx < 0) return false;
        
        float innov_norm = 0.0f;
        for(int i=0; i<innovation.rows; i++) innov_norm += innovation(i,0) * innovation(i,0);
        innov_norm = sqrtf(innov_norm);
        
        // 巨大な外れ値はスキップ
        if(innov_norm > max_allowed_innov_ * 1e6f) {
            try {
                std::ofstream dd("Results/divergence_debug.txt", std::ios::app);
                if(dd) {
                    uint64_t sample_idx = ++g_log_counter;
                    auto now = std::chrono::system_clock::now();
                    double ts = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
                    dd << "MARKER=DIV_LOG sensor=" << sensor_name << " sample=" << sample_idx << " time=" << ts << " reason=OUTLIER_SKIP innov_norm=" << innov_norm << std::endl;
                }
            } catch(...) {}
            sensor_log("MARKER=DIV_LOG sensor=%s reason=OUTLIER_SKIP innov_norm=%.9g\n", sensor_name, innov_norm);
            return true;
        }
        
        // イノベーション制限
        if(innov_norm > max_allowed_innov_) {
            float target_norm = max_allowed_innov_ * max_innov_cap_fraction_;
            if(target_norm <= 0.0f) target_norm = max_allowed_innov_;
            float scale = target_norm / innov_norm;
            for(int i=0; i<innovation.rows; i++) innovation(i,0) *= scale;
            innov_norm = target_norm;
            was_attenuated = true;
            try {
                std::ofstream dd("Results/divergence_debug.txt", std::ios::app);
                if(dd) {
                    uint64_t sample_idx = ++g_log_counter;
                    auto now = std::chrono::system_clock::now();
                    double ts = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
                    dd << "MARKER=DIV_LOG sensor=" << sensor_name << " sample=" << sample_idx << " time=" << ts << " reason=INNOV_CAP original_norm=" << (innov_norm/scale) << " capped_norm=" << innov_norm << std::endl;
                }
            } catch(...) {}
            sensor_log("MARKER=DIV_LOG sensor=%s reason=INNOV_CAP original_norm=%.9g capped_norm=%.9g\n", sensor_name, innov_norm/scale, innov_norm);
        }
        
        // 変化率チェック
        SensorHistory& hist = history_[idx];
        if(hist.valid && hist.update_count > 0) {
            float prev_norm = 0.0f;
            for(int i=0; i<hist.prev_innovation.rows; i++) prev_norm += hist.prev_innovation(i,0) * hist.prev_innovation(i,0);
            prev_norm = sqrtf(prev_norm);
            
            if(prev_norm > 1e-6f) {
                float change = 0.0f;
                for(int i=0; i<innovation.rows; i++) {
                    float diff = innovation(i,0) - hist.prev_innovation(i,0);
                    change += diff * diff;
                }
                change = sqrtf(change);
                float ratio = change / prev_norm;
                
                if(ratio > innov_change_ratio_threshold_) {
                    for(int i=0; i<dx.rows; i++) dx(i,0) *= attenuation_factor_;
                    was_attenuated = true;
                    try {
                        std::ofstream dd("Results/divergence_debug.txt", std::ios::app);
                        if(dd) {
                            uint64_t sample_idx = ++g_log_counter;
                            auto now = std::chrono::system_clock::now();
                            double ts = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
                            dd << "MARKER=DIV_LOG sensor=" << sensor_name << " sample=" << sample_idx << " time=" << ts << " reason=RATIO_ATTENUATE ratio=" << ratio << " attenuation=" << attenuation_factor_ << std::endl;
                        }
                    } catch(...) {}
                    sensor_log("MARKER=DIV_LOG sensor=%s reason=RATIO_ATTENUATE ratio=%.9g attenuation=%.9g\n", sensor_name, ratio, attenuation_factor_);
                }
            }
        }
        
        // 履歴更新
        hist.prev_innovation = innovation;
        hist.update_count++;
        hist.valid = true;
        
        return false;
    }
    
    void regularize_covariance(cm& P) {
        int n = P.rows;
        
        // 対称性強制
        for(int i=0; i<n; i++) {
            for(int j=i+1; j<n; j++) {
                float avg = (P(i,j) + P(j,i)) * 0.5f;
                P(i,j) = avg;
                P(j,i) = avg;
            }
        }
        
        // ジッターと最小対角値の保証
        float max_diag = 0.0f;
        for(int i=0; i<n; i++) max_diag = fmaxf(max_diag, fabsf(P(i,i)));
        if(max_diag < 1e-20f) max_diag = 1e-20f;

        // 絶対的最小値と相対的最小値の両方を用いて下限を決定する。
        // 小さいスケールの共分散（例えば diag(~1e-12)）でも十分に大きな下限を与えるため、
        // 絶対下限 eps_abs を設ける。
        const float eps_abs = 1e-9f;
        const float eps_rel = min_eigenvalue_factor_; // class member, default 1e-8f
        float min_diag = fmaxf(eps_abs, eps_rel * max_diag);

        // 基本的なジッターは最大対角要素比例で加える（小さすぎる行列に対しては最小下限を適用）
        float jitter = jitter_base_ * max_diag;
        for(int i=0; i<n; i++) {
            if(P(i,i) < min_diag) P(i,i) = min_diag;
            else P(i,i) += jitter;
        }

        // 必要なら状態別キャップ（15次元ESKF用）
        if(n == 15) {
            float caps[15] = {1e6f, 1e6f, 1e6f,  // pos
                              1e4f, 1e4f, 1e4f,  // vel
                              100.0f, 100.0f, 100.0f,  // att
                              1e2f, 1e2f, 1e2f,  // ba
                              1e-1f, 1e-1f, 1e-1f}; // bg
            for(int i=0; i<15; i++) P(i,i) = fminf(P(i,i), caps[i]);
        }

        // rcond に相当する簡易チェック: 対角の最小/最大比を用いて条件悪化を検出し、ブーストを追加
        {
            float max_diag2 = 0.0f;
            float min_diag2 = FLT_MAX;
            for(int i=0; i<n; i++) {
                float v = fabsf(P(i,i));
                max_diag2 = fmaxf(max_diag2, v);
                min_diag2 = fminf(min_diag2, v);
            }
            if(max_diag2 < 1e-20f) max_diag2 = 1e-20f;
            if(min_diag2 < 1e-20f) min_diag2 = 1e-20f;
            float rcond_est = min_diag2 / max_diag2;
            const float min_rcond = 1e-12f;
            if(rcond_est < min_rcond) {
                float boost = 1e-8f * max_diag2;
                for(int i=0; i<n; i++) P(i,i) += boost;
            }
        }

        // 非常に小さい要素をゼロ化して数値ノイズを抑える
        const float tiny_thresh = 1e-15f;
        for(int i=0; i<n; i++) {
            for(int j=0; j<n; j++) {
                if(fabsf(P(i,j)) < tiny_thresh) P(i,j) = 0.0f;
            }
        }

        // 最終対称化
        for(int i=0; i<n; i++) {
            for(int j=i+1; j<n; j++) {
                float avg = (P(i,j) + P(j,i)) * 0.5f;
                P(i,j) = avg;
                P(j,i) = avg;
            }
        }
    }
    
    void clip_state_change(cm& dx) {
        if(dx.rows < 15) return;
        
        float max_pos = 10.0f, max_vel = 5.0f, max_att = 0.5f, max_ba = 0.5f, max_bg = 0.1f;
        
        float pos_norm = 0.0f; for(int i=0; i<3; i++) pos_norm += dx(i,0) * dx(i,0);
        pos_norm = sqrtf(pos_norm);
        if(pos_norm > max_pos) { float s = max_pos/pos_norm; for(int i=0; i<3; i++) dx(i,0) *= s; }
        
        float vel_norm = 0.0f; for(int i=3; i<6; i++) vel_norm += dx(i,0) * dx(i,0);
        vel_norm = sqrtf(vel_norm);
        if(vel_norm > max_vel) { float s = max_vel/vel_norm; for(int i=3; i<6; i++) dx(i,0) *= s; }
        
        float att_norm = 0.0f; for(int i=6; i<9; i++) att_norm += dx(i,0) * dx(i,0);
        att_norm = sqrtf(att_norm);
        if(att_norm > max_att) { float s = max_att/att_norm; for(int i=6; i<9; i++) dx(i,0) *= s; }
        
        float ba_norm = 0.0f; for(int i=9; i<12; i++) ba_norm += dx(i,0) * dx(i,0);
        ba_norm = sqrtf(ba_norm);
        if(ba_norm > max_ba) { float s = max_ba/ba_norm; for(int i=9; i<12; i++) dx(i,0) *= s; }
        
        float bg_norm = 0.0f; for(int i=12; i<15; i++) bg_norm += dx(i,0) * dx(i,0);
        bg_norm = sqrtf(bg_norm);
        if(bg_norm > max_bg) { float s = max_bg/bg_norm; for(int i=12; i<15; i++) dx(i,0) *= s; }
    }
};

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
        float a_norm = sqrtf(a_norm_sq);
        
        if (a_norm < gravity_range_min_ || a_norm > gravity_range_max_) {
            is_outlier = true;
            // 外れ値の場合は前回のフィルタ済み値を返す
            return accel_filter.get_value();
        }
        
            // イノベーション（3x1）を作成し、統一 Mahalanobis 判定へ委譲
            common::math::cm innov; innov.resize(3,1);
            for (int i = 0; i < 3; ++i) innov(i,0) = a_meas(i,0) - a_expected(i,0);

            // センサーのノイズ推定から R を取得して S として用いる（簡易モデル）
            common::math::cm Rmat = noise_estimator.get_R_matrix("accel");
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
        residual_norm = sqrtf(residual_norm);
        
            // イノベーション（3x1）を作成し、統一 Mahalanobis 判定へ委譲
            common::math::cm innov; innov.resize(3,1);
            for (int i = 0; i < 3; ++i) innov(i,0) = m_meas(i,0) - m_expected(i,0);
            common::math::cm Rmat = noise_estimator.get_R_matrix("mag");
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

