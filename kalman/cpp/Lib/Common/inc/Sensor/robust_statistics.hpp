#pragma once

#ifndef COMMON_SENSOR_ROBUST_STATISTICS_HPP
#define COMMON_SENSOR_ROBUST_STATISTICS_HPP

#include "../../../Matrix/fixed_matrix.hpp"
#include <fstream>
#include <chrono>
#include <string>
#include <cmath>
#include <cfloat>

using cm = cmath_fx::FixedMatrix;

// ========== ノイズ推定器 ==========
class NoiseEstimator {
private:
    static const int MAX_WARMUP = 10;
    static const float R_ABS_MIN;
    static const float R_ABS_MAX;
    static const float OUTLIER_FACTOR;
    
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
        R_gyro_.resize(3, 1); for(int i=0; i<3; i++) R_gyro_(i,0) = 1.74e-4f;
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
        // デバッグ出力（呼び出し側で sensor_log/g_log_counter が定義されていることを想定）
        return R;
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
    static const int MAX_SENSORS = 5;
    
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
        
        float max_diag = 0.0f;
        for(int i=0; i<n; i++) max_diag = fmaxf(max_diag, fabsf(P(i,i)));
        if(max_diag < 1e-20f) max_diag = 1e-20f;

        const float eps_abs = 1e-9f;
        const float eps_rel = min_eigenvalue_factor_;
        float min_diag = fmaxf(eps_abs, eps_rel * max_diag);

        float jitter = jitter_base_ * max_diag;
        for(int i=0; i<n; i++) {
            if(P(i,i) < min_diag) P(i,i) = min_diag;
            else P(i,i) += jitter;
        }

        if(n == 15) {
            float caps[15] = {1e6f, 1e6f, 1e6f,
                              1e4f, 1e4f, 1e4f,
                              100.0f, 100.0f, 100.0f,
                              1e2f, 1e2f, 1e2f,
                              1e-1f, 1e-1f, 1e-1f};
            for(int i=0; i<15; i++) P(i,i) = fminf(P(i,i), caps[i]);
        }

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

        const float tiny_thresh = 1e-15f;
        for(int i=0; i<n; i++) {
            for(int j=0; j<n; j++) {
                if(fabsf(P(i,j)) < tiny_thresh) P(i,j) = 0.0f;
            }
        }

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

#endif // COMMON_SENSOR_ROBUST_STATISTICS_HPP
#pragma once

#ifndef COMMON_SENSOR_ROBUST_STATISTICS_HPP
#define COMMON_SENSOR_ROBUST_STATISTICS_HPP

/**
 * @file robust_statistics.hpp
 * @brief Robust noise estimation and divergence prevention
 * 
 * Provides adaptive noise covariance estimation and divergence guards
 * for Kalman filter stability.
 */

#include "../../Matrix/fixed_matrix.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <cstring>
#include <fstream>
#include <atomic>
#include <chrono>
#include <cfloat>

namespace common {
namespace sensor {

using cm = cmath_fx::FixedMatrix;

// Forward declarations for logging
extern std::atomic<uint64_t> g_log_counter;
extern void sensor_log(const char* fmt, ...);

// ========== ノイズ推定器 ==========
class NoiseEstimator {
private:
    static const int MAX_WARMUP;
    static const float R_ABS_MIN;
    static const float R_ABS_MAX;
    static const float OUTLIER_FACTOR;
    
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
        // 出力は Results/noise_debug.txt に追記されます
        try {
            std::ofstream nd("Results/noise_debug.txt", std::ios::app);
            if(!nd) {
                // fallback
            } else {
                uint64_t sample_idx = ++g_log_counter;
                std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
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
        } catch(...) {}
        
        uint64_t sample_idx = ++g_log_counter;
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        double ts = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
        if(strcmp(sensor_type, "gps") == 0 && R.rows >= 3) {
            sensor_log("MARKER=NOISE_LOG sensor=gps sample=%llu time=%.9g R=%.9g,%.9g,%.9g\n", (unsigned long long)sample_idx, ts, R(0,0), R(1,1), R(2,2));
        } else if(strcmp(sensor_type, "baro") == 0 && R.rows >= 1) {
            sensor_log("MARKER=NOISE_LOG sensor=baro sample=%llu time=%.9g R=%.9g\n", (unsigned long long)sample_idx, ts, R(0,0));
        } else if(R.rows > 0) {
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
    static const int MAX_SENSORS;
    static const int MAX_INNOV_HISTORY;
    
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
                    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
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
                    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
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
                            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
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

        const float eps_abs = 1e-9f;
        const float eps_rel = min_eigenvalue_factor_;
        float min_diag = fmaxf(eps_abs, eps_rel * max_diag);

        float jitter = jitter_base_ * max_diag;
        for(int i=0; i<n; i++) {
            if(P(i,i) < min_diag) P(i,i) = min_diag;
            else P(i,i) += jitter;
        }

        // 15次元ESKF用キャップ
        if(n == 15) {
            float caps[15] = {1e6f, 1e6f, 1e6f,  // pos
                              1e4f, 1e4f, 1e4f,  // vel
                              100.0f, 100.0f, 100.0f,  // att
                              1e2f, 1e2f, 1e2f,  // ba
                              1e-1f, 1e-1f, 1e-1f}; // bg
            for(int i=0; i<15; i++) P(i,i) = fminf(P(i,i), caps[i]);
        }

        // rcond チェック
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

        // 微小要素のゼロ化
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

} // namespace sensor
} // namespace common

#endif // COMMON_SENSOR_ROBUST_STATISTICS_HPP
