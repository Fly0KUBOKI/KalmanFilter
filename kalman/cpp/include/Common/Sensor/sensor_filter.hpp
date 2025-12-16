#pragma once

#include "../Math/fixed_matrix.hpp"
#include <cmath>
#include <algorithm>

namespace common {
namespace sensor {

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
    
    void reset() {
        initialized_ = false;
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
    
    void set_parameters(float alpha, float beta) {
        alpha_ = alpha;
        beta_ = beta;
    }
};

// ========== 外れ値検出器 ==========
class OutlierDetector {
private:
    static constexpr int MAX_HISTORY = 20;
    float history_[MAX_HISTORY];
    int count_;
    
public:
    OutlierDetector() : count_(0) {}
    
    bool detect(float residual_norm, float threshold_sigma = 3.0f) {
        // ノイズ標準偏差推定
        float noise_std = 0.1f;  // デフォルト
        if (count_ > 0) {
            // 標準偏差計算
            float sum = 0.0f;
            float sum_sq = 0.0f;
            for (int i = 0; i < count_; ++i) {
                sum += history_[i];
                sum_sq += history_[i] * history_[i];
            }
            float mean = sum / count_;
            noise_std = sqrtf(sum_sq / count_ - mean * mean);
            noise_std = fmaxf(noise_std, 0.1f);
        }
        
        // 外れ値判定
        float threshold = threshold_sigma * noise_std;
        bool is_outlier = (residual_norm > threshold);

        // 履歴更新（MATLABのMEX経路と一致させるため、
        // 外れ値判定の有無に関わらず履歴に残す）
        if (count_ < MAX_HISTORY) {
            history_[count_++] = residual_norm;
        } else {
            // シフト
            for (int i = 0; i < MAX_HISTORY - 1; ++i) {
                history_[i] = history_[i+1];
            }
            history_[MAX_HISTORY-1] = residual_norm;
        }

        return is_outlier;
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
        if(innov_norm > max_allowed_innov_ * 1e6f) return true;
        
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
        
        // ジッター追加
        float max_diag = 0.0f;
        for(int i=0; i<n; i++) max_diag = fmaxf(max_diag, fabsf(P(i,i)));
        if(max_diag < 1e-10f) max_diag = 1e-10f;
        float jitter = jitter_base_ * max_diag;
        for(int i=0; i<n; i++) P(i,i) += jitter;
        
        // 固有値補正は省略（簡易版）
        // 必要に応じて固有値分解を追加
        
        // 状態別キャップ（15次元ESKF用）
        if(n == 15) {
            float caps[15] = {1e6f, 1e6f, 1e6f,  // pos
                              1e4f, 1e4f, 1e4f,  // vel
                              100.0f, 100.0f, 100.0f,  // att
                              1e2f, 1e2f, 1e2f,  // ba
                              1e-1f, 1e-1f, 1e-1f}; // bg
            for(int i=0; i<15; i++) P(i,i) = fminf(P(i,i), caps[i]);
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
    
    NoiseEstimator noise_estimator;
    DivergenceGuard divergence_guard;
    
    SensorFilterLib() : noise_estimator(10), accel_threshold_sigma_(3.0f), accel_min_std_(0.1f) {
        accel_filter.set_alpha(0.3f);
        mag_filter.set_alpha(0.2f);
        baro_filter.set_alpha(0.4f);
        gps_filter.set_parameters(0.5f, 0.1f);
    }
    
    // 加速度フィルタ（外れ値検出付き）
    cm filter_accel(const cm& a_meas, const cm& a_expected, bool& is_outlier) {
        // 残差計算
        float residual_norm = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float diff = a_meas(i,0) - a_expected(i,0);
            residual_norm += diff * diff;
        }
        residual_norm = sqrtf(residual_norm);
        
        // 外れ値検出（設定で制御）
        is_outlier = accel_outlier.detect(residual_norm, accel_threshold_sigma_, accel_min_std_);
        
        if (is_outlier) {
            // 外れ値の場合は前回値を返す
            return accel_filter.get_value();
        } else {
            return accel_filter.filter(a_meas);
        }
    }

    void set_accel_config(float ema_alpha, int history_size, float threshold_sigma, float min_std) {
        accel_filter.set_alpha(ema_alpha);
        accel_outlier.set_max_history(history_size);
        accel_outlier.set_default_threshold_sigma(threshold_sigma);
        accel_outlier.set_default_min_std(min_std);
        accel_threshold_sigma_ = threshold_sigma;
        accel_min_std_ = min_std;
    }
    
    // 磁気計フィルタ（外れ値検出付き）
    cm filter_mag(const cm& m_meas, const cm& m_expected, bool& is_outlier) {
        // 残差計算
        float residual_norm = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float diff = m_meas(i,0) - m_expected(i,0);
            residual_norm += diff * diff;
        }
        residual_norm = sqrtf(residual_norm);
        
        // 外れ値検出
        is_outlier = mag_outlier.detect(residual_norm);
        
        if (is_outlier) {
            return mag_filter.filter(m_meas);
        } else {
            return mag_filter.filter(m_meas);
        }
    }
    
    // GPSフィルタ
    void filter_gps(const cm& gps_pos, float dt, cm& pos_out, cm& vel_out) {
        gps_filter.filter(gps_pos, dt, pos_out, vel_out);
    }
    
    // 気圧計フィルタ
    float filter_baro(float pressure) {
        cm p_in; p_in.resize(1,1); p_in(0,0) = pressure;
        cm p_out = baro_filter.filter(p_in);
        return p_out(0,0);
    }
    
    void reset_all() {
        accel_filter.reset();
        // gyro_filter removed (deprecated)
        mag_filter.reset();
        gps_filter.reset();
        baro_filter.reset();
        accel_outlier.reset();
        mag_outlier.reset();
    }
};

} // namespace sensor
} // namespace common
