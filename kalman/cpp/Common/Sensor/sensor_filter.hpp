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
        result.resize(input.rows(), input.cols());
        for (int i = 0; i < input.rows(); ++i) {
            for (int j = 0; j < input.cols(); ++j) {
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
        alpha_ = std::fmaxf(0.0f, std::fminf(1.0f, alpha));
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
        float K = std::tanf(omega * dt / 2.0f);
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
        result.resize(input.rows(), input.cols());
        
        for (int i = 0; i < input.rows(); ++i) {
            for (int j = 0; j < input.cols(); ++j) {
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
            velocity_.resize(measurement.rows(), measurement.cols());
            for (int i = 0; i < velocity_.rows(); ++i) {
                for (int j = 0; j < velocity_.cols(); ++j) {
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
        pos_pred.resize(position_.rows(), position_.cols());
        for (int i = 0; i < position_.rows(); ++i) {
            for (int j = 0; j < position_.cols(); ++j) {
                pos_pred(i,j) = position_(i,j) + velocity_(i,j) * dt;
            }
        }
        
        // 残差
        cm residual;
        residual.resize(measurement.rows(), measurement.cols());
        for (int i = 0; i < residual.rows(); ++i) {
            for (int j = 0; j < residual.cols(); ++j) {
                residual(i,j) = measurement(i,j) - pos_pred(i,j);
            }
        }
        
        // 更新
        pos_out.resize(position_.rows(), position_.cols());
        vel_out.resize(velocity_.rows(), velocity_.cols());
        for (int i = 0; i < position_.rows(); ++i) {
            for (int j = 0; j < position_.cols(); ++j) {
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
            noise_std = std::sqrtf(sum_sq / count_ - mean * mean);
            noise_std = std::fmaxf(noise_std, 0.1f);
        }
        
        // 外れ値判定
        float threshold = threshold_sigma * noise_std;
        bool is_outlier = (residual_norm > threshold);
        
        // 履歴更新（外れ値でない場合のみ）
        if (!is_outlier) {
            if (count_ < MAX_HISTORY) {
                history_[count_++] = residual_norm;
            } else {
                // シフト
                for (int i = 0; i < MAX_HISTORY - 1; ++i) {
                    history_[i] = history_[i+1];
                }
                history_[MAX_HISTORY-1] = residual_norm;
            }
        }
        
        return is_outlier;
    }
    
    void reset() {
        count_ = 0;
    }
};

// ========== センサーフィルタ統合クラス ==========
class SensorFilterLib {
public:
    EMAFilter accel_filter;
    BiquadLowpassFilter gyro_filter;
    EMAFilter mag_filter;
    AlphaBetaFilter gps_filter;
    EMAFilter baro_filter;
    
    OutlierDetector accel_outlier;
    OutlierDetector mag_outlier;
    
    SensorFilterLib() {
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
        residual_norm = std::sqrtf(residual_norm);
        
        // 外れ値検出
        is_outlier = accel_outlier.detect(residual_norm);
        
        if (is_outlier) {
            // 外れ値の場合は前回値を返す
            return accel_filter.filter(a_meas);  // 実際は前回値を保持
        } else {
            return accel_filter.filter(a_meas);
        }
    }
    
    // ジャイロフィルタ
    cm filter_gyro(const cm& w_meas, float dt, float cutoff_freq = 50.0f) {
        gyro_filter.configure(dt, cutoff_freq);
        return gyro_filter.filter(w_meas);
    }
    
    // 磁気計フィルタ（外れ値検出付き）
    cm filter_mag(const cm& m_meas, const cm& m_expected, bool& is_outlier) {
        // 残差計算
        float residual_norm = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float diff = m_meas(i,0) - m_expected(i,0);
            residual_norm += diff * diff;
        }
        residual_norm = std::sqrtf(residual_norm);
        
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
        gyro_filter.reset();
        mag_filter.reset();
        gps_filter.reset();
        baro_filter.reset();
        accel_outlier.reset();
        mag_outlier.reset();
    }
};

} // namespace sensor
} // namespace common
