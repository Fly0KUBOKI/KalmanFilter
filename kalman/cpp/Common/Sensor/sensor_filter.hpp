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
            // MATLAB互換: 0初期化からスタート
            filtered_.resize(input.rows, input.cols);
            for(int i=0; i<filtered_.rows*filtered_.cols; ++i) filtered_.data[i] = 0.0f;
            initialized_ = true;
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

    void set_value(const cm& val) {
        filtered_ = val;
        initialized_ = true;
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
    bool coeffs_set_;     // 係数が設定されたかのフラグ
    float current_dt_;
    float current_cutoff_;
    
public:
    BiquadLowpassFilter() : initialized_(false), coeffs_set_(false), current_dt_(0.0f), current_cutoff_(0.0f) {
        b0_ = b1_ = b2_ = 0.0f;
        a1_ = a2_ = 0.0f;
    }
    
    void configure(float dt, float cutoff_freq) {
        // パラメータが変更された場合のみ再計算
        if (coeffs_set_ && fabsf(dt - current_dt_) < 1e-6f && fabsf(cutoff_freq - current_cutoff_) < 1e-6f) {
            return;
        }
        
        current_dt_ = dt;
        current_cutoff_ = cutoff_freq;

        // Biquad係数計算 (MATLAB互換)
        float sample_rate = 1.0f / dt;
        float omega = 2.0f * 3.14159265358979323846f * cutoff_freq / sample_rate;
        float K = tanf(omega / 2.0f);
        float Q = 1.0f / sqrtf(2.0f);  // 0.7071...
        float norm = 1.0f / (1.0f + K / Q + K * K);
        
        b0_ = K * K * norm;
        b1_ = 2.0f * b0_;
        b2_ = b0_;
        a1_ = 2.0f * (K * K - 1.0f) * norm;
        a2_ = (1.0f - K / Q + K * K) * norm;
        
        coeffs_set_ = true;
        // MATLABではパラメータ変更時に状態をリセットしない
    }
    
    cm filter(const cm& input) {
        if (!initialized_) {
            // MATLAB互換: 0初期化からスタート
            x1_.resize(input.rows, input.cols);
            x2_.resize(input.rows, input.cols);
            y1_.resize(input.rows, input.cols);
            y2_.resize(input.rows, input.cols);
            
            // リセット時のために明示的に0クリア
            for(int i=0; i<input.rows*input.cols; ++i) {
                x1_.data[i] = 0.0f;
                x2_.data[i] = 0.0f;
                y1_.data[i] = 0.0f;
                y2_.data[i] = 0.0f;
            }
            initialized_ = true;
        }
        
        cm result;
        result.resize(input.rows, input.cols);
        
        for (int i = 0; i < input.rows; ++i) {
            for (int j = 0; j < input.cols; ++j) {
                // MATLAB実装に合わせたロジック
                // w = input - a1*y1 - a2*y2
                float w = input(i,j) - a1_ * y1_(i,j) - a2_ * y2_(i,j);
                
                // result = b0*w + b1*x1 + b2*x2
                result(i,j) = b0_ * w + b1_ * x1_(i,j) + b2_ * x2_(i,j);
                
                // 状態更新
                x2_(i,j) = x1_(i,j);
                x1_(i,j) = w;
                y2_(i,j) = y1_(i,j);
                y1_(i,j) = result(i,j);
            }
        }
        
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
            // MATLAB互換: 0初期化
            position_.resize(measurement.rows, measurement.cols);
            velocity_.resize(measurement.rows, measurement.cols);
            
            // リセット時のために明示的に0クリア
            for(int i=0; i<position_.rows*position_.cols; ++i) {
                position_.data[i] = 0.0f;
                velocity_.data[i] = 0.0f;
            }
            
            initialized_ = true;
            
            // 初回更新 (0からの予測)
            // MATLAB: p_pred = 0 + 0*dt = 0
            // innov = meas - 0 = meas
            // p_filt = 0 + alpha * meas
            // v_filt = 0 + beta/dt * meas
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
    
    bool detect(float residual_norm, float threshold_sigma = 3.0f, float min_std = 0.1f) {
        // ノイズ標準偏差推定
        float noise_std;
        if (count_ == 0) {
            noise_std = residual_norm;
        } else {
            // 標準偏差計算
            float sum = 0.0f;
            float sum_sq = 0.0f;
            for (int i = 0; i < count_; ++i) {
                sum += history_[i];
                sum_sq += history_[i] * history_[i];
            }
            float mean = sum / count_;
            // 分散が負にならないように保護
            float var = sum_sq / count_ - mean * mean;
            noise_std = sqrtf(fmaxf(var, 0.0f));
        }
        noise_std = fmaxf(noise_std, min_std);
        
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
        residual_norm = sqrtf(residual_norm);
        
        // 外れ値検出 (3σ, min_std=0.1)
        is_outlier = accel_outlier.detect(residual_norm, 3.0f, 0.1f);
        
        if (is_outlier) {
            // 外れ値の場合は前回値を返す (更新しない)
            return accel_filter.get_value();
        } else {
            return accel_filter.filter(a_meas);
        }
    }
    
    // ジャイロフィルタ
    cm filter_gyro(const cm& w_meas, float dt, float cutoff_freq = 20.0f) {
        gyro_filter.configure(dt, cutoff_freq);
        return gyro_filter.filter(w_meas);
    }
    
    // 磁気計フィルタ（外れ値検出付き）
    cm filter_mag(const cm& m_meas, const cm& m_expected, bool& is_outlier) {
        // 前回値取得
        cm prev_filtered = mag_filter.get_value();
        
        // 初回チェック (norm < 1e-6)
        float prev_norm = 0.0f;
        if (prev_filtered.rows == 3 && prev_filtered.cols == 1) {
            for(int i=0; i<3; ++i) prev_norm += prev_filtered(i,0)*prev_filtered(i,0);
            prev_norm = sqrtf(prev_norm);
        }
        
        if (prev_norm < 1e-6f) {
            mag_filter.set_value(m_meas);
            is_outlier = false;
            return m_meas;
        }

        // 残差計算 (対 前回値)
        float residual_norm = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float diff = m_meas(i,0) - prev_filtered(i,0);
            residual_norm += diff * diff;
        }
        residual_norm = sqrtf(residual_norm);
        
        // 外れ値検出 (5σ, min_std=5.0)
        is_outlier = mag_outlier.detect(residual_norm, 5.0f, 5.0f);
        
        if (is_outlier) {
            return mag_filter.get_value();
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
