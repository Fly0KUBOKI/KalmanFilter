#pragma once

#ifndef COMMON_SENSOR_BIQUAD_FILTER_HPP
#define COMMON_SENSOR_BIQUAD_FILTER_HPP

#include "../../../Matrix/fixed_matrix.hpp"
#include <cmath>

namespace common {
namespace sensor {

// 型エイリアス（sensor_filter.hpp との互換性）
using cm = cmath_fx::FixedMatrix;

/**
 * @class BiquadLowpassFilter
 * @brief Biquad（2次）ローパスフィルタ
 * 
 * デジタル信号処理で広く使用される2次IIRフィルタ。
 * センサーデータのノイズ低減に使用されます。
 */
class BiquadLowpassFilter {
private:
    cm x1_, x2_;          ///< 入力値の履歴 (x[n-1], x[n-2])
    cm y1_, y2_;          ///< 出力値の履歴 (y[n-1], y[n-2])
    float b0_, b1_, b2_;  ///< 分子係数 (差分方程式の前方項)
    float a1_, a2_;       ///< 分母係数 (差分方程式の後方項)
    bool initialized_;    ///< 初期化済みフラグ
    
public:
    /**
     * @brief コンストラクタ
     */
    BiquadLowpassFilter() : initialized_(false) {
        b0_ = b1_ = b2_ = 0.0f;
        a1_ = a2_ = 0.0f;
    }
    
    /**
     * @brief フィルタ係数を設定
     * @param dt サンプリング周期 [秒]
     * @param cutoff_freq カットオフ周波数 [Hz]
     */
    void configure(float dt, float cutoff_freq) {
        // Biquad係数計算（Butterworth特性）
        float omega = 2.0f * 3.14159265358979323846f * cutoff_freq;
        float K = tanf(omega * dt / 2.0f);
        float norm = 1.0f / (1.0f + K / 0.7071f + K * K);
        
        b0_ = K * K * norm;
        b1_ = 2.0f * b0_;
        b2_ = b0_;
        a1_ = 2.0f * (K * K - 1.0f) * norm;
        a2_ = (1.0f - K / 0.7071f + K * K) * norm;
        
        initialized_ = false;  // 設定後、履歴をリセット
    }
    
    /**
     * @brief フィルタリングを実行
     * @param input 入力値
     * @return フィルタ済み出力
     */
    cm filter(const cm& input) {
        if (!initialized_) {
            // 初回呼び出し時は入力値を直通
            x1_ = input;
            x2_ = input;
            y1_ = input;
            y2_ = input;
            initialized_ = true;
            return input;
        }
        
        cm result;
        result.resize(input.rows, input.cols);
        
        // 各要素にフィルタを適用
        for (int i = 0; i < input.rows; ++i) {
            for (int j = 0; j < input.cols; ++j) {
                result(i,j) = b0_ * input(i,j) + b1_ * x1_(i,j) + b2_ * x2_(i,j)
                            - a1_ * y1_(i,j) - a2_ * y2_(i,j);
            }
        }
        
        // 履歴を更新
        x2_ = x1_;
        x1_ = input;
        y2_ = y1_;
        y1_ = result;
        
        return result;
    }
    
    /**
     * @brief フィルタをリセット
     */
    void reset() {
        initialized_ = false;
    }
};

} // namespace sensor
} // namespace common

#endif // COMMON_SENSOR_BIQUAD_FILTER_HPP
