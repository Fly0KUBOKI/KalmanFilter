#pragma once

#ifndef COMMON_SENSOR_EMA_FILTER_HPP
#define COMMON_SENSOR_EMA_FILTER_HPP

#include "../../Matrix/fixed_matrix.hpp"
#include <cmath>

namespace common {
namespace sensor {

// 型エイリアス（sensor_filter.hpp との互換性）
using cm = cmath_fx::FixedMatrix;

/**
 * @class EMAFilter
 * @brief Exponential Moving Average (EMA) フィルタ
 * 
 * センサーデータの平滑化に使用される単純なEMAフィルタ。
 * 初期化時は入力値をそのまま使用し、その後指数移動平均を適用します。
 */
class EMAFilter {
private:
    cm filtered_;           ///< フィルタ済みの値
    float alpha_;           ///< スムージング係数 (0.0 - 1.0)
    bool initialized_;      ///< 初期化済みフラグ
    
public:
    /**
     * @brief コンストラクタ
     * @param alpha スムージング係数（デフォルト: 0.3）
     */
    EMAFilter(float alpha = 0.3f) 
        : alpha_(alpha), initialized_(false) {}
    
    /**
     * @brief EMAフィルタリングを実行
     * @param input 入力値
     * @return フィルタ済み出力
     */
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
    
    /**
     * @brief フィルタの現在値を取得
     * @return フィルタ済み値
     */
    cm get_value() const {
        return filtered_;
    }
    
    /**
     * @brief フィルタをリセット
     */
    void reset() {
        initialized_ = false;
    }
    
    /**
     * @brief フィルタをゼロリセット
     */
    void reset_zero() {
        for(int i=0; i<filtered_.rows*filtered_.cols; ++i) 
            filtered_.data[i] = 0.0f;
        initialized_ = true;
    }
    
    /**
     * @brief スムージング係数を設定
     * @param alpha 係数（0.0 - 1.0の範囲にクリップ）
     */
    void set_alpha(float alpha) {
        alpha_ = fmaxf(0.0f, fminf(1.0f, alpha));
    }
};

} // namespace sensor
} // namespace common

#endif // COMMON_SENSOR_EMA_FILTER_HPP
