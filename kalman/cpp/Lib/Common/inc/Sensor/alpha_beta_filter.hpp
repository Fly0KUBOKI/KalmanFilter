#pragma once

#ifndef COMMON_SENSOR_ALPHA_BETA_FILTER_HPP
#define COMMON_SENSOR_ALPHA_BETA_FILTER_HPP

#include "../../Matrix/fixed_matrix.hpp"
#include <cmath>

namespace common {
namespace sensor {

// 型エイリアス
using cm = cmath_fx::FixedMatrix;

/**
 * @class AlphaBetaFilter
 * @brief Alpha-Beta トラッキングフィルタ
 * 
 * 位置と速度の同時トラッキングに使用されます。
 * GPS信号などの位置データにフィットさせるのに有効です。
 */
class AlphaBetaFilter {
private:
    cm position_;           ///< 現在の位置推定値
    cm velocity_;           ///< 現在の速度推定値
    float alpha_;           ///< 位置更新係数
    float beta_;            ///< 速度更新係数
    bool initialized_;      ///< 初期化済みフラグ
    
public:
    /**
     * @brief コンストラクタ
     * @param alpha 位置係数（デフォルト: 0.5）
     * @param beta 速度係数（デフォルト: 0.1）
     */
    AlphaBetaFilter(float alpha = 0.5f, float beta = 0.1f) 
        : alpha_(alpha), beta_(beta), initialized_(false) {}
    
    /**
     * @brief フィルタリング実行
     * @param measurement 測定値（位置）
     * @param dt 時間ステップ
     * @param pos_out 位置出力
     * @param vel_out 速度出力
     */
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
        
        // 予測ステップ
        cm pos_pred;
        pos_pred.resize(position_.rows, position_.cols);
        for (int i = 0; i < position_.rows; ++i) {
            for (int j = 0; j < position_.cols; ++j) {
                pos_pred(i,j) = position_(i,j) + velocity_(i,j) * dt;
            }
        }
        
        // 残差計算
        cm residual;
        residual.resize(measurement.rows, measurement.cols);
        for (int i = 0; i < residual.rows; ++i) {
            for (int j = 0; j < residual.cols; ++j) {
                residual(i,j) = measurement(i,j) - pos_pred(i,j);
            }
        }
        
        // 更新ステップ
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
        position_.resize(0, 0);
        velocity_.resize(0, 0);
        initialized_ = false;
    }
    
    /**
     * @brief パラメータを設定
     * @param alpha 位置係数
     * @param beta 速度係数
     */
    void set_parameters(float alpha, float beta) {
        alpha_ = alpha;
        beta_ = beta;
    }
};

} // namespace sensor
} // namespace common

#endif // COMMON_SENSOR_ALPHA_BETA_FILTER_HPP
