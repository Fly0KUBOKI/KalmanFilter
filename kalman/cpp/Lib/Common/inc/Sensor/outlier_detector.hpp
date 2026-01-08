#pragma once

#ifndef COMMON_SENSOR_OUTLIER_DETECTOR_HPP
#define COMMON_SENSOR_OUTLIER_DETECTOR_HPP

#include "../../Matrix/fixed_matrix.hpp"
#include "../Math/math_utils.hpp"
#include <cmath>

namespace common {
namespace sensor {

// 型エイリアス（sensor_filter.hpp との互換性）
using cm = cmath_fx::FixedMatrix;

/**
 * @class OutlierDetector
 * @brief センサー異常値（外れ値）検出器
 * 
 * 残差ノルムとその履歴統計に基づいて異常値を判定します。
 * MATLAB実装とのパリティを維持するための設計になっています。
 */
class OutlierDetector {
private:
    static constexpr int MAX_HISTORY = 20;  ///< 履歴の最大保持数
    float history_[MAX_HISTORY];            ///< 正常な残差ノルムの履歴
    int count_;                             ///< 現在の履歴数
    
public:
    /**
     * @brief コンストラクタ
     */
    OutlierDetector() : count_(0) {}
    
    /**
     * @brief 残差ノルムに基づいて外れ値を検出
     * 
     * 統計的な閾値判定を行います。正常なサンプルのみを履歴に加えることで、
     * 外れ値がノイズ推定を汚さないようにしています。
     * 
     * @param residual_norm 残差ノルム（innovation norm など）
     * @param threshold_sigma 閾値の倍数（デフォルト: 3.0σ）
     * @param min_std 標準偏差の最小値 [rad, m, etc]
     * @return true: 外れ値, false: 正常
     */
    bool detect(float residual_norm, float threshold_sigma = 3.0f, float min_std = 0.1f) {
        float noise_std;
        
        if (count_ == 0) {
            // 履歴が空の場合: 残差ノルムを基準にする
            // MATLAB互換: 最初のデータは外れ値と判定されない
            noise_std = fmaxf(residual_norm, min_std);
        } else if (count_ < 5) {
            // 履歴が少ない場合: 現在の履歴の標準偏差を使用
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
            // ノイズ推定が過度に小さくならないように下限を設定
            noise_std = fmaxf(noise_std, fmaxf(residual_norm / 3.0f, min_std));
        }
        
        // 閾値で判定
        float threshold = threshold_sigma * noise_std;
        bool is_outlier = (residual_norm > threshold);

        // MATLAB互換: 正常なサンプルのみを履歴に追加
        // これにより外れ値がノイズ推定を汚さない
        if (!is_outlier) {
            if (count_ < MAX_HISTORY) {
                history_[count_++] = residual_norm;
            } else {
                // 古い値を削除してシフト
                for (int i = 0; i < MAX_HISTORY - 1; ++i) {
                    history_[i] = history_[i+1];
                }
                history_[MAX_HISTORY-1] = residual_norm;
            }
        }

        return is_outlier;
    }

    /**
     * @brief Mahalanobis距離による静的判定
     * 
     * 履歴を使わない一度限りの判定。innovation と共分散から直接計算。
     * 
     * @param innovation イノベーション（測定 - 推定）
     * @param S イノベーション共分散行列
     * @param threshold_sigma 閾値の倍数
     * @return true: 外れ値, false: 正常
     */
    static bool detect_mahalanobis_static(
        const common::math::cm& innovation, 
        const common::math::cm& S, 
        float threshold_sigma = 3.0f
    ) {
        float dist_sq = kf::ops::mahalanobis_distance_squared(innovation, S);
        float dist = sqrtf(dist_sq);
        return dist > threshold_sigma;
    }
    
    /**
     * @brief 履歴をリセット
     */
    void reset() {
        count_ = 0;
    }
};

} // namespace sensor
} // namespace common

#endif // COMMON_SENSOR_OUTLIER_DETECTOR_HPP
