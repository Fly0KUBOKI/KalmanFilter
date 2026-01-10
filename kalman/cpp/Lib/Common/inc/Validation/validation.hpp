#pragma once

#include "../../../Matrix/fixed_matrix.hpp"
#include "../../../KF/inc/kf_operations.hpp"
#include "../Math/statistics.hpp"
#include "../Math/geometry.hpp"
#include "../Math/numerical.hpp"
#include "../Math/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace common {
namespace validation {

using cm = cmath_fx::FixedMatrix;
using MathUtils = common::math::MathUtils;

// ========== 共分散正則化 ==========
class CovarianceRegularizer {
public:
    static const float MIN_VARIANCE;
    static const float MAX_VARIANCE;
    static const float MIN_EIGENVALUE;
    
    // 共分散行列の正則化
    static cm regularize(const cm& P) {
        if (P.rows != P.cols) return P;
        
        int n = P.rows;
        cm P_reg = P;
        
        // 1. 対称性の強制
        cmath_fx::utils::symmetrize(P_reg);
        
        // 2. 対角成分の範囲制限
        for (int i = 0; i < n; ++i) {
            if (P_reg(i,i) < MIN_VARIANCE) {
                P_reg(i,i) = MIN_VARIANCE;
            } else if (P_reg(i,i) > MAX_VARIANCE) {
                P_reg(i,i) = MAX_VARIANCE;
            }
        }
        
        // 3. 正定値性チェック（簡易版：対角優位性の確保）
        for (int i = 0; i < n; ++i) {
            float row_sum = 0.0f;
            for (int j = 0; j < n; ++j) {
                if (i != j) {
                    row_sum += fabsf(P_reg(i,j));
                }
            }
            if (P_reg(i,i) < row_sum) {
                P_reg(i,i) = row_sum + MIN_EIGENVALUE;
            }
        }
        
        return P_reg;
    }
    
    // Joseph形式の共分散更新（数値安定性向上）
    static cm joseph_form_update(const cm& P_pred, const cm& K, const cm& H, const cm& R) {
        cm P_upd;
        kf::ops::joseph_form_update(P_pred, K, H, R, P_upd);
        return regularize(P_upd);
    }
};

// ========== 外れ値検出器（非推奨：math_utils.hpp の関数を使用） ==========
class OutlierDetector {
public:
    // Mahalanobis距離による外れ値検出（統一実装は math_utils.hpp::MathUtils::mahalanobis_distance_squared）
    static bool detect_mahalanobis(const cm& innovation, const cm& S, float threshold = 9.0f) {
        float dist_sq = kf::ops::mahalanobis_distance_squared(innovation, S);
        return (dist_sq > threshold);
    }
    
    // シンプルな外れ値検出（閾値ベース）
    static bool detect_threshold(const cm& innovation, float threshold) {
        float norm_sq = 0.0f;
        for (int i = 0; i < innovation.rows; ++i) {
            norm_sq += innovation(i,0) * innovation(i,0);
        }
        float norm = sqrtf(norm_sq);
        return (norm > threshold);
    }
    
    // Chi-square検定による外れ値検出（統一実装は math_utils.hpp::MathUtils::is_outlier_chi_square）
    static bool detect_chi_square(const cm& innovation, const cm& S, float alpha = 0.05f) {
        int dof = innovation.rows;
        return MathUtils::is_outlier_chi_square(innovation, S, dof, alpha);
    }
};

// ========== 状態検証器 ==========
class StateValidator {
public:
    static const float MAX_POSITION;      // m
    static const float MAX_VELOCITY;    // m/s
    static const float MAX_ACCELERATION;  // m/s^2
    static const float MAX_ANGULAR_RATE;  // rad/s
    
    // 位置の妥当性チェック
    static bool validate_position(const cm& p) {
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(p(i,0))) return false;
                if (fabsf(p(i,0)) > MAX_POSITION) return false;
        }
        return true;
    }
    
    // 速度の妥当性チェック
    static bool validate_velocity(const cm& v) {
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(v(i,0))) return false;
                if (fabsf(v(i,0)) > MAX_VELOCITY) return false;
        }
        return true;
    }
    
    // 加速度の妥当性チェック
    static bool validate_acceleration(const cm& a) {
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(a(i,0))) return false;
                if (fabsf(a(i,0)) > MAX_ACCELERATION) return false;
        }
        return true;
    }
    
    // 角速度の妥当性チェック
    static bool validate_angular_rate(const cm& w) {
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(w(i,0))) return false;
                if (fabsf(w(i,0)) > MAX_ANGULAR_RATE) return false;
        }
        return true;
    }
    
    // クォータニオンの妥当性チェック
    static bool validate_quaternion(const cm& q) {
        if (q.rows < 4) return false;
        
        // 有限性チェック
        for (int i = 0; i < 4; ++i) {
            if (!std::isfinite(q(i,0))) return false;
        }
        
        // ノルムチェック
        float norm_sq = 0.0f;
        for (int i = 0; i < 4; ++i) {
            norm_sq += q(i,0) * q(i,0);
        }
        float norm = sqrtf(norm_sq);
        
        // ノルムが1に近いかチェック（±10%の許容範囲）
        if (norm < 0.9f || norm > 1.1f) return false;
        
        return true;
    }
    
    // 共分散行列の妥当性チェック
    static bool validate_covariance(const cm& P) {
        if (P.rows != P.cols) return false;
        
        int n = P.rows;
        
        // 1. 有限性チェック
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (!std::isfinite(P(i,j))) return false;
            }
        }
        
        // 2. 対称性チェック
        for (int i = 0; i < n; ++i) {
            for (int j = i+1; j < n; ++j) {
                    if (fabsf(P(i,j) - P(j,i)) > 1e-6f) return false;
            }
        }
        
        // 3. 対角成分の正値性チェック
        for (int i = 0; i < n; ++i) {
            if (P(i,i) <= 0.0f) return false;
        }
        
        return true;
    }
    
    // ESKF状態全体の妥当性チェック
    static bool validate_eskf_state(const cm& p, const cm& v, const cm& q, 
                                   const cm& ba, const cm& bg, const cm& P) {
        if (!validate_position(p)) return false;
        if (!validate_velocity(v)) return false;
        if (!validate_quaternion(q)) return false;
        if (!validate_acceleration(ba)) return false;
        if (!validate_angular_rate(bg)) return false;
        if (!validate_covariance(P)) return false;
        
        return true;
    }
};

// ========== ノイズ推定器 ==========
class NoiseEstimator {
private:
    static const int WINDOW_SIZE;
    float innovation_history_[WINDOW_SIZE];
    int count_;
    
public:
    NoiseEstimator() : count_(0) {}
    
    // イノベーションからノイズを推定
    float estimate(const cm& innovation) {
        float norm_sq = 0.0f;
        for (int i = 0; i < innovation.rows; ++i) {
            norm_sq += innovation(i,0) * innovation(i,0);
        }
            float norm = sqrtf(norm_sq);
        
        // 履歴追加
        if (count_ < WINDOW_SIZE) {
            innovation_history_[count_++] = norm;
        } else {
            // シフト
            for (int i = 0; i < WINDOW_SIZE - 1; ++i) {
                innovation_history_[i] = innovation_history_[i+1];
            }
            innovation_history_[WINDOW_SIZE-1] = norm;
        }
        
        // 標準偏差計算
        if (count_ < 2) return 0.1f;  // デフォルト
        
        float sum = 0.0f;
        float sum_sq = 0.0f;
        for (int i = 0; i < count_; ++i) {
            sum += innovation_history_[i];
            sum_sq += innovation_history_[i] * innovation_history_[i];
        }
        float mean = sum / count_;
        float variance = sum_sq / count_ - mean * mean;
        
            return sqrtf(fmaxf(variance, 0.0f));
    }
    
    void reset() {
        count_ = 0;
    }
};

} // namespace validation
} // namespace common
