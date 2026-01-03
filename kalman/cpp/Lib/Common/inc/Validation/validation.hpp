#pragma once

#include "../../Lib/Matrix/fixed_matrix.hpp"
#include <cmath>
#include <algorithm>

namespace common {
namespace validation {

using cm = cmath_fx::FixedMatrix;

// ========== 共分散正則化 ==========
class CovarianceRegularizer {
public:
    static constexpr float MIN_VARIANCE = 1.0e-12f;
    static constexpr float MAX_VARIANCE = 1.0e6f;
    static constexpr float MIN_EIGENVALUE = 1.0e-9f;
    
    // 共分散行列の正則化
    static cm regularize(const cm& P) {
        if (P.rows != P.cols) return P;
        
        int n = P.rows;
        cm P_reg = P;
        
        // 1. 対称性の強制
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                P_reg(i,j) = 0.5f * (P(i,j) + P(j,i));
            }
        }
        
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
        // P = (I - K*H) * P_pred * (I - K*H)' + K*R*K'
        int n = P_pred.rows;
        int m = H.rows;
        
        // I - K*H
        cm KH, IKH;
        cmath_fx::multiply(K, H, KH);
        IKH.resize(n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                IKH(i,j) = (i == j ? 1.0f : 0.0f) - KH(i,j);
            }
        }
        
        // (I - K*H) * P_pred
        cm temp1;
        cmath_fx::multiply(IKH, P_pred, temp1);
        
        // (I - K*H) * P_pred * (I - K*H)'
        cm IKH_t, temp2;
        cmath_fx::transpose(IKH, IKH_t);
        cmath_fx::multiply(temp1, IKH_t, temp2);
        
        // K*R*K'
        cm KR, K_t, temp3;
        cmath_fx::multiply(K, R, KR);
        cmath_fx::transpose(K, K_t);
        cmath_fx::multiply(KR, K_t, temp3);
        
        // 合計
        cm P_upd;
        P_upd.resize(n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                P_upd(i,j) = temp2(i,j) + temp3(i,j);
            }
        }
        
        return regularize(P_upd);
    }
};

// ========== 外れ値検出器 ==========
class OutlierDetector {
public:
    // マハラノビス距離による外れ値検出
    static bool detect_mahalanobis(const cm& innovation, const cm& S, float threshold = 9.0f) {
        // マハラノビス距離^2 = y' * S^-1 * y
        // 簡易版: S^-1を計算せず、ノルムベースで判定
        
        // イノベーション共分散行列の対角成分から閾値計算
        float max_var = 0.0f;
        for (int i = 0; i < S.rows; ++i) {
            if (S(i,i) > max_var) max_var = S(i,i);
        }
        
        float innovation_norm_sq = 0.0f;
        for (int i = 0; i < innovation.rows; ++i) {
            innovation_norm_sq += innovation(i,0) * innovation(i,0);
        }
        
        // 正規化された距離
        float normalized_dist = innovation_norm_sq / (max_var + 1e-9f);
        
        return (normalized_dist > threshold);
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
    
    // Chi-square検定による外れ値検出
    static bool detect_chi_square(const cm& innovation, const cm& S, float alpha = 0.05f) {
        // 簡易版: 3σ基準を使用
        int dof = innovation.rows;
        float threshold = 3.0f * dof;  // 保守的な閾値
        
        return detect_mahalanobis(innovation, S, threshold);
    }
};

// ========== 状態検証器 ==========
class StateValidator {
public:
    static constexpr float MAX_POSITION = 1.0e6f;      // m
    static constexpr float MAX_VELOCITY = 1000.0f;    // m/s
    static constexpr float MAX_ACCELERATION = 100.0f;  // m/s^2
    static constexpr float MAX_ANGULAR_RATE = 10.0f;  // rad/s
    
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
    static constexpr int WINDOW_SIZE = 50;
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
