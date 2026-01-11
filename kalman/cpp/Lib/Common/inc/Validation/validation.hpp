#pragma once

#include "../../../Matrix/fixed_matrix.hpp"
#include "../../../KF/inc/kf_operations.hpp"
#include "../Math/statistics.hpp"
#include "../Math/geometry.hpp"
#include "../Math/numerical.hpp"
#include "../Math/math_utils.hpp"
#include <cmath>
#include <algorithm>
#include "../Math/portable_math.hpp"

namespace common {
namespace validation {

using cm = cmath_fx::FixedMatrix;
using MathUtils = common::math::MathUtils;

// ========== 共分散正則化（名前空間 API） ==========
namespace covariance {
    extern const float MIN_VARIANCE;
    extern const float MAX_VARIANCE;
    extern const float MIN_EIGENVALUE;

    inline cm regularize(const cm& P) {
        if (P.rows != P.cols) return P;
        int n = P.rows;
        cm P_reg = P;
        cmath_fx::utils::symmetrize(P_reg);
        for (int i = 0; i < n; ++i) {
            if (P_reg(i,i) < MIN_VARIANCE) P_reg(i,i) = MIN_VARIANCE;
            else if (P_reg(i,i) > MAX_VARIANCE) P_reg(i,i) = MAX_VARIANCE;
        }
        for (int i = 0; i < n; ++i) {
            float row_sum = 0.0f;
            for (int j = 0; j < n; ++j) if (i != j) row_sum += fabsf(P_reg(i,j));
            if (P_reg(i,i) < row_sum) P_reg(i,i) = row_sum + MIN_EIGENVALUE;
        }
        return P_reg;
    }

    inline cm joseph_form_update(const cm& P_pred, const cm& K, const cm& H, const cm& R) {
        cm P_upd; kf::ops::joseph_form_update(P_pred, K, H, R, P_upd); return regularize(P_upd);
    }
}

// Backward-compatible namespace wrappers (preferred API)
namespace covariance {
    inline cm regularize_cov(const cm& P) { return covariance::regularize(P); }
    inline cm joseph_form_update_cov(const cm& P_pred, const cm& K, const cm& H, const cm& R) { return covariance::joseph_form_update(P_pred,K,H,R); }
}

namespace state {
    // Forwarders to StateValidator (if exists) — keep compatibility with planned API.
    inline bool check_quaternion_norm(const float q[4], float tol = 1e-4f) {
        // If StateValidator provides specific implementation, call here. Fallback: basic check.
        float norm = common::math::portable_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        return fabsf(norm - 1.0f) < tol;
    }
}

// Additional backward-compatible wrappers for covariance/state operations
namespace covariance {
    inline void symmetrize(cmath_fx::Matrix<15,15,float>& P) { cmath_fx::utils::symmetrize(P); }
    inline void ensure_positive_definite(cmath_fx::Matrix<15,15,float>& P, float min_diag = 1e-6f) {
        for (int i = 0; i < 15; ++i) if (P(i,i) < min_diag) P(i,i) = min_diag;
    }
    inline void add_process_noise(cmath_fx::Matrix<15,15,float>& P, const cmath_fx::Matrix<15,15,float>& Q) {
        for (int i = 0; i < 15*15; ++i) P.data()[i] += Q.data()[i];
    }
    // aliases to the primary implementations above
    inline cm regularize(const cm& P) { return covariance::regularize(P); }
    inline cm joseph_form_update(const cm& P_pred, const cm& K, const cm& H, const cm& R) { return covariance::joseph_form_update(P_pred,K,H,R); }
}

namespace state {
    inline bool check_finite(const float* arr, int n) {
        for (int i=0;i<n;++i) if (!std::isfinite(arr[i])) return false;
        return true;
    }
    inline bool check_covariance(const float P_arr[15*15]) {
        for (int i=0;i<15;++i) { float v = P_arr[i*15+i]; if (!(std::isfinite(v) && v > 0.0f)) return false; }
        return true;
    }
}

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
        float norm = common::math::portable_sqrt(norm_sq);
        return (norm > threshold);
    }
    
    // Chi-square検定による外れ値検出（統一実装は math_utils.hpp::MathUtils::is_outlier_chi_square）
    static bool detect_chi_square(const cm& innovation, const cm& S, float alpha = 0.05f) {
        int dof = innovation.rows;
        return common::math::is_outlier_chi_square(innovation, S, dof, alpha);
    }
};
            #pragma once

            #ifndef COMMON_VALIDATION_HPP
            #define COMMON_VALIDATION_HPP

            #include "../../../Matrix/fixed_matrix.hpp"
            #include "../Sensor/outlier_detector.hpp"
            #include "../Sensor/robust_statistics.hpp"
            #include <cmath>
            #include <cfloat>

            namespace common {
            namespace validation {

            using cm = cmath_fx::FixedMatrix;

            // 後方互換用エイリアス
            using OutlierDetector = common::sensor::OutlierDetector;
            using NoiseEstimator = common::sensor::NoiseEstimator;
            using DivergenceGuard = common::sensor::DivergenceGuard;

            // 共分散の基本チェック
            template<typename T>
            bool validate_covariance(const cmath_fx::Matrix<15,15,T>& P, T tolerance = (T)1e-6) {
                // 対称性
                for (int i = 0; i < 15; ++i) {
                    for (int j = i+1; j < 15; ++j) {
                        if (std::fabs(P(i,j) - P(j,i)) > tolerance) return false;
                    }
                }
                // 対角正値
                for (int i = 0; i < 15; ++i) {
                    if (P(i,i) <= 0) return false;
                }
                return true;
            }

            template<typename T>
            bool validate_quaternion(const T q[4], T tolerance = (T)1e-4) {
                T norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
                return std::fabs(norm - 1.0) < tolerance;
            }

            template<typename T>
            bool is_finite(const T* arr, int n) {
                for (int i = 0; i < n; ++i) if (!std::isfinite(arr[i])) return false;
                return true;
            }

            } // namespace validation
            } // namespace common

            #endif
