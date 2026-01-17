#pragma once
#ifndef LIB_ESKF_INC_VALIDATION_VALIDATION_HPP
#define LIB_ESKF_INC_VALIDATION_VALIDATION_HPP

#include "../../../Matrix/fixed_matrix.hpp"
#include "../../../KF/inc/kf_operations.hpp"
#include "../../../Matrix/Math/statistics.hpp"
#include <cmath>
#include <algorithm>

namespace common {
namespace validation {

using cm = cmath_fx::FixedMatrix;

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
    // Mahalanobis距離による外れ値検出（統一実装は common::math::mahalanobis_distance_squared相当）
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
        float norm = std::sqrt(norm_sq);
        return (norm > threshold);
    }
    
    // Chi-square検定による外れ値検出（統一実装は common::math::is_outlier_chi_square）
    static bool detect_chi_square(const cm& innovation, const cm& S, float alpha = 0.05f) {
        int dof = innovation.rows;
        return common::math::is_outlier_chi_square(innovation, S, dof, alpha);
    }
};
            #pragma once

            #ifndef COMMON_VALIDATION_HPP
            #define COMMON_VALIDATION_HPP

            #include "../../../Matrix/fixed_matrix.hpp"
            #include "../../../Sensor/sensor_filters.hpp"
            #include <cmath>
            #include <cfloat>

            namespace common {
            namespace validation {

            using cm = cmath_fx::FixedMatrix;

            // 後方互換用エイリアス
            using OutlierDetector = sensor::filter::OutlierDetector;
            using NoiseEstimator = sensor::filter::NoiseEstimator;
            using DivergenceGuard = sensor::filter::DivergenceGuard;

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
