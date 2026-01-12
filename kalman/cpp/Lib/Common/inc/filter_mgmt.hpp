#pragma once
#ifndef LIB_COMMON_INC_FILTER_MGMT_HPP
#define LIB_COMMON_INC_FILTER_MGMT_HPP


#include "../../Matrix/fixed_matrix.hpp"
#include <cstddef>

namespace common {
namespace filter {

// 共分散行列にNaNまたはInfが含まれているかチェック
// P: 共分散行列 (15x15)
// 戻り値: NaNまたはInfが含まれていればtrue
bool hasNaNOrInf(const cmath_fx::Matrix<15, 15, float>& P);

// 共分散行列をスケール付き単位行列に設定
// P: 共分散行列 (15x15) [出力]
// scale: スケール値
void setIdentityScaled(cmath_fx::Matrix<15, 15, float>& P, float scale);

// 発散チェック
// P: 共分散行列 (15x15)
// 戻り値: 発散していればtrue
bool check_divergence(const cmath_fx::Matrix<15, 15, float>& P);

// ZUPT適用（速度をゼロにして共分散を減らす）
// v_in: 速度ベクトル (3x1) [入力]
// P_in: 共分散行列 (15x15) [入力]
// v_out: 速度ベクトル (3x1) [出力]
// P_out: 共分散行列 (15x15) [出力]
void apply_zupt(
    const cmath_fx::Vector<3, float>& v_in,
    const cmath_fx::Matrix<15, 15, float>& P_in,
    cmath_fx::Vector<3, float>& v_out,
    cmath_fx::Matrix<15, 15, float>& P_out
);

// 共分散行列の正規化（最大分散チェック）
// P: 共分散行列 (15x15) [入出力]
// 各状態変数の最大分散を超える場合、スケーリングして正規化
void normalize_covariance(cmath_fx::Matrix<15, 15, float>& P);

// 共分散行列の強制対称化: P = (P + P^T)/2
// P: 共分散行列 (15x15) [入出力]
void symmetrize_covariance(cmath_fx::Matrix<15, 15, float>& P);
// 状態の発散チェック（拡張版）
// p: 位置ベクトル (3x1) [入力]
// v: 速度ベクトル (3x1) [入力]
// q: クォータニオン (4x1) [入力]
// ba: 加速度バイアス (3x1) [入力]
// bg: ジャイロバイアス (3x1) [入力]
// P: 共分散行列 (15x15) [入力]
// 戻り値: 発散していればtrue
bool check_state_divergence(
    const cmath_fx::Vector<3, float>& p,
    const cmath_fx::Vector<3, float>& v,
    const cmath_fx::Vector<4, float>& q,
    const cmath_fx::Vector<3, float>& ba,
    const cmath_fx::Vector<3, float>& bg,
    const cmath_fx::Matrix<15, 15, float>& P
);

// ZUPTチェック
// a_meas: 加速度測定値 (3x1) [入力]
// w_meas: 角速度測定値 (3x1) [入力]
// zupt_threshold_accel: 加速度閾値 [入力]
// zupt_threshold_gyro: 角速度閾値 [入力]
// 戻り値: 静止状態であればtrue
bool check_zupt_condition(
    const cmath_fx::Vector<3, float>& a_meas,
    const cmath_fx::Vector<3, float>& w_meas,
    float zupt_threshold_accel,
    float zupt_threshold_gyro
);

// 状態リセット（発散時のリセット処理）
// v: 速度ベクトル (3x1) [入出力]
// ba: 加速度バイアス (3x1) [入出力]
// bg: ジャイロバイアス (3x1) [入出力]
// q: クォータニオン (4x1) [入出力]
// P: 共分散行列 (15x15) [入出力]
void reset_state_on_divergence(
    cmath_fx::Vector<3, float>& v,
    cmath_fx::Vector<3, float>& ba,
    cmath_fx::Vector<3, float>& bg,
    cmath_fx::Vector<4, float>& q,
    cmath_fx::Matrix<15, 15, float>& P
);

} // namespace filter
// Backward-compatible namespaces for gradual migration from
// CovarianceRegularizer / StateValidator -> common::covariance / common::state
namespace covariance {
    using cm = cmath_fx::Matrix<15, 15, float>;

    inline void symmetrize(cm& P) { common::filter::symmetrize_covariance(P); }

    inline void ensure_positive_definite(cm& P) { common::filter::normalize_covariance(P); }

    // Simple add_process_noise: add scaled identity to covariance
    inline void add_process_noise(cm& P, float scale) { common::filter::setIdentityScaled(P, scale); }
}

namespace state {
    using vec3 = cmath_fx::Vector<3, float>;
    using vec4 = cmath_fx::Vector<4, float>;
    using cm = cmath_fx::Matrix<15, 15, float>;

    inline bool check_quaternion_norm(const vec4& q, float tol = 1e-3f) {
        float sum = 0.0f;
        for (size_t i = 0; i < 4; ++i) sum += q(i,0) * q(i,0);
        return std::fabs(sum - 1.0f) <= tol;
    }

    template <typename T>
    inline bool check_finite(const T* arr, size_t n) {
        for (size_t i = 0; i < n; ++i) if (!std::isfinite(static_cast<double>(arr[i]))) return false;
        return true;
    }

    inline bool check_covariance(const cm& P) { return !common::filter::hasNaNOrInf(P); }
}

} // namespace common

#endif // COMMON_FILTER_MANAGEMENT_HPP

