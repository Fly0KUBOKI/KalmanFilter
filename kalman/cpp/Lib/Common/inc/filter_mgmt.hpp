#pragma once

#ifndef COMMON_FILTER_MANAGEMENT_HPP
#define COMMON_FILTER_MANAGEMENT_HPP

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
} // namespace common

#endif // COMMON_FILTER_MANAGEMENT_HPP

