#pragma once

#ifndef ESKF_ESKF_POSTPROCESS_HPP
#define ESKF_ESKF_POSTPROCESS_HPP

#include "../Lib/Matrix/fixed_matrix.hpp"
#include <cstddef>

namespace eskf {

// Predict後処理パラメータ
struct PredictPostprocessParams {
    bool enable_accel_z_integration = false;
    float accel_z_threshold = 0.5f;
    float accel_z_damping = 0.1f;
    float velocity_damping = 0.0f;
};

// Predict後処理結果
struct PredictPostprocessResult {
    cmath_fx::Vector<3, float> v;
    cmath_fx::Matrix<15, 15, float> P;
};

// Predict後処理
// v: 速度ベクトル (3x1) [入出力]
// q: クォータニオン (4x1) [入力]
// P: 共分散行列 (15x15) [入出力]
// a_for_vel: 速度計算用加速度 (3x1) [入力]
// dt: サンプリング時間 [入力]
// g: 重力ベクトル (3x1) [入力]
// params: 後処理パラメータ [入力]
// 戻り値: 後処理結果
void predict_postprocess(
    cmath_fx::Vector<3, float>& v,
    const cmath_fx::Vector<4, float>& q,
    cmath_fx::Matrix<15, 15, float>& P,
    const cmath_fx::Vector<3, float>& a_for_vel,
    float dt,
    const cmath_fx::Vector<3, float>& g,
    const PredictPostprocessParams& params
);

// Update後処理パラメータ
struct UpdatePostprocessParams {
    // 必要に応じて追加
};

// Update後処理結果
struct UpdatePostprocessResult {
    cmath_fx::Vector<3, float> p;
    cmath_fx::Vector<3, float> v;
    cmath_fx::Vector<4, float> q;
    cmath_fx::Vector<3, float> ba;
    cmath_fx::Vector<3, float> bg;
    cmath_fx::Matrix<15, 15, float> P;
    bool should_skip = false;
};

// Update後処理: 状態更新とクォータニオン更新
// dx: 状態更新量 (15x1) [入力]
// state_p, state_v, state_q, state_ba, state_bg: 現在の状態 [入力]
// new_state_P: 更新後の共分散行列 [入力]
// 戻り値: 更新後の状態
UpdatePostprocessResult update_state_from_dx(
    const cmath_fx::Vector<15, float>& dx,
    const cmath_fx::Vector<3, float>& state_p,
    const cmath_fx::Vector<3, float>& state_v,
    const cmath_fx::Vector<4, float>& state_q,
    const cmath_fx::Vector<3, float>& state_ba,
    const cmath_fx::Vector<3, float>& state_bg,
    const cmath_fx::Matrix<15, 15, float>& new_state_P
);

// P行列の対称化
// P: 共分散行列 (15x15) [入出力]
void symmetrize_covariance(cmath_fx::Matrix<15, 15, float>& P);

} // namespace eskf

#endif // ESKF_ESKF_POSTPROCESS_HPP

