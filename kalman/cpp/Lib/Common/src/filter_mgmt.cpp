#include "../inc/filter_mgmt.hpp"
#include <cmath>

namespace common {
namespace filter {

bool hasNaNOrInf(const cmath_fx::Matrix<15, 15, float>& P) {
    for (int j = 0; j < 15; ++j) {
        for (int i = 0; i < 15; ++i) {
            float v = P(i, j);
            if (std::isnan(v) || std::isinf(v)) {
                return true;
            }
        }
    }
    return false;
}

void setIdentityScaled(cmath_fx::Matrix<15, 15, float>& P, float scale) {
    for (int j = 0; j < 15; ++j) {
        for (int i = 0; i < 15; ++i) {
            P(i, j) = 0.0f;
        }
    }
    for (int i = 0; i < 15; ++i) {
        P(i, i) = scale;
    }
}

bool check_divergence(const cmath_fx::Matrix<15, 15, float>& P) {
    if (hasNaNOrInf(P)) {
        return true;
    }
    
    // 対角要素の大きな分散をチェック
    for (int i = 0; i < 15; ++i) {
        if (P(i, i) > 1e8f) {
            return true;
        }
    }
    
    return false;
}

void apply_zupt(
    const cmath_fx::Vector<3, float>& v_in,
    const cmath_fx::Matrix<15, 15, float>& P_in,
    cmath_fx::Vector<3, float>& v_out,
    cmath_fx::Matrix<15, 15, float>& P_out
) {
    // 速度をゼロに
    v_out(0, 0) = 0.0f;
    v_out(1, 0) = 0.0f;
    v_out(2, 0) = 0.0f;
    
    // 共分散行列をコピー
    P_out = P_in;
    
    // 速度分散を減らす（インデックス3,4,5）
    float factor = 0.01f;
    for (int idx = 3; idx < 6; ++idx) {
        P_out(idx, idx) = P_out(idx, idx) * factor;
    }
}

void normalize_covariance(cmath_fx::Matrix<15, 15, float>& P) {
    // 最大分散の定義（MATLAB実装と一致）
    float max_var[15];
    max_var[0] = max_var[1] = max_var[2] = 100.0f * 100.0f;  // position
    max_var[3] = max_var[4] = max_var[5] = 20.0f * 20.0f;    // velocity
    // deg2rad(45) = 0.7853981633974483 (double precision)
    float deg45_rad = 0.7853981633974483f;
    max_var[6] = max_var[7] = max_var[8] = deg45_rad * deg45_rad;  // attitude
    max_var[9] = max_var[10] = max_var[11] = 0.1f;  // accel bias
    max_var[12] = max_var[13] = max_var[14] = 0.01f;  // gyro bias
    
    for (int i = 0; i < 15; ++i) {
        if (P(i, i) > max_var[i]) {
            float factor = std::sqrt(max_var[i] / P(i, i));
            for (int j = 0; j < 15; ++j) {
                P(i, j) = P(i, j) * factor;
                P(j, i) = P(j, i) * factor;
            }
            P(i, i) = max_var[i];
        }
    }
}

void symmetrize_covariance(cmath_fx::Matrix<15, 15, float>& P) {
    for (int i = 0; i < 15; ++i) {
        for (int j = i+1; j < 15; ++j) {
            float avg = 0.5f * (P(i, j) + P(j, i));
            P(i, j) = avg;
            P(j, i) = avg;
        }
    }
}

bool check_state_divergence(
    const cmath_fx::Vector<3, float>& p,
    const cmath_fx::Vector<3, float>& v,
    const cmath_fx::Vector<4, float>& q,
    const cmath_fx::Vector<3, float>& ba,
    const cmath_fx::Vector<3, float>& bg,
    const cmath_fx::Matrix<15, 15, float>& P
) {
    // 共分散行列の発散チェック
    if (check_divergence(P)) {
        return true;
    }
    
    // ノルム計算
    float p_norm = std::sqrt(p(0,0)*p(0,0) + p(1,0)*p(1,0) + p(2,0)*p(2,0));
    float v_norm = std::sqrt(v(0,0)*v(0,0) + v(1,0)*v(1,0) + v(2,0)*v(2,0));
    float ba_norm = std::sqrt(ba(0,0)*ba(0,0) + ba(1,0)*ba(1,0) + ba(2,0)*ba(2,0));
    float bg_norm = std::sqrt(bg(0,0)*bg(0,0) + bg(1,0)*bg(1,0) + bg(2,0)*bg(2,0));
    
    // NaN/Infチェック
    if (std::isnan(p(0,0)) || std::isnan(v(0,0)) || std::isnan(q(0,0))) return true;
    if (std::isnan(ba(0,0)) || std::isnan(bg(0,0))) return true;
    if (std::isinf(p(0,0)) || std::isinf(v(0,0))) return true;
    
    // 不合理な値のチェック
    if (v_norm > 10.0f || p_norm > 1000.0f) return true;
    
    // バイアス発散チェック（bg > 1 rad/s = 57 deg/s）
    if (bg_norm > 1.0f) return true;
    
    return false;
}

bool check_zupt_condition(
    const cmath_fx::Vector<3, float>& a_meas,
    const cmath_fx::Vector<3, float>& w_meas,
    float zupt_threshold_accel,
    float zupt_threshold_gyro
) {
    // 加速度ノルム計算
    float a_norm = std::sqrt(a_meas(0,0)*a_meas(0,0) + a_meas(1,0)*a_meas(1,0) + a_meas(2,0)*a_meas(2,0));
    
    // 角速度ノルム計算
    float w_norm = std::sqrt(w_meas(0,0)*w_meas(0,0) + w_meas(1,0)*w_meas(1,0) + w_meas(2,0)*w_meas(2,0));
    
    // 静止状態チェック（重力加速度に近く、角速度が小さい）
    const float GRAVITY = 9.80665f;
    bool stationary = (std::abs(a_norm - GRAVITY) < zupt_threshold_accel) && (w_norm < zupt_threshold_gyro);
    
    return stationary;
}

void reset_state_on_divergence(
    cmath_fx::Vector<3, float>& v,
    cmath_fx::Vector<3, float>& ba,
    cmath_fx::Vector<3, float>& bg,
    cmath_fx::Vector<4, float>& q,
    cmath_fx::Matrix<15, 15, float>& P
) {
    // 速度とバイアスをゼロに
    v(0, 0) = v(1, 0) = v(2, 0) = 0.0f;
    ba(0, 0) = ba(1, 0) = ba(2, 0) = 0.0f;
    bg(0, 0) = bg(1, 0) = bg(2, 0) = 0.0f;
    
    // P行列の対角ブロックをリセット
    // 位置: 20.0
    P(0, 0) = P(1, 1) = P(2, 2) = 20.0f;
    // 速度: 2.0
    P(3, 3) = P(4, 4) = P(5, 5) = 2.0f;
    // 姿勢: deg2rad(30)^2
    const float deg30_rad = 30.0f * 3.14159265f / 180.0f;
    const float deg30_rad_sq = deg30_rad * deg30_rad;
    P(6, 6) = P(7, 7) = P(8, 8) = deg30_rad_sq;
    
    // クォータニオンのノルムチェックとリセット
    float q_norm = std::sqrt(q(0,0)*q(0,0) + q(1,0)*q(1,0) + q(2,0)*q(2,0) + q(3,0)*q(3,0));
    if (std::isnan(q_norm) || q_norm < 0.5f) {
        q(0, 0) = 1.0f;
        q(1, 0) = q(2, 0) = q(3, 0) = 0.0f;
    }
}

} // namespace filter
} // namespace common

