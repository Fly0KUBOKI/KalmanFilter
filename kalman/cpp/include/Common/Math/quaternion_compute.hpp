#pragma once

#include "compute_types.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace kalman_compute {

/**
 * Quaternion計算ライブラリ
 * 
 * 設計方針:
 * - すべての関数は状態非依存の純粋計算関数
 * - 入力: input (行列/ベクトル)
 * - 出力: output (行列/ベクトル)
 * - クォータニオンは [w, x, y, z] の4x1ベクトルとして扱う
 */
class QuaternionCompute {
public:
    
    /**
     * クォータニオン積
     * input: [q1(4x1); q2(4x1)] -> 8x1
     * output: q_result(4x1)
     */
    static void multiply(const Scalar* input, Scalar* output);
    
    /**
     * クォータニオン正規化
     * input: q(4x1)
     * output: q_normalized(4x1)
     */
    static void normalize(const Scalar* input, Scalar* output);
    
    /**
     * クォータニオン共役
     * input: q(4x1)
     * output: q_conjugate(4x1)
     */
    static void conjugate(const Scalar* input, Scalar* output);
    
    /**
     * クォータニオン逆元
     * input: q(4x1)
     * output: q_inverse(4x1)
     */
    static void inverse(const Scalar* input, Scalar* output);
    
    /**
     * クォータニオン→回転行列変換
     * input: q(4x1)
     * output: R(9x1) - row-major order [R11,R12,R13,R21,R22,R23,R31,R32,R33]
     */
    static void to_rotation_matrix(const Scalar* input, Scalar* output);
    
    /**
     * クォータニオン→オイラー角変換 (度数法)
     * input: q(4x1)
     * output: euler(3x1) - [roll, pitch, yaw] in degrees
     */
    static void to_euler(const Scalar* input, Scalar* output);
    
    /**
     * オイラー角→クォータニオン変換 (度数法)
     * input: euler(3x1) - [roll, pitch, yaw] in degrees
     * output: q(4x1)
     */
    static void from_euler(const Scalar* input, Scalar* output);
    
    /**
     * 小角度回転ベクトル→クォータニオン
     * input: theta(3x1) - [theta_x, theta_y, theta_z] in radians
     * output: q(4x1)
     */
    static void from_small_angle(const Scalar* input, Scalar* output);
    
    /**
     * 角速度積分によるクォータニオン更新
     * input: [q(4x1); omega(3x1); dt(1x1)] -> 8x1
     * output: q_new(4x1)
     */
    static void integrate(const Scalar* input, Scalar* output);
    
    /**
     * 2つのクォータニオン間の角度 (度数法)
     * input: [q1(4x1); q2(4x1)] -> 8x1
     * output: angle(1x1) in degrees
     */
    static void angle_between(const Scalar* input, Scalar* output);
    
    /**
     * 球面線形補間 (SLERP)
     * input: [q1(4x1); q2(4x1); t(1x1)] -> 9x1, t in [0,1]
     * output: q_interp(4x1)
     */
    static void slerp(const Scalar* input, Scalar* output);
    
    /**
     * 軸角→クォータニオン
     * input: [axis(3x1); angle(1x1)] -> 4x1, angle in radians
     * output: q(4x1)
     */
    static void from_axis_angle(const Scalar* input, Scalar* output);
    
    /**
     * クォータニオン→軸角
     * input: q(4x1)
     * output: [axis(3x1); angle(1x1)] -> 4x1, angle in radians
     */
    static void to_axis_angle(const Scalar* input, Scalar* output);
    
    /**
     * クォータニオン内積
     * input: [q1(4x1); q2(4x1)] -> 8x1
     * output: dot(1x1)
     */
    static void dot(const Scalar* input, Scalar* output);

private:
    static constexpr Scalar EPS = 1e-9f;
    
    // ヘルパー関数
    static inline Scalar safe_sqrt(Scalar x) {
        return (x > 0.0f) ? std::sqrt(x) : 0.0f;
    }
    
    static inline Scalar safe_asin(Scalar x) {
        return std::asin(std::max(-1.0f, std::min(1.0f, x)));
    }
};

} // namespace kalman_compute
