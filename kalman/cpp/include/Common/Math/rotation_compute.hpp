#pragma once

#include "compute_types.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace kalman_compute {

/**
 * Rotation計算ライブラリ
 * 
 * 設計方針:
 * - すべての関数は状態非依存の純粋計算関数
 * - 入力: input (行列/ベクトル)
 * - 出力: output (行列/ベクトル)
 * - 回転行列は3x3のrow-major order (9x1ベクトル) として扱う
 */
class RotationCompute {
public:
    
    /**
     * 歪対称行列生成
     * input: v(3x1)
     * output: skew(9x1) - row-major [0,-vz,vy; vz,0,-vx; -vy,vx,0]
     */
    static void skew_symmetric(const Scalar* input, Scalar* output);
    
    /**
     * X軸周りの回転行列
     * input: angle(1x1) - radians
     * output: R(9x1) - row-major
     */
    static void rotation_x(const Scalar* input, Scalar* output);
    
    /**
     * Y軸周りの回転行列
     * input: angle(1x1) - radians
     * output: R(9x1) - row-major
     */
    static void rotation_y(const Scalar* input, Scalar* output);
    
    /**
     * Z軸周りの回転行列
     * input: angle(1x1) - radians
     * output: R(9x1) - row-major
     */
    static void rotation_z(const Scalar* input, Scalar* output);
    
    /**
     * オイラー角→回転行列 (度数法)
     * input: euler(3x1) - [roll, pitch, yaw] in degrees
     * output: R(9x1) - row-major
     */
    static void from_euler(const Scalar* input, Scalar* output);
    
    /**
     * 回転行列→オイラー角 (度数法)
     * input: R(9x1) - row-major
     * output: euler(3x1) - [roll, pitch, yaw] in degrees
     */
    static void to_euler(const Scalar* input, Scalar* output);
    
    /**
     * ロドリゲスの公式による回転
     * input: [axis(3x1); angle(1x1)] -> 4x1, angle in radians
     * output: R(9x1) - row-major
     */
    static void rodrigues(const Scalar* input, Scalar* output);
    
    /**
     * 軸角→回転行列
     * input: [axis(3x1); angle(1x1)] -> 4x1, angle in radians
     * output: R(9x1) - row-major
     */
    static void from_axis_angle(const Scalar* input, Scalar* output);
    
    /**
     * 回転行列→軸角
     * input: R(9x1) - row-major
     * output: [axis(3x1); angle(1x1)] -> 4x1, angle in radians
     */
    static void to_axis_angle(const Scalar* input, Scalar* output);
    
    /**
     * 回転行列の正規直交化 (Gram-Schmidt)
     * input: R(9x1) - row-major
     * output: R_ortho(9x1) - row-major orthonormalized
     */
    static void orthonormalize(const Scalar* input, Scalar* output);
    
    /**
     * 回転行列の逆 (転置)
     * input: R(9x1) - row-major
     * output: R_inv(9x1) - row-major
     */
    static void inverse(const Scalar* input, Scalar* output);
    
    /**
     * 回転行列の正当性検証
     * input: R(9x1) - row-major
     * output: is_valid(1x1) - 1.0 if valid, 0.0 if invalid
     */
    static void is_valid(const Scalar* input, Scalar* output);
    
    /**
     * ベクトルに回転を適用
     * input: [R(9x1); v(3x1)] -> 12x1
     * output: v_rotated(3x1) = R * v
     */
    static void apply_rotation(const Scalar* input, Scalar* output);
    
    /**
     * 2つの回転行列の合成
     * input: [R1(9x1); R2(9x1)] -> 18x1
     * output: R_result(9x1) = R1 * R2
     */
    static void compose(const Scalar* input, Scalar* output);

private:
    static constexpr Scalar EPS = 1e-9f;
    
    // ヘルパー関数
    static inline Scalar safe_sqrt(Scalar x) {
        return (x > 0.0f) ? std::sqrt(x) : 0.0f;
    }
    
    static inline Scalar safe_acos(Scalar x) {
        return std::acos(std::max(-1.0f, std::min(1.0f, x)));
    }
    
    // 3x3行列乗算 (row-major)
    static void matrix_multiply_3x3(const Scalar* A, const Scalar* B, Scalar* C);
    
    // ベクトル正規化
    static void normalize_vector_3(Scalar* v);
};

} // namespace kalman_compute
