#pragma once

#include "compute_types.hpp"

namespace kalman_compute {

/**
 * ESKF計算ライブラリ
 * 
 * 設計方針:
 * - すべての関数は状態非依存の純粋計算関数
 * - 入力: input (行列/ベクトル)
 * - 出力: output (行列/ベクトル)
 * - センサー種類などの状態管理はMATLAB側で実施
 */
class ESKFCompute {
public:
    
    /**
     * 共分散予測: P_new = F * P * F' + Q
     * input: [P(flatten 15x15); F(flatten 15x15); Q(flatten 15x15)] -> 675x1
     * output: P_new(flatten 15x15) -> 225x1
     * Note: 行列はrow-major orderでflattenされる
     */
    static void covariance_prediction(const Scalar* input, Scalar* output);
    
    /**
     * 状態遷移行列F計算
     * input: [q(4); a_meas(3); ba(3); w_meas(3); bg(3); dt(1)] -> 17x1
     * output: F(flatten 15x15) -> 225x1
     */
    static void compute_F_matrix(const Scalar* input, Scalar* output);
    
    /**
     * 誤差状態注入
     * input: [p(3); v(3); q(4); ba(3); bg(3); dx(15)] -> 31x1
     * output: [p_new(3); v_new(3); q_new(4); ba_new(3); bg_new(3)] -> 16x1
     */
    static void inject_error_state(const Scalar* input, Scalar* output);
    
    /**
     * カルマン更新 (汎用)
     * input: [x(Nx1); P(NxN flat); y(Mx1); H(MxN flat); R(MxM flat); N(1); M(1)]
     * output: [x_new(Nx1); P_new(NxN flat); K(NxM flat); S(MxM flat)]
     */
    static void kalman_update(const Scalar* input, Scalar* output);
    
    /**
     * 位置速度積分 (RK2/AB2)
     * input: [p(3); v(3); a_world(3); g(3); dt(1); prev_a(3); prev_v(3); 
     *         use_ab2(1); max_accel(1); max_vel(1)] -> 23x1
     * output: [p_new(3); v_new(3); a_out(3); v_out(3)] -> 12x1
     */
    static void pv_integration(const Scalar* input, Scalar* output);
    
    /**
     * GPS座標→ローカル座標変換
     * input: [gps_pos(3); origin_pos(3)] -> 6x1
     * output: local_pos(3)
     */
    static void gps_to_local(const Scalar* input, Scalar* output);
    
    /**
     * 気圧→高度変換
     * input: pressure(1) in Pa
     * output: altitude(1) in meters
     */
    static void pressure_to_altitude(const Scalar* input, Scalar* output);
    
    /**
     * 磁気ベクトル予測 (ワールド→ボディ)
     * input: [q(4); m_world(3)] -> 7x1
     * output: m_body(3)
     */
    static void mag_observation_prediction(const Scalar* input, Scalar* output);

private:
    static constexpr Scalar EPS = 1e-9f;
    static constexpr uint8_t STATE_DIM = 15;  // ESKF state dimension
    
    // ヘルパー関数
    static void matrix_multiply(
        const Scalar* A, const Scalar* B, Scalar* C,
        uint8_t rows_A, uint8_t cols_A, uint8_t cols_B
    );
    
    static void matrix_transpose(
        const Scalar* A, Scalar* A_T,
        uint8_t rows, uint8_t cols
    );
    
    static void matrix_add(
        const Scalar* A, const Scalar* B, Scalar* C,
        uint8_t rows, uint8_t cols
    );
    
    static bool matrix_inverse(
        const Scalar* A, Scalar* A_inv,
        uint8_t n
    );
    
    static void symmetrize(Scalar* M, uint8_t n);
};

} // namespace kalman_compute
