#pragma once

#ifndef ESKF_ESKF_RUNNER_HPP
#define ESKF_ESKF_RUNNER_HPP

#include "eskf_state.hpp"
#include "eskf_core.hpp"
#include "eskf_postprocess.hpp"
#include "../Lib/Matrix/fixed_matrix.hpp"
#include "Common/Sensor/sensor_filter.hpp"
#include <cstddef>

namespace eskf {

/**
 * ESKF Runner - 統合実行クラス
 * 
 * 予測ステップ、後処理、共分散正規化などの統合機能を提供
 */
class ESKFRunner {
public:
    ESKFRunner();
    ~ESKFRunner() = default;
    
    /**
     * 予測ステップ（統合版）
     * 
     * @param s ESKFState構造体へのポインタ [入出力]
     * @param a_meas 加速度測定値 (3x1) [入力]
     * @param w_meas 角速度測定値 (3x1) [入力]
     * 
     * この関数は以下を実行：
     * 1. Adaptive Q scaling
     * 2. Nominal state integration
     * 3. Covariance prediction
     * 4. Predict postprocess (accel_z_integration, velocity_damping, etc.)
     * 5. Covariance regularization
     * 6. Velocity clipping
     */
    static void predict(ESKFState* s, const double* a_meas, const double* w_meas);
    
private:
    // 内部ヘルパー関数
    static void apply_accel_z_integration(
        cmath_fx::Vector<3, float>& v,
        const cmath_fx::Vector<4, float>& q,
        const cmath_fx::Vector<3, float>& a_for_vel,
        float dt,
        const cmath_fx::Vector<3, float>& g,
        float accel_z_threshold,
        float accel_z_damping
    );
    
    static void apply_velocity_clipping(
        cmath_fx::Vector<3, float>& v,
        cmath_fx::Matrix<15, 15, float>& P,
        float max_vel
    );
    
    static void regularize_covariance(
        cmath_fx::Matrix<15, 15, float>& P
    );
};

} // namespace eskf

#endif // ESKF_ESKF_RUNNER_HPP


