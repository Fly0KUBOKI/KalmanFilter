// Copied from Inc/ESKF/eskf_core.hpp
#pragma once

// Implementation: Src/ESKF/eskf_core.cpp

#include "../../Matrix/fixed_matrix.hpp"
#include "../../Quaternion/quaternion_functions.hpp"

namespace eskf {

// Use float for performance
using Scalar = float;

using Vector3 = cmath_fx::Vector<3, Scalar>;
using Vector4 = cmath_fx::Vector<4, Scalar>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, Scalar>;
using Matrix15x15 = cmath_fx::Matrix<15, 15, Scalar>;
using Vector15 = cmath_fx::Vector<15, Scalar>;

class ESKFCore {
public:
    // ノミナル状態積分（RK2/台形則）
    static void integrate_nominal(
        Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg,
        const Vector3& a_meas, const Vector3& w_meas,
        Scalar dt, const Vector3& g,
        const Vector3& gyro_noise_threshold,
        const Vector3& accel_noise_threshold
    );
    
    // ====================================================================
    // NOTE: 以下のセンサー更新関数は MEUKF で置き換え済み（未使用）
    // 実際のセンサー更新は MEUKFCore::step() で実行される
    // ====================================================================
    
    // 加速度更新（Roll/Pitch）- UNUSED (replaced by MEUKFCore::update_accel_meukf)
    static void update_accel(
        Vector4& q,
        const Vector3& a_meas,
        Scalar scale_factor = 1.0
    );
    
    // 磁気計更新 - UNUSED (replaced by MEUKFCore::update_mag_meukf)
    static void update_mag(
        Vector4& q, Matrix15x15& P,
        const Vector3& m_meas,
        const Vector3& m_world,
        const Matrix3x3& R_mag,
        cmath_fx::Matrix<15, 3, Scalar>& K_out,
        Vector15& dx_out
    );
    
    // GPS更新 - UNUSED (replaced by MEUKFCore::update_gps_meukf_ukf_version)
    static void update_gps(
        Vector3& p, Vector3& v, Matrix15x15& P,
        const Vector3& gps_pos,
        const Vector3& gps_origin,
        const Matrix3x3& R_gps,
        cmath_fx::Matrix<15, 3, Scalar>& K_out,
        Vector15& dx_out
    );
    
    // 気圧計更新 - UNUSED (replaced by MEUKFCore::update_baro_meukf_ukf_version)
    static void update_baro(
        Vector3& p, Matrix15x15& P,
        Scalar altitude,
        const Vector3& gps_origin,
        Scalar R_baro,
        cmath_fx::Matrix<15, 1, Scalar>& K_out,
        Vector15& dx_out
    );
    
    // ====================================================================
    // 以下は ESKF の予測・共分散管理で使用中（削除不可）
    // ====================================================================
    
    // 共分散予測
    static void predict_covariance(const Matrix15x15& P, const Vector4& q, const Vector3& a_meas, const Vector3& ba,
                                   const Vector3& w_meas, const Vector3& bg, const Matrix15x15& Q, Scalar dt,
                                   Matrix15x15& P_new);
    
    // Adaptive Q scaling（MATLAB実装に合わせた）
    static void compute_adaptive_Q(
        const Matrix15x15& Q_nominal,
        const Vector3& a_meas,
        const Vector3& w_meas,
        Matrix15x15& Q_adapted
    );
    
    // 状態遷移行列計算
    static void compute_F_matrix(const Vector4& q, const Vector3& a_meas, const Vector3& ba,
                                 const Vector3& w_meas, const Vector3& bg, Scalar dt,
                                 Matrix15x15& F);
    
    // 誤差状態注入
    static void inject_error_state(Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg, const Vector15& dx);

    // ZUPT更新（Kalman filter update）
    static void update_zupt(
        const Vector3& v_in,
        const Matrix15x15& P_in,
        Vector3& v_out,
        Matrix15x15& P_out
    );

private:
    static Vector3 prev_a_world;
    static Vector3 prev_v;
    static bool prev_initialized;
    
    static Scalar pressure_to_altitude(Scalar pressure);
    static void gps_to_local(const Vector3& gps_pos, const Vector3& origin, Vector3& local_pos);
};

} // namespace eskf
