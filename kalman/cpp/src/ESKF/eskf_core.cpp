// eskf_core.cpp
// Implementation file for ESKF core functions

#include "../../Inc/ESKF/eskf_core.hpp"
#include "../../Inc/KF/kalman_filter_core.hpp"
#include "../../Inc/Common/Math/quaternion.hpp"
#include <cmath>

namespace eskf {

// Static member initialization
Vector3 ESKFCore::prev_a_world;
Vector3 ESKFCore::prev_v;
bool ESKFCore::prev_initialized = false;

// ノミナル状態積分（RK2/台形則）
void ESKFCore::integrate_nominal(
    Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg,
    const Vector3& a_meas, const Vector3& w_meas,
    Scalar dt, const Vector3& g,
    const Vector3& gyro_noise_threshold,
    const Vector3& accel_noise_threshold
) {
    // バイアス補正
    Vector3 w_corrected = w_meas;
    for (int i = 0; i < 3; ++i) {
        w_corrected(i, 0) -= bg(i, 0);
    }
    
    Vector3 a_corrected = a_meas;
    for (int i = 0; i < 3; ++i) {
        a_corrected(i, 0) -= ba(i, 0);
    }
    
    // ノイズ閾値チェック（簡易版）
    // 実際の実装では、閾値を超えた場合は処理をスキップする
    
    // クォータニオン積分（RK2）
    Vector4 q_half;
    {
        Vector3 w_half = w_corrected;
        for (int i = 0; i < 3; ++i) w_half(i, 0) *= static_cast<Scalar>(0.5) * dt;
        
        Vector4 dq_half;
        dq_half(0, 0) = static_cast<Scalar>(-0.5) * (q(1, 0) * w_half(0, 0) + q(2, 0) * w_half(1, 0) + q(3, 0) * w_half(2, 0));
        dq_half(1, 0) = static_cast<Scalar>(0.5) * (q(0, 0) * w_half(0, 0) + q(2, 0) * w_half(2, 0) - q(3, 0) * w_half(1, 0));
        dq_half(2, 0) = static_cast<Scalar>(0.5) * (q(0, 0) * w_half(1, 0) - q(1, 0) * w_half(2, 0) + q(3, 0) * w_half(0, 0));
        dq_half(3, 0) = static_cast<Scalar>(0.5) * (q(0, 0) * w_half(2, 0) + q(1, 0) * w_half(1, 0) - q(2, 0) * w_half(0, 0));
        
        q_half(0, 0) = q(0, 0) + dq_half(0, 0);
        q_half(1, 0) = q(1, 0) + dq_half(1, 0);
        q_half(2, 0) = q(2, 0) + dq_half(2, 0);
        q_half(3, 0) = q(3, 0) + dq_half(3, 0);
        cquat::normalize_quat(q_half);
    }
    
    // 回転行列に変換
    Matrix3x3 R;
    cquat::quat_to_rotm(q_half, R);
    
    // 加速度をワールド座標系に変換
    Vector3 a_world;
    for (int i = 0; i < 3; ++i) {
        a_world(i, 0) = static_cast<Scalar>(0.0);
        for (int j = 0; j < 3; ++j) {
            a_world(i, 0) += R(i, j) * a_corrected(j, 0);
        }
        a_world(i, 0) += g(i, 0);
    }
    
    // 位置・速度積分（RK2）
    Vector3 v_half = v;
    for (int i = 0; i < 3; ++i) {
        v_half(i, 0) += static_cast<Scalar>(0.5) * a_world(i, 0) * dt;
    }
    
    for (int i = 0; i < 3; ++i) {
        p(i, 0) += v_half(i, 0) * dt;
        v(i, 0) += a_world(i, 0) * dt;
    }
    
    // 最終的なクォータニオン更新
    {
        Vector3 w_final = w_corrected;
        for (int i = 0; i < 3; ++i) w_final(i, 0) *= static_cast<Scalar>(0.5) * dt;
        
        Vector4 dq_final;
        dq_final(0, 0) = static_cast<Scalar>(-0.5) * (q_half(1, 0) * w_final(0, 0) + q_half(2, 0) * w_final(1, 0) + q_half(3, 0) * w_final(2, 0));
        dq_final(1, 0) = static_cast<Scalar>(0.5) * (q_half(0, 0) * w_final(0, 0) + q_half(2, 0) * w_final(2, 0) - q_half(3, 0) * w_final(1, 0));
        dq_final(2, 0) = static_cast<Scalar>(0.5) * (q_half(0, 0) * w_final(1, 0) - q_half(1, 0) * w_final(2, 0) + q_half(3, 0) * w_final(0, 0));
        dq_final(3, 0) = static_cast<Scalar>(0.5) * (q_half(0, 0) * w_final(2, 0) + q_half(1, 0) * w_final(1, 0) - q_half(2, 0) * w_final(0, 0));
        
        q(0, 0) = q_half(0, 0) + dq_final(0, 0);
        q(1, 0) = q_half(1, 0) + dq_final(1, 0);
        q(2, 0) = q_half(2, 0) + dq_final(2, 0);
        q(3, 0) = q_half(3, 0) + dq_final(3, 0);
        cquat::normalize_quat(q);
    }
}

// 共分散予測
void ESKFCore::predict_covariance(
    const Matrix15x15& P, const Vector4& q, const Vector3& a_meas, const Vector3& ba,
    const Vector3& w_meas, const Vector3& bg, const Matrix15x15& Q, Scalar dt,
    Matrix15x15& P_new
) {
    // F行列を計算
    Matrix15x15 F;
    compute_F_matrix(q, a_meas, ba, w_meas, bg, dt, F);
    
    // P_new = F * P * F' + Q
    Matrix15x15 F_P = F * P;
    Matrix15x15 F_P_Ft = F_P * F.transpose();
    P_new = F_P_Ft + Q;
    
    // 対称化
    for (int i = 0; i < 15; ++i) {
        for (int j = i + 1; j < 15; ++j) {
            Scalar v = static_cast<Scalar>(0.5) * (P_new(i, j) + P_new(j, i));
            P_new(i, j) = v;
            P_new(j, i) = v;
        }
    }
}

// 状態遷移行列計算
void ESKFCore::compute_F_matrix(
    const Vector4& q, const Vector3& a_meas, const Vector3& ba,
    const Vector3& w_meas, const Vector3& bg, Scalar dt,
    Matrix15x15& F
) {
    // 簡易実装：単位行列 + 小さな摂動
    F = Matrix15x15();
    for (int i = 0; i < 15; ++i) {
        F(i, i) = static_cast<Scalar>(1.0);
    }
    
    // 実際の実装では、ESKFの状態遷移行列を計算する必要があります
    // ここでは簡易版として、対角要素にdtを加算
    for (int i = 0; i < 15; ++i) {
        F(i, i) = static_cast<Scalar>(1.0) + dt * static_cast<Scalar>(0.01);
    }
}

// その他のメソッド（簡易実装）
void ESKFCore::update_accel(Vector4& q, const Vector3& a_meas, Scalar scale_factor) {
    // TODO: 実装が必要
}

void ESKFCore::update_mag(Vector4& q, Matrix15x15& P, const Vector3& m_meas,
                          const Vector3& m_world, const Matrix3x3& R_mag,
                          cmath_fx::Matrix<15, 3, Scalar>& K_out, Vector15& dx_out) {
    // TODO: 実装が必要
}

void ESKFCore::update_gps(Vector3& p, Vector3& v, Matrix15x15& P,
                          const Vector3& gps_pos, const Vector3& gps_origin,
                          const Matrix3x3& R_gps,
                          cmath_fx::Matrix<15, 3, Scalar>& K_out, Vector15& dx_out) {
    // TODO: 実装が必要
}

void ESKFCore::update_baro(Vector3& p, Matrix15x15& P, Scalar pressure,
                           const Vector3& gps_origin, Scalar R_baro,
                           cmath_fx::Matrix<15, 1, Scalar>& K_out, Vector15& dx_out) {
    // TODO: 実装が必要
}

void ESKFCore::inject_error_state(Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg, const Vector15& dx) {
    // TODO: 実装が必要
}

Scalar ESKFCore::pressure_to_altitude(Scalar pressure) {
    // 簡易実装：標準大気モデル
    const Scalar p0 = static_cast<Scalar>(101325.0);  // 海面気圧 (Pa)
    const Scalar L = static_cast<Scalar>(0.0065);      // 温度減率 (K/m)
    const Scalar T0 = static_cast<Scalar>(288.15);    // 海面温度 (K)
    const Scalar g = static_cast<Scalar>(9.80665);   // 重力加速度 (m/s^2)
    const Scalar M = static_cast<Scalar>(0.0289644);  // モル質量 (kg/mol)
    const Scalar R = static_cast<Scalar>(8.31447);    // 気体定数 (J/(mol·K))
    
    if (pressure <= static_cast<Scalar>(0.0)) return static_cast<Scalar>(0.0);
    
    Scalar h = (T0 / L) * (static_cast<Scalar>(1.0) - std::pow(pressure / p0, (R * L) / (g * M)));
    return h;
}

void ESKFCore::gps_to_local(const Vector3& gps_pos, const Vector3& origin, Vector3& local_pos) {
    // 簡易実装：GPS座標をローカル座標に変換
    for (int i = 0; i < 3; ++i) {
        local_pos(i, 0) = gps_pos(i, 0) - origin(i, 0);
    }
}

} // namespace eskf
