// eskf_core.cpp
// Implementation file for ESKF core functions

#include "../../Inc/ESKF/eskf_core.hpp"
#include "../../Inc/ESKF/eskf_math.hpp"
#include "../../Inc/KF/kalman_filter_core.hpp"
#include "../../Lib/Quaternion/quaternion_functions.hpp"
#include "../../Inc/Common/Math/math_utils.hpp"
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
    // 加速度からRoll/Pitchを計算（Yawは変更しない）
    // 実装はeskf_math.hppのaccel_to_quaternionを使用
    using namespace eskf_math;
    Vector4 q_rp;
    ESKFMath::accel_to_quaternion(a_meas, scale_factor, q_rp);
    
    // 現在のYawを保持してRoll/Pitchのみ更新
    // 簡易実装：q_rpとqを合成（実際にはより複雑な処理が必要）
    Scalar q0 = q(0, 0), q1 = q(1, 0), q2 = q(2, 0), q3 = q(3, 0);
    Scalar qrp0 = q_rp(0, 0), qrp1 = q_rp(1, 0), qrp2 = q_rp(2, 0), qrp3 = q_rp(3, 0);
    
    // Roll/Pitchのみの更新（簡易版）
    q(0, 0) = qrp0 * q0 - qrp1 * q1 - qrp2 * q2 - qrp3 * q3;
    q(1, 0) = qrp0 * q1 + qrp1 * q0 + qrp2 * q3 - qrp3 * q2;
    q(2, 0) = qrp0 * q2 - qrp1 * q3 + qrp2 * q0 + qrp3 * q1;
    q(3, 0) = qrp0 * q3 + qrp1 * q2 - qrp2 * q1 + qrp3 * q0;
    
    // 正規化
    Scalar norm = std::sqrt(q(0,0)*q(0,0) + q(1,0)*q(1,0) + q(2,0)*q(2,0) + q(3,0)*q(3,0));
    if (norm > 1e-6f) {
        for (int i = 0; i < 4; ++i) q(i, 0) /= norm;
    }
}

void ESKFCore::update_mag(Vector4& q, Matrix15x15& P, const Vector3& m_meas,
                          const Vector3& m_world, const Matrix3x3& R_mag,
                          cmath_fx::Matrix<15, 3, Scalar>& K_out, Vector15& dx_out) {
    // ESKF磁気計更新（簡易実装）
    // クォータニオンから回転行列を計算
    using namespace common::math;
    Matrix3x3 R;
    cquat::quat_to_rotm(q, R);
    
    // 磁気参照ベクトルをボディ座標に変換: m_body = R^T * m_world
    Vector3 m_body = R.transpose() * m_world;
    
    // 観測行列H = skew(m_body) (3x3)
    Matrix3x3 H_sub;
    H_sub(0, 0) = 0.0f; H_sub(0, 1) = -m_body(2, 0); H_sub(0, 2) = m_body(1, 0);
    H_sub(1, 0) = m_body(2, 0); H_sub(1, 1) = 0.0f; H_sub(1, 2) = -m_body(0, 0);
    H_sub(2, 0) = -m_body(1, 0); H_sub(2, 1) = m_body(0, 0); H_sub(2, 2) = 0.0f;
    
    // 予測値: z_pred = R * m_meas (正規化)
    Vector3 m_meas_norm = m_meas;
    Scalar m_meas_len = 0.0f;
    for(int i=0; i<3; ++i) m_meas_len += m_meas(i,0) * m_meas(i,0);
    m_meas_len = std::sqrt(m_meas_len);
    if (m_meas_len > 1e-9f) {
        for(int i=0; i<3; ++i) m_meas_norm(i, 0) /= m_meas_len;
    }
    
    Vector3 z_pred = R * m_meas_norm;
    
    // イノベーション: y = m_world - z_pred
    Vector3 y = m_world - z_pred;
    
    // P_att = P(6:8, 6:8) (attitude covariance)
    Matrix3x3 P_att;
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            P_att(i, j) = P(6+i, 6+j);
        }
    }
    
    // S = H_sub * P_att * H_sub^T + R_mag
    Matrix3x3 H_P = H_sub * P_att;
    Matrix3x3 H_P_Ht = H_P * H_sub.transpose();
    Matrix3x3 S = H_P_Ht + R_mag;
    
    // S_inv
    Matrix3x3 S_inv;
    if(!S.inverse(S_inv)) {
        K_out = cmath_fx::Matrix<15, 3, Scalar>::Zero();
        dx_out = Vector15::Zero();
        return;
    }
    
    // P_cross = P(:, 6:8) (15x3)
    cmath_fx::Matrix<15, 3, Scalar> P_cross;
    for(int i=0; i<15; ++i) {
        for(int j=0; j<3; ++j) {
            P_cross(i, j) = P(i, 6+j);
        }
    }
    
    // K_full = P_cross * H_sub^T * S_inv
    cmath_fx::Matrix<15, 3, Scalar> tmp = P_cross * H_sub.transpose();
    K_out = tmp * S_inv;
    
    // dx = K * y
    dx_out = K_out * y;
    
    // Update covariance: P = (I - K*H) * P (Joseph form)
    // H = [0,0,0,0,0,0, H_sub, 0...] where H_sub is at columns 6:8
    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 KH_full = Matrix15x15::Zero();
    // Build K*H where H has H_sub at columns 6:8
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            if(j >= 6 && j < 9) {
                for(int k=0; k<3; ++k) {
                    KH_full(i, j) += K_out(i, k) * H_sub(k, j-6);
                }
            }
        }
    }
    Matrix15x15 I_KH = I - KH_full;
    Matrix15x15 P_tmp = I_KH * P * I_KH.transpose();
    // Add K*R*K^T term
    Matrix15x15 K_R_Kt = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            for(int k=0; k<3; ++k) {
                K_R_Kt(i, j) += K_out(i, k) * R_mag(k, k) * K_out(j, k);
            }
        }
    }
    P = P_tmp + K_R_Kt;
    // Symmetrize
    for(int i=0; i<15; ++i) {
        for(int j=i+1; j<15; ++j) {
            Scalar avg = 0.5f * (P(i, j) + P(j, i));
            P(i, j) = avg;
            P(j, i) = avg;
        }
    }
}

void ESKFCore::update_gps(Vector3& p, Vector3& v, Matrix15x15& P,
                          const Vector3& gps_pos, const Vector3& gps_origin,
                          const Matrix3x3& R_gps,
                          cmath_fx::Matrix<15, 3, Scalar>& K_out, Vector15& dx_out) {
    // GPS更新：H = [I3, 0...], z_pred = p, y = gps_pos - p
    Vector3 z_pred = p;
    Vector3 y = gps_pos - z_pred;
    
    // S = H*P*H' + R = P_pos + R
    Matrix3x3 P_pos;
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            P_pos(i, j) = P(i, j);
        }
    }
    
    Matrix3x3 S = P_pos + R_gps;
    
    // S_inv
    Matrix3x3 S_inv;
    if(!S.inverse(S_inv)) {
        // 逆行列計算失敗
        K_out = cmath_fx::Matrix<15, 3, Scalar>::Zero();
        dx_out = Vector15::Zero();
        return;
    }
    
    // K = P*H' * S_inv = P(:,1:3) * S_inv
    cmath_fx::Matrix<15, 3, Scalar> PHt;
    for(int i=0; i<15; ++i) {
        for(int j=0; j<3; ++j) {
            PHt(i, j) = P(i, j);
        }
    }
    
    K_out = PHt * S_inv;
    
    // dx = K * y
    dx_out = K_out * y;
    
    // Update covariance: P = (I - K*H) * P (Joseph form for numerical stability)
    // H = [I3, 0...] for GPS, so K*H = K(:,1:3) at columns 0:2
    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 KH = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<3; ++j) {
            KH(i, j) = K_out(i, j);
        }
    }
    Matrix15x15 I_KH = I - KH;
    Matrix15x15 P_tmp = I_KH * P * I_KH.transpose();
    // Add K*R*K^T term
    Matrix15x15 K_R_Kt = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            for(int k=0; k<3; ++k) {
                K_R_Kt(i, j) += K_out(i, k) * R_gps(k, k) * K_out(j, k);
            }
        }
    }
    P = P_tmp + K_R_Kt;
    // Symmetrize
    for(int i=0; i<15; ++i) {
        for(int j=i+1; j<15; ++j) {
            Scalar avg = 0.5f * (P(i, j) + P(j, i));
            P(i, j) = avg;
            P(j, i) = avg;
        }
    }
}

void ESKFCore::update_baro(Vector3& p, Matrix15x15& P, Scalar altitude,
                           const Vector3& gps_origin, Scalar R_baro,
                           cmath_fx::Matrix<15, 1, Scalar>& K_out, Vector15& dx_out) {
    // Baro更新：H = [0,0,1, zeros(1,12)], z_pred = p(2), y = altitude - p(2)
    Scalar z_pred = p(2, 0);
    Scalar y = altitude - z_pred;
    
    // S = H*P*H' + R = P(2,2) + R_baro
    Scalar P_z = P(2, 2);
    Scalar S = P_z + R_baro;
    
    if (S < 1e-9f) {
        K_out = cmath_fx::Matrix<15, 1, Scalar>::Zero();
        dx_out = Vector15::Zero();
        return;
    }
    
    Scalar S_inv = 1.0f / S;
    
    // K = P*H' / S = P(:,2) / S
    for(int i=0; i<15; ++i) {
        K_out(i, 0) = P(i, 2) * S_inv;
    }
    
    // dx = K * y
    for(int i=0; i<15; ++i) {
        dx_out(i, 0) = K_out(i, 0) * y;
    }
    
    // Update covariance: P = (I - K*H) * P (Joseph form)
    // H = [0,0,1, zeros(1,12)], so K*H has K at column 2
    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 KH = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        KH(i, 2) = K_out(i, 0);
    }
    Matrix15x15 I_KH = I - KH;
    Matrix15x15 P_tmp = I_KH * P * I_KH.transpose();
    // Add K*R*K^T term (R is scalar)
    Matrix15x15 K_R_Kt = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            K_R_Kt(i, j) = K_out(i, 0) * R_baro * K_out(j, 0);
        }
    }
    P = P_tmp + K_R_Kt;
    // Symmetrize
    for(int i=0; i<15; ++i) {
        for(int j=i+1; j<15; ++j) {
            Scalar avg = 0.5f * (P(i, j) + P(j, i));
            P(i, j) = avg;
            P(j, i) = avg;
        }
    }
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

void ESKFCore::compute_adaptive_Q(
    const Matrix15x15& Q_nominal,
    const Vector3& a_meas,
    const Vector3& w_meas,
    Matrix15x15& Q_adapted
) {
    // 初期化
    Q_adapted = Q_nominal;
    
    // 加速度ノルム計算
    Scalar a_norm = static_cast<Scalar>(0.0);
    for (int i = 0; i < 3; ++i) {
        a_norm += a_meas(i, 0) * a_meas(i, 0);
    }
    a_norm = std::sqrt(a_norm);
    
    // 重力誤差に基づくスケール
    Scalar gravity_error = std::fabs(a_norm - static_cast<Scalar>(9.81));
    Scalar accel_scale = static_cast<Scalar>(1.0) + (gravity_error / static_cast<Scalar>(3.0));
    
    // 角速度ノルム計算
    Scalar w_norm = static_cast<Scalar>(0.0);
    for (int i = 0; i < 3; ++i) {
        w_norm += w_meas(i, 0) * w_meas(i, 0);
    }
    w_norm = std::sqrt(w_norm);
    
    // 角速度に基づくスケール（15 deg/s = 15 * π/180 rad/s）
    Scalar deg2rad15 = static_cast<Scalar>(15.0) * static_cast<Scalar>(3.14159265) / static_cast<Scalar>(180.0);
    Scalar gyro_scale = static_cast<Scalar>(1.0) + (w_norm / deg2rad15);
    
    // 最大スケールを選択し、上限を適用
    Scalar q_scale = std::fmax(accel_scale, gyro_scale);
    if (q_scale > static_cast<Scalar>(5.0)) {
        q_scale = static_cast<Scalar>(5.0);
    }
    
    // Q_nominalをスケール
    for (int j = 0; j < 15; ++j) {
        for (int i = 0; i < 15; ++i) {
            Q_adapted(i, j) = Q_nominal(i, j) * q_scale;
        }
    }
}

// ZUPT更新（Kalman filter update）
void ESKFCore::update_zupt(
    const Vector3& v_in,
    const Matrix15x15& P_in,
    Vector3& v_out,
    Matrix15x15& P_out
) {
    using Matrix15x3 = cmath_fx::Matrix<15, 3, Scalar>;
    using namespace common::math;
    
    // ZUPT: Observe velocity = 0
    // z = [0;0;0], h = v
    // y = z - h = -v
    Vector3 y;
    y(0, 0) = -v_in(0, 0);
    y(1, 0) = -v_in(1, 0);
    y(2, 0) = -v_in(2, 0);
    
    // H = [0, I, 0, 0, 0] (observation matrix)
    // S = H*P*H' + R
    // H*P*H' is simply the velocity block of P (indices 3,4,5)
    Matrix3x3 P_vv;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_vv(i, j) = P_in(3 + i, 3 + j);
        }
    }
    
    // R: ZUPT noise covariance (diagonal)
    // MATLAB実装では noise_zupt = [0.01^2; 0.01^2; 0.01^2] = [0.0001; 0.0001; 0.0001]
    Matrix3x3 R = Matrix3x3::Zero();
    float noise_zupt[3] = {0.0001f, 0.0001f, 0.0001f}; // ZUPT noise variance (0.01^2)
    for (int i = 0; i < 3; ++i) {
        R(i, i) = noise_zupt[i];
    }
    
    // S = P_vv + R
    Matrix3x3 S = P_vv + R;
    
    // Invert S
    Matrix3x3 S_inv;
    bool S_is_singular = !MathUtils::invert3x3(S, S_inv);
    
    if (S_is_singular) {
        // Fallback: Sが特異な場合は、単純に速度を0にして共分散を減らす
        v_out(0, 0) = 0.0f;
        v_out(1, 0) = 0.0f;
        v_out(2, 0) = 0.0f;
        
        // P_out = P_in (copy)
        P_out = P_in;
        
        // Reduce velocity variances by factor (indices 3,4,5)
        float factor = 0.01f;
        P_out(3, 3) *= factor;
        P_out(4, 4) *= factor;
        P_out(5, 5) *= factor;
        
        // Also update off-diagonal elements for symmetry
        for (int i = 0; i < 15; ++i) {
            if (i != 3) {
                float val = 0.5f * (P_out(i, 3) + P_out(3, i));
                P_out(i, 3) = val * factor;
                P_out(3, i) = val * factor;
            }
            if (i != 4) {
                float val = 0.5f * (P_out(i, 4) + P_out(4, i));
                P_out(i, 4) = val * factor;
                P_out(4, i) = val * factor;
            }
            if (i != 5) {
                float val = 0.5f * (P_out(i, 5) + P_out(5, i));
                P_out(i, 5) = val * factor;
                P_out(5, i) = val * factor;
            }
        }
    } else {
        // Normal Kalman filter update
        // K = P * H' * S_inv
        // P * H' is the block of columns 3,4,5 of P
        Matrix15x3 PHt;
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 3; ++j) {
                PHt(i, j) = P_in(i, 3 + j);
            }
        }
        
        // K = PHt * S_inv
        Matrix15x3 K = PHt * S_inv;
        
        // dx = K * y
        Vector15 dx = K * y;
        
        // Update velocity: v_out = v_in + dx[3:5]
        v_out(0, 0) = v_in(0, 0) + dx(3, 0);
        v_out(1, 0) = v_in(1, 0) + dx(4, 0);
        v_out(2, 0) = v_in(2, 0) + dx(5, 0);
        
        // Update Covariance: P = (I - K*H) * P
        // K*H is 15x15, but only columns 3,4,5 are non-zero (equal to K)
        Matrix15x15 KH = Matrix15x15::Zero();
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 3; ++j) {
                KH(i, 3 + j) = K(i, j);
            }
        }
        
        Matrix15x15 I_mat = Matrix15x15::Identity();
        P_out = (I_mat - KH) * P_in;
        
        // Symmetrize P_out
        for (int i = 0; i < 15; ++i) {
            for (int j = i + 1; j < 15; ++j) {
                float val = 0.5f * (P_out(i, j) + P_out(j, i));
                P_out(i, j) = val;
                P_out(j, i) = val;
            }
        }
    }
}

} // namespace eskf
