#include "eskf_core.hpp"
#include "../KF/Core/kalman_filter_core.hpp"
#include "../Common/Math/quaternion.hpp"
#include <cmath>
#include <algorithm>

namespace eskf {

// 静的メンバ初期化
Vector3 ESKFCore::prev_a_world;
Vector3 ESKFCore::prev_v;
bool ESKFCore::prev_initialized = false;

void ESKFCore::integrate_nominal(
    Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg,
    const Vector3& a_meas, const Vector3& w_meas,
    float dt, const Vector3& g,
    const Vector3& gyro_noise_threshold,
    const Vector3& accel_noise_threshold
) {
    // バイアス補正
    Vector3 a = a_meas - ba;
    Vector3 w = w_meas - bg;
    
    // 加速度閾値処理
    for (int i=0; i<3; ++i) {
        if (std::abs(a(i,0)) < accel_noise_threshold(i,0) * dt) {
            a(i,0) = 0.0f;
        }
    }
    
    // 角速度積分
    Vector3 w_dt = w * dt;
    
    float w_dt_norm = 0.0f;
    for (int i=0; i<3; ++i) w_dt_norm += w_dt(i,0) * w_dt(i,0);
    w_dt_norm = std::sqrt(w_dt_norm);
    
    float threshold = 1e-15f;
    if (w_dt_norm > threshold) {
        float half_angle = w_dt_norm / 2.0f;
        Vector4 delta_q;
        
        if (half_angle > 1e-6f) {
            float sin_half = std::sin(half_angle);
            float cos_half = std::cos(half_angle);
            Vector3 w_unit = w_dt * (1.0f / w_dt_norm);
            
            delta_q(0,0) = cos_half;
            for (int i=0; i<3; ++i) delta_q(i+1,0) = w_unit(i,0) * sin_half;
        } else {
            // Taylor展開近似
            float w_norm_sq = w_dt_norm * w_dt_norm;
            delta_q(0,0) = 1.0f - w_norm_sq/8.0f;
            for (int i=0; i<3; ++i) {
                delta_q(i+1,0) = w_dt(i,0) * 0.5f * (1.0f - w_norm_sq/24.0f);
            }
        }
        
        Vector4 q_new;
        cquat::multiply_quat(q, delta_q, q_new);
        q = q_new;
        
        // 正規化
        cquat::normalize_quat(q);
    }
    
    // 回転行列（body->world）
    Matrix3x3 Rb;
    cquat::quat_to_rotm(q, Rb);
    
    // 世界座標系での比力
    Vector3 a_world = Rb * a;
    
    // Adams-Bashforth2 積分
    if (!prev_initialized) {
        // 初回: Forward Euler
        Vector3 v_prev = v;
        Vector3 v_candidate = v + (a_world + g) * dt;
        
        // 速度発散防止
        float max_accel = 200.0f;
        Vector3 dv = v_candidate - v;
        
        float dv_norm = 0.0f;
        for (int i=0; i<3; ++i) dv_norm += dv(i,0) * dv(i,0);
        dv_norm = std::sqrt(dv_norm);
        
        float max_dv = max_accel * dt;
        if (dv_norm > max_dv) {
            float scale = max_dv / dv_norm;
            dv = dv * scale;
        }
        
        v = v + dv;
        
        // 速度クリップ
        float max_velocity = 200.0f;
        for (int i=0; i<3; ++i) {
            v(i,0) = std::max(std::min(v(i,0), max_velocity), -max_velocity);
        }
        
        // 位置更新
        p = p + v * dt + (a_world + g) * (0.5f * dt * dt);
        
        prev_a_world = a_world;
        prev_v = v_prev;
        prev_initialized = true;
    } else {
        // AB2
        Vector3 v_old = v;
        v = v + (a_world + g) * (1.5f * dt) - (prev_a_world + g) * (0.5f * dt);
        
        // 速度発散防止
        float max_accel = 200.0f;
        Vector3 dv = v - v_old;
        
        float dv_norm = 0.0f;
        for (int i=0; i<3; ++i) dv_norm += dv(i,0) * dv(i,0);
        dv_norm = std::sqrt(dv_norm);
        
        float max_dv = max_accel * dt;
        if (dv_norm > max_dv) {
            float scale = max_dv / dv_norm;
            v = v_old + dv * scale;
        }
        
        // 速度クリップ
        float max_velocity = 200.0f;
        for (int i=0; i<3; ++i) {
            v(i,0) = std::max(std::min(v(i,0), max_velocity), -max_velocity);
        }
        
        // 位置更新
        p = p + v_old * (1.5f * dt) - prev_v * (0.5f * dt);
        
        prev_v = v_old;
        prev_a_world = a_world;
    }
}

void ESKFCore::update_accel(
    Vector4& q,
    const Vector3& a_meas,
    float scale_factor
) {
    // 健全性チェック
    float a_norm = 0.0f;
    for (int i=0; i<3; ++i) a_norm += a_meas(i,0) * a_meas(i,0);
    a_norm = std::sqrt(a_norm);
    
    if (a_norm < 0.1f || std::abs(a_norm - 9.81f) > 3.0f) {
        return;
    }
    
    // 現在のYaw取得
    Vector3 euler_current; 
    cquat::to_euler_deg(q, euler_current);
    float yaw_current = euler_current(2,0);
    
    // Roll/Pitch計算
    float ax = a_meas(0,0);
    float ay = a_meas(1,0);
    float az = a_meas(2,0);
    
    float roll_measured = atan2f(ay, az) * 180.0f / static_cast<float>(M_PI);
    float pitch_measured = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / static_cast<float>(M_PI);
    
    float roll_current = euler_current(0,0);
    float pitch_current = euler_current(1,0);
    
    // スケーリング適用
    float roll_target = roll_current + (roll_measured - roll_current) * scale_factor;
    float pitch_target = pitch_current + (pitch_measured - pitch_current) * scale_factor;
    
    // 新しいクォータニオン生成
    cquat::from_euler_deg(roll_target, pitch_target, yaw_current, q);
    cquat::normalize_quat(q);
}

void ESKFCore::update_mag(
    Vector4& q, Matrix15x15& P,
    const Vector3& m_meas,
    const Vector3& m_world,
    const Matrix3x3& R_mag,
    cmath_fx::Matrix<15, 3, float>& K_out,
    Vector15& dx_out
) {
    // 回転行列取得
    Matrix3x3 Rb;
    cquat::quat_to_rotm(q, Rb);
    
    // 予測磁場(body座標系)
    Vector3 h_mag = Rb.transpose() * m_world;
    
    // 観測行列 H (3x15)
    cmath_fx::Matrix<3, 15, float> H;
    for (int i=0; i<3; ++i) for (int j=0; j<15; ++j) H(i,j) = 0.0f;
    
    // H(1:3, 7:9) = skew(h)
    Matrix3x3 h_skew = kf::KalmanFilterCore::skew_symmetric(h_mag);
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
            H(i, 6+j) = h_skew(i,j);
        }
    }
    
    // イノベーション
    Vector3 y = m_meas - h_mag;
    
    // S = H*P*H' + R (3x3)
    auto temp = H * P;  // 3x15
    Matrix3x3 S = temp * H.transpose() + R_mag;
    
    // カルマンゲイン K = P*H'*S^-1 (15x3)
    K_out = kf::KalmanFilterCore::compute_kalman_gain<15, 3, float>(P, H, S);
    
    // dx = K * y (15x1)
    dx_out = K_out * y;
    
    // 状態更新（クォータニオンのみ）
    Vector3 dtheta;
    dtheta(0,0) = 0.0f;
    dtheta(1,0) = 0.0f;
    dtheta(2,0) = dx_out(8,0);
    
    Vector4 dq;
    cquat::from_euler_deg(dtheta(0,0), dtheta(1,0), dtheta(2,0), dq);
    
    Vector4 q_new;
    cquat::multiply_quat(q, dq, q_new);
    q = q_new;
    cquat::normalize_quat(q);
    
    // 共分散更新 (Joseph形式)
    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 KH = K_out * H;
    Matrix15x15 IKH = I - KH;
    
    Matrix15x15 term1 = IKH * P * IKH.transpose();
    auto KR = K_out * R_mag;  // 15x3
    Matrix15x15 term2 = KR * K_out.transpose();
    
    P = term1 + term2;
    
    // 対称化
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            P(i,j) = 0.5f * (P(i,j) + P(j,i));
        }
    }
}

void ESKFCore::update_gps(
    Vector3& p, Vector3& v, Matrix15x15& P,
    const Vector3& gps_pos,
    const Vector3& gps_origin,
    const Matrix3x3& R_gps,
    cmath_fx::Matrix<15, 3, float>& K_out,
    Vector15& dx_out
) {
    // GPS->ローカル座標変換
    Vector3 p_local;
    gps_to_local(gps_pos, gps_origin, p_local);
    
    // 観測行列（位置のみ）H (3x15)
    cmath_fx::Matrix<3, 15, float> H;
    for (int i=0; i<3; ++i) {
        for (int j=0; j<15; ++j) {
            H(i,j) = (j == i) ? 1.0f : 0.0f;
        }
    }
    
    // イノベーション
    Vector3 y = p_local - p;
    
    // S = H*P*H' + R (3x3)
    auto temp = H * P;  // 3x15
    Matrix3x3 S = temp * H.transpose() + R_gps;
    
    // カルマンゲイン K = P*H'*S^-1 (15x3)
    K_out = kf::KalmanFilterCore::compute_kalman_gain<15, 3, float>(P, H, S);
    
    // dx = K * y (15x1)
    dx_out = K_out * y;
    
    // 状態更新
    for (int i=0; i<3; ++i) {
        p(i,0) += dx_out(i,0);
        v(i,0) += dx_out(i+3,0);
    }
    
    // 共分散更新（Joseph形式）
    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 KH = K_out * H;
    Matrix15x15 IKH = I - KH;
    
    Matrix15x15 term1 = IKH * P * IKH.transpose();
    auto KR = K_out * R_gps;  // 15x3
    Matrix15x15 term2 = KR * K_out.transpose();
    
    P = term1 + term2;
    
    // 対称化
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            P(i,j) = 0.5f * (P(i,j) + P(j,i));
        }
    }
}

void ESKFCore::update_baro(
    Vector3& p, Matrix15x15& P,
    float pressure,
    const Vector3& gps_origin,
    float R_baro,
    cmath_fx::Matrix<15, 1, float>& K_out,
    Vector15& dx_out
) {
    // 気圧->高度変換
    float alt_meas = pressure_to_altitude(pressure);
    float alt_origin = gps_origin(2,0);
    float z_local = alt_meas - alt_origin;
    
    // 観測行列（Z軸のみ）H (1x15)
    cmath_fx::Matrix<1, 15, float> H;
    for (int j=0; j<15; ++j) H(0,j) = (j == 2) ? 1.0f : 0.0f;
    
    // イノベーション
    cmath_fx::Matrix<1, 1, float> y;
    y(0,0) = z_local - p(2,0);
    
    // S = H*P*H' + R (1x1)
    auto temp = H * P;  // 1x15
    cmath_fx::Matrix<1, 1, float> S = temp * H.transpose();
    S(0,0) += R_baro;
    
    // カルマンゲイン K = P*H'*S^-1 (15x1)
    K_out = kf::KalmanFilterCore::compute_kalman_gain<15, 1, float>(P, H, S);
    
    // dx = K * y (15x1)
    dx_out = K_out * y;
    
    // 状態更新（位置のみ）
    p(2,0) += dx_out(2,0);
    
    // 共分散更新
    Matrix15x15 I = Matrix15x15::Identity();
    auto KH = K_out * H;  // 15x15
    Matrix15x15 IKH = I - KH;
    
    Matrix15x15 term1 = IKH * P * IKH.transpose();
    
    cmath_fx::Matrix<1, 1, float> R_mat;
    R_mat(0,0) = R_baro;
    auto KR = K_out * R_mat;  // 15x1
    Matrix15x15 term2 = KR * K_out.transpose();
    
    P = term1 + term2;
    
    // 対称化
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            P(i,j) = 0.5f * (P(i,j) + P(j,i));
        }
    }
}

float ESKFCore::pressure_to_altitude(float pressure) {
    const float P0 = 101325.0f;
    return 44330.0f * (1.0f - powf(pressure / P0, 0.1903f));
}

void ESKFCore::gps_to_local(const Vector3& gps_pos, const Vector3& origin, Vector3& local_pos) {
    float lat = gps_pos(0,0);
    float lon = gps_pos(1,0);
    float alt = gps_pos(2,0);
    
    float lat0 = origin(0,0);
    float lon0 = origin(1,0);
    float alt0 = origin(2,0);
    
    // 簡易変換（緯度経度->メートル）
    float cos_lat0 = cosf(lat0 * static_cast<float>(M_PI) / 180.0f);
    local_pos(1,0) = (lat - lat0) / 9.0e-6f;  // Y (North)
    local_pos(0,0) = (lon - lon0) / (9.0e-6f / cos_lat0);  // X (East)
    local_pos(2,0) = alt - alt0;  // Z (Up)
}

// 共分散予測（ブロック化最適化版）
void ESKFCore::predict_covariance(const Matrix15x15& P, const Vector4& q, const Vector3& a_meas, const Vector3& ba,
                                  const Vector3& w_meas, const Vector3& bg, const Matrix15x15& Q, float dt,
                                  Matrix15x15& P_new) {
    // 回転行列と中間変数を事前計算
    Matrix3x3 R;
    cquat::quat_to_rotm(q, R);
    
    Vector3 a_corrected = a_meas - ba;
    Vector3 w_corrected = w_meas - bg;
    
    // skew(a) と skew(w)
    Matrix3x3 skew_a = kf::KalmanFilterCore::skew_symmetric(a_corrected);
    Matrix3x3 skew_w = kf::KalmanFilterCore::skew_symmetric(w_corrected);
    
    // R * skew(a)
    Matrix3x3 R_skew = R * skew_a;
    
    float dt2 = dt * dt;
    
    // ブロック(0:2, 0:2) 位置-位置
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_new(i,j) = P(i,j) + dt*(P(i,j+3) + P(i+3,j)) + dt2*P(i+3,j+3) + Q(i,j);
        }
    }
    
    // ブロック(0:2, 3:5) 位置-速度
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_new(i,j+3) = P(i,j+3) + dt*P(i+3,j+3);
            for (int k = 0; k < 3; ++k) {
                P_new(i,j+3) += dt*(-R_skew(j,k)*P(i,k+6)*dt - R(j,k)*P(i,k+9)*dt);
            }
        }
    }
    
    // ブロック(3:5, 0:2) 速度-位置(対称)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_new(i+3,j) = P_new(j,i+3);
        }
    }
    
    // ブロック(3:5, 3:5) 速度-速度
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_new(i+3,j+3) = P(i+3,j+3);
            for (int k = 0; k < 3; ++k) {
                for (int l = 0; l < 3; ++l) {
                    P_new(i+3,j+3) += -R_skew(i,k)*P(k+6,j+3)*dt - R_skew(j,l)*P(i+3,l+6)*dt;
                    P_new(i+3,j+3) += R_skew(i,k)*R_skew(j,l)*P(k+6,l+6)*dt2;
                }
            }
            for (int k = 0; k < 3; ++k) {
                for (int l = 0; l < 3; ++l) {
                    P_new(i+3,j+3) += -R(i,k)*P(k+9,j+3)*dt - R(j,l)*P(i+3,l+9)*dt;
                    P_new(i+3,j+3) += R(i,k)*R(j,l)*P(k+9,l+9)*dt2;
                }
            }
            P_new(i+3,j+3) += Q(i+3,j+3);
        }
    }
    
    // ブロック(6:8, 6:8) 姿勢-姿勢
    // F_theta = I - [w]× * dt
    Matrix3x3 F_theta;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F_theta(i,j) = (i == j ? 1.0f : 0.0f) - skew_w(i,j) * dt;
        }
    }
    
    // P_theta_theta = F_theta * P(7:9,7:9) * F_theta'
    Matrix3x3 P_theta;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_theta(i,j) = P(i+6,j+6);
        }
    }
    
    auto F_P = F_theta * P_theta;
    auto F_P_Ft = F_P * F_theta.transpose();
    
    // ジャイロバイアスとのクロス項を含む完全な更新
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_new(i+6,j+6) = F_P_Ft(i,j);
            // クロス項: F_theta * P(7:9, 13:15) * (-dt)
            for (int k = 0; k < 3; ++k) {
                P_new(i+6,j+6) += F_theta(i,k) * P(k+6,j+12) * (-dt);
                P_new(i+6,j+6) += (-dt) * P(i+12,k+6) * F_theta(j,k);
            }
            // ジャイロバイアス項: dt^2 * P(13:15, 13:15)
            P_new(i+6,j+6) += dt2 * P(i+12,j+12);
            // プロセスノイズ
            P_new(i+6,j+6) += Q(i+6,j+6);
        }
    }
    
    // ブロック(9:11, 9:11) 加速度バイアス
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_new(i+9,j+9) = P(i+9,j+9) + Q(i+9,j+9);
        }
    }
    
    // ブロック(12:14, 12:14) 角速度バイアス
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_new(i+12,j+12) = P(i+12,j+12) + Q(i+12,j+12);
        }
    }
    
    // クロス項
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_new(i,j+6) = P(i,j+6) + dt*P(i+3,j+6);
            P_new(j+6,i) = P_new(i,j+6);
            P_new(i,j+9) = P(i,j+9) + dt*P(i+3,j+9);
            P_new(j+9,i) = P_new(i,j+9);
            P_new(i,j+12) = P(i,j+12) + dt*P(i+3,j+12);
            P_new(j+12,i) = P_new(i,j+12);
            P_new(i+3,j+6) = P(i+3,j+6);
            for (int k = 0; k < 3; ++k) {
                P_new(i+3,j+6) += -R_skew(i,k)*P(k+6,j+6)*dt - R(i,k)*P(k+9,j+6)*dt;
            }
            P_new(j+6,i+3) = P_new(i+3,j+6);
            P_new(i+3,j+9) = P(i+3,j+9);
            for (int k = 0; k < 3; ++k) {
                P_new(i+3,j+9) += -R_skew(i,k)*P(k+6,j+9)*dt - R(i,k)*P(k+9,j+9)*dt;
            }
            P_new(j+9,i+3) = P_new(i+3,j+9);
            P_new(i+3,j+12) = P(i+3,j+12);
            for (int k = 0; k < 3; ++k) {
                P_new(i+3,j+12) += -R_skew(i,k)*P(k+6,j+12)*dt - R(i,k)*P(k+9,j+12)*dt;
            }
            P_new(j+12,i+3) = P_new(i+3,j+12);
            P_new(i+6,j+9) = P(i+6,j+9) + dt*(-P(i+12,j+9));
            P_new(j+9,i+6) = P_new(i+6,j+9);
            P_new(i+6,j+12) = P(i+6,j+12) + dt*(-P(i+12,j+12));
            P_new(j+12,i+6) = P_new(i+6,j+12);
            P_new(i+9,j+12) = P(i+9,j+12);
            P_new(j+12,i+9) = P_new(i+9,j+12);
        }
    }
    
    // 対称性強制と正定値保証
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_new(i,j) = 0.5f * (P_new(i,j) + P_new(j,i));
        }
    }
    
    const float min_var = 1.0e-9f;
    for (int i = 0; i < 15; ++i) {
        if (P_new(i,i) < min_var) P_new(i,i) = min_var;
    }
}

// 状態遷移行列F計算
void ESKFCore::compute_F_matrix(const Vector4& q, const Vector3& a_meas, const Vector3& ba,
                                const Vector3& w_meas, const Vector3& bg, float dt,
                                Matrix15x15& F) {
    // 単位行列で初期化
    F = Matrix15x15::Identity();
    
    // 位置-速度カップリング: F(1:3, 4:6) = I * dt
    F(0,3) = dt; F(1,4) = dt; F(2,5) = dt;
    
    // 回転行列取得
    Matrix3x3 R;
    cquat::quat_to_rotm(q, R);
    
    // 加速度補正
    Vector3 a_corrected = a_meas - ba;
    
    // 歪対称行列: skew(a_corrected)
    Matrix3x3 skew_a = kf::KalmanFilterCore::skew_symmetric(a_corrected);
    
    // 速度-姿勢カップリング: F(4:6, 7:9) = -R * skew(a) * dt
    Matrix3x3 R_skew = R * skew_a;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F(3+i, 6+j) = -R_skew(i,j) * dt;
        }
    }
    
    // 速度-加速度バイアスカップリング: F(4:6, 10:12) = -R * dt
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F(3+i, 9+j) = -R(i,j) * dt;
        }
    }
    
    // 姿勢-角速度バイアスカップリング: F(7:9, 13:15) = -I * dt
    F(6,12) = -dt; F(7,13) = -dt; F(8,14) = -dt;
}

// 誤差状態注入
void ESKFCore::inject_error_state(Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg, const Vector15& dx) {
    // 位置更新
    for (int i = 0; i < 3; ++i) {
        p(i,0) += dx(i,0);
    }
    
    // 速度更新
    for (int i = 0; i < 3; ++i) {
        v(i,0) += dx(3+i,0);
    }
    
    // 姿勢更新（小角度クォータニオン）
    Vector3 dtheta;
    for (int i = 0; i < 3; ++i) {
        dtheta(i,0) = dx(6+i,0);
    }
    
    // 小角度近似: dq ≈ [1; dtheta/2]
    float theta_norm = sqrtf(dtheta(0,0)*dtheta(0,0) + 
                                   dtheta(1,0)*dtheta(1,0) + 
                                   dtheta(2,0)*dtheta(2,0));
    
    Vector4 dq;
    if (theta_norm < 1e-6f) {
        // 極小角度
        dq(0,0) = 1.0f;
        dq(1,0) = dtheta(0,0) * 0.5f;
        dq(2,0) = dtheta(1,0) * 0.5f;
        dq(3,0) = dtheta(2,0) * 0.5f;
    } else {
        // 通常の小角度クォータニオン
        float half_theta = theta_norm * 0.5f;
        float sin_half = sinf(half_theta);
        float cos_half = cosf(half_theta);
        
        dq(0,0) = cos_half;
        dq(1,0) = sin_half * dtheta(0,0) / theta_norm;
        dq(2,0) = sin_half * dtheta(1,0) / theta_norm;
        dq(3,0) = sin_half * dtheta(2,0) / theta_norm;
    }
    
    // クォータニオン積: q_new = q * dq
    Vector4 q_new;
    cquat::multiply_quat(q, dq, q_new);
    cquat::normalize_quat(q_new);
    
    for (int i = 0; i < 4; ++i) {
        q(i,0) = q_new(i,0);
    }
    
    // バイアス更新
    for (int i = 0; i < 3; ++i) {
        ba(i,0) += dx(9+i,0);
        bg(i,0) += dx(12+i,0);
    }
}

} // namespace eskf
