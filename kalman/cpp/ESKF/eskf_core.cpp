#include "eskf_core.hpp"
#include "../KF/Core/kalman_filter_core.hpp"
#include "../Common/Math/quaternion.hpp"
#include <cmath>
#include <algorithm>

namespace eskf {

// 静的メンバ初期化
cm ESKFCore::prev_a_world;
cm ESKFCore::prev_v;
bool ESKFCore::prev_initialized = false;

void ESKFCore::integrate_nominal(
    cm& p, cm& v, cm& q, cm& ba, cm& bg,
    const cm& a_meas, const cm& w_meas,
    float dt, const cm& g,
    const cm& gyro_noise_threshold,
    const cm& accel_noise_threshold
) {
    // バイアス補正
    cm a; a.resize(3,1);
    cm w; w.resize(3,1);
    for (int i=0; i<3; ++i) {
        a(i,0) = a_meas(i,0) - ba(i,0);
        w(i,0) = w_meas(i,0) - bg(i,0);
    }
    
    // 加速度閾値処理
    for (int i=0; i<3; ++i) {
        if (std::abs(a(i,0)) < accel_noise_threshold(i,0) * dt) {
            a(i,0) = 0.0f;
        }
    }
    
    // 角速度積分
    cm w_dt; w_dt.resize(3,1);
    for (int i=0; i<3; ++i) w_dt(i,0) = w(i,0) * dt;
    
    float w_dt_norm = 0.0f;
    for (int i=0; i<3; ++i) w_dt_norm += w_dt(i,0) * w_dt(i,0);
    w_dt_norm = std::sqrt(w_dt_norm);
    
    float threshold = 1e-15f;
    if (w_dt_norm > threshold) {
        float half_angle = w_dt_norm / 2.0f;
        cm delta_q; delta_q.resize(4,1);
        
        if (half_angle > 1e-6f) {
            float sin_half = std::sin(half_angle);
            float cos_half = std::cos(half_angle);
            cm w_unit; w_unit.resize(3,1);
            for (int i=0; i<3; ++i) w_unit(i,0) = w_dt(i,0) / w_dt_norm;
            
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
        
        cm q_new;
        cquat::multiply_quat(q, delta_q, q_new);
        q = q_new;
        
        // 正規化
        cquat::normalize_quat(q);
    }
    
    // 回転行列（body->world）
    cm Rb; cquat::quat_to_rotm(q, Rb);
    
    // 世界座標系での比力
    cm a_world; a_world.resize(3,1);
    cmath_fx::multiply(Rb, a, a_world);
    
    // Adams-Bashforth2 積分
    if (!prev_initialized) {
        // 初回: Forward Euler
        cm v_prev = v;
        cm v_candidate; v_candidate.resize(3,1);
        for (int i=0; i<3; ++i) {
            v_candidate(i,0) = v(i,0) + (a_world(i,0) + g(i,0)) * dt;
        }
        
        // 速度発散防止
        float max_accel = 2.0f;
        cm dv; dv.resize(3,1);
        for (int i=0; i<3; ++i) dv(i,0) = v_candidate(i,0) - v(i,0);
        
        float dv_norm = 0.0f;
        for (int i=0; i<3; ++i) dv_norm += dv(i,0) * dv(i,0);
        dv_norm = std::sqrt(dv_norm);
        
        float max_dv = max_accel * dt;
        if (dv_norm > max_dv) {
            float scale = max_dv / dv_norm;
            for (int i=0; i<3; ++i) dv(i,0) *= scale;
        }
        
        for (int i=0; i<3; ++i) v(i,0) += dv(i,0);
        
        // 速度クリップ
        float max_velocity = 50.0f;
        for (int i=0; i<3; ++i) {
            v(i,0) = std::max(std::min(v(i,0), max_velocity), -max_velocity);
        }
        
        // 位置更新
        for (int i=0; i<3; ++i) {
            p(i,0) += v(i,0) * dt + 0.5f * (a_world(i,0) + g(i,0)) * dt * dt;
        }
        
        prev_a_world = a_world;
        prev_v = v_prev;
        prev_initialized = true;
    } else {
        // AB2
        cm v_old = v;
        for (int i=0; i<3; ++i) {
            v(i,0) += dt * (1.5f * (a_world(i,0) + g(i,0)) - 0.5f * (prev_a_world(i,0) + g(i,0)));
        }
        
        // 速度発散防止
        float max_accel = 2.0f;
        cm dv; dv.resize(3,1);
        for (int i=0; i<3; ++i) dv(i,0) = v(i,0) - v_old(i,0);
        
        float dv_norm = 0.0f;
        for (int i=0; i<3; ++i) dv_norm += dv(i,0) * dv(i,0);
        dv_norm = std::sqrt(dv_norm);
        
        float max_dv = max_accel * dt;
        if (dv_norm > max_dv) {
            float scale = max_dv / dv_norm;
            for (int i=0; i<3; ++i) dv(i,0) *= scale;
            for (int i=0; i<3; ++i) v(i,0) = v_old(i,0) + dv(i,0);
        }
        
        // 速度クリップ
        float max_velocity = 50.0f;
        for (int i=0; i<3; ++i) {
            v(i,0) = std::max(std::min(v(i,0), max_velocity), -max_velocity);
        }
        
        // 位置更新
        for (int i=0; i<3; ++i) {
            p(i,0) += dt * (1.5f * v_old(i,0) - 0.5f * prev_v(i,0));
        }
        
        prev_v = v_old;
        prev_a_world = a_world;
    }
}

void ESKFCore::update_accel(
    cm& q,
    const cm& a_meas,
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
    cm euler_current; 
    cquat::to_euler_deg(q, euler_current);
    float yaw_current = euler_current(2,0);
    
    // Roll/Pitch計算
    float ax = a_meas(0,0);
    float ay = a_meas(1,0);
    float az = a_meas(2,0);
    
    float roll_measured = std::atan2f(ay, az) * 180.0f / static_cast<float>(M_PI);
    float pitch_measured = std::atan2f(-ax, std::sqrtf(ay*ay + az*az)) * 180.0f / static_cast<float>(M_PI);
    
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
    cm& q, cm& P,
    const cm& m_meas,
    const cm& m_world,
    const cm& R_mag,
    cm& K_out,
    cm& dx_out
) {
    // 回転行列取得
    cm Rb; cquat::quat_to_rotm(q, Rb);
    
    // 予測磁場（body座標系）
    cm Rb_t; cmath_fx::transpose(Rb, Rb_t);
    cm h_mag; cmath_fx::multiply(Rb_t, m_world, h_mag);
    
    // 観測行列
    cm H; H.resize(3,15);
    for (int i=0; i<3; ++i) for (int j=0; j<15; ++j) H(i,j) = 0.0f;
    
    // H(1:3, 7:9) = skew(h)
    cm h_skew = kf::KalmanFilterCore::skew_symmetric(h_mag);
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
            H(i, 6+j) = h_skew(i,j);
        }
    }
    
    // イノベーション
    cm y; y.resize(3,1);
    for (int i=0; i<3; ++i) y(i,0) = m_meas(i,0) - h_mag(i,0);
    
    // S = H*P*H' + R
    cm temp, temp2, Ht;
    cmath_fx::transpose(H, Ht);
    cmath_fx::multiply(H, P, temp);
    cmath_fx::multiply(temp, Ht, temp2);
    cm S; S.resize(3,3);
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
            S(i,j) = temp2(i,j) + R_mag(i,j);
        }
    }
    
    // カルマンゲイン
    K_out = kf::KalmanFilterCore::compute_kalman_gain(P, H, S);
    
    // dx = K * y
    cmath_fx::multiply(K_out, y, dx_out);
    
    // 状態更新（クォータニオンのみ）
    if (dx_out.rows >= 9) {
        cm dtheta; dtheta.resize(3,1);
        dtheta(0,0) = 0.0f;
        dtheta(1,0) = 0.0f;
        dtheta(2,0) = dx_out(8,0);
        
        cm dq;
        cquat::from_euler_deg(dtheta(0,0), dtheta(1,0), dtheta(2,0), dq);
        
        cm q_new;
        cquat::multiply_quat(q, dq, q_new);
        q = q_new;
        cquat::normalize_quat(q);
    }
    
    // 共分散更新
    cm I = cm::Identity(15);
    cm KH, IKH;
    cmath_fx::multiply(K_out, H, KH);
    IKH.resize(15,15);
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            IKH(i,j) = I(i,j) - KH(i,j);
        }
    }
    
    cm temp1, IKHt, term1;
    cmath_fx::multiply(IKH, P, temp1);
    cmath_fx::transpose(IKH, IKHt);
    cmath_fx::multiply(temp1, IKHt, term1);
    
    cm KR, KRt, term2;
    cmath_fx::multiply(K_out, R_mag, temp);
    cmath_fx::transpose(K_out, KRt);
    cmath_fx::multiply(temp, KRt, term2);
    
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            P(i,j) = term1(i,j) + term2(i,j);
        }
    }
    
    // 対称化
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            P(i,j) = 0.5f * (P(i,j) + P(j,i));
        }
    }
}

void ESKFCore::update_gps(
    cm& p, cm& v, cm& P,
    const cm& gps_pos,
    const cm& gps_origin,
    const cm& R_gps,
    cm& K_out,
    cm& dx_out
) {
    // GPS->ローカル座標変換
    cm p_local;
    gps_to_local(gps_pos, gps_origin, p_local);
    
    // 観測行列（位置のみ）
    cm H; H.resize(3,15);
    for (int i=0; i<3; ++i) {
        for (int j=0; j<15; ++j) {
            H(i,j) = (j == i) ? 1.0f : 0.0f;
        }
    }
    
    // イノベーション
    cm y; y.resize(3,1);
    for (int i=0; i<3; ++i) y(i,0) = p_local(i,0) - p(i,0);
    
    // S = H*P*H' + R
    cm temp, temp2, Ht;
    cmath_fx::transpose(H, Ht);
    cmath_fx::multiply(H, P, temp);
    cmath_fx::multiply(temp, Ht, temp2);
    cm S; S.resize(3,3);
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
            S(i,j) = temp2(i,j) + R_gps(i,j);
        }
    }
    
    // カルマンゲイン
    K_out = kf::KalmanFilterCore::compute_kalman_gain(P, H, S);
    
    // dx = K * y
    cmath_fx::multiply(K_out, y, dx_out);
    
    // 状態更新
    if (dx_out.rows >= 6) {
        for (int i=0; i<3; ++i) {
            p(i,0) += dx_out(i,0);
            v(i,0) += dx_out(i+3,0);
        }
    }
    
    // 共分散更新（Joseph形式）
    cm I = cm::Identity(15);
    cm KH, IKH;
    cmath_fx::multiply(K_out, H, KH);
    IKH.resize(15,15);
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            IKH(i,j) = I(i,j) - KH(i,j);
        }
    }
    
    cm temp1, IKHt, term1;
    cmath_fx::multiply(IKH, P, temp1);
    cmath_fx::transpose(IKH, IKHt);
    cmath_fx::multiply(temp1, IKHt, term1);
    
    cm KR, KRt, term2;
    cmath_fx::multiply(K_out, R_gps, temp);
    cmath_fx::transpose(K_out, KRt);
    cmath_fx::multiply(temp, KRt, term2);
    
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            P(i,j) = term1(i,j) + term2(i,j);
        }
    }
    
    // 対称化
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            P(i,j) = 0.5f * (P(i,j) + P(j,i));
        }
    }
}

void ESKFCore::update_baro(
    cm& p, cm& P,
    float pressure,
    const cm& gps_origin,
    float R_baro,
    cm& K_out,
    cm& dx_out
) {
    // 気圧->高度変換
    float alt_meas = pressure_to_altitude(pressure);
    float alt_origin = gps_origin(2,0);
    float z_local = alt_meas - alt_origin;
    
    // 観測行列（Z軸のみ）
    cm H; H.resize(1,15);
    for (int j=0; j<15; ++j) H(0,j) = (j == 2) ? 1.0f : 0.0f;
    
    // イノベーション
    cm y; y.resize(1,1);
    y(0,0) = z_local - p(2,0);
    
    // S = H*P*H' + R
    cm temp, temp2, Ht;
    cmath_fx::transpose(H, Ht);
    cmath_fx::multiply(H, P, temp);
    cmath_fx::multiply(temp, Ht, temp2);
    cm S; S.resize(1,1);
    S(0,0) = temp2(0,0) + R_baro;
    
    // カルマンゲイン
    K_out = kf::KalmanFilterCore::compute_kalman_gain(P, H, S);
    
    // dx = K * y
    cmath_fx::multiply(K_out, y, dx_out);
    
    // 状態更新（位置のみ）
    if (dx_out.rows >= 3) {
        p(2,0) += dx_out(2,0);
    }
    
    // 共分散更新
    cm I = cm::Identity(15);
    cm KH, IKH;
    cmath_fx::multiply(K_out, H, KH);
    IKH.resize(15,15);
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            IKH(i,j) = I(i,j) - KH(i,j);
        }
    }
    
    cm temp1, IKHt, term1;
    cmath_fx::multiply(IKH, P, temp1);
    cmath_fx::transpose(IKH, IKHt);
    cmath_fx::multiply(temp1, IKHt, term1);
    
    cm R_mat; R_mat.resize(1,1);
    R_mat(0,0) = R_baro;
    cm KR, KRt, term2;
    cmath_fx::multiply(K_out, R_mat, temp);
    cmath_fx::transpose(K_out, KRt);
    cmath_fx::multiply(temp, KRt, term2);
    
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            P(i,j) = term1(i,j) + term2(i,j);
        }
    }
    
    // 対称化
    for (int i=0; i<15; ++i) {
        for (int j=0; j<15; ++j) {
            P(i,j) = 0.5f * (P(i,j) + P(j,i));
        }
    }
}

float ESKFCore::pressure_to_altitude(float pressure) {
    const float P0 = 101325.0f;
    return 44330.0f * (1.0f - std::powf(pressure / P0, 0.1903f));
}

void ESKFCore::gps_to_local(const cm& gps_pos, const cm& origin, cm& local_pos) {
    local_pos.resize(3,1);
    
    float lat = gps_pos(0,0);
    float lon = gps_pos(1,0);
    float alt = gps_pos(2,0);
    
    float lat0 = origin(0,0);
    float lon0 = origin(1,0);
    float alt0 = origin(2,0);
    
    // 簡易変換（緯度経度->メートル）
    float cos_lat0 = std::cosf(lat0 * static_cast<float>(M_PI) / 180.0f);
    local_pos(1,0) = (lat - lat0) / 9.0e-6f;  // Y (North)
    local_pos(0,0) = (lon - lon0) / (9.0e-6f / cos_lat0);  // X (East)
    local_pos(2,0) = alt - alt0;  // Z (Up)
}

// 共分散予測
void ESKFCore::predict_covariance(const cm& P, const cm& q, const cm& a_meas, const cm& ba,
                                  const cm& w_meas, const cm& bg, const cm& Q, float dt,
                                  cm& P_new) {
    // 状態遷移行列F計算
    cm F;
    compute_F_matrix(q, a_meas, ba, w_meas, bg, dt, F);
    
    // P_new = F * P * F' + Q
    cm FP, Ft, FPFt;
    cmath_fx::multiply(F, P, FP);
    cmath_fx::transpose(F, Ft);
    cmath_fx::multiply(FP, Ft, FPFt);
    
    P_new.resize(15, 15);
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_new(i,j) = FPFt(i,j) + Q(i,j);
        }
    }
    
    // 対称性強制
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_new(i,j) = 0.5f * (P_new(i,j) + P_new(j,i));
        }
    }
    
    // 対角成分の正値性保証
    const float min_var = 1.0e-9f;
    for (int i = 0; i < 15; ++i) {
        if (P_new(i,i) < min_var) {
            P_new(i,i) = min_var;
        }
    }
}

// 状態遷移行列F計算
void ESKFCore::compute_F_matrix(const cm& q, const cm& a_meas, const cm& ba,
                                const cm& w_meas, const cm& bg, float dt,
                                cm& F) {
    F.resize(15, 15);
    
    // 単位行列で初期化
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            F(i,j) = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 位置-速度カップリング: F(1:3, 4:6) = I * dt
    F(0,3) = dt; F(1,4) = dt; F(2,5) = dt;
    
    // 回転行列取得
    cm R; R.resize(3,3);
    cquat::quat_to_rotm(q, R);
    
    // 加速度補正
    cm a_corrected; a_corrected.resize(3,1);
    for (int i = 0; i < 3; ++i) {
        a_corrected(i,0) = a_meas(i,0) - ba(i,0);
    }
    
    // 歪対称行列: skew(a_corrected)
    cm skew_a; skew_a.resize(3,3);
    skew_a(0,0) = 0.0f;           skew_a(0,1) = -a_corrected(2,0); skew_a(0,2) = a_corrected(1,0);
    skew_a(1,0) = a_corrected(2,0); skew_a(1,1) = 0.0f;            skew_a(1,2) = -a_corrected(0,0);
    skew_a(2,0) = -a_corrected(1,0); skew_a(2,1) = a_corrected(0,0); skew_a(2,2) = 0.0f;
    
    // 速度-姿勢カップリング: F(4:6, 7:9) = -R * skew(a) * dt
    cm R_skew, temp;
    cmath_fx::multiply(R, skew_a, R_skew);
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
void ESKFCore::inject_error_state(cm& p, cm& v, cm& q, cm& ba, cm& bg, const cm& dx) {
    // 位置更新
    for (int i = 0; i < 3; ++i) {
        p(i,0) += dx(i,0);
    }
    
    // 速度更新
    for (int i = 0; i < 3; ++i) {
        v(i,0) += dx(3+i,0);
    }
    
    // 姿勢更新（小角度クォータニオン）
    cm dtheta; dtheta.resize(3,1);
    for (int i = 0; i < 3; ++i) {
        dtheta(i,0) = dx(6+i,0);
    }
    
    // 小角度近似: dq ≈ [1; dtheta/2]
    float theta_norm = std::sqrtf(dtheta(0,0)*dtheta(0,0) + 
                                   dtheta(1,0)*dtheta(1,0) + 
                                   dtheta(2,0)*dtheta(2,0));
    
    cm dq; dq.resize(4,1);
    if (theta_norm < 1e-6f) {
        // 極小角度
        dq(0,0) = 1.0f;
        dq(1,0) = dtheta(0,0) * 0.5f;
        dq(2,0) = dtheta(1,0) * 0.5f;
        dq(3,0) = dtheta(2,0) * 0.5f;
    } else {
        // 通常の小角度クォータニオン
        float half_theta = theta_norm * 0.5f;
        float sin_half = std::sinf(half_theta);
        float cos_half = std::cosf(half_theta);
        
        dq(0,0) = cos_half;
        dq(1,0) = sin_half * dtheta(0,0) / theta_norm;
        dq(2,0) = sin_half * dtheta(1,0) / theta_norm;
        dq(3,0) = sin_half * dtheta(2,0) / theta_norm;
    }
    
    // クォータニオン積: q_new = q * dq
    cm q_new;
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
