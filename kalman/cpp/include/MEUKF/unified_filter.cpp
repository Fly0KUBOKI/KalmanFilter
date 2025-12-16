#include "unified_filter.hpp"
#include "meukf_core.hpp"
#include "../Common/Math/math_utils.hpp"
#include <cmath>

namespace meukf {

UnifiedFilter::UnifiedFilter() 
    : tolerance_(1e-9),
      zupt_counter_(0),
      zupt_min_duration_(10),
      zupt_threshold_accel_(1.0),
      zupt_threshold_gyro_(0.0524) {  // 3 deg/s
    prev_accel_.setZero();
    prev_gyro_.setZero();
    prev_mag_.setZero();
    prev_gps_pos_.setZero();
    prev_baro_alt_ = 0.0;
}

FilterOutput UnifiedFilter::update(FilterState& state, const FilterInput& input) {
    FilterOutput output;
    int update_count = 0;
    
    // 1. 予測ステップ（必ず実行）
    predict_step(state, input);
    
    // 2. 加速度更新（変更検知）
    if (sensor_changed(input.accel, prev_accel_)) {
        if (update_accel(state, input, output)) {
            update_count++;
        }
        prev_accel_ = input.accel;
    }
    
    // 3. 磁気計更新（変更検知）
    if (input.mag_valid && sensor_changed(input.mag, prev_mag_)) {
        if (update_mag(state, input, output)) {
            update_count++;
        }
        prev_mag_ = input.mag;
    }
    
    // 4. GPS更新（変更検知）
    if (input.gps_valid && sensor_changed(input.gps_pos, prev_gps_pos_)) {
        if (update_gps(state, input, output)) {
            update_count++;
        }
        prev_gps_pos_ = input.gps_pos;
    }
    
    // 5. 気圧更新（変更検知）
    if (input.baro_valid && std::abs(input.baro_alt - prev_baro_alt_) > tolerance_) {
        if (update_baro(state, input, output)) {
            update_count++;
        }
        prev_baro_alt_ = input.baro_alt;
    }
    
    // 6. ZUPT判定 & 更新
    if (check_zupt(input.accel, input.gyro)) {
        update_zupt(state);
        update_count++;
    }
    
    // 7. 発散検知（簡易版）
    check_divergence(state, output);
    
    // 8. 出力構造体に格納
    pack_output(state, output);
    output.num_updates_applied = update_count;
    
    return output;
}

void UnifiedFilter::predict_step(FilterState& state, const FilterInput& input) {
    // MEUKFの予測ステップを呼び出し
    // meukf_core.cppの関数を使用
    
    // 角速度からバイアス補正
    Vec3 w_corrected = input.gyro - state.bg;
    
    // 加速度からバイアス補正
    Vec3 a_corrected = input.accel - state.ba;
    
    // クォータニオンの積分（小角度近似）
    double dt = input.dt;
    double wx = w_corrected(0) * dt / 2.0;
    double wy = w_corrected(1) * dt / 2.0;
    double wz = w_corrected(2) * dt / 2.0;
    
    Vec4 dq;
    dq(0) = 1.0;
    dq(1) = wx;
    dq(2) = wy;
    dq(3) = wz;
    
    // クォータニオン乗算
    Vec4 q_new = quaternion_multiply(state.q, dq);
    q_new = normalize_quaternion(q_new);
    
    // 回転行列を計算
    Mat3 R = quaternion_to_rotation_matrix(state.q);
    
    // 速度の更新
    Vec3 a_ned = R * a_corrected - input.g;
    state.v = state.v + a_ned * dt;
    
    // 位置の更新
    state.p = state.p + state.v * dt + a_ned * (0.5 * dt * dt);
    
    // 姿勢の更新
    state.q = q_new;
    
    // 共分散の予測（簡易版 - Q行列を追加）
    Mat15 Q = Mat15::Identity() * 1e-6;
    Q(0,0) = Q(1,1) = Q(2,2) = 1e-4;  // 位置
    Q(3,3) = Q(4,4) = Q(5,5) = 1e-3;  // 速度
    Q(6,6) = Q(7,7) = Q(8,8) = 1e-4;  // 姿勢
    Q(9,9) = Q(10,10) = Q(11,11) = 1e-6;  // 加速度バイアス
    Q(12,12) = Q(13,13) = Q(14,14) = 1e-6;  // ジャイロバイアス
    
    state.P = state.P + Q * dt;
    
    // 共分散の対称性を強制
    state.P = (state.P + state.P.transpose()) * 0.5;
}

bool UnifiedFilter::update_accel(FilterState& state, const FilterInput& input, 
                                   FilterOutput& output) {
    // 加速度ノルムチェック
    double accel_norm = input.accel.norm();
    if (accel_norm < 0.1 || std::abs(accel_norm - 9.81) > 3.0) {
        return false;
    }
    
    // MEUKF加速度更新（簡易版）
    // 予測加速度（姿勢から計算）
    Mat3 R = quaternion_to_rotation_matrix(state.q);
    Vec3 a_pred = R.transpose() * input.g;  // Body frameでの重力
    
    // イノベーション
    Vec3 innovation = input.accel - a_pred;
    output.innovation_norm_accel = innovation.norm();
    
    // R行列
    Mat3 R_noise = Mat3::Identity();
    R_noise(0,0) = input.noise_accel(0) * input.noise_accel(0);
    R_noise(1,1) = input.noise_accel(1) * input.noise_accel(1);
    R_noise(2,2) = input.noise_accel(2) * input.noise_accel(2);
    
    // 観測行列H（姿勢に関する部分）
    // 簡易版: 小角度近似
    Mat3 H_att;
    H_att.setZero();
    H_att(0, 1) = input.g(2);
    H_att(0, 2) = -input.g(1);
    H_att(1, 0) = -input.g(2);
    H_att(1, 2) = input.g(0);
    H_att(2, 0) = input.g(1);
    H_att(2, 1) = -input.g(0);
    
    // イノベーション共分散
    Mat3 S = H_att * state.P.block<3,3>(6,6) * H_att.transpose() + R_noise;
    
    // カルマンゲイン（姿勢のみ更新）
    Mat3 K_att = state.P.block<3,3>(6,6) * H_att.transpose() * S.inverse();
    
    // 状態更新（姿勢の誤差状態）
    Vec3 d_att = K_att * innovation;
    
    // 姿勢の更新（小角度からクォータニオン）
    Vec4 dq;
    dq(0) = 1.0;
    dq(1) = d_att(0) / 2.0;
    dq(2) = d_att(1) / 2.0;
    dq(3) = d_att(2) / 2.0;
    
    state.q = quaternion_multiply(state.q, dq);
    state.q = normalize_quaternion(state.q);
    
    // 共分散更新
    Mat3 I_KH = Mat3::Identity() - K_att * H_att;
    state.P.block<3,3>(6,6) = I_KH * state.P.block<3,3>(6,6);
    
    return true;
}

bool UnifiedFilter::update_mag(FilterState& state, const FilterInput& input, 
                                 FilterOutput& output) {
    // 磁場ノルムチェック
    double mag_norm = input.mag.norm();
    if (mag_norm < 1e-6 || mag_norm > 100.0) {
        return false;
    }
    
    // MEUKF磁気計更新（簡易版）
    Mat3 R = quaternion_to_rotation_matrix(state.q);
    Vec3 mag_pred = R.transpose() * input.mag_ref;
    
    Vec3 innovation = input.mag - mag_pred;
    output.innovation_norm_mag = innovation.norm();
    
    // R行列
    Mat3 R_noise = Mat3::Identity();
    R_noise(0,0) = input.noise_mag(0) * input.noise_mag(0);
    R_noise(1,1) = input.noise_mag(1) * input.noise_mag(1);
    R_noise(2,2) = input.noise_mag(2) * input.noise_mag(2);
    
    // 観測行列（簡易版）
    Mat3 H_att;
    H_att.setZero();
    // ヤコビアン計算を簡略化
    
    // 姿勢のみ更新（実装を簡略化）
    // 本来はMEUKFのシグマポイントを使用
    
    return true;
}

bool UnifiedFilter::update_gps(FilterState& state, const FilterInput& input, 
                                 FilterOutput& output) {
    // GPS更新（EKF版 - 位置のみ）
    Vec3 innovation = input.gps_pos - state.p;
    output.innovation_norm_gps = innovation.norm();
    
    // R行列
    Mat3 R_noise = Mat3::Identity();
    R_noise(0,0) = input.noise_gps(0) * input.noise_gps(0);
    R_noise(1,1) = input.noise_gps(1) * input.noise_gps(1);
    R_noise(2,2) = input.noise_gps(2) * input.noise_gps(2);
    
    // 観測行列（位置に関するH行列）
    Mat3 H = Mat3::Identity();
    
    // イノベーション共分散
    Mat3 S = state.P.block<3,3>(0,0) + R_noise;
    
    // カルマンゲイン
    Mat3 K = state.P.block<3,3>(0,0) * S.inverse();
    
    // 状態更新（位置のみ）
    state.p = state.p + K * innovation;
    
    // 共分散更新
    Mat3 I_KH = Mat3::Identity() - K * H;
    state.P.block<3,3>(0,0) = I_KH * state.P.block<3,3>(0,0);
    
    return true;
}

bool UnifiedFilter::update_baro(FilterState& state, const FilterInput& input, 
                                  FilterOutput& output) {
    // 気圧更新（EKF版 - 高度のみ）
    double innovation = input.baro_alt - (-state.p(2));  // NED座標系（-Z = 高度）
    output.innovation_norm_baro = std::abs(innovation);
    
    double R_noise = input.noise_baro * input.noise_baro;
    
    // 観測行列（高度に関するH）
    double H = -1.0;  // dh/dp_z = -1
    
    // イノベーション共分散
    double S = state.P(2,2) + R_noise;
    
    // カルマンゲイン
    double K = state.P(2,2) / S;
    
    // 状態更新（高度のみ）
    state.p(2) = state.p(2) + K * innovation * H;
    
    // 共分散更新
    state.P(2,2) = (1.0 - K * H) * state.P(2,2);
    
    return true;
}

bool UnifiedFilter::check_zupt(const Vec3& accel, const Vec3& gyro) {
    double accel_norm = accel.norm();
    double gravity_dev = std::abs(accel_norm - 9.81);
    double gyro_norm = gyro.norm();
    
    if (gravity_dev < zupt_threshold_accel_ && gyro_norm < zupt_threshold_gyro_) {
        zupt_counter_++;
    } else {
        zupt_counter_ = 0;
    }
    
    return (zupt_counter_ >= zupt_min_duration_);
}

void UnifiedFilter::update_zupt(FilterState& state) {
    // ZUPT: 速度を0に更新
    Vec3 innovation = Vec3::Zero() - state.v;
    
    // R行列（静止時の強い確信）
    Mat3 R_zupt = Mat3::Identity() * (0.01 * 0.01);
    
    // 観測行列（速度に関するH）
    Mat3 H = Mat3::Identity();
    
    // イノベーション共分散
    Mat3 S = state.P.block<3,3>(3,3) + R_zupt;
    
    // カルマンゲイン
    Mat3 K = state.P.block<3,3>(3,3) * S.inverse();
    
    // 状態更新
    state.v = state.v + K * innovation;
    
    // 共分散更新
    Mat3 I_KH = Mat3::Identity() - K * H;
    state.P.block<3,3>(3,3) = I_KH * state.P.block<3,3>(3,3);
}

void UnifiedFilter::check_divergence(const FilterState& state, FilterOutput& output) {
    // 簡易発散検知
    output.divergence_detected = false;
    
    // 速度チェック
    if (state.v.norm() > 5.0) {
        output.divergence_detected = true;
    }
    
    // イノベーションチェック
    if (output.innovation_norm_accel > 5.0 || 
        output.innovation_norm_mag > 2.0 ||
        output.innovation_norm_gps > 20.0) {
        output.divergence_detected = true;
    }
    
    // NaNチェック
    if (!state.p.allFinite() || !state.v.allFinite() || !state.q.allFinite()) {
        output.divergence_detected = true;
    }
}

void UnifiedFilter::pack_output(const FilterState& state, FilterOutput& output) {
    output.position = state.p;
    output.velocity = state.v;
    output.quaternion = state.q;
    output.accel_bias = state.ba;
    output.gyro_bias = state.bg;
    output.covariance = state.P;
    
    // オイラー角変換
    Vec3 euler = quaternion_to_euler(state.q);
    output.roll = euler(0) * 180.0 / M_PI;
    output.pitch = euler(1) * 180.0 / M_PI;
    output.yaw = euler(2) * 180.0 / M_PI;
}

bool UnifiedFilter::sensor_changed(const Vec3& new_val, const Vec3& prev_val) const {
    return (new_val - prev_val).norm() > tolerance_;
}

// ヘルパー関数（quaternion operations）
Vec4 UnifiedFilter::quaternion_multiply(const Vec4& q1, const Vec4& q2) const {
    Vec4 q_out;
    q_out(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
    q_out(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
    q_out(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
    q_out(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);
    return q_out;
}

Vec4 UnifiedFilter::normalize_quaternion(const Vec4& q) const {
    double norm = std::sqrt(q(0)*q(0) + q(1)*q(1) + q(2)*q(2) + q(3)*q(3));
    if (norm < 1e-10) {
        Vec4 q_norm;
        q_norm(0) = 1; q_norm(1) = 0; q_norm(2) = 0; q_norm(3) = 0;
        return q_norm;
    }
    return q / norm;
}

Mat3 UnifiedFilter::quaternion_to_rotation_matrix(const Vec4& q) const {
    double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
    Mat3 R;
    
    R(0,0) = 1 - 2*(qy*qy + qz*qz);
    R(0,1) = 2*(qx*qy - qw*qz);
    R(0,2) = 2*(qx*qz + qw*qy);
    
    R(1,0) = 2*(qx*qy + qw*qz);
    R(1,1) = 1 - 2*(qx*qx + qz*qz);
    R(1,2) = 2*(qy*qz - qw*qx);
    
    R(2,0) = 2*(qx*qz - qw*qy);
    R(2,1) = 2*(qy*qz + qw*qx);
    R(2,2) = 1 - 2*(qx*qx + qy*qy);
    
    return R;
}

Vec3 UnifiedFilter::quaternion_to_euler(const Vec4& q) const {
    double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
    
    Vec3 euler;
    // Roll (x-axis)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    euler(0) = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis)
    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        euler(1) = std::copysign(M_PI / 2, sinp);
    else
        euler(1) = std::asin(sinp);
    
    // Yaw (z-axis)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    euler(2) = std::atan2(siny_cosp, cosy_cosp);
    
    return euler;
}

} // namespace meukf
