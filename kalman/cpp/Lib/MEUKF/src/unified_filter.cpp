#include "../inc/unified_filter.hpp"
#include "../../Common/inc/Sensor/sensor_filter.hpp"
// Matrix operations consolidated into fixed_matrix.hpp
#include "../../Common/inc/Math/statistics.hpp"
#include "../../Common/inc/Math/geometry.hpp"
#include "../../Common/inc/Math/numerical.hpp"
#include "../inc/meukf_core.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include <cstring>
#include <cmath>

namespace meukf {

UnifiedFilter::UnifiedFilter() {
    // デフォルトコンストラクタ
    // 内部状態を初期化
    prev_accel_(0,0) = 0.0f; prev_accel_(1,0) = 0.0f; prev_accel_(2,0) = 0.0f;
    prev_gyro_(0,0) = 0.0f; prev_gyro_(1,0) = 0.0f; prev_gyro_(2,0) = 0.0f;
    prev_mag_(0,0) = 0.0f; prev_mag_(1,0) = 0.0f; prev_mag_(2,0) = 0.0f;
    prev_gps_pos_(0,0) = 0.0f; prev_gps_pos_(1,0) = 0.0f; prev_gps_pos_(2,0) = 0.0f;
    prev_baro_alt_ = 0.0f;
    tolerance_ = 1e-6f;
    zupt_counter_ = 0;
    zupt_min_duration_ = 5;
    zupt_threshold_accel_ = 0.5f;
    zupt_threshold_gyro_ = 0.1f;
}

FilterOutput UnifiedFilter::update(FilterState& state, const FilterInput& input) {
    FilterOutput output;
    
    // 1. センサーデータの変更検知
    bool accel_changed = true;  // IMUは毎回更新
    bool gyro_changed = true;
    
    bool mag_changed = false;
    if (input.mag_valid) {
        float diff = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float d = input.mag(i,0) - prev_mag_(i,0);
            diff += d * d;
        }
        mag_changed = (std::sqrt(diff) > tolerance_);
    }
    
    bool gps_changed = false;
    if (input.gps_valid) {
        float diff = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float d = input.gps_pos(i,0) - prev_gps_pos_(i,0);
            diff += d * d;
        }
        gps_changed = (std::sqrt(diff) > tolerance_);
    }
    
    bool baro_changed = false;
    if (input.baro_valid) {
        float diff = std::abs(input.baro_alt - prev_baro_alt_);
        baro_changed = (diff > tolerance_);
    }
    
    // 2. MEUKFコアを呼び出す (既存の実装を使用)
    meukf::State meukf_state;
    meukf::SensorData sensor;
    meukf::Params params;
    
    // FilterStateからmeukf::Stateへ変換
    for (int i = 0; i < 3; ++i) {
        meukf_state.p[i] = state.p(i,0);
        meukf_state.v[i] = state.v(i,0);
        meukf_state.ba[i] = state.ba(i,0);
        meukf_state.bg[i] = state.bg(i,0);
    }
    meukf_state.q[0] = state.q(0,0);
    meukf_state.q[1] = state.q(1,0);
    meukf_state.q[2] = state.q(2,0);
    meukf_state.q[3] = state.q(3,0);
    
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            meukf_state.P[i*15 + j] = state.P(i, j);
        }
    }
    
    // センサーデータをコピー
    sensor.dt = input.dt;
    for (int i = 0; i < 3; ++i) {
        sensor.accel[i] = input.accel(i,0);
        sensor.gyro[i] = input.gyro(i,0);
        sensor.mag[i] = input.mag(i,0);
        sensor.gps_pos[i] = input.gps_pos(i,0);
    }
    sensor.alt_baro = input.baro_alt;
    
    // 前回のセンサー値を設定
    for (int i = 0; i < 3; ++i) {
        sensor.prev_mag[i] = prev_mag_(i,0);
        sensor.prev_gps_pos[i] = prev_gps_pos_(i,0);
    }
    sensor.prev_baro_alt = prev_baro_alt_;
    
    // 更新フラグ (変更検知結果を使用)
    sensor.update_accel = accel_changed ? 1 : 0;
    sensor.update_mag = (input.mag_valid && mag_changed) ? 1 : 0;
    sensor.update_gps = (input.gps_valid && gps_changed) ? 1 : 0;
    sensor.update_baro = (input.baro_valid && baro_changed) ? 1 : 0;
    sensor.update_zupt = 0;  // ZUPTは別途実装
    
    // パラメータをコピー
    for (int i = 0; i < 3; ++i) {
        params.g[i] = input.g(i,0);
        params.mag_ref[i] = input.mag_ref(i,0);
        params.noise_accel[i] = input.noise_accel(i,0);
        params.noise_gyro[i] = input.noise_gyro(i,0);
        params.noise_mag[i] = input.noise_mag(i,0);
        params.noise_gps[i] = input.noise_gps(i,0);
        params.noise_ba[i] = 1e-6f;  // 固定値
        params.noise_bg[i] = 1e-8f;  // 固定値
        params.noise_zupt[i] = 1e-3f;  // 固定値
    }
    params.noise_baro = input.noise_baro;
    params.alpha = input.alpha;
    params.beta = input.beta;
    params.kappa = input.kappa;
    
    // MEUKFステップ実行
    meukf::MEUKFInput meukf_input;
    meukf_input.prev_state = meukf_state;
    meukf_input.sensor = sensor;
    meukf_input.params = params;
    
    meukf::MEUKFOutput meukf_output;
    meukf::MEUKFCore::step(meukf_input, meukf_output);
    
    // meukf::StateからFilterStateへ変換
    for (int i = 0; i < 3; ++i) {
        state.p(i,0) = meukf_output.new_state.p[i];
        state.v(i,0) = meukf_output.new_state.v[i];
        state.ba(i,0) = meukf_output.new_state.ba[i];
        state.bg(i,0) = meukf_output.new_state.bg[i];
    }
    state.q(0,0) = meukf_output.new_state.q[0];
    state.q(1,0) = meukf_output.new_state.q[1];
    state.q(2,0) = meukf_output.new_state.q[2];
    state.q(3,0) = meukf_output.new_state.q[3];
    
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            state.P(i, j) = meukf_output.new_state.P[i*15 + j];
        }
    }
    
    // FilterOutputを構築
    for (int i = 0; i < 3; ++i) {
        output.position(i,0) = state.p(i,0);
        output.velocity(i,0) = state.v(i,0);
        output.accel_bias(i,0) = state.ba(i,0);
        output.gyro_bias(i,0) = state.bg(i,0);
    }
    output.quaternion(0,0) = state.q(0,0);
    output.quaternion(1,0) = state.q(1,0);
    output.quaternion(2,0) = state.q(2,0);
    output.quaternion(3,0) = state.q(3,0);
    
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            output.covariance(i, j) = state.P(i, j);
        }
    }
    
    // オイラー角計算
    Vec4 q_vec = state.q;
    Vec3 euler_deg;
    cquat::to_euler_deg(q_vec, euler_deg);
    output.roll = euler_deg(0,0);
    output.pitch = euler_deg(1,0);
    output.yaw = euler_deg(2,0);
    
    // 診断情報
    output.innovation_norm_accel = meukf_output.debug_info[0];
    output.innovation_norm_mag = meukf_output.debug_info[1];
    output.innovation_norm_gps = meukf_output.debug_info[2];
    output.innovation_norm_baro = meukf_output.debug_info[3];
    
    output.divergence_detected = false;
    output.reset_occurred = false;
    output.num_updates_applied = 1;
    
    // 前回のセンサー値を更新
    for (int i = 0; i < 3; ++i) {
        prev_accel_(i,0) = input.accel(i,0);
        prev_gyro_(i,0) = input.gyro(i,0);
        prev_mag_(i,0) = input.mag(i,0);
        prev_gps_pos_(i,0) = input.gps_pos(i,0);
    }
    prev_baro_alt_ = input.baro_alt;
    
    return output;
}

void UnifiedFilter::reset() {
    // 内部状態をリセット
    for (int i = 0; i < 3; ++i) {
        prev_accel_(i,0) = 0.0f;
        prev_gyro_(i,0) = 0.0f;
        prev_mag_(i,0) = 0.0f;
        prev_gps_pos_(i,0) = 0.0f;
    }
    prev_baro_alt_ = 0.0f;
    zupt_counter_ = 0;
}

// Quaternion helper: normalize and return a Vec4 (delegates to canonical cquat)
Vec4 UnifiedFilter::normalize_quaternion(const Vec4& q) const {
    Vec4 out = q;
    cquat::normalize_quat(out);
    return out;
}

} // namespace meukf
