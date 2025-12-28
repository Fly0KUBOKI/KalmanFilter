#include "../../Inc/MEUKF/unified_filter.hpp"
#include "../../Inc/Common/Sensor/sensor_filter.hpp"
#include "../../Inc/Common/Math/math_utils.hpp"
#include "../../Inc/MEUKF/meukf_core.hpp"
#include <cstring>
#include <cmath>

namespace meukf {

UnifiedFilter::UnifiedFilter() {
    // デフォルトコンストラクタ
}

void UnifiedFilter::initialize(const FilterOutput& initial_state) {
    current_state_ = initial_state;
}

void UnifiedFilter::update(const FilterInput& input, FilterOutput& output) {
    // TODO: 完全な実装
    // このプロトタイプでは基本的な構造のみ
    
    // 1. センサーデータの変更検知
    bool accel_changed = true;  // IMUは毎回更新
    bool gyro_changed = true;
    
    bool mag_changed = false;
    if (input.mag_valid) {
        float diff = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float d = input.mag[i] - input.prev_mag[i];
            diff += d * d;
        }
        mag_changed = (std::sqrt(diff) > 1e-6f);
    }
    
    bool gps_changed = false;
    if (input.gps_valid) {
        float diff = 0.0f;
        for (int i = 0; i < 3; ++i) {
            float d = input.gps_pos[i] - input.prev_gps_pos[i];
            diff += d * d;
        }
        gps_changed = (std::sqrt(diff) > 1e-6f);
    }
    
    bool baro_changed = false;
    if (input.baro_valid) {
        float diff = std::abs(input.baro_alt - input.prev_baro_alt);
        baro_changed = (diff > 1e-6f);
    }
    
    // 2. MEUKFコアを呼び出す (既存の実装を使用)
    meukf::State state;
    meukf::SensorData sensor;
    meukf::Params params;
    
    // 状態をコピー
    for (int i = 0; i < 3; ++i) {
        state.p[i] = current_state_.position[i];
        state.v[i] = current_state_.velocity[i];
        state.ba[i] = current_state_.accel_bias[i];
        state.bg[i] = current_state_.gyro_bias[i];
    }
    state.q[0] = current_state_.quaternion[0];
    state.q[1] = current_state_.quaternion[1];
    state.q[2] = current_state_.quaternion[2];
    state.q[3] = current_state_.quaternion[3];
    
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            state.P[i*15 + j] = current_state_.covariance(i, j);
        }
    }
    
    // センサーデータをコピー
    sensor.dt = input.dt;
    for (int i = 0; i < 3; ++i) {
        sensor.accel[i] = input.accel[i];
        sensor.gyro[i] = input.gyro[i];
        sensor.mag[i] = input.mag[i];
        sensor.gps_pos[i] = input.gps_pos[i];
    }
    sensor.alt_baro = input.baro_alt;
    
    // 更新フラグ (変更検知結果を使用)
    sensor.update_accel = accel_changed ? 1 : 0;
    sensor.update_mag = (input.mag_valid && mag_changed) ? 1 : 0;
    sensor.update_gps = (input.gps_valid && gps_changed) ? 1 : 0;
    sensor.update_baro = (input.baro_valid && baro_changed) ? 1 : 0;
    sensor.update_zupt = 0;  // ZUPTは別途実装
    
    // パラメータをコピー
    for (int i = 0; i < 3; ++i) {
        params.g[i] = input.g[i];
        params.mag_ref[i] = input.mag_ref[i];
        params.noise_accel[i] = input.noise_accel;
        params.noise_gyro[i] = input.noise_gyro;
        params.noise_mag[i] = input.noise_mag;
        params.noise_gps[i] = input.noise_gps;
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
    meukf_input.prev_state = state;
    meukf_input.sensor = sensor;
    meukf_input.params = params;
    
    meukf::MEUKFOutput meukf_output;
    meukf::MEUKFCore::step(meukf_input, meukf_output);
    
    // 出力をコピー
    for (int i = 0; i < 3; ++i) {
        output.position[i] = meukf_output.new_state.p[i];
        output.velocity[i] = meukf_output.new_state.v[i];
        output.accel_bias[i] = meukf_output.new_state.ba[i];
        output.gyro_bias[i] = meukf_output.new_state.bg[i];
    }
    
    output.quaternion[0] = meukf_output.new_state.q[0];
    output.quaternion[1] = meukf_output.new_state.q[1];
    output.quaternion[2] = meukf_output.new_state.q[2];
    output.quaternion[3] = meukf_output.new_state.q[3];
    
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            output.covariance(i, j) = meukf_output.new_state.P[i*15 + j];
        }
    }
    
    // オイラー角計算
    cmath_fx::Vector<4, float> q_vec;
    for (int i = 0; i < 4; ++i) q_vec(i, 0) = output.quaternion[i];
    cmath_fx::Vector<3, float> euler_deg;
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
    
    // 現在の状態を更新
    current_state_ = output;
}

void UnifiedFilter::reset() {
    current_state_ = FilterOutput();
}

} // namespace meukf
