# 完全C++化実装計画

**目標**: MATLAB/C++ハイブリッドを完全C++実装に移行  
**日付**: 2025年12月13日  
**ステータス**: 実装計画策定完了

---

## アーキテクチャ概要

### 現在の構成（ハイブリッド）
```
MATLAB (69%)                         C++ MEX (31%)
├─ run_simulation.m                 ├─ mex_meukf_step_v2.cpp
├─ ESKF/@ESKF/                      ├─ meukf_core.cpp
│   ├─ ESKF.m (初期化)              └─ [個別のセンサー更新関数]
│   ├─ predict.m
│   ├─ sensor_updates.m (4種類)
│   ├─ zupt.m
│   └─ reset.m
├─ KF/Utils/
│   ├─ NoiseEstimator.m
│   ├─ SensorFilter.m
│   ├─ DivergenceGuard.m
│   └─ [各種フィルタ]
└─ Common/Math/QuaternionLib.m
```

### 目標構成（完全C++）
```
MATLAB (5%)                          C++ MEX (95%)
├─ run_simulation.m                 ├─ mex_unified_filter.cpp (統一I/F)
├─ GenerateData/sim_generate.m      └─ unified_filter.cpp
│   (周波数制御用データ複製)            ├─ meukf_core.cpp
├─ ESKF/@ESKF/                          ├─ sensor_filter.hpp
│   ├─ ESKF.m (最小限の初期化)          ├─ noise_estimator.hpp
│   └─ call_unified_filter.m            ├─ divergence_guard.hpp
│       (構造体のpack/unpackのみ)       ├─ quaternion.hpp
└─ Graph/plot_csv_file.m                └─ [変更検知ロジック]
```

---

## フェーズ1: 統一インターフェース構築

### 1.1 入出力構造体設計

**C++側構造体定義** (`cpp/include/MEUKF/unified_types.hpp`):

```cpp
struct FilterInput {
    // タイムステップ
    double dt;
    
    // センサーデータ（全て毎回送信、C++側で変更検知）
    Vec3 accel;          // 加速度 [m/s^2] (100 Hz)
    Vec3 gyro;           // 角速度 [rad/s] (100 Hz)
    Vec3 mag;            // 磁場 (25 Hz → 4回重複)
    Vec3 gps_pos;        // GPS位置 [m] NED座標系 (4 Hz → 25回重複)
    double baro_alt;     // 気圧高度 [m] (2 Hz → 50回重複)
    
    // 前回値（変更検知用、C++内部で保持）
    Vec3 prev_accel;
    Vec3 prev_gyro;
    Vec3 prev_mag;
    Vec3 prev_gps_pos;
    double prev_baro_alt;
    
    // 有効フラグ（NaN検知等）
    bool mag_valid;
    bool gps_valid;
    bool baro_valid;
    
    // 基準値
    Vec3 g;              // 重力ベクトル [0, 0, 9.80665]
    Vec3 mag_ref;        // 磁場基準 [Bx, 0, Bz]
    
    // ノイズパラメータ
    Vec3 noise_accel;
    Vec3 noise_gyro;
    Vec3 noise_mag;
    Vec3 noise_gps;
    double noise_baro;
    
    // UKFパラメータ
    double alpha;
    double beta;
    double kappa;
};

struct FilterOutput {
    // 推定状態
    Vec3 position;       // 位置 [m] (NED)
    Vec3 velocity;       // 速度 [m/s] (NED)
    Quaternion quaternion; // 姿勢 [qw, qx, qy, qz]
    Vec3 accel_bias;     // 加速度バイアス
    Vec3 gyro_bias;      // ジャイロバイアス
    Mat15 covariance;    // 共分散行列 (15x15)
    
    // オイラー角（可視化用）
    double roll;         // [deg]
    double pitch;        // [deg]
    double yaw;          // [deg]
    
    // 診断情報
    double innovation_norm_accel;
    double innovation_norm_mag;
    double innovation_norm_gps;
    double innovation_norm_baro;
    bool divergence_detected;
    bool reset_occurred;
    int num_updates_applied;  // このステップで実行された更新数
};

struct FilterState {
    Vec3 p;              // 位置
    Vec3 v;              // 速度
    Quaternion q;        // 姿勢
    Vec3 ba;             // 加速度バイアス
    Vec3 bg;             // ジャイロバイアス
    Mat15 P;             // 共分散
};
```

### 1.2 統一フィルタ関数実装

**メイン関数** (`cpp/MEUKF/unified_filter.cpp`):

```cpp
#include "unified_types.hpp"
#include "meukf_core.hpp"
#include "sensor_filter.hpp"
#include "noise_estimator.hpp"
#include "divergence_guard.hpp"

class UnifiedFilter {
private:
    // 内部状態（前回値保持）
    Vec3 prev_accel_;
    Vec3 prev_gyro_;
    Vec3 prev_mag_;
    Vec3 prev_gps_pos_;
    double prev_baro_alt_;
    
    // ヘルパークラス
    SensorFilter sensor_filter_;
    NoiseEstimator noise_estimator_;
    DivergenceGuard divergence_guard_;
    
    // 変更検知閾値
    static constexpr double TOLERANCE = 1e-9;
    
public:
    FilterOutput update(FilterState& state, const FilterInput& input) {
        FilterOutput output;
        int update_count = 0;
        
        // 1. 予測ステップ（必ず実行）
        predict_step(state, input.accel, input.gyro, input.dt);
        
        // 2. センサーフィルタリング（C++側で実施）
        Vec3 accel_filtered = sensor_filter_.filter_accel(input.accel);
        Vec3 gyro_filtered = sensor_filter_.filter_gyro(input.gyro);
        Vec3 mag_filtered = sensor_filter_.filter_mag(input.mag);
        
        // 3. 加速度更新（変更検知）
        if (sensor_changed(accel_filtered, prev_accel_)) {
            if (update_accel(state, accel_filtered, input)) {
                update_count++;
            }
            prev_accel_ = accel_filtered;
        }
        
        // 4. 磁気計更新（変更検知）
        if (input.mag_valid && sensor_changed(mag_filtered, prev_mag_)) {
            if (update_mag(state, mag_filtered, input)) {
                update_count++;
            }
            prev_mag_ = mag_filtered;
        }
        
        // 5. GPS更新（変更検知）
        if (input.gps_valid && sensor_changed(input.gps_pos, prev_gps_pos_)) {
            if (update_gps(state, input.gps_pos, input)) {
                update_count++;
            }
            prev_gps_pos_ = input.gps_pos;
        }
        
        // 6. 気圧更新（変更検知）
        if (input.baro_valid && std::abs(input.baro_alt - prev_baro_alt_) > TOLERANCE) {
            if (update_baro(state, input.baro_alt, input)) {
                update_count++;
            }
            prev_baro_alt_ = input.baro_alt;
        }
        
        // 7. ZUPT判定 & 更新
        if (check_zupt(accel_filtered, gyro_filtered)) {
            update_zupt(state);
            update_count++;
        }
        
        // 8. 発散検知 & リセット
        if (divergence_guard_.check(state, output)) {
            reset_filter(state, accel_filtered, mag_filtered);
            output.reset_occurred = true;
        }
        
        // 9. 適応ノイズ推定
        noise_estimator_.update(output.innovation_norm_accel,
                                 output.innovation_norm_mag,
                                 output.innovation_norm_gps,
                                 output.innovation_norm_baro);
        
        // 10. 出力構造体に格納
        pack_output(state, output);
        output.num_updates_applied = update_count;
        
        return output;
    }
    
private:
    bool sensor_changed(const Vec3& new_val, const Vec3& prev_val) {
        return (new_val - prev_val).norm() > TOLERANCE;
    }
    
    void predict_step(FilterState& state, const Vec3& accel, 
                      const Vec3& gyro, double dt) {
        // meukf_core.cppの予測関数を呼び出し
        meukf_predict(state.p, state.v, state.q, state.ba, state.bg, 
                      state.P, accel, gyro, dt);
    }
    
    bool update_accel(FilterState& state, const Vec3& accel_meas,
                      const FilterInput& input) {
        // 外れ値チェック
        if (!sensor_filter_.check_accel_valid(accel_meas)) {
            return false;
        }
        
        // MEUKF更新（姿勢のみ）
        double innov_norm = meukf_update_accel(state.q, state.P, accel_meas,
                                                input.g, input.noise_accel);
        output.innovation_norm_accel = innov_norm;
        return true;
    }
    
    bool update_mag(FilterState& state, const Vec3& mag_meas,
                    const FilterInput& input) {
        // 外れ値チェック
        if (!sensor_filter_.check_mag_valid(mag_meas)) {
            return false;
        }
        
        // MEUKF更新（ヨー角）
        double innov_norm = meukf_update_mag(state.q, state.P, mag_meas,
                                              input.mag_ref, input.noise_mag);
        output.innovation_norm_mag = innov_norm;
        return true;
    }
    
    bool update_gps(FilterState& state, const Vec3& gps_pos,
                    const FilterInput& input) {
        // UKF更新（位置・速度）
        double innov_norm = ukf_update_gps(state.p, state.v, state.P, gps_pos,
                                            input.noise_gps, input.alpha,
                                            input.beta, input.kappa);
        output.innovation_norm_gps = innov_norm;
        return true;
    }
    
    bool update_baro(FilterState& state, double baro_alt,
                     const FilterInput& input) {
        // EKF更新（高度のみ）
        double innov_norm = ekf_update_baro(state.p(2), state.P, baro_alt,
                                             input.noise_baro);
        output.innovation_norm_baro = innov_norm;
        return true;
    }
    
    bool check_zupt(const Vec3& accel, const Vec3& gyro) {
        // 静止判定ロジック
        double accel_norm = accel.norm();
        double gravity_dev = std::abs(accel_norm - 9.81);
        double gyro_norm = gyro.norm();
        
        static int zupt_counter = 0;
        static constexpr int ZUPT_MIN_DURATION = 10;
        static constexpr double ZUPT_ACCEL_THRESHOLD = 1.0;
        static constexpr double ZUPT_GYRO_THRESHOLD = 0.0524;  // 3 deg/s
        
        if (gravity_dev < ZUPT_ACCEL_THRESHOLD && gyro_norm < ZUPT_GYRO_THRESHOLD) {
            zupt_counter++;
        } else {
            zupt_counter = 0;
        }
        
        return (zupt_counter >= ZUPT_MIN_DURATION);
    }
    
    void update_zupt(FilterState& state) {
        // 速度を0に更新（カルマンゲイン使用）
        Vec3 z_zero(0, 0, 0);
        // 簡易実装: 速度を0に設定
        state.v = z_zero;
    }
    
    void reset_filter(FilterState& state, const Vec3& accel, const Vec3& mag) {
        // 姿勢を再初期化
        double phi = std::atan2(-accel(1), -accel(2));
        double theta = std::atan2(accel(0), 
                                  std::sqrt(accel(1)*accel(1) + accel(2)*accel(2)));
        double psi = std::atan2(mag(1), mag(0));
        
        state.q = quaternion_from_euler(phi, theta, psi);
        
        // 共分散を拡大
        state.P(6, 6) = state.P(7, 7) = state.P(8, 8) = 0.5;  // 姿勢の分散を増加
    }
    
    void pack_output(const FilterState& state, FilterOutput& output) {
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
};
```

### 1.3 MEXラッパー実装

**MEXインターフェース** (`cpp/MEX/mex_unified_filter.cpp`):

```cpp
#include "mex.h"
#include "unified_filter.hpp"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // 引数チェック
    if (nrhs != 3) {
        mexErrMsgIdAndTxt("MEUKF:unified:nrhs", "3 inputs required (state, input, params)");
    }
    if (nlhs != 1) {
        mexErrMsgIdAndTxt("MEUKF:unified:nlhs", "1 output required (output struct)");
    }
    
    // 入力1: 状態構造体
    const mxArray* state_struct = prhs[0];
    FilterState state;
    state.p = get_vec3(mxGetField(state_struct, 0, "p"));
    state.v = get_vec3(mxGetField(state_struct, 0, "v"));
    state.q = get_quaternion(mxGetField(state_struct, 0, "q"));
    state.ba = get_vec3(mxGetField(state_struct, 0, "ba"));
    state.bg = get_vec3(mxGetField(state_struct, 0, "bg"));
    state.P = get_mat15(mxGetField(state_struct, 0, "P"));
    
    // 入力2: センサー入力構造体
    const mxArray* input_struct = prhs[1];
    FilterInput input;
    input.dt = mxGetScalar(mxGetField(input_struct, 0, "dt"));
    input.accel = get_vec3(mxGetField(input_struct, 0, "accel"));
    input.gyro = get_vec3(mxGetField(input_struct, 0, "gyro"));
    input.mag = get_vec3(mxGetField(input_struct, 0, "mag"));
    input.gps_pos = get_vec3(mxGetField(input_struct, 0, "gps_pos"));
    input.baro_alt = mxGetScalar(mxGetField(input_struct, 0, "baro_alt"));
    
    input.mag_valid = mxGetScalar(mxGetField(input_struct, 0, "mag_valid"));
    input.gps_valid = mxGetScalar(mxGetField(input_struct, 0, "gps_valid"));
    input.baro_valid = mxGetScalar(mxGetField(input_struct, 0, "baro_valid"));
    
    // 入力3: パラメータ構造体
    const mxArray* params_struct = prhs[2];
    input.g = get_vec3(mxGetField(params_struct, 0, "g"));
    input.mag_ref = get_vec3(mxGetField(params_struct, 0, "mag_ref"));
    input.noise_accel = get_vec3(mxGetField(params_struct, 0, "noise_accel"));
    input.noise_gyro = get_vec3(mxGetField(params_struct, 0, "noise_gyro"));
    input.noise_mag = get_vec3(mxGetField(params_struct, 0, "noise_mag"));
    input.noise_gps = get_vec3(mxGetField(params_struct, 0, "noise_gps"));
    input.noise_baro = mxGetScalar(mxGetField(params_struct, 0, "noise_baro"));
    input.alpha = mxGetScalar(mxGetField(params_struct, 0, "alpha"));
    input.beta = mxGetScalar(mxGetField(params_struct, 0, "beta"));
    input.kappa = mxGetScalar(mxGetField(params_struct, 0, "kappa"));
    
    // 統一フィルタ実行
    UnifiedFilter filter;
    FilterOutput output = filter.update(state, input);
    
    // 出力構造体作成
    const char* output_fields[] = {
        "position", "velocity", "quaternion", "accel_bias", "gyro_bias",
        "covariance", "roll", "pitch", "yaw",
        "innovation_norm_accel", "innovation_norm_mag",
        "innovation_norm_gps", "innovation_norm_baro",
        "divergence_detected", "reset_occurred", "num_updates_applied"
    };
    plhs[0] = mxCreateStructMatrix(1, 1, 16, output_fields);
    
    mxSetField(plhs[0], 0, "position", create_vec3(output.position));
    mxSetField(plhs[0], 0, "velocity", create_vec3(output.velocity));
    mxSetField(plhs[0], 0, "quaternion", create_quaternion(output.quaternion));
    mxSetField(plhs[0], 0, "accel_bias", create_vec3(output.accel_bias));
    mxSetField(plhs[0], 0, "gyro_bias", create_vec3(output.gyro_bias));
    mxSetField(plhs[0], 0, "covariance", create_mat15(output.covariance));
    
    mxSetField(plhs[0], 0, "roll", mxCreateDoubleScalar(output.roll));
    mxSetField(plhs[0], 0, "pitch", mxCreateDoubleScalar(output.pitch));
    mxSetField(plhs[0], 0, "yaw", mxCreateDoubleScalar(output.yaw));
    
    mxSetField(plhs[0], 0, "innovation_norm_accel", 
               mxCreateDoubleScalar(output.innovation_norm_accel));
    mxSetField(plhs[0], 0, "innovation_norm_mag", 
               mxCreateDoubleScalar(output.innovation_norm_mag));
    mxSetField(plhs[0], 0, "innovation_norm_gps", 
               mxCreateDoubleScalar(output.innovation_norm_gps));
    mxSetField(plhs[0], 0, "innovation_norm_baro", 
               mxCreateDoubleScalar(output.innovation_norm_baro));
    
    mxSetField(plhs[0], 0, "divergence_detected", 
               mxCreateLogicalScalar(output.divergence_detected));
    mxSetField(plhs[0], 0, "reset_occurred", 
               mxCreateLogicalScalar(output.reset_occurred));
    mxSetField(plhs[0], 0, "num_updates_applied", 
               mxCreateDoubleScalar(output.num_updates_applied));
}
```

---

## フェーズ2: MATLAB側簡素化

### 2.1 データ生成の周波数制御

**`sim_generate.m`の修正**:

```matlab
function sim_generate()
    params = config_params();
    
    % 基本レート: 100 Hz
    base_freq = 100;
    dt = 1 / base_freq;
    duration = params.duration;
    N = floor(duration / dt);
    
    % 真値生成（100 Hz）
    truth = generate_motion(params.motion_type, N, dt);
    
    % IMUセンサー（100 Hz - 複製不要）
    obs.time = truth.time;
    obs.accel_x = truth.accel_body_x + randn(N,1) * params.sigma_accel;
    obs.accel_y = truth.accel_body_y + randn(N,1) * params.sigma_accel;
    obs.accel_z = truth.accel_body_z + randn(N,1) * params.sigma_accel;
    obs.gyro_x = truth.gyro_body_x + randn(N,1) * params.sigma_gyro;
    obs.gyro_y = truth.gyro_body_y + randn(N,1) * params.sigma_gyro;
    obs.gyro_z = truth.gyro_body_z + randn(N,1) * params.sigma_gyro;
    
    % 磁気計（25 Hz → 4回複製）
    mag_freq = 25;
    mag_ratio = base_freq / mag_freq;  % = 4
    N_mag = floor(N / mag_ratio);
    mag_data_x = truth.mag_body_x(1:mag_ratio:end) + randn(N_mag,1) * params.sigma_mag;
    mag_data_y = truth.mag_body_y(1:mag_ratio:end) + randn(N_mag,1) * params.sigma_mag;
    mag_data_z = truth.mag_body_z(1:mag_ratio:end) + randn(N_mag,1) * params.sigma_mag;
    
    % 複製して100 Hzに拡張
    obs.mag_x = repelem(mag_data_x, mag_ratio);
    obs.mag_y = repelem(mag_data_y, mag_ratio);
    obs.mag_z = repelem(mag_data_z, mag_ratio);
    
    % GPS（4 Hz → 25回複製）
    gps_freq = 4;
    gps_ratio = base_freq / gps_freq;  % = 25
    N_gps = floor(N / gps_ratio);
    gps_lat = truth.lat(1:gps_ratio:end) + randn(N_gps,1) * params.sigma_gps_deg;
    gps_lon = truth.lon(1:gps_ratio:end) + randn(N_gps,1) * params.sigma_gps_deg;
    gps_alt = truth.alt(1:gps_ratio:end) + randn(N_gps,1) * params.sigma_gps_alt;
    
    obs.gps_lat = repelem(gps_lat, gps_ratio);
    obs.gps_lon = repelem(gps_lon, gps_ratio);
    obs.gps_alt = repelem(gps_alt, gps_ratio);
    
    % 気圧計（2 Hz → 50回複製）
    baro_freq = 2;
    baro_ratio = base_freq / baro_freq;  % = 50
    N_baro = floor(N / baro_ratio);
    baro_data = truth.alt(1:baro_ratio:end) + randn(N_baro,1) * params.sigma_baro;
    
    obs.baro = repelem(baro_data, baro_ratio);
    
    % CSVに保存
    save_csv('GenerateData/sensor_data.csv', obs);
    save_csv('GenerateData/truth_data.csv', truth);
    
    fprintf('データ生成完了: %d サンプル (100 Hz)\n', N);
    fprintf('  磁気計: %d回複製 (25 Hz)\n', mag_ratio);
    fprintf('  GPS: %d回複製 (4 Hz)\n', gps_ratio);
    fprintf('  気圧計: %d回複製 (2 Hz)\n', baro_ratio);
end
```

### 2.2 ESKF.mの簡素化

**`ESKF/@ESKF/ESKF.m`の最小化**:

```matlab
classdef ESKF < handle
    properties
        % 状態（C++から返却された値を保持）
        p; v; q; ba; bg; P;
        
        % パラメータ（初期化時に設定、C++に渡す）
        dt; g; mag_ref;
        noise_accel; noise_gyro; noise_mag; noise_gps; noise_baro;
        alpha; beta; kappa;
        gps_origin;
    end
    
    methods
        function obj = ESKF(obs, static_time, dt)
            % 最小限の初期化（C++に渡すパラメータのみ設定）
            
            obj.dt = dt;
            obj.g = [0; 0; 9.80665];
            
            % 静止期間からノイズ推定
            if nargin >= 2 && ~isempty(static_time) && static_time > 0
                N_static = floor(static_time / dt);
                static_idx = 1:N_static;
                
                % 加速度からロール・ピッチ初期化
                accel_static = [obs.accel_x(static_idx), ...
                                obs.accel_y(static_idx), ...
                                obs.accel_z(static_idx)];
                accel_mean = mean(accel_static, 1);
                obj.noise_accel = ones(3,1) * std(accel_static(:));
                
                phi = atan2(-accel_mean(2), -accel_mean(3));
                theta = atan2(accel_mean(1), sqrt(accel_mean(2)^2 + accel_mean(3)^2));
                
                % 磁気計からヨー初期化
                mag_static = [obs.mag_x(static_idx), ...
                              obs.mag_y(static_idx), ...
                              obs.mag_z(static_idx)];
                mag_mean = mean(mag_static, 1);
                obj.noise_mag = ones(3,1) * std(mag_static(:));
                
                psi = atan2(mag_mean(2), mag_mean(1));
                obj.q = mex_quaternion_lib('from_euler', rad2deg([phi; theta; psi]));
                obj.mag_ref = [mag_mean(1); 0; mag_mean(3)];
                
                % ジャイロノイズ
                gyro_static = [obs.gyro_x(static_idx), ...
                               obs.gyro_y(static_idx), ...
                               obs.gyro_z(static_idx)];
                obj.noise_gyro = ones(3,1) * deg2rad(std(gyro_static(:)));
                
                % GPS・気圧ノイズ
                obj.noise_gps = ones(3,1) * 5.0;  % デフォルト 5m
                obj.noise_baro = 1.0;  % デフォルト 1m
                
                % GPS原点
                obj.gps_origin = [mean(obs.gps_lat(static_idx)); ...
                                  mean(obs.gps_lon(static_idx)); ...
                                  mean(obs.gps_alt(static_idx))];
            else
                % デフォルト値
                obj.q = [1; 0; 0; 0];
                obj.mag_ref = [1; 0; 0];
                obj.noise_accel = ones(3,1) * 0.01;
                obj.noise_gyro = ones(3,1) * deg2rad(0.1);
                obj.noise_mag = ones(3,1) * 0.1;
                obj.noise_gps = ones(3,1) * 5.0;
                obj.noise_baro = 1.0;
                obj.gps_origin = [0; 0; 0];
            end
            
            % 初期状態
            obj.p = zeros(3,1);
            obj.v = zeros(3,1);
            obj.ba = zeros(3,1);
            obj.bg = zeros(3,1);
            obj.P = eye(15) * 0.01;
            obj.P(1:3, 1:3) = eye(3) * 5.0;   % 位置の初期分散
            obj.P(4:6, 4:6) = eye(3) * 0.5;   % 速度の初期分散
            
            % UKFパラメータ
            obj.alpha = 1e-3;
            obj.beta = 2;
            obj.kappa = 0;
            
            % MEXパス追加
            addpath(genpath('cpp'));
        end
        
        function update_filter(obj, obs, k)
            % C++統一フィルタを1回呼び出すだけ
            
            % 入力構造体パック
            input = struct();
            input.dt = obj.dt;
            input.accel = [obs.accel_x(k); obs.accel_y(k); obs.accel_z(k)];
            input.gyro = deg2rad([obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)]);
            input.mag = [obs.mag_x(k); obs.mag_y(k); obs.mag_z(k)];
            
            % GPS座標変換（NED）
            lat0 = obj.gps_origin(1);
            lon0 = obj.gps_origin(2);
            alt0 = obj.gps_origin(3);
            y_m = (obs.gps_lat(k) - lat0) / (9.0e-6);
            x_m = (obs.gps_lon(k) - lon0) / (9.0e-6 / cosd(lat0));
            z_m = obs.gps_alt(k) - alt0;
            input.gps_pos = [y_m; x_m; -z_m];
            
            input.baro_alt = obs.baro(k);
            
            % 有効フラグ
            input.mag_valid = true;
            input.gps_valid = ~isnan(obs.gps_lat(k));
            input.baro_valid = ~isnan(obs.baro(k));
            
            % パラメータ構造体パック
            params = struct();
            params.g = obj.g;
            params.mag_ref = obj.mag_ref;
            params.noise_accel = obj.noise_accel;
            params.noise_gyro = obj.noise_gyro;
            params.noise_mag = obj.noise_mag;
            params.noise_gps = obj.noise_gps;
            params.noise_baro = obj.noise_baro;
            params.alpha = obj.alpha;
            params.beta = obj.beta;
            params.kappa = obj.kappa;
            
            % 状態構造体パック
            state = struct();
            state.p = obj.p;
            state.v = obj.v;
            state.q = obj.q;
            state.ba = obj.ba;
            state.bg = obj.bg;
            state.P = obj.P;
            
            % C++統一フィルタ呼び出し
            output = mex_unified_filter(state, input, params);
            
            % 状態更新
            obj.p = output.position;
            obj.v = output.velocity;
            obj.q = output.quaternion;
            obj.ba = output.accel_bias;
            obj.bg = output.gyro_bias;
            obj.P = output.covariance;
        end
        
        function euler = get_euler(obj)
            % オイラー角取得（可視化用）
            euler = mex_quaternion_lib('to_euler', obj.q);
        end
    end
end
```

### 2.3 run_simulation.mの簡素化

**`run_simulation.m`の修正**:

```matlab
function run_simulation(seed, skip_data_gen)
    clc; rehash; clear functions;
    if nargin >= 1 && ~isempty(seed); rng(seed, 'twister'); end
    if nargin < 2; skip_data_gen = false; end
    
    proj_root = fileparts(mfilename('fullpath'));
    addpath(genpath(fullfile(proj_root, 'ESKF')));
    addpath(fullfile(proj_root, 'GenerateData'));
    addpath(fullfile(proj_root, 'Graph'));
    addpath(genpath(fullfile(proj_root, 'cpp')));
    
    if ~skip_data_gen; sim_generate(); end
    
    % データ読み込み
    obs_file = fullfile(proj_root, 'GenerateData', 'sensor_data.csv');
    obs = read_csv(obs_file);
    
    % ESKF初期化
    params = config_params();
    dt = mean(diff(obs.time));
    eskf = ESKF(obs, params.static_time, dt);
    
    % メインループ（シンプル化）
    n_samples = numel(obs.time);
    static_samples = floor(params.static_time / dt);
    
    % 結果格納
    results.time = obs.time(:)';
    results.p = zeros(3, n_samples);
    results.v = zeros(3, n_samples);
    results.euler = zeros(3, n_samples);
    results.ba = zeros(3, n_samples);
    results.bg = zeros(3, n_samples);
    
    fprintf('推定開始: %d サンプル\n', n_samples);
    
    for k = 1:n_samples
        if k > static_samples
            % C++統一フィルタを呼び出すだけ
            eskf.update_filter(obs, k);
        end
        
        % 結果格納
        results.p(:,k) = eskf.p;
        results.v(:,k) = eskf.v;
        results.euler(:,k) = eskf.get_euler();
        results.ba(:,k) = eskf.ba;
        results.bg(:,k) = eskf.bg;
        
        if mod(k, 1000) == 0
            fprintf('Step %d / %d\n', k, n_samples);
        end
    end
    
    % 結果保存
    save_results(proj_root, results);
    fprintf('推定完了\n');
end

function save_results(proj_root, results)
    out_dir = fullfile(proj_root, 'Results');
    if ~exist(out_dir,'dir'); mkdir(out_dir); end
    
    T = table(results.time(:), ...
              results.p(1,:)', results.p(2,:)', results.p(3,:)', ...
              results.v(1,:)', results.v(2,:)', results.v(3,:)', ...
              results.euler(1,:)', results.euler(2,:)', results.euler(3,:)', ...
              results.ba(1,:)', results.ba(2,:)', results.ba(3,:)', ...
              results.bg(1,:)', results.bg(2,:)', results.bg(3,:)', ...
              'VariableNames', {'time', 'px', 'py', 'pz', 'vx', 'vy', 'vz', ...
                                'roll', 'pitch', 'yaw', 'bax', 'bay', 'baz', ...
                                'bgx', 'bgy', 'bgz'});
    writetable(T, fullfile(out_dir, 'estimation.csv'));
end
```

---

## フェーズ3: C++ヘルパークラス移植

### 3.1 センサーフィルタ

**`cpp/include/Common/Sensor/sensor_filter.hpp`** (既存を拡張):

```cpp
class SensorFilter {
private:
    // EMAフィルタ
    EMAFilter accel_ema_{0.3};
    EMAFilter gyro_ema_{0.2};
    EMAFilter mag_ema_{0.1};
    
    // Biquadローパス
    BiquadLowpassFilter accel_biquad_;
    BiquadLowpassFilter gyro_biquad_;
    
    // 外れ値検出
    OutlierDetector outlier_detector_;
    
public:
    SensorFilter() {
        // 初期化（サンプリング周期は固定: 100 Hz）
        double dt = 0.01;
        accel_biquad_.configure(dt, 30.0);  // 30 Hz cutoff
        gyro_biquad_.configure(dt, 50.0);   // 50 Hz cutoff
    }
    
    Vec3 filter_accel(const Vec3& accel_raw) {
        Vec3 filtered = accel_ema_.filter(accel_raw);
        return accel_biquad_.filter(filtered);
    }
    
    Vec3 filter_gyro(const Vec3& gyro_raw) {
        Vec3 filtered = gyro_ema_.filter(gyro_raw);
        return gyro_biquad_.filter(filtered);
    }
    
    Vec3 filter_mag(const Vec3& mag_raw) {
        return mag_ema_.filter(mag_raw);
    }
    
    bool check_accel_valid(const Vec3& accel) {
        double norm = accel.norm();
        return (norm > 0.1 && std::abs(norm - 9.81) < 3.0);
    }
    
    bool check_mag_valid(const Vec3& mag) {
        double norm = mag.norm();
        return (norm > 1e-6 && norm < 100.0);  // 妥当な範囲
    }
};
```

### 3.2 適応ノイズ推定

**`cpp/include/Common/Estimation/noise_estimator.hpp`**:

```cpp
class NoiseEstimator {
private:
    static constexpr double ALPHA = 0.01;  // EMA平滑化係数
    static constexpr double R_MIN = 1e-10;
    static constexpr double R_MAX = 1e6;
    
    double R_accel_avg_;
    double R_mag_avg_;
    double R_gps_avg_;
    double R_baro_avg_;
    
public:
    NoiseEstimator() 
        : R_accel_avg_(0.01), R_mag_avg_(0.1), 
          R_gps_avg_(5.0), R_baro_avg_(1.0) {}
    
    void update(double innov_accel, double innov_mag,
                double innov_gps, double innov_baro) {
        // イノベーションから動的にR行列を更新
        if (innov_accel > 0) {
            R_accel_avg_ = (1 - ALPHA) * R_accel_avg_ + ALPHA * innov_accel;
            R_accel_avg_ = std::clamp(R_accel_avg_, R_MIN, R_MAX);
        }
        
        if (innov_mag > 0) {
            R_mag_avg_ = (1 - ALPHA) * R_mag_avg_ + ALPHA * innov_mag;
            R_mag_avg_ = std::clamp(R_mag_avg_, R_MIN, R_MAX);
        }
        
        if (innov_gps > 0) {
            R_gps_avg_ = (1 - ALPHA) * R_gps_avg_ + ALPHA * innov_gps;
            R_gps_avg_ = std::clamp(R_gps_avg_, R_MIN, R_MAX);
        }
        
        if (innov_baro > 0) {
            R_baro_avg_ = (1 - ALPHA) * R_baro_avg_ + ALPHA * innov_baro;
            R_baro_avg_ = std::clamp(R_baro_avg_, R_MIN, R_MAX);
        }
    }
    
    double get_R_accel() const { return R_accel_avg_; }
    double get_R_mag() const { return R_mag_avg_; }
    double get_R_gps() const { return R_gps_avg_; }
    double get_R_baro() const { return R_baro_avg_; }
};
```

### 3.3 発散検知

**`cpp/include/Common/Validation/divergence_guard.hpp`**:

```cpp
class DivergenceGuard {
private:
    static constexpr double MAX_VELOCITY = 5.0;  // m/s
    static constexpr double MAX_INNOVATION_ACCEL = 5.0;
    static constexpr double MAX_INNOVATION_MAG = 2.0;
    static constexpr double MAX_INNOVATION_GPS = 20.0;
    static constexpr double MAX_TRACE_P = 500.0;
    
public:
    bool check(const FilterState& state, const FilterOutput& output) {
        // 速度チェック
        if (state.v.norm() > MAX_VELOCITY) {
            return true;
        }
        
        // イノベーションチェック
        if (output.innovation_norm_accel > MAX_INNOVATION_ACCEL ||
            output.innovation_norm_mag > MAX_INNOVATION_MAG ||
            output.innovation_norm_gps > MAX_INNOVATION_GPS) {
            return true;
        }
        
        // 共分散トレースチェック
        double trace_P = state.P.trace();
        if (trace_P > MAX_TRACE_P) {
            return true;
        }
        
        // NaN/Infチェック
        if (!state.p.allFinite() || !state.v.allFinite() || 
            !state.q.coeffs().allFinite()) {
            return true;
        }
        
        return false;
    }
    
    Mat15 regularize_covariance(const Mat15& P) {
        // 対称性を強制
        Mat15 P_sym = (P + P.transpose()) / 2.0;
        
        // 最小固有値を確保（正定値性）
        Eigen::SelfAdjointEigenSolver<Mat15> es(P_sym);
        if (es.eigenvalues().minCoeff() < 1e-10) {
            Mat15 P_reg = P_sym + Mat15::Identity() * 1e-8;
            return P_reg;
        }
        
        return P_sym;
    }
};
```

---

## フェーズ4: ビルド & テスト

### 4.1 ビルドスクリプト更新

**`cpp/build/build_mex.m`**:

```matlab
function build_mex()
    % 統一フィルタのビルド
    
    fprintf('=== 統一フィルタC++実装のビルド開始 ===\n');
    
    % インクルードパス
    inc_path = '-I../include';
    
    % 最適化フラグ
    opt_flags = 'OPTIMFLAGS="-O3 -DNDEBUG"';
    cpp_std = 'CXXFLAGS="$CXXFLAGS -std=c++17"';
    
    % mex_unified_filter.mexw64のビルド
    fprintf('Building mex_unified_filter...\n');
    mex('-v', opt_flags, cpp_std, inc_path, ...
        '../MEX/mex_unified_filter.cpp', ...
        '../MEUKF/unified_filter.cpp', ...
        '../MEUKF/meukf_core.cpp', ...
        '-output', '../bin/mex_unified_filter');
    
    % ユーティリティMEX（QuaternionLib等）のビルド
    fprintf('Building mex_quaternion_lib...\n');
    mex('-v', opt_flags, cpp_std, inc_path, ...
        '../MEX/mex_quaternion_lib.cpp', ...
        '-output', '../bin/mex_quaternion_lib');
    
    fprintf('=== ビルド完了 ===\n');
    fprintf('出力先: ../bin/\n');
    fprintf('  - mex_unified_filter.mexw64\n');
    fprintf('  - mex_quaternion_lib.mexw64\n');
end
```

### 4.2 検証テスト

**`test_unified_filter.m`** (新規作成):

```matlab
function test_unified_filter()
    % 統一フィルタの単体テスト
    
    fprintf('=== 統一フィルタ検証テスト ===\n');
    
    % 1. MEXファイルの存在確認
    if exist('mex_unified_filter', 'file') ~= 3
        error('mex_unified_filter.mexw64 が見つかりません');
    end
    fprintf('✓ MEXファイル検出\n');
    
    % 2. 初期状態設定
    state = struct();
    state.p = zeros(3,1);
    state.v = zeros(3,1);
    state.q = [1;0;0;0];
    state.ba = zeros(3,1);
    state.bg = zeros(3,1);
    state.P = eye(15) * 0.01;
    
    % 3. 入力データ（静止状態）
    input = struct();
    input.dt = 0.01;
    input.accel = [0; 0; -9.81];  % 静止（重力のみ）
    input.gyro = zeros(3,1);
    input.mag = [1; 0; 0];
    input.gps_pos = zeros(3,1);
    input.baro_alt = 0;
    input.mag_valid = true;
    input.gps_valid = true;
    input.baro_valid = true;
    
    % 4. パラメータ
    params = struct();
    params.g = [0; 0; 9.80665];
    params.mag_ref = [1; 0; 0];
    params.noise_accel = ones(3,1) * 0.01;
    params.noise_gyro = ones(3,1) * deg2rad(0.1);
    params.noise_mag = ones(3,1) * 0.1;
    params.noise_gps = ones(3,1) * 5.0;
    params.noise_baro = 1.0;
    params.alpha = 1e-3;
    params.beta = 2;
    params.kappa = 0;
    
    % 5. 実行テスト（10ステップ）
    for k = 1:10
        output = mex_unified_filter(state, input, params);
        
        % 出力チェック
        assert(~any(isnan(output.position)), 'NaN in position');
        assert(~any(isnan(output.velocity)), 'NaN in velocity');
        assert(~any(isnan(output.quaternion)), 'NaN in quaternion');
        assert(norm(output.quaternion) > 0.99, 'Quaternion not normalized');
        
        % 状態更新
        state.p = output.position;
        state.v = output.velocity;
        state.q = output.quaternion;
        state.ba = output.accel_bias;
        state.bg = output.gyro_bias;
        state.P = output.covariance;
    end
    
    fprintf('✓ 10ステップ実行成功\n');
    
    % 6. 変更検知テスト
    % 磁気計データを変更
    input.mag = [0.5; 0; 0];
    output = mex_unified_filter(state, input, params);
    fprintf('✓ センサー変更検知テスト成功\n');
    
    % 7. GPS更新テスト
    input.gps_pos = [10; 5; -2];  % 10m北、5m東、2m下
    output = mex_unified_filter(state, input, params);
    assert(norm(output.position) > 0, 'GPS update failed');
    fprintf('✓ GPS更新テスト成功\n');
    
    fprintf('\n=== 全テスト成功 ===\n');
end
```

---

## 実装順序

### ステップ1: 基盤整備（1週間）
1. `unified_types.hpp` - 構造体定義
2. `unified_filter.cpp` - 骨格実装（予測 + センサー更新のスタブ）
3. `mex_unified_filter.cpp` - MEXラッパー
4. `build_mex.m` - ビルドスクリプト更新
5. `test_unified_filter.m` - 検証スクリプト

### ステップ2: センサー更新実装（2週間）
1. `update_accel()` - 加速度更新（MEUKF）
2. `update_mag()` - 磁気計更新（MEUKF）
3. `update_gps()` - GPS更新（UKF）
4. `update_baro()` - 気圧更新（EKF）
5. 変更検知ロジック追加

### ステップ3: ヘルパークラス移植（1週間）
1. `sensor_filter.hpp` - フィルタ実装完成
2. `noise_estimator.hpp` - 適応ノイズ推定
3. `divergence_guard.hpp` - 発散検知

### ステップ4: MATLAB側簡素化（1週間）
1. `sim_generate.m` - 周波数制御データ生成
2. `ESKF.m` - 最小限のラッパーに変更
3. `run_simulation.m` - シンプル化

### ステップ5: 統合テスト（1週間）
1. 単体テスト（各センサー更新）
2. 統合テスト（run_simulation）
3. バッチテスト（run_batch_10sets）
4. 精度検証（既存結果と比較）

---

## 成功基準

- [ ] `mex_unified_filter.mexw64` が正常にビルドできる
- [ ] `test_unified_filter.m` が全テスト成功
- [ ] `run_simulation()` が従来と同じ精度で動作
- [ ] `run_batch_10sets()` の RMSE が従来と同等（Position < 5m, Attitude < 2°）
- [ ] MATLAB コード行数が 80% 削減（2000行 → 400行程度）
- [ ] 実行速度が同等以上（C++化による高速化）

---

## リスク管理

| リスク | 影響 | 対策 |
|--------|------|------|
| MEXビルドエラー | 高 | 段階的ビルド、詳細なエラーログ |
| 精度劣化 | 高 | 各フェーズで精度検証、rollback可能に |
| 変更検知の誤動作 | 中 | 閾値調整、テストケース追加 |
| NaN発生 | 中 | 入力チェック強化、安全な数値計算 |

---

## 次のステップ

1. このドキュメントをレビュー
2. フェーズ1のコード実装開始
3. 段階的にテスト・デバッグ
4. 各フェーズ完了後に `.github/copilot-instructions.md` 更新
