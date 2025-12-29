# Phase 1: ESKFコンストラクタのMEX化 - 詳細設計

**作成日**: 2025-12-27  
**目標**: `ESKF()` コンストラクタを完全にMEX化

---

## 📋 現状分析

### ESKF.m コンストラクタの処理内容

1. **パラメータ初期化** (23-33行)
   - `dt` の設定
   - `static_idx` の計算

2. **基本状態の初期化** (35-37行)
   - `p = zeros(3,1)`
   - `v = zeros(3,1)`
   - `g = [0;0;9.80665]`

3. **静止データからの初期化** (39-99行)
   - 加速度データから初期姿勢（roll/pitch）計算
   - ジャイロデータからノイズ推定
   - 磁気データから初期yaw計算
   - 気圧データからノイズ推定
   - GPSデータから原点とノイズ推定

4. **デフォルト値の設定** (100-104行)
   - 静止データがない場合のデフォルト値

5. **バイアス初期化** (106-107行)
   - `ba = zeros(3,1)`
   - `bg = zeros(3,1)`

6. **Q行列の初期化** (109-113行)
   - プロセスノイズ共分散行列

7. **P行列の初期化** (115-119行)
   - 状態共分散行列

8. **state_handleの初期化** (121-127行)
   - `mex_eskf_init` の呼び出し（オプション）

9. **noiseEstimatorの初期化** (129-136行)
   - ノイズ推定器の設定

10. **その他の初期化** (138-167行)
    - `sensor_filters`, `divergence_guard`, `prev_*`, `freq_*`, `zupt_*`, etc.

---

## 🎯 MEX関数設計

### 関数名: `mex_eskf_constructor`

### インターフェース

```matlab
% 使用例
init_data = mex_eskf_constructor('init', obs, static_time, dt);
```

### 入力引数

| 引数 | 型 | 説明 |
|------|-----|------|
| `obs` | struct | 観測データ構造体 |
| `static_time` | double | 静止時間 [秒] |
| `dt` | double | サンプリング時間 [秒] |

### 出力引数

```matlab
init_data = struct(
    'p', [3×1],           % 初期位置
    'v', [3×1],           % 初期速度
    'q', [4×1],           % 初期クォータニオン
    'ba', [3×1],          % 加速度バイアス
    'bg', [3×1],          % ジャイロバイアス
    'P', [15×15],         % 状態共分散行列
    'Q', [15×15],         % プロセスノイズ共分散行列
    'g', [3×1],           % 重力ベクトル
    'gps_origin', [3×1],  % GPS原点
    'gyro_noise_threshold', double,  % ジャイロノイズ閾値
    'noiseEstimator', struct(         % ノイズ推定器
        'R_accel', [3×1],
        'R_gyro', [3×1],
        'R_mag', [3×1],
        'R_baro', double,
        'R_gps', [3×1]
    ),
    'prev_accel', [3×1],
    'prev_gyro', [3×1],
    'prev_mag', [3×1],
    'prev_gps_lat', double,
    'prev_gps_lon', double,
    'prev_gps_alt', double,
    'prev_baro', double,
    'freq_accel', int32,
    'freq_mag', int32,
    'freq_baro', int32,
    'freq_gps', int32,
    'zupt_threshold_accel', double,
    'zupt_threshold_gyro', double,
    'zupt_min_duration', int32,
    'zupt_counter', int32,
    'is_stationary', logical,
    'Q_nominal', [15×15],
    'adaptive_q_enabled', logical,
    'last_reset_step', [],
    'velocity_damping', double,
    'w_body', [3×1],
    'quaternion_norm', double,
    'accel_innovation_norm', double,
    'enable_accel_z_integration', logical,
    'accel_z_threshold', double,
    'accel_z_damping', double,
    'baro_weight', double,
    'buffer_tolerance', double
);
```

---

## 🔧 C++実装詳細

### ファイル: `kalman/cpp/MEX/mex_eskf_constructor.cpp`

### 主要関数

```cpp
// メイン処理
static mxArray* handle_init(const mxArray* obs, double static_time, double dt);

// ヘルパー関数
static bool get_field(const mxArray* s, const char* name, mxArray** out);
static bool has_field(const mxArray* s, const char* name);
static double* get_field_data(const mxArray* arr, int n_elements);
static void compute_initial_attitude(const mxArray* accel_data, int n_samples, 
                                     double* phi, double* theta);
static void compute_initial_yaw(const mxArray* mag_data, int n_samples,
                                 double phi, double theta, double* psi);
static void compute_noise_std(const mxArray* data, int n_samples, int n_dims,
                               double* sigma);
static void compute_gps_origin(const mxArray* lat_data, const mxArray* lon_data,
                                const mxArray* alt_data, int n_samples,
                                double* lat0, double* lon0, double* alt0);
static void initialize_Q_matrix(double sigma_a, double sigma_g, double* Q);
static void initialize_P_matrix(double* P);
```

### 実装のポイント

1. **既存MEX関数の活用**
   - `mex_matlab_helpers('get_field')` でフィールド取得
   - `mex_quaternion_lib('from_euler')` でクォータニオン生成
   - `mex_quaternion_lib('to_rotation_matrix')` で回転行列生成

2. **数値計算**
   - MATLABの`mean()`, `std()` をC++で実装
   - `atan2()`, `sqrt()`, `cos()` などの数学関数を使用

3. **エラーハンドリング**
   - フィールドが存在しない場合の処理
   - データが不足している場合のデフォルト値設定

---

## 📝 ESKF.m の修正

### 修正前

```matlab
function obj = ESKF(obs, static_time, dt)
    if nargin < 3 || isempty(dt), dt = 1/100; end
    obj.dt = dt;
    % ... 既存の初期化コード ...
end
```

### 修正後

```matlab
function obj = ESKF(obs, static_time, dt)
    if nargin < 3 || isempty(dt), dt = 1/100; end
    obj.dt = dt;
    
    % MEX版コンストラクタを使用
    if exist('mex_eskf_constructor', 'file') == 3
        try
            init_data = mex_eskf_constructor('init', obs, static_time, dt);
            obj = obj.set_from_init_data(init_data);
            return;
        catch ME
            warning('MEX constructor failed, using MATLAB fallback: %s', ME.message);
        end
    end
    
    % MATLAB版（フォールバック）
    % ... 既存の初期化コード ...
end

function obj = set_from_init_data(obj, init_data)
    obj.p = init_data.p(:);
    obj.v = init_data.v(:);
    obj.q = init_data.q(:);
    obj.ba = init_data.ba(:);
    obj.bg = init_data.bg(:);
    obj.P = init_data.P;
    obj.Q = init_data.Q;
    obj.g = init_data.g(:);
    obj.gps_origin = init_data.gps_origin(:);
    obj.gyro_noise_threshold = init_data.gyro_noise_threshold;
    
    obj.noiseEstimator = struct();
    obj.noiseEstimator.getRnoise = @(s) mex_sensor_filter('get_R', s);
    obj.noiseEstimator.estimate = @(s, innov, H, P) mex_sensor_filter('noise_estimate', s, innov, H, P);
    obj.noiseEstimator.R_accel = init_data.noiseEstimator.R_accel(:);
    obj.noiseEstimator.R_gyro = init_data.noiseEstimator.R_gyro(:);
    obj.noiseEstimator.R_mag = init_data.noiseEstimator.R_mag(:);
    obj.noiseEstimator.R_baro = init_data.noiseEstimator.R_baro;
    obj.noiseEstimator.R_gps = init_data.noiseEstimator.R_gps(:);
    
    obj.sensor_filters = struct('accel', [], 'gyro', [], 'mag', [], 'gps', [], 'baro', []);
    obj.accel_filter = [];
    
    config = struct('max_velocity', 3, 'max_acceleration', 3, 'max_allowed_innov', 100, ...
        'max_innov_cap_fraction', 0.6, 'max_gain_norm', 150, 'innov_change_ratio_threshold', 2.5, ...
        'attenuation_factor', 0.6, 'max_attitude_variance', (deg2rad(15))^2, 'max_mag_gain_element', 0.2);
    obj.divergence_guard = struct();
    obj.divergence_guard.check_and_attenuate_update = @(s, i, d, c) mex_sensor_filter('divergence_check', s, i, d);
    obj.divergence_guard.regularize_covariance = @(P) mex_sensor_filter('divergence_regularize', P);
    obj.divergence_guard.check_and_clip_velocity = @(v, P, idx) mex_sensor_filter('divergence_clip_velocity', v, P, idx);
    
    obj.prev_accel = init_data.prev_accel(:);
    obj.prev_gyro = init_data.prev_gyro(:);
    obj.prev_mag = init_data.prev_mag(:);
    obj.prev_gps_lat = init_data.prev_gps_lat;
    obj.prev_gps_lon = init_data.prev_gps_lon;
    obj.prev_gps_alt = init_data.prev_gps_alt;
    obj.prev_baro = init_data.prev_baro;
    obj.buffer_tolerance = init_data.buffer_tolerance;
    obj.freq_accel = init_data.freq_accel;
    obj.freq_mag = init_data.freq_mag;
    obj.freq_baro = init_data.freq_baro;
    obj.freq_gps = init_data.freq_gps;
    
    obj.zupt_threshold_accel = init_data.zupt_threshold_accel;
    obj.zupt_threshold_gyro = init_data.zupt_threshold_gyro;
    obj.zupt_min_duration = init_data.zupt_min_duration;
    obj.zupt_counter = init_data.zupt_counter;
    obj.is_stationary = init_data.is_stationary;
    
    obj.Q_nominal = init_data.Q_nominal;
    obj.adaptive_q_enabled = init_data.adaptive_q_enabled;
    
    obj.w_body = init_data.w_body(:);
    obj.last_reset_step = init_data.last_reset_step;
    obj.velocity_damping = init_data.velocity_damping;
    obj.accel_innovation_norm = init_data.accel_innovation_norm;
    obj.quaternion_norm = init_data.quaternion_norm;
    
    obj.enable_accel_z_integration = init_data.enable_accel_z_integration;
    obj.accel_z_threshold = init_data.accel_z_threshold;
    obj.accel_z_damping = init_data.accel_z_damping;
    obj.baro_weight = init_data.baro_weight;
    
    obj.options = struct('preproc_in_matlab', true);
    
    if exist('mex_eskf_init', 'file') == 3
        state_in = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
            'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P);
        obj.state_handle = mex_eskf_init(state_in, struct());
    else
        obj.state_handle = uint64(0);
    end
end
```

---

## ✅ 検証方法

### 1. 単体テスト

```matlab
function test_eskf_constructor()
    % テストデータ生成
    obs = struct();
    obs.ax = randn(2000, 1);
    obs.ay = randn(2000, 1);
    obs.az = randn(2000, 1) - 9.80665;
    obs.wx = randn(2000, 1);
    obs.wy = randn(2000, 1);
    obs.wz = randn(2000, 1);
    obs.mx = randn(2000, 1) * 50;
    obs.my = randn(2000, 1) * 50;
    obs.mz = randn(2000, 1) * 50;
    obs.pressure = 101325 + randn(2000, 1) * 100;
    obs.lat = 35.0 + randn(2000, 1) * 0.001;
    obs.lon = 139.0 + randn(2000, 1) * 0.001;
    obs.alt = 100 + randn(2000, 1) * 10;
    
    static_time = 20.0;
    dt = 0.01;
    
    % MATLAB版
    eskf_matlab = ESKF(obs, static_time, dt);
    
    % MEX版
    init_data = mex_eskf_constructor('init', obs, static_time, dt);
    eskf_mex = ESKF();
    eskf_mex = eskf_mex.set_from_init_data(init_data);
    
    % 比較
    assert(norm(eskf_matlab.p - eskf_mex.p) < 1e-10, 'p mismatch');
    assert(norm(eskf_matlab.v - eskf_mex.v) < 1e-10, 'v mismatch');
    assert(norm(eskf_matlab.q - eskf_mex.q) < 1e-10, 'q mismatch');
    assert(norm(eskf_matlab.ba - eskf_mex.ba) < 1e-10, 'ba mismatch');
    assert(norm(eskf_matlab.bg - eskf_mex.bg) < 1e-10, 'bg mismatch');
    assert(norm(eskf_matlab.P(:) - eskf_mex.P(:)) < 1e-10, 'P mismatch');
    
    fprintf('Phase 1 test: PASS\n');
end
```

### 2. バッチテスト

```matlab
% run_batch_10sets(false) を実行
% 10/10 成功を確認
```

---

## 🚀 実装開始

Phase 1の実装を開始する準備が整いました。




