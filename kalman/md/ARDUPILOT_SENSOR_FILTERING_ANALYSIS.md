# ArduPilot センサーフィルタリング実装分析

*このドキュメントはArduPilotのセンサーフィルタリング、ノイズ除去、平滑化の実装を詳細に分析したものです*

## 目次

1. [フィルタの種類](#フィルタの種類)
2. [フィルタの構成](#フィルタの構成)
3. [センサー別の実装](#センサー別の実装)
4. [特殊な処理](#特殊な処理)
5. [実装の特徴](#実装の特徴)
6. [コードスニペット](#コードスニペット)

---

## フィルタの種類

### 1. Low Pass Filter (LPF) - ローパスフィルタ

#### 1.1 単純EMA (指数移動平均) フィルタ
**用途**: 可変時間ステップでの高速フィルタリング

```cpp
// LowPassFilter: 各サンプルで dt を提供
output = filter.apply(sample, dt);

// 実装原理: α = dt / (dt + RC), RC = 1/(2π*f_cutoff)
T _apply(const T &sample, const float &alpha) {
    output += (sample - output) * alpha;
    return output;
}
```

**特徴**:
- CPU効率的（毎サンプルで1回の乗算・加算）
- 可変サンプリングレート対応
- メモリ効率的（状態変数1つのみ）

#### 1.2 Biquad Filter (2次ローパスフィルタ)
**用途**: より急峻なカットオフ特性が必要な場合

```cpp
// 構成: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
T apply(const T &sample, const struct biquad_params &params) {
    T delay_element_0 = sample - _delay_element_1 * params.a1 - _delay_element_2 * params.a2;
    T output = delay_element_0 * params.b0 + _delay_element_1 * params.b1 + _delay_element_2 * params.b2;
    
    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;
    
    return output;
}

// パラメータ計算
void compute_params(float sample_freq, float cutoff_freq) {
    float fr = sample_freq / cutoff_freq;
    float ohm = tan(π/fr);
    float c = 1.0f + 2.0f*cos(π/4.0f)*ohm + ohm*ohm;
    
    b0 = ohm*ohm/c;
    b1 = 2.0f*b0;
    b2 = b0;
    a1 = 2.0f*(ohm*ohm-1.0f)/c;
    a2 = (1.0f-2.0f*cos(π/4.0f)*ohm+ohm*ohm)/c;
}
```

**特徴**:
- -12dB/octaveのカットオフ勾配（EMAより急峻）
- 遅延は同等（2サンプル遅延）
- 2次バターワース特性

#### 1.3 Notch Filter (ノッチフィルタ)
**用途**: 特定周波数成分の除去（振動周波数など）

```cpp
// 構造: 帯域除去フィルタ (Band Stop Filter)
T apply(const T &sample) {
    T output = sample*b0 + ntchsig1*b1 + ntchsig2*b2 - signal1*a1 - signal2*a2;
    
    ntchsig2 = ntchsig1;
    ntchsig1 = sample;
    signal2 = signal1;
    signal1 = output;
    
    return output;
}

// パラメータ:
// - center_freq_hz: ノッチ中心周波数
// - bandwidth_hz: ノッチ帯域幅
// - attenuation_dB: 減衰量（10-20dB推奨）
void calculate_A_and_Q(float center_freq, float bandwidth, float attenuation_dB) {
    A = pow(10, -attenuation_dB / 40.0f);
    if (center_freq > 0.5 * bandwidth) {
        float octaves = log2(center_freq / (center_freq - bandwidth/2)) * 2;
        Q = sqrt(pow(2, octaves)) / (pow(2, octaves) - 1);
    }
}
```

**特徴**:
- モータ振動（50-400Hz帯）の除去に効果的
- 複数の調和周波数に対応可能
- 複数ノッチフィルタの直列接続可能

### 2. Harmonic Notch Filter

**用途**: RPM同期振動の除去

```cpp
// AP_InertialSensor内で実装
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    class HarmonicNotch {
        // RPM同期で中心周波数を動的に更新
        // 複数の調和周波数成分に対応
    } harmonic_notches[HAL_INS_NUM_HARMONIC_NOTCH_FILTERS];
#endif
```

### 3. Mode Filter (中央値フィルタ)

**用途**: 外れ値除去

```cpp
template <class T, uint8_t FILTER_SIZE>
T ModeFilter<T,FILTER_SIZE>::apply(T sample) {
    isort(sample, drop_high_sample);
    drop_high_sample = !drop_high_sample;
    
    if (sample_index < FILTER_SIZE) {
        return _output = samples[FILTER_SIZE/2];  // 中央値
    } else {
        return _output = samples[_return_element];
    }
}
```

**特徴**:
- 外れ値の影響を排除
- アルゴリズム: 挿入ソート＆交互削除法
- バッファサイズは奇数（推奨）

### 4. Average Filter (平均フィルタ)

**用途**: ホワイトノイズの除去

```cpp
template <class T, class U, uint8_t FILTER_SIZE>
T AverageFilter<T,U,FILTER_SIZE>::apply(T sample) {
    U result = 0;
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);
    
    _num_samples++;
    if (_num_samples > FILTER_SIZE) _num_samples = FILTER_SIZE;
    
    for (uint8_t i = 0; i < FILTER_SIZE; i++)
        result += samples[i];
    
    return (T)(result / _num_samples);
}
```

### 5. Derivative Filter (微分フィルタ)

**用途**: スロープ（変化率）の計算

```cpp
template <class T, uint8_t FILTER_SIZE>
void DerivativeFilter<T,FILTER_SIZE>::update(T sample, uint32_t timestamp) {
    _timestamps[sample_index] = timestamp;
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);
    _new_data = true;
}

float slope() {
    // Savitzky-Golay微分フィルタを使用
    // タイムスタンプを考慮した非均一サンプリング対応
}
```

---

## フィルタの構成

### フィルタ階層構造

```
Filter (基底インターフェース)
├── DigitalLPF (EMAベースフィルタ)
│   ├── LowPassFilterConstDt (固定dt)
│   └── LowPassFilter (可変dt)
├── DigitalBiquadFilter
│   └── LowPassFilter2p (2次ローパス)
├── FilterWithBuffer
│   ├── AverageFilter
│   │   └── AverageIntegralFilter
│   ├── ModeFilter (中央値フィルタ)
│   └── DerivativeFilter
├── NotchFilter (ノッチフィルタ)
└── AP_Filter (パラメータ統合インターフェース)
```

### 共通インターフェース

```cpp
template <class T>
class Filter {
public:
    virtual T apply(T sample) = 0;
    virtual void reset() = 0;
};
```

### フィルタ初期化パラメータ

#### EMAフィルタ
```cpp
// 方法1: 固定dt
filter.set_cutoff_frequency(sample_freq, cutoff_freq_hz);
output = filter.apply(sample);

// 方法2: 可変dt
filter.set_cutoff_frequency(cutoff_freq_hz);
output = filter.apply(sample, dt);
```

#### Biquadフィルタ
```cpp
// 初期化
filter.set_cutoff_frequency(sample_freq_hz, cutoff_freq_hz);

// 自動的にパラメータ計算
// b0, b1, b2, a1, a2 が設定される
```

#### ノッチフィルタ
```cpp
// 初期化
filter.init(
    sample_freq_hz,      // 1000Hz等
    center_freq_hz,      // 250Hz等
    bandwidth_hz,        // 20Hz等
    attenuation_dB       // 10-20dB
);
```

### フィルタ選択メカニズム

```cpp
enum class FilterType : uint8_t {
    FILTER_NONE            = 0,
    FILTER_NOTCH           = 1,
};

class AP_Filter {
    virtual bool setup_notch_filter(NotchFilterFloat& filter, 
                                   float sample_rate) { 
        return false; 
    }
};

struct AP_NotchFilter_params : public AP_Filter {
    AP_Float _center_freq_hz;
    AP_Float _quality;
    AP_Float _attenuation_dB;
};
```

---

## センサー別の実装

### 1. IMU（加速度計・ジャイロ）

#### 加速度計フィルタ構成
```cpp
// libraries/AP_InertialSensor/AP_InertialSensor.h

class AP_InertialSensor {
private:
    // ローパスフィルタ（ソフトウェア）
    LowPassFilter2pVector3f _accel_filter[INS_MAX_INSTANCES];
    LowPassFilter2pVector3f _gyro_filter[INS_MAX_INSTANCES];
    
    // 振動検出用フィルタ
    LowPassFilterVector3f _accel_vibe_floor_filter[INS_VIBRATION_CHECK_INSTANCES];
    LowPassFilterVector3f _accel_vibe_filter[INS_VIBRATION_CHECK_INSTANCES];
    
    // ハーモニックノッチフィルタ
    #if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        class HarmonicNotch {
            // 複数の調和周波数対応
        } harmonic_notches[HAL_INS_NUM_HARMONIC_NOTCH_FILTERS];
    #endif
    
    // 温度フィルタ
    LowPassFilterFloat _temp_filter;
    
    // 設定パラメータ
    AP_Int16 _accel_filter_cutoff;  // Hz
    AP_Int16 _gyro_filter_cutoff;   // Hz
};
```

#### デフォルトカットオフ周波数
```cpp
// ArduCopter/Quadcopter
#define DEFAULT_GYRO_FILTER  20      // 20 Hz
#define DEFAULT_ACCEL_FILTER 20      // 20 Hz

// Rover
#define DEFAULT_GYRO_FILTER  4       // 4 Hz
#define DEFAULT_ACCEL_FILTER 10      // 10 Hz

// Plane
#define DEFAULT_GYRO_FILTER  20      // 20 Hz
#define DEFAULT_ACCEL_FILTER 20      // 20 Hz
```

#### 加速度計フィルタ処理フロー
```cpp
void AP_InertialSensor_Backend::apply_accel_filters(
    const uint8_t instance, 
    const Vector3f &accel) {
    
    Vector3f accel_filtered = accel;
    
    // 1. ハーモニックノッチフィルタ（オプション）
    #if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        if (!inactive) {
            accel_filtered = notch.filter[instance].apply(accel_filtered);
        }
    #endif
    
    // 2. ソフトウェアローパスフィルタ
    accel_filtered = _imu._accel_filter[instance].apply(accel_filtered);
    
    // 3. エラーチェック
    if (accel_filtered.is_nan() || accel_filtered.is_inf()) {
        _imu._accel_filter[instance].reset();
        return;  // 前回値を保持
    }
}
```

#### ジャイロフィルタ処理フロー
```cpp
void AP_InertialSensor_Backend::apply_gyro_filters(
    const uint8_t instance, 
    const Vector3f &gyro) {
    
    Vector3f gyro_filtered = gyro;
    
    // 1. ハーモニックノッチフィルタ
    #if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        if (!inactive) {
            gyro_filtered = notch.filter[instance].apply(gyro_filtered);
        }
    #endif
    
    // 2. ローパスフィルタ（最後に適用して、ノッチノイズを減衰）
    gyro_filtered = _imu._gyro_filter[instance].apply(gyro_filtered);
}
```

### 2. 高速サンプリング時のダウンサンプリング

#### Invensenseセンサー（8-9kHz）
```cpp
// 1次ローパスフィルタ: 188Hz → 平均化8サンプル → 1kHz
bool AP_InertialSensor_Invensense::_accumulate_sensor_rate_sampling(
    uint8_t *samples, 
    uint8_t n_samples) {
    
    // 1. 188Hz 1次ローパス適用
    Vector3f accel_lp_188hz = lpf_188hz.apply(raw_accel);
    
    // 2. 8サンプル蓄積
    _accum.accel += accel_lp_188hz;
    _accum.accel_count++;
    
    // 3. 8サンプルごとに平均化＆出力
    if (_accum.accel_count % 8 == 0) {
        accel_output = _accum.accel / 8;  // 1kHz
        _notify_new_accel_raw_sample(accel_instance, accel_output);
        _accum.accel.zero();
    }
}
```

**効果**:
- 非常に良いエイリアシング除去（Nyquist上のノイズ抑制）
- メモリ効率的（FFT不要）
- リアルタイム処理可能

### 3. 磁気計フィルタ

```cpp
// libraries/AP_Compass/AP_Compass_*.cpp

Vector3f noise = rand_vec3f() * _sitl->mag_noise;
Vector3f new_mag_data = state.bodyMagField + noise;

// 遅延バッファ＆移動平均フィルタ
update_delay_buffer(new_mag_data);
filtered_mag = moving_average(delay_buffer, window_size);
```

### 4. 気圧計フィルタ

```cpp
// libraries/AP_Baro/AP_Baro_BMP085.cpp

class AP_Baro_BMP085 : public AP_Baro_Backend {
private:
    AverageIntegralFilter<int32_t, int32_t, 10> _pressure_filter;
    
    void _calculate() {
        // 気圧計算
        int32_t raw_pressure = calculate_raw_pressure();
        
        // 積分型平均フィルタ
        _pressure_filter.apply(raw_pressure);
        
        // 結果取得
        int32_t filtered_pressure = _pressure_filter.getf();
    }
};
```

### 5. GPS フィルタ

```cpp
// NavEKF2/NavEKF3によるカルマンフィルタ統合

// GPS測定信頼度設定
AP_GROUPINFO("GPS_M_NSE", GPS_POS_M_NSE, NavEKF3, _gpsNoise, GPS_POS_M_NSE_DEFAULT);

// EKF内での使用
if (gps.fix_type >= AP_GPS_FixType::FIX_3D) {
    Vector3f gps_position = extract_gps_position();
    // EKFで融合
    update_position(gps_position, gps_confidence);
}
```

---

## 特殊な処理

### 1. 外れ値検出（Outlier Detection）

#### 方法1: Hampel フィルタ
```cpp
// センサーに応じた実装
// 高加速度サンプルの検出＆除外

bool is_outlier(Vector3f sample, Vector3f recent_mean, float threshold) {
    float deviation = (sample - recent_mean).length();
    return deviation > threshold;
}
```

#### 方法2: クリッピング検出
```cpp
// Invensenseセンサー
const int32_t unscaled_clip_limit = _clip_limit / _accel_scale;

// FIFOバッファからデータ読出し時に確認
if (accel_raw.x >= unscaled_clip_limit || 
    accel_raw.y >= unscaled_clip_limit) {
    // クリップフラグ設定
    clipped = true;
    _accel_clip_count[accel_instance]++;
}
```

#### 方法3: ビイビリティ判定
```cpp
// EKF内での創発性判定
void check_filter_divergence() {
    if (innovation_magnitude > innovation_gate_width * sqrt(innovation_variance)) {
        // 外れ値として処理
        // フィルタを再初期化するか、更新をスキップ
    }
}
```

### 2. 動的パラメータ調整

#### Harmonic Notch の周波数追従
```cpp
// RPM から ノッチ周波数を計算
float rpm = get_motor_rpm();
float notch_freq_hz = rpm * motor_poles / 60.0f;

// 複数調和周波数対応
for (int harmonic = 1; harmonic <= num_harmonics; harmonic++) {
    float harmonic_freq = notch_freq_hz * harmonic;
    harmonic_notch[harmonic].set_center_frequency(harmonic_freq);
}
```

#### フィルタカットオフの適応制御
```cpp
// 振動レベルに応じた適応フィルタ調整
float vibration_level = estimate_vibration();

if (vibration_level > HIGH_VIBRATION_THRESHOLD) {
    // 振動が多い → カットオフを低くする
    filter.set_cutoff_frequency(1000, 10);  // 10Hz
} else if (vibration_level < LOW_VIBRATION_THRESHOLD) {
    // 振動が少ない → カットオフを高くする
    filter.set_cutoff_frequency(1000, 25);  // 25Hz
}
```

### 3. 振動検出と監視

```cpp
class AP_InertialSensor {
    struct PeakHoldState {
        Vector3f accel_peak;
        uint32_t peak_detect_time_us;
    } _peak_hold_state;
    
    // 振動チェック用フィルタ
    LowPassFilterVector3f _accel_vibe_floor_filter;    // 5Hz
    LowPassFilterVector3f _accel_vibe_filter;           // 2Hz
    
    void check_vibration() {
        // FFT分析（有効な場合）
        if (has_fft_notch()) {
            perform_fft_analysis();
        }
        
        // ピークホールド検出
        update_peak_hold();
        
        // 動的なフィルタ調整
        adapt_filter_settings();
    }
};
```

### 4. FIFO破損検出

```cpp
// Invensenseセンサー
bool AP_InertialSensor_Invensense::_accumulate(uint8_t *samples, uint8_t n_samples) {
    for (uint8_t i = 0; i < n_samples; i++) {
        // 温度から FIFO 破損を検出
        int16_t t2 = extract_temperature(samples);
        
        // 温度の急激な変化は FIFO 破損の兆候
        if (abs(t2 - _raw_temp) > TEMP_SPIKE_THRESHOLD) {
            debug("temp reset IMU[%u] %d %d", accel_instance, _raw_temp, t2);
            _fifo_reset(true);
            return false;
        }
        _raw_temp = t2;
    }
    return true;
}
```

---

## 実装の特徴

### 1. メモリ効率

#### テンプレート実装
```cpp
// 異なるデータ型に対応（同じコード）
typedef LowPassFilterConstDt<float>    LowPassFilterConstDtFloat;
typedef LowPassFilterConstDt<Vector2f> LowPassFilterConstDtVector2f;
typedef LowPassFilterConstDt<Vector3f> LowPassFilterConstDtVector3f;

// コンパイル時型チェック＆最適化
template class LowPassFilterConstDt<float>;
template class LowPassFilterConstDt<Vector3f>;
```

#### 状態変数の最小化
| フィルタタイプ | 必要メモリ | 遅延 |
|---|---|---|
| EMA（単一） | 1×sizeof(T) | 1サンプル |
| Biquad | 2×sizeof(T) | 2サンプル |
| Mode（5サンプル） | 5×sizeof(T) | 2.5サンプル |
| Average（10サンプル） | 10×sizeof(T) | 5サンプル |

### 2. 計算量の最適化

#### 低遅延アルゴリズム
```cpp
// EMAフィルタ: 2演算（乗算＋加算）
output += (sample - output) * alpha;  // ~2 CPU cycles

// Biquad: 5演算（乗算2回、加算3回）
output = delay_element_0 * b0 + 
         _delay_element_1 * b1 + 
         _delay_element_2 * b2 - 
         signal1 * a1 - 
         signal2 * a2;  // ~10-15 CPU cycles
```

#### オンボード前処理
```cpp
// センサー内部フィルタを活用（ハードウェア）
// → ソフトウェアフィルタ負荷を軽減
```

### 3. マルチスレッド対応

#### 同期機構
```cpp
class AP_InertialSensor {
private:
    // スレッドセーフなバッファ
    struct {
        WITH_SEMAPHORE(sem);
        Vector3f _accel_filtered;
        Vector3f _gyro_filtered;
    } state;
};

// 使用例
Vector3f get_accel() {
    WITH_SEMAPHORE(state.sem);
    return state._accel_filtered;
}
```

#### FIFOバッファ管理
```cpp
// Sensor読み込みスレッド
while (!stop_reading) {
    read_fifo_samples();
    apply_filters();
    notify_update();
    delay(1000/sample_rate_hz);  // ~1ms
}
```

### 4. Real-time 保証

#### 周期的フィルタ更新
```cpp
// AP_InertialSensor: 1kHz or 8kHz
_dev->register_periodic_callback(
    1250,  // microseconds (800Hz)
    FUNCTOR_BIND_MEMBER(
        &AP_InertialSensor_L3G4200D::_accumulate_gyro, void
    )
);
```

#### タイムスタンプ追跡
```cpp
// Derivative Filter
void DerivativeFilter<T,FILTER_SIZE>::update(T sample, uint32_t timestamp) {
    uint8_t i = sample_index;
    _timestamps[i] = timestamp;  // 非均一サンプリング対応
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);
}
```

---

## コードスニペット

### 実装例1: IMUフィルタスタック

```cpp
// complete_imu_filtering.cpp
class IMUSensorProcessing {
private:
    // フィルタスタック
    LowPassFilter2pVector3f accel_lpf;      // 20Hz
    LowPassFilter2pVector3f gyro_lpf;       // 20Hz
    NotchFilterVector3f gyro_notch;         // 250Hz中心
    ModeFilter<Vector3f, 5> outlier_filter; // 外れ値除去
    
public:
    void initialize(float sample_rate_hz, float accel_cutoff, float gyro_cutoff) {
        accel_lpf.set_cutoff_frequency(sample_rate_hz, accel_cutoff);
        gyro_lpf.set_cutoff_frequency(sample_rate_hz, gyro_cutoff);
        
        // ノッチ: 250Hz中心、20Hz帯域、15dB減衰
        gyro_notch.init(sample_rate_hz, 250.0f, 20.0f, 15.0f);
    }
    
    Vector3f process_accel(Vector3f raw_accel) {
        // 1. ローパスフィルタ
        Vector3f filtered = accel_lpf.apply(raw_accel);
        
        // 2. エラーチェック
        if (filtered.is_nan()) {
            accel_lpf.reset();
            return last_valid_accel;
        }
        
        return filtered;
    }
    
    Vector3f process_gyro(Vector3f raw_gyro) {
        // 1. ノッチフィルタ（振動除去）
        Vector3f filtered = gyro_notch.apply(raw_gyro);
        
        // 2. ローパスフィルタ
        filtered = gyro_lpf.apply(filtered);
        
        return filtered;
    }
};
```

### 実装例2: 適応的ノッチフィルタ

```cpp
// adaptive_notch_filter.cpp
class AdaptiveNotchFilter {
private:
    NotchFilterVector3f harmonic_notches[3];  // 基本周波数と2次、3次
    float target_freq_hz;
    float update_rate_hz;
    
public:
    void set_target_frequency(float rpm_value, int motor_poles) {
        // RPMから基本周波数を計算
        float base_freq = (rpm_value / 60.0f) * motor_poles;
        
        if (abs(base_freq - target_freq_hz) < 5.0f) {
            return;  // 更新が不要な場合はスキップ
        }
        
        target_freq_hz = base_freq;
        
        // 調和周波数を更新
        for (int h = 0; h < 3; h++) {
            float harmonic_freq = base_freq * (h + 1);
            
            // Nyquist制限チェック
            if (harmonic_freq < sample_rate_hz * 0.4f) {
                harmonic_notches[h].init(
                    sample_rate_hz,
                    harmonic_freq,
                    20.0f,        // 帯域幅
                    15.0f         // 減衰
                );
            }
        }
    }
    
    Vector3f apply(Vector3f sample) {
        Vector3f output = sample;
        
        // 全ノッチフィルタを直列に適用
        for (int h = 0; h < 3; h++) {
            output = harmonic_notches[h].apply(output);
        }
        
        return output;
    }
};
```

### 実装例3: FFTベースの振動解析

```cpp
// fft_vibration_analysis.cpp
class FFTVibrationAnalyzer {
private:
    FloatBuffer gyro_window;  // FFT入力バッファ
    bool fft_enabled;
    
public:
    void update_fft_window(Vector3f gyro_sample) {
        // ジャイロデータをFFT用バッファに蓄積
        if (gyro_window.space() > 0) {
            gyro_window.add(gyro_sample.length());
        } else {
            // バッファ満杯 → FFT実行
            perform_fft_analysis();
            gyro_window.reset();
        }
    }
    
    void perform_fft_analysis() {
        // FFT計算（外部ライブラリ）
        fft_results = compute_fft(gyro_window.data(), gyro_window.size());
        
        // 支配周波数を検出
        float peak_freq = find_peak_frequency(fft_results);
        
        // ノッチフィルタを動的に調整
        if (peak_freq > 50.0f && peak_freq < 500.0f) {
            notch_filter.init(sample_rate_hz, peak_freq, 25.0f, 20.0f);
        }
    }
};
```

### 実装例4: マルチセンサー統合フィルタ

```cpp
// multi_sensor_fusion.cpp
class MultiSensorFilter {
private:
    // IMU フィルタ
    LowPassFilter2pVector3f imu_accel_lpf;
    LowPassFilter2pVector3f imu_gyro_lpf;
    
    // 気圧計フィルタ
    AverageIntegralFilter<int32_t, int32_t, 10> baro_pressure_filter;
    
    // 磁気計フィルタ
    LowPassFilterVector3f mag_lpf;
    
public:
    struct SensorOutput {
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        float pressure;
        float temperature;
    };
    
    SensorOutput process_all_sensors(
        Vector3f raw_accel,
        Vector3f raw_gyro,
        Vector3f raw_mag,
        int32_t raw_pressure) {
        
        SensorOutput result;
        
        // 加速度計処理
        result.accel = imu_accel_lpf.apply(raw_accel);
        
        // ジャイロ処理
        result.gyro = imu_gyro_lpf.apply(raw_gyro);
        
        // 磁気計処理
        result.mag = mag_lpf.apply(raw_mag);
        
        // 気圧計処理
        baro_pressure_filter.apply(raw_pressure);
        result.pressure = baro_pressure_filter.getf();
        
        // バリデーション
        if (result.accel.is_nan() || result.gyro.is_nan()) {
            // エラーリカバリー
            reset_all_filters();
        }
        
        return result;
    }
};
```

---

## まとめと推奨事項

### フィルタ選択ガイドライン

| 用途 | 推奨フィルタ | カットオフ | 特徴 |
|---|---|---|---|
| IMU ローパス | Biquad 2次 | 20-25 Hz | バランスの取れた遅延＆減衰 |
| 高振動対応 | ノッチ＋Biquad | 中心250Hz | 特定周波数除去 |
| 高速サンプリング | EMA＆ダウンサンプリング | N/A | メモリ効率 |
| 外れ値対応 | Mode フィルタ | 5サンプル | ロバスト性 |
| 気圧計 | 積分型平均 | 10サンプル | スムーズな高度推定 |
| GPS | EKF（オンボード） | 適応的 | 動的信頼度 |

### 実装のベストプラクティス

1. **フィルタチェーンの順序**
   - ノッチ → ローパス → 統計的処理

2. **エラーハンドリング**
   - NaN/Inf チェック
   - FIFO 破損検出
   - 自動リセット機能

3. **リアルタイム性**
   - 周期的タイムアウト設定
   - タイムスタンプ追跡
   - バッファオーバーフロー防止

4. **テスト方法**
   - FFT分析（周波数応答確認）
   - ステップ応答テスト
   - ノイズロバストネステスト

---

**参考資料**：
- ArduPilot GitHub: https://github.com/ArduPilot/ardupilot
- Filter Library: libraries/Filter/
- InertialSensor Library: libraries/AP_InertialSensor/
- Compass Library: libraries/AP_Compass/
- Baro Library: libraries/AP_Baro/
