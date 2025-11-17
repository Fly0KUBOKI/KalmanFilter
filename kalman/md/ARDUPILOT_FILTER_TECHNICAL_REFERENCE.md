# ArduPilot Filter Library - 技術リファレンス

## Filter ライブラリ構造

### ディレクトリ構成
```
libraries/Filter/
├── LowPassFilter.h           # EMA ローパスフィルタ
├── LowPassFilter.cpp
├── LowPassFilter2p.h         # 2次 Biquad フィルタ
├── LowPassFilter2p.cpp
├── NotchFilter.h             # ノッチフィルタ
├── NotchFilter.cpp
├── AverageFilter.h           # 平均フィルタ
├── ModeFilter.h              # 中央値フィルタ
├── ModeFilter.cpp
├── DerivativeFilter.h        # 微分フィルタ
├── DerivativeFilter.cpp
├── FilterWithBuffer.h        # バッファ付きフィルタベース
├── FilterWithBuffer.cpp
├── FilterClass.h             # インターフェース
├── Butter.h                  # Butterworth係数
├── AP_Filter.h               # パラメータ管理
├── AP_Filter.cpp
├── AP_Filter_params.cpp
└── examples/
    └── LowPassFilter/
    └── LowPassFilter2p/
```

---

## 各フィルタの詳細仕様

### LowPassFilter（EMA ローパスフィルタ）

#### ヘッダファイル
```cpp
template <class T>
class DigitalLPF {
private:
    T output;
    bool initialised;
protected:
    T _apply(const T &sample, const float &alpha);
};

// 可変dt版
template <class T>
class LowPassFilter : public DigitalLPF<T> {
public:
    LowPassFilter(const float &cutoff_freq);
    void set_cutoff_frequency(const float &cutoff_freq);
    float get_cutoff_freq() const;
    T apply(const T &sample, const float &dt);
private:
    float cutoff_freq;
};

// 固定dt版
template <class T>
class LowPassFilterConstDt : public DigitalLPF<T> {
public:
    LowPassFilterConstDt(const float &sample_freq, const float &cutoff_freq);
    void set_cutoff_frequency(const float &sample_freq, const float &cutoff_freq);
    float get_cutoff_freq() const;
    T apply(const T &sample);
private:
    float cutoff_freq;
    float alpha;
};
```

#### アルゴリズム
```cpp
// ステップ1: α（アルファ）係数の計算
float rc = 1.0f / (2π * cutoff_freq);
float alpha = dt / (dt + rc);

// ステップ2: フィルタリング
output += (sample - output) * alpha;

// 数式：
// H(z) = α / (1 - (1-α)*z^-1)
// 周波数応答: -3dB @ cutoff_freq
// 遅延: ~1サンプル
```

#### 使用例
```cpp
// 例1: 固定サンプリングレート
LowPassFilter2pFloat filter(1000, 20);  // 1kHz, 20Hz cutoff
float output = filter.apply(raw_sample);

// 例2: 可変サンプリングレート
LowPassFilterFloat filter(20.0f);  // 20Hz cutoff
float dt = time_since_last_call;
float output = filter.apply(raw_sample, dt);
```

#### 性能特性
| パラメータ | 値 |
|---|---|
| カットオフ特性 | -3dB @ f_cutoff |
| 勾配 | -20dB/decade（1次） |
| 遅延 | ~1サンプル @ f_cutoff |
| リップル | なし（Butterworth） |

---

### LowPassFilter2p（Biquad 2次ローパスフィルタ）

#### ヘッダファイル
```cpp
struct biquad_params {
    float cutoff_freq;
    float sample_freq;
    float a1, a2;  // フィードバック係数
    float b0, b1, b2;  // フィードフォワード係数
};

template <class T>
class DigitalBiquadFilter {
private:
    T _delay_element_1, _delay_element_2;
    bool initialised;
public:
    T apply(const T &sample, const struct biquad_params &params);
    void reset();
    static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret);
};

template <class T>
class LowPassFilter2p {
public:
    LowPassFilter2p(float sample_freq, float cutoff_freq);
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);
    float get_cutoff_freq() const;
    float get_sample_freq() const;
    T apply(const T &sample);
    void reset(void);
    void reset(const T &value);
private:
    struct biquad_params _params;
    DigitalBiquadFilter<T> _filter;
};
```

#### 計算式
```cpp
// Butterworth 2次ローパスフィルタ

// Step 1: 正規化周波数
fr = sample_freq / cutoff_freq
ohm = tan(π / fr)

// Step 2: パラメータ計算
c = 1.0 + 2*cos(π/4)*ohm + ohm²
b0 = ohm² / c
b1 = 2*b0
b2 = b0
a1 = 2*(ohm² - 1) / c
a2 = (1 - 2*cos(π/4)*ohm + ohm²) / c

// Step 3: 応用
delay_0 = sample - delay_1*a1 - delay_2*a2
output = delay_0*b0 + delay_1*b1 + delay_2*b2
delay_2 = delay_1
delay_1 = delay_0
```

#### 使用例
```cpp
// 初期化: 1kHz サンプリング, 30Hz カットオフ
LowPassFilter2pFloat filter(1000, 30);

// 処理ループ
for (int i = 0; i < num_samples; i++) {
    float raw_sample = get_sensor_data();
    float filtered = filter.apply(raw_sample);
}

// リセット
filter.reset(initial_value);
```

#### 性能特性
| パラメータ | 値 |
|---|---|
| フィルタ型 | Butterworth 2次 |
| カットオフ特性 | -3dB @ f_cutoff |
| 勾配 | -40dB/decade（2次） |
| 遅延 | ~2サンプル @ f_cutoff |
| リップル | なし |
| Q値 | 0.707（最大平坦） |

---

### NotchFilter（ノッチフィルタ）

#### ヘッダファイル
```cpp
template <class T>
class NotchFilter {
private:
    T signal1, signal2;
    T ntchsig1, ntchsig2;
    float b0, b1, b2;
    float a1, a2;
    float _center_freq_hz;
    float _sample_freq_hz;
    float _bandwidth_hz;
    float _attenuation_dB;
    bool initialised;
    bool need_reset;
public:
    void init(float sample_freq_hz, float center_freq_hz, 
              float bandwidth_hz, float attenuation_dB);
    void init_with_A_and_Q(float sample_freq_hz, float center_freq_hz,
                          float A, float Q);
    T apply(const T &sample);
    void reset();
    static void calculate_A_and_Q(float center_freq_hz, float bandwidth_hz,
                                 float attenuation_dB, float& A, float& Q);
};
```

#### パラメータ計算
```cpp
// Step 1: A と Q の計算
A = 10^(-attenuation_dB / 40)
octaves = log2(center_freq / (center_freq - bandwidth/2)) * 2
Q = sqrt(2^octaves) / (2^octaves - 1)

// Step 2: Biquad 係数計算
omega = 2π * center_freq / sample_freq
sin_omega = sin(omega)
cos_omega = cos(omega)
alpha = sin_omega * sinh(ln(2)/2 * bandwidth * omega * sin_omega)

b0 = 1.0
b1 = -2 * cos_omega
b2 = 1.0
a0 = 1 + alpha / A
a1 = -2 * cos_omega
a2 = 1 - alpha / A
```

#### 使用例
```cpp
// 初期化: 1kHz サンプリング, 250Hz中心, 20Hz帯域, 15dB減衰
NotchFilterFloat filter;
filter.init(1000, 250, 20, 15);

// 処理
for (int i = 0; i < num_samples; i++) {
    float raw = get_gyro_data();
    float filtered = filter.apply(raw);
}

// RPM同期型（動的周波数更新）
for (int harmonic = 1; harmonic <= 3; harmonic++) {
    float harmonic_freq = rpm_freq * harmonic;
    harmonic_notch[harmonic].init(1000, harmonic_freq, 20, 15);
}
```

#### 性能特性
| パラメータ | 値 |
|---|---|
| 形状 | バンドストップ（ノッチ） |
| 中心周波数 | 可変（RPM同期可） |
| 帯域幅 | 通常 10-50 Hz |
| 減衰量 | 10-20 dB 推奨 |
| 群遅延 | 中心周波数付近で最小 |

---

### ModeFilter（中央値フィルタ）

#### 実装
```cpp
template <class T, uint8_t FILTER_SIZE>
class ModeFilter : public FilterWithBuffer<T, FILTER_SIZE> {
public:
    ModeFilter(uint8_t return_element);
    virtual T apply(T sample) override;
    virtual T get() const { return _output; }
private:
    uint8_t _return_element;
    T _output;
    void isort(T sample, bool drop_high_sample);
    bool drop_high_sample;
};
```

#### アルゴリズム
```cpp
T ModeFilter<T,FILTER_SIZE>::apply(T sample) {
    // 新しいサンプルを挿入ソートで配列に挿入
    isort(sample, drop_high_sample);
    
    // 次回は高側/低側から削除を交互に実行
    drop_high_sample = !drop_high_sample;
    
    // 中央値を返す
    if (sample_index < FILTER_SIZE) {
        return _output = samples[FILTER_SIZE/2];
    } else {
        return _output = samples[_return_element];
    }
}
```

#### 使用例
```cpp
// 5サンプル中央値フィルタ（外れ値2つまで除去）
ModeFilter<int16_t, 5> filter(2);  // 中央値を返す

for (int i = 0; i < 100; i++) {
    int16_t raw = read_sensor();
    int16_t median = filter.apply(raw);
}
```

#### 特性
| パラメータ | 値 |
|---|---|
| アルゴリズム | 挿入ソート＆交互削除 |
| 外れ値除去 | (FILTER_SIZE-1)/2 個 |
| 計算量 | O(FILTER_SIZE) per sample |
| 遅延 | FILTER_SIZE/2 サンプル |

---

### AverageFilter（平均フィルタ）

#### 実装
```cpp
template <class T, class U, uint8_t FILTER_SIZE>
class AverageFilter : public FilterWithBuffer<T, FILTER_SIZE> {
public:
    virtual T apply(T sample) override;
    virtual void reset() override;
protected:
    uint8_t _num_samples;
};

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

#### 型定義
```cpp
typedef AverageFilter<int16_t, int32_t, 5> AverageFilterInt16_Size5;
typedef AverageFilter<float, float, 10> AverageFilterFloat_Size10;
```

---

### AverageIntegralFilter（積分型平均フィルタ）

#### 実装
```cpp
template <class T, class U, uint8_t FILTER_SIZE>
class AverageIntegralFilter : public AverageFilter<T, U, FILTER_SIZE> {
public:
    virtual T apply(T sample) override;
    virtual float getf();
    virtual double getd();
protected:
    U _sum = 0;
};

T AverageIntegralFilter<T,U,FILTER_SIZE>::apply(T sample) {
    T curr = samples[sample_index];
    
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);
    this->_num_samples++;
    if (this->_num_samples > FILTER_SIZE) 
        this->_num_samples = FILTER_SIZE;
    
    _sum -= curr;
    _sum += sample;
    
    return 0;  // get()またはgetf()で取得
}

float AverageIntegralFilter<T,U,FILTER_SIZE>::getf() {
    if (this->_num_samples == 0) return 0.f;
    return (float)_sum / this->_num_samples;
}
```

#### 使用例
```cpp
// 気圧フィルタ用
AverageIntegralFilter<int32_t, int32_t, 10> pressure_filter;

for (int i = 0; i < samples; i++) {
    int32_t raw_pressure = read_baro();
    pressure_filter.apply(raw_pressure);  // 戻り値は不使用
    
    float filtered = pressure_filter.getf();  // フィルタ値を取得
}
```

---

### DerivativeFilter（微分フィルタ）

#### 実装
```cpp
template <class T, uint8_t FILTER_SIZE>
class DerivativeFilter : public FilterWithBuffer<T, FILTER_SIZE> {
public:
    void update(T sample, uint32_t timestamp);
    float slope(void);
    virtual void reset() override;
private:
    bool _new_data;
    float _last_slope;
    uint32_t _timestamps[FILTER_SIZE];
};

void DerivativeFilter<T,FILTER_SIZE>::update(T sample, uint32_t timestamp) {
    uint8_t i = sample_index;
    uint8_t i1 = (i == 0) ? FILTER_SIZE - 1 : i - 1;
    
    if (_timestamps[i1] == timestamp) {
        return;  // 新規タイムスタンプでない
    }
    
    _timestamps[i] = timestamp;
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);
    _new_data = true;
}

float DerivativeFilter<T,FILTER_SIZE>::slope() {
    // Savitzky-Golay微分フィルタ
    // タイムスタンプを考慮した非均一サンプリング対応
    
    if (!_new_data) return _last_slope;
    
    // サンプル5個の場合:
    // slope = [-2*f(-2Δt) - f(-Δt) + f(Δt) + 2*f(2Δt)] / (10*Δt)
}
```

#### 使用例
```cpp
DerivativeFilterFloat_Size5 deriv_filter;

for (int i = 0; i < 100; i++) {
    float sample = read_sensor();
    uint32_t now = micros();
    
    deriv_filter.update(sample, now);
    float rate = deriv_filter.slope();  // dSample/dt
}
```

---

## AP_InertialSensor での実装

### フィルタの統合

```cpp
class AP_InertialSensor {
private:
    // 各インスタンス（IMUが複数存在する場合）
    static const uint8_t INS_MAX_INSTANCES = 3;
    
    // ローパスフィルタ
    LowPassFilter2pVector3f _accel_filter[INS_MAX_INSTANCES];
    LowPassFilter2pVector3f _gyro_filter[INS_MAX_INSTANCES];
    
    // 振動検出用フィルタ
    LowPassFilterVector3f _accel_vibe_floor_filter[INS_VIBRATION_CHECK_INSTANCES];
    LowPassFilterVector3f _accel_vibe_filter[INS_VIBRATION_CHECK_INSTANCES];
    
    // 温度フィルタ
    LowPassFilterFloat _temp_filter;
    
    // ハーモニックノッチフィルタ
    #if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        class HarmonicNotch {
            NotchFilterVector3f filter[INS_MAX_INSTANCES];
            float center_freq_hz;
            float bandwidth_hz;
            float attenuation_dB;
        } harmonic_notches[HAL_INS_NUM_HARMONIC_NOTCH_FILTERS];
    #endif
    
    // パラメータ
    AP_Int16 _accel_filter_cutoff;
    AP_Int16 _gyro_filter_cutoff;
    
    // フィルタ適用メソッド
    void apply_accel_filters(uint8_t instance, const Vector3f &accel);
    void apply_gyro_filters(uint8_t instance, const Vector3f &gyro);
};
```

### ノッチフィルタ有効化コンパイルフラグ

```cpp
// AP_InertialSensor_config.h
#ifndef AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
#define AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED  (AP_VEHICLE_ENABLED && !AP_BAREMETAL)
#endif

#ifndef HAL_INS_NUM_HARMONIC_NOTCH_FILTERS
#define HAL_INS_NUM_HARMONIC_NOTCH_FILTERS 2
#endif
```

---

## 計算最適化テクニック

### 1. Fixed-Point 演算への変換

```cpp
// フローティング版（遅い）
float alpha = calc_lowpass_alpha_dt(dt, cutoff_freq);
output = output + (sample - output) * alpha;

// 固定小数点版（高速）
// alpha を 16bit 固定小数点で表現
int16_t alpha_fixed = (int16_t)(alpha * 65536);
int16_t delta = sample - output;
output += (delta * alpha_fixed) >> 16;
```

### 2. テーブル参照による高速化

```cpp
// 事前計算テーブル
static const float butterworth_coeffs[11][5] = {
    // [cutoff_hz][b0, b1, b2, a1, a2]
    {0.00249995f, 0.00499989f, 0.00249995f, 1.92767143f, -0.93357563f},
    // ... for 10Hz, 20Hz, 30Hz, ...
};

// 使用
void set_cutoff_frequency_fast(uint8_t index) {
    _params.b0 = butterworth_coeffs[index][0];
    _params.b1 = butterworth_coeffs[index][1];
    _params.b2 = butterworth_coeffs[index][2];
    _params.a1 = butterworth_coeffs[index][3];
    _params.a2 = butterworth_coeffs[index][4];
}
```

### 3. SIMD 活用例

```cpp
// Vector3f に対する並列フィルタリング
// GCC の自動ベクトル化により以下が高速化される:
Vector3f delay_0 = sample - delay_1 * a1 - delay_2 * a2;
Vector3f output = delay_0 * b0 + delay_1 * b1 + delay_2 * b2;

// x, y, z成分が並列に計算される
// （オプション最適化: -O3 -march=native）
```

---

## デバッグとテスト

### Filter Transfer Function テスト

```cpp
// libraries/Filter/examples/TransferFunctionCheck/

// 目的: フィルタの周波数応答を検証

class LowPassConstDtHelper : public LowPassFilterConstDtFloat {
public:
    void print_transfer_function() {
        hal.console->printf("LowPassFilterConstDtFloat\n");
        hal.console->printf("Sample rate: %.9f Hz, Cutoff: %.9f Hz\n", 
                           sample_freq, cutoff_freq);
        hal.console->printf("a: %.9f\n", alpha);
    }
};

// 周波数スイープテスト
void sweep(uint16_t num_samples, uint16_t max_freq, float sample_rate) {
    for (uint16_t freq = 1; freq < max_freq; freq++) {
        float phase = 0;
        float output_peak = 0;
        
        for (uint16_t i = 0; i < num_samples; i++) {
            float input = sinf(phase);
            float output = filter.apply(input);
            
            if (output > output_peak) {
                output_peak = output;
            }
            
            phase += 2 * M_PI * freq / sample_rate;
        }
        
        float gain_dB = 20 * log10f(output_peak);
        printf("%d Hz: %f dB\n", freq, gain_dB);
    }
}
```

### フィルタ性能検証

```cpp
// テスト項目
1. DC応答: output == input（定常状態）
2. 周波数応答: -3dB @ cutoff_freq
3. 勾配: -20dB/decade（1次）または -40dB/decade（2次）
4. 位相遅延: arctan(-freq/cutoff_freq)
5. 段階応答: 立ち上がり時間、オーバーシュート、整定時間
6. ノイズ抑圧: ホワイトノイズ入力時の減衰
```

---

## パフォーマンス指標

### メモリ使用量（バイト）

| フィルタタイプ | Vector3f | 説明 |
|---|---|---|
| EMA | 16 | 状態変数1つ |
| Biquad | 32 | 遅延素子2つ |
| Notch | 32 | 遅延素子2つ |
| Mode(5) | 92 | バッファ+状態 |
| Average(10) | 160 | バッファ+カウンタ |

### CPU コスト（Cortex-M4 @ 168MHz）

| フィルタタイプ | cycles | μs @ 168MHz |
|---|---|---|
| EMA apply | 6-10 | 0.04-0.06 |
| Biquad apply | 15-20 | 0.09-0.12 |
| Notch apply | 20-25 | 0.12-0.15 |
| Mode apply | 50-100 | 0.3-0.6 |

### リアルタイム性

```cpp
// フィルタ適用時間の最大値（タイムアウト）
// 1kHz IMU サンプリングレート
// → 1ms 以内に完了する必要あり

#define INS_MAX_FILTER_TIME_US 500  // 0.5ms マージン確保
```

---

## まとめ

### 推奨フィルタの組み合わせ

#### 標準的なドローン（ACQ, Heli）
```cpp
// IMU処理パイプライン
Raw Data 
  ↓ (ハードウェアフィルタ)
Sensor FIFO
  ↓ (ダウンサンプリング)
1kHz データ
  ↓ (ハーモニックノッチ)
ノッチフィルタ 250Hz中心
  ↓ (ローパスフィルタ)
2次Biquad 20Hz
  ↓
EKF入力
```

#### 高振動環境
```cpp
// 複数ノッチ + ゆるいローパス
Raw Data 
  ↓
ノッチ1: 150Hz
ノッチ2: 300Hz
ノッチ3: 450Hz（3次調和）
  ↓
Biquad 15Hz（広い帯域）
  ↓
EKF入力
```

#### GPS/気圧計（低周波）
```cpp
// 強い平均化
Raw Data
  ↓
平均フィルタ 10サンプル
  ↓
中央値フィルタ 5サンプル（外れ値対応）
  ↓
EKF入力
```
