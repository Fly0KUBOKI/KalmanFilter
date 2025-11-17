# ArduPilot フィルタ実装ガイド

## 目的別フィルタ選択と実装パターン

---

## 1. センサーフィルタリングの基本フロー

### 一般的なセンサー処理パイプライン

```
Raw Sensor Value (from SPI/I2C)
    ↓
┌─────────────────────────────┐
│ 1. Range Check              │ ← 物理的有効範囲確認
│    (e.g., -500 to +500 m/s²)│
└─────────────────────────────┘
    ↓
┌─────────────────────────────┐
│ 2. Hardware Filtering       │ ← センサー内フィルタ（BW制限）
│    (if available)           │
└─────────────────────────────┘
    ↓
┌─────────────────────────────┐
│ 3. Outlier Detection        │ ← 外れ値検出＆除去
│    (Mode/Median Filter)     │
└─────────────────────────────┘
    ↓
┌─────────────────────────────┐
│ 4. Harmonic Notch           │ ← 振動周波数抑圧
│    (if RPM data available)  │
└─────────────────────────────┘
    ↓
┌─────────────────────────────┐
│ 5. Software LPF             │ ← 最終ローパスフィルタ
│    (2nd order Biquad)       │
└─────────────────────────────┘
    ↓
Filtered Output to EKF/Controller
```

---

## 2. IMU フィルタリング実装例

### 例: Quadcopter 加速度計フィルタ

```cpp
// File: sensor_processing.cpp

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <Filter/LowPassFilter2p.h>

class IMUSensorProcessor {
private:
    // フィルタ インスタンス
    LowPassFilter2pVector3f accel_filter;
    LowPassFilter2pVector3f gyro_filter;
    
    // パラメータ
    const float SAMPLE_RATE_HZ = 1000.0f;
    const float ACCEL_CUTOFF_HZ = 20.0f;
    const float GYRO_CUTOFF_HZ = 20.0f;
    
    // 統計
    struct {
        uint32_t process_count;
        uint32_t error_count;
        float max_accel_change;
    } stats;
    
public:
    IMUSensorProcessor() : process_count(0), error_count(0) {
        initialize_filters();
    }
    
    void initialize_filters() {
        // 加速度計フィルタ初期化
        accel_filter.set_cutoff_frequency(SAMPLE_RATE_HZ, ACCEL_CUTOFF_HZ);
        
        // ジャイロ フィルタ初期化
        gyro_filter.set_cutoff_frequency(SAMPLE_RATE_HZ, GYRO_CUTOFF_HZ);
    }
    
    void process_accel(Vector3f raw_accel, Vector3f &filtered_accel) {
        // ステップ 1: 物理的有効範囲確認
        if (is_out_of_range(raw_accel)) {
            // 前回値を使用（エラーカウント）
            stats.error_count++;
            filtered_accel = last_filtered_accel;
            return;
        }
        
        // ステップ 2: フィルタ適用
        filtered_accel = accel_filter.apply(raw_accel);
        
        // ステップ 3: NaN/Inf チェック
        if (filtered_accel.is_nan() || filtered_accel.is_inf()) {
            stats.error_count++;
            accel_filter.reset();  // 自動リセット
            filtered_accel = raw_accel;  // Rawデータにフォールバック
            return;
        }
        
        // ステップ 4: 統計更新
        stats.max_accel_change = MAX(
            stats.max_accel_change,
            (filtered_accel - last_filtered_accel).length()
        );
        
        last_filtered_accel = filtered_accel;
        stats.process_count++;
    }
    
    void process_gyro(Vector3f raw_gyro, Vector3f &filtered_gyro) {
        // ステップ 1: レンジチェック
        if (raw_gyro.length() > 500.0f) {  // rad/s
            stats.error_count++;
            filtered_gyro = last_filtered_gyro;
            return;
        }
        
        // ステップ 2: フィルタ適用
        filtered_gyro = gyro_filter.apply(raw_gyro);
        
        // ステップ 3: エラーチェック
        if (filtered_gyro.is_nan() || filtered_gyro.is_inf()) {
            stats.error_count++;
            gyro_filter.reset();
            filtered_gyro = raw_gyro;
            return;
        }
        
        last_filtered_gyro = filtered_gyro;
        stats.process_count++;
    }
    
    bool is_out_of_range(Vector3f sample) {
        // 加速度: -60 to +60 m/s²
        return sample.length() > 70.0f;
    }
    
private:
    Vector3f last_filtered_accel;
    Vector3f last_filtered_gyro;
};
```

### 使用例

```cpp
void main_loop() {
    IMUSensorProcessor processor;
    
    while (true) {
        // センサー読み込み
        Vector3f raw_accel = get_raw_accel_from_hardware();
        Vector3f raw_gyro = get_raw_gyro_from_hardware();
        
        // フィルタ処理
        Vector3f filtered_accel;
        Vector3f filtered_gyro;
        
        processor.process_accel(raw_accel, filtered_accel);
        processor.process_gyro(raw_gyro, filtered_gyro);
        
        // EKFへ供給
        ekf.set_accel(filtered_accel);
        ekf.set_gyro(filtered_gyro);
        
        // 制御
        update_controller(filtered_accel, filtered_gyro);
    }
}
```

---

## 3. ノッチフィルタの動的周波数追従

### RPM同期ノッチフィルタ

```cpp
// File: adaptive_harmonic_notch.cpp

#include <Filter/NotchFilter.h>

class AdaptiveHarmonicNotchFilter {
private:
    static const uint8_t NUM_HARMONICS = 3;
    NotchFilterVector3f harmonic_notches[NUM_HARMONICS];
    
    float current_target_freq_hz;
    float last_update_time_ms;
    const float UPDATE_PERIOD_MS = 100.0f;  // 10Hz 更新レート
    
    const uint8_t MOTOR_POLES = 14;  // モーター極数
    const float SAMPLE_RATE_HZ = 1000.0f;
    const float NOTCH_BANDWIDTH_HZ = 20.0f;
    const float NOTCH_ATTENUATION_DB = 15.0f;
    
public:
    AdaptiveHarmonicNotchFilter() 
        : current_target_freq_hz(0), last_update_time_ms(0) {
        // 初期化は update() で行う
    }
    
    void update(float rpm_value, float current_time_ms) {
        // 更新頻度を制限
        if (current_time_ms - last_update_time_ms < UPDATE_PERIOD_MS) {
            return;
        }
        last_update_time_ms = current_time_ms;
        
        // ステップ 1: RPM から基本周波数を計算
        float base_freq_hz = calculate_base_frequency(rpm_value);
        
        // ステップ 2: 周波数が大きく変わった場合のみ更新
        if (abs(base_freq_hz - current_target_freq_hz) < 2.0f) {
            return;  // ヒステリシス: 2Hz 未満は更新しない
        }
        
        current_target_freq_hz = base_freq_hz;
        
        // ステップ 3: 調和周波数を設定
        for (uint8_t h = 0; h < NUM_HARMONICS; h++) {
            float harmonic_freq = base_freq_hz * (h + 1);
            
            // Nyquist制限チェック（サンプリング周波数の 40% 以下）
            if (harmonic_freq > SAMPLE_RATE_HZ * 0.4f) {
                continue;  // この調和周波数はスキップ
            }
            
            // ノッチフィルタ初期化
            harmonic_notches[h].init(
                SAMPLE_RATE_HZ,
                harmonic_freq,
                NOTCH_BANDWIDTH_HZ,
                NOTCH_ATTENUATION_DB
            );
            
            #if AP_DEBUG_ENABLED
            gcs().send_text(MAV_SEVERITY_DEBUG,
                "Notch H%d: %f Hz", h+1, harmonic_freq);
            #endif
        }
    }
    
    Vector3f apply(Vector3f gyro_sample) {
        Vector3f output = gyro_sample;
        
        // 全ノッチフィルタを直列に適用
        for (uint8_t h = 0; h < NUM_HARMONICS; h++) {
            output = harmonic_notches[h].apply(output);
        }
        
        return output;
    }
    
    float calculate_base_frequency(float rpm_value) {
        // RPM → Hz 変換: Hz = (RPM / 60) * num_poles
        return (rpm_value / 60.0f) * MOTOR_POLES;
    }
    
    bool is_active() const {
        return current_target_freq_hz > 10.0f;  // 10Hz 以上で有効
    }
};
```

### メイン処理での使用

```cpp
void gyro_processing_loop() {
    AdaptiveHarmonicNotchFilter notch_filter;
    LowPassFilter2pVector3f final_lpf(1000, 20);
    
    while (is_running) {
        // RPM 更新
        float current_rpm = read_motor_rpm();
        uint32_t now = AP_HAL::millis();
        
        // ノッチフィルタを更新
        notch_filter.update(current_rpm, now);
        
        // センサー読み込み
        Vector3f raw_gyro = read_gyro();
        
        // ステップ 1: ノッチフィルタ適用
        Vector3f after_notch = notch_filter.apply(raw_gyro);
        
        // ステップ 2: 最終ローパスフィルタ適用
        Vector3f filtered_gyro = final_lpf.apply(after_notch);
        
        // 結果を使用
        ekf.set_gyro(filtered_gyro);
    }
}
```

---

## 4. 外れ値除去フィルタの実装

### Mode Filter 組み込み例

```cpp
// File: outlier_rejection.cpp

class OutlierRejectionFilter {
private:
    ModeFilter<int16_t, 5> mode_filter;
    
    // 統計情報
    struct {
        uint32_t outlier_count;
        int16_t last_valid_value;
    } stats;
    
public:
    OutlierRejectionFilter() : stats{0, 0} {}
    
    int16_t apply(int16_t raw_sample) {
        // ステップ 1: Mode フィルタ（中央値）適用
        int16_t median_value = mode_filter.apply(raw_sample);
        
        // ステップ 2: 外れ値チェック
        //   中央値と大きく異なる場合は外れ値
        int16_t deviation = abs(raw_sample - median_value);
        const int16_t OUTLIER_THRESHOLD = 100;  // センサーに応じて調整
        
        if (deviation > OUTLIER_THRESHOLD && stats.last_valid_value != 0) {
            stats.outlier_count++;
            
            // 前回の有効値を使用
            return stats.last_valid_value;
        }
        
        // ステップ 3: 有効な値を保存
        stats.last_valid_value = median_value;
        return median_value;
    }
    
    void reset() {
        mode_filter.reset();
        stats.outlier_count = 0;
    }
    
    uint32_t get_outlier_count() const {
        return stats.outlier_count;
    }
};
```

### GPS 外れ値除去

```cpp
void process_gps_position(Vector2f raw_position) {
    static OutlierRejectionFilter gps_filter;
    
    // GPS位置の急激な変化をチェック
    static Vector2f last_valid_position;
    float position_change = (raw_position - last_valid_position).length();
    
    const float MAX_POSITION_CHANGE_M = 50.0f;  // 1秒で50m以上は外れ値
    
    if (position_change > MAX_POSITION_CHANGE_M) {
        // 外れ値として棄却
        gcs().send_text(MAV_SEVERITY_WARNING,
            "GPS outlier: %f m change", position_change);
        return;
    }
    
    last_valid_position = raw_position;
    
    // EKFに供給
    ekf.set_gps_position(raw_position);
}
```

---

## 5. 気圧計フィルタの実装

### Barometer Filtering Pattern

```cpp
// File: baro_filtering.cpp

class BarometerFilter {
private:
    // 積分型平均フィルタ（気圧用）
    AverageIntegralFilter<int32_t, int32_t, 10> pressure_filter;
    
    // 中央値フィルタ（外れ値対応）
    ModeFilter<int32_t, 5> mode_filter;
    
    // 統計
    struct {
        float altitude_m;
        float climb_rate_ms;
        uint32_t sample_count;
    } state;
    
    const float SAMPLE_RATE_HZ = 50.0f;  // 気圧計は通常低速
    
public:
    void process_raw_pressure(int32_t raw_pressure_pa) {
        // ステップ 1: レンジチェック
        if (raw_pressure_pa < 30000 || raw_pressure_pa > 110000) {
            // 物理的に不可能な値
            return;
        }
        
        // ステップ 2: 中央値フィルタで外れ値除去
        int32_t median_pressure = mode_filter.apply(raw_pressure_pa);
        
        // ステップ 3: 積分型平均フィルタ
        pressure_filter.apply(median_pressure);
        
        // ステップ 4: フィルタ値を取得
        float filtered_pressure = pressure_filter.getf();
        
        // ステップ 5: 高度計算
        update_altitude(filtered_pressure);
        
        state.sample_count++;
    }
    
    void update_altitude(float pressure_pa) {
        // ISA大気モデルで高度を計算
        // h = 44330 * [1 - (P/P0)^(1/5.255)]
        
        static float last_altitude = 0;
        
        float P0 = 101325.0f;  // 海面気圧 (Pa)
        float altitude = 44330.0f * (1.0f - powf(pressure_pa / P0, 1.0f / 5.255f));
        
        // クライムレート計算
        state.climb_rate_ms = (altitude - last_altitude) * SAMPLE_RATE_HZ;
        state.altitude_m = altitude;
        last_altitude = altitude;
        
        // EKF更新
        ekf.set_altitude(altitude);
        ekf.set_climb_rate(state.climb_rate_ms);
    }
    
    float get_altitude() const {
        return state.altitude_m;
    }
    
    float get_climb_rate() const {
        return state.climb_rate_ms;
    }
};
```

---

## 6. 温度補正付きフィルタ

```cpp
// File: temperature_compensated_filtering.cpp

class TemperatureCompensatedIMU {
private:
    LowPassFilter2pVector3f accel_filter;
    LowPassFilter2pVector3f gyro_filter;
    LowPassFilterFloat temp_filter;
    
    // 温度テーブル
    struct TemperatureCalibration {
        float temperature_c;
        Vector3f accel_bias;
        Vector3f gyro_bias;
        float scale_factor;
    } cal_points[5];  // -20, 0, 25, 50, 75℃
    
public:
    void load_calibration_data() {
        // キャリブレーションデータをロード
        // 通常は EEPROM または NVM から
        cal_points[0] = {-20.0f, {0.05f, -0.08f, 0.02f}, ...};
        cal_points[1] = { 0.0f, {0.01f, -0.02f, 0.00f}, ...};
        cal_points[2] = {25.0f, {0.00f,  0.00f, 0.00f}, ...};
        cal_points[3] = {50.0f, {-0.02f, 0.02f, 0.01f}, ...};
        cal_points[4] = {75.0f, {-0.05f, 0.05f, 0.03f}, ...};
    }
    
    Vector3f get_accel_with_temperature_compensation(
        Vector3f raw_accel, 
        float temperature_c) {
        
        // ステップ 1: 温度フィルタ（瞬間値ノイズ対応）
        float filtered_temp = temp_filter.apply(temperature_c);
        
        // ステップ 2: 温度に応じたバイアス補正
        Vector3f temp_bias = interpolate_calibration(filtered_temp);
        
        // ステップ 3: バイアス除去
        Vector3f compensated = raw_accel - temp_bias;
        
        // ステップ 4: ローパスフィルタ
        Vector3f filtered_accel = accel_filter.apply(compensated);
        
        return filtered_accel;
    }
    
private:
    Vector3f interpolate_calibration(float temperature_c) {
        // 線形補間で該当温度のバイアスを計算
        // ...
    }
};
```

---

## 7. フィルタパラメータの動的最適化

```cpp
// File: adaptive_filter_tuning.cpp

class AdaptiveFilterTuning {
private:
    float current_cutoff_hz;
    float min_cutoff_hz = 5.0f;
    float max_cutoff_hz = 50.0f;
    
    // 振動レベル推定
    float estimated_noise_level;
    
public:
    void update_filter_parameters(
        Vector3f sensor_data,
        LowPassFilter2pVector3f &filter,
        float sample_rate) {
        
        // ステップ 1: 振動レベルを推定
        estimated_noise_level = estimate_noise_level(sensor_data);
        
        // ステップ 2: 振動レベルに応じてカットオフを調整
        float new_cutoff = calculate_optimal_cutoff();
        
        // ステップ 3: 更新が必要な場合のみ再計算
        if (abs(new_cutoff - current_cutoff_hz) > 1.0f) {
            filter.set_cutoff_frequency(sample_rate, new_cutoff);
            current_cutoff_hz = new_cutoff;
            
            #if AP_DEBUG_ENABLED
            gcs().send_text(MAV_SEVERITY_INFO,
                "LPF updated to %f Hz (noise: %f)",
                new_cutoff, estimated_noise_level);
            #endif
        }
    }
    
private:
    float estimate_noise_level(Vector3f sample) {
        // 移動標準偏差を計算
        static RingBuffer<Vector3f, 20> buffer;
        buffer.add(sample);
        
        if (buffer.size() < buffer.capacity()) {
            return 0;  // 十分なデータが無い
        }
        
        Vector3f mean;
        for (uint8_t i = 0; i < buffer.capacity(); i++) {
            mean += buffer.get(i);
        }
        mean *= (1.0f / buffer.capacity());
        
        float variance = 0;
        for (uint8_t i = 0; i < buffer.capacity(); i++) {
            Vector3f dev = buffer.get(i) - mean;
            variance += dev.length_squared();
        }
        variance *= (1.0f / buffer.capacity());
        
        return sqrt(variance);  // 標準偏差
    }
    
    float calculate_optimal_cutoff() {
        // 利用可能なノイズの量に基づいて選択
        if (estimated_noise_level < 0.1f) {
            // ノイズが少ない → カットオフを高く
            return max_cutoff_hz;
        } else if (estimated_noise_level > 1.0f) {
            // ノイズが多い → カットオフを低く
            return min_cutoff_hz;
        } else {
            // 中程度 → 線形補間
            float factor = (estimated_noise_level - 0.1f) / 0.9f;  // 0-1
            return min_cutoff_hz + factor * (max_cutoff_hz - min_cutoff_hz);
        }
    }
};
```

---

## 8. フィルタ性能モニタリング

```cpp
// File: filter_performance_monitoring.cpp

class FilterPerformanceMonitor {
private:
    struct FilterMetrics {
        uint32_t total_samples;
        uint32_t error_count;
        float max_value;
        float min_value;
        float mean_value;
        float std_deviation;
        uint32_t last_error_time_ms;
    } metrics;
    
    static const uint32_t ERROR_REPORT_INTERVAL_MS = 5000;
    
public:
    void update(float filtered_value, Vector3f raw_data) {
        metrics.total_samples++;
        
        // ステップ 1: 統計更新
        update_statistics(filtered_value);
        
        // ステップ 2: 異常検出
        check_for_anomalies(raw_data, filtered_value);
        
        // ステップ 3: 定期報告
        report_if_needed();
    }
    
    void print_summary() {
        gcs().send_text(MAV_SEVERITY_INFO,
            "Filter Stats: samples=%lu errors=%lu std=%.3f",
            metrics.total_samples,
            metrics.error_count,
            metrics.std_deviation);
    }
    
private:
    void update_statistics(float value) {
        if (metrics.total_samples == 1) {
            metrics.max_value = value;
            metrics.min_value = value;
            metrics.mean_value = value;
        } else {
            metrics.max_value = MAX(metrics.max_value, value);
            metrics.min_value = MIN(metrics.min_value, value);
            
            // 移動平均
            metrics.mean_value = (metrics.mean_value * 0.99f) + (value * 0.01f);
        }
    }
    
    void check_for_anomalies(Vector3f raw, float filtered) {
        float change = abs(raw.length() - filtered);
        
        if (change > 50.0f) {  // センサーに応じて調整
            metrics.error_count++;
            metrics.last_error_time_ms = AP_HAL::millis();
        }
    }
    
    void report_if_needed() {
        static uint32_t last_report_time = 0;
        uint32_t now = AP_HAL::millis();
        
        if (now - last_report_time > ERROR_REPORT_INTERVAL_MS) {
            last_report_time = now;
            print_summary();
        }
    }
};
```

---

## ベストプラクティス まとめ

### ✅ すべき こと

1. **フィルタチェーンを構築**
   - 外れ値除去 → ノッチ → ローパス

2. **エラーハンドリングを実装**
   - NaN/Inf チェック
   - 自動リセット
   - フォールバック処理

3. **統計情報を記録**
   - エラー数
   - パフォーマンス指標

4. **定期的に検証**
   - 周波数応答テスト
   - ステップ応答テスト
   - ノイズロバストネステスト

5. **パラメータを文書化**
   ```cpp
   // 初期化
   accel_filter.set_cutoff_frequency(
       1000.0f,  // sample_freq_hz: IMUは通常1kHz
       20.0f     // cutoff_freq_hz: 低周波ノイズ除去用
   );
   ```

### ❌ すべきでない こと

1. **単一フィルタだけに依存**
   - 複合フィルタを使用

2. **エラー検出を省略**
   - NaN/Inf は EKF を崩壊させる

3. **固定パラメータの過度な使用**
   - 環境に応じた調整が必要

4. **計算コストの無視**
   - リアルタイム制約を考慮

5. **キャリブレーションデータの欠落**
   - 温度補正が必須な場合がある

---

## デバッグのヒント

### フィルタが正しく機能していない場合

```cpp
// 1. フィルタ入出力をログ出力
gcs().send_text(MAV_SEVERITY_DEBUG,
    "Raw: %.2f, Filt: %.2f",
    raw_value, filtered_value);

// 2. 周波数応答をテスト
sweep_frequency(10, 200, 1);  // 10-200Hz, 1sec/freq

// 3. ステップ応答をテスト
step_response_test(100.0f);  // 100 m/s² ステップ

// 4. 安定性を確認
check_stability(10000);  // 10,000サンプル

// 5. CPU負荷を測定
uint32_t start = micros();
for (int i = 0; i < 1000; i++) {
    filter.apply(test_data);
}
uint32_t elapsed = micros() - start;
gcs().send_text(MAV_SEVERITY_INFO,
    "Filter CPU: %.1f µs/sample", 
    elapsed / 1000.0f);
```
