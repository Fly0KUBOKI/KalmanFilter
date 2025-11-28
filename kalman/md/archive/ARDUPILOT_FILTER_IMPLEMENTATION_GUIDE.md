````markdown
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
````