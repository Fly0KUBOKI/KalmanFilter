# C++ アーキテクチャ詳細

## 🎯 概要

C++ MEX実装は、**高性能数値計算エンジン**として、15次元状態ベクトルのKalmanフィルタ処理を最適化された形で提供します。

## 🏗️ アーキテクチャ階層

```
┌─ MEX Interface Layer ─────────────────┐
│ mex_run_eskf.cpp                      │ ← MATLABとのデータ交換・ハンドル管理
│ mex_eskf_initializer.cpp              │ ← 初期化専用MEXインターフェース
│ mex_meukf_step.cpp                    │ ← MEUKF実装エントリーポイント
└───────────────────────────────────────┘
                     │ 型変換・エラーハンドリング
                     ▼
┌─ ESKF Core Library ───────────────────┐
│ Lib/ESKF/                             │
│  ├── eskf_core.{hpp,cpp}              │ ← 主フィルタ実装・状態管理
│  ├── eskf_sensor_updates.{hpp,cpp}    │ ← センサー統合・Innovation計算  
│  ├── eskf_initializer.{hpp,cpp}       │ ← 初期状態推定・バイアス較正
│  ├── eskf_math.{hpp,cpp}              │ ← 数学関数・行列演算
│  └── eskf_postprocess.{hpp,cpp}       │ ← 結果後処理・正規化
└───────────────────────────────────────┘
                     │ アルゴリズム共通処理
                     ▼
┌─ Common Libraries ────────────────────┐
│ Lib/Common/                           │
│  ├── Math/math_utils.hpp              │ ← 統計・ロバスト推定
│  ├── Sensor/sensor_filter.hpp         │ ← 外れ値検出・前処理
│  └── Validation/validation.hpp        │ ← 数値安定性チェック
│                                       │
│ Lib/Matrix/fixed_matrix.hpp           │ ← 固定サイズ行列演算・Cholesky分解
│ Lib/Quaternion/quaternion_functions.hpp │ ← 四元数正規化・回転合成
└───────────────────────────────────────┘
```

---

## 🔧 MEX Interface Layer

### `mex_run_eskf.cpp` — メインエントリーポイント

**指令ディスパッチャ**:
```cpp
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    std::string cmd = mxArrayToString(prhs[0]);
    
    if (cmd == "init") {
        plhs[0] = do_init(prhs[1], prhs[2], prhs[3]);        // obs, static_time, dt
    } else if (cmd == "step") {
        do_step(prhs[1], prhs[2], prhs[3]);                  // handle, obs, k  
    } else if (cmd == "get_state") {
        plhs[0] = do_get_state(prhs[1]);                     // handle
    } else if (cmd == "free") {
        do_free(prhs[1]);                                    // handle
    } else {
        mexErrMsgIdAndTxt("ESKF:InvalidCommand", "Unknown command: %s", cmd.c_str());
    }
}
```

**ハンドル管理システム**:
```cpp
// 内部状態管理 (メモリ安全・マルチインスタンス対応)
static std::map<uint64_t, eskf::State*> g_state_map;
static uint64_t g_next_handle = 1;

uint64_t create_handle(eskf::State* state) {
    uint64_t handle = g_next_handle++;
    g_state_map[handle] = state;
    return handle;
}

eskf::State* get_state_from_handle(uint64_t handle) {
    auto it = g_state_map.find(handle);
    return (it != g_state_map.end()) ? it->second : nullptr;
}
```

### 型変換・安全性保証

**MATLAB → C++ 変換**:
```cpp
// GPS座標のみ double、他センサーは float 厳密分離
double extract_gps_coordinate(const mxArray* arr, const char* field_name) {
    if (!mxIsDouble(arr)) {
        mexErrMsgIdAndTxt("ESKF:TypeMismatch", 
            "GPS座標 %s は double 型である必要があります", field_name);
    }
    return mxGetScalar(arr);
}

void extract_sensor_float(const mxArray* arr, float* output, int size, const char* field_name) {
    if (!mxIsSingle(arr)) {
        mexErrMsgIdAndTxt("ESKF:TypeMismatch", 
            "センサーデータ %s は single 型である必要があります", field_name);
    }
    float* data = (float*)mxGetData(arr);
    memcpy(output, data, sizeof(float) * size);
}
```

**C++ → MATLAB 変換**:
```cpp
mxArray* create_state_struct(const eskf::ESKFState& state) {
    const char* field_names[] = {"p", "v", "q", "euler", "ba", "bg", "P"};
    mxArray* result = mxCreateStructMatrix(1, 1, 7, field_names);
    
    // 位置・速度・バイアス (float32 3x1)
    mxSetField(result, 0, "p", create_float_vector(state.p, 3));
    mxSetField(result, 0, "v", create_float_vector(state.v, 3));
    mxSetField(result, 0, "ba", create_float_vector(state.ba, 3));
    mxSetField(result, 0, "bg", create_float_vector(state.bg, 3));
    
    // 四元数 (float32 4x1, [w,x,y,z] 順序固定)
    mxSetField(result, 0, "q", create_float_vector(state.q, 4));
    
    // 共分散行列 (float32 15x15, column-major, 対称化済み)  
    mxSetField(result, 0, "P", create_covariance_matrix(state.P));
    
    return result;
}
```

---

## 🧮 ESKF Core Library

### `eskf_core.cpp` — フィルタ主実装

**状態予測** (Predict Step):
```cpp
void ESKFCore::predict(double dt) {
    // IMU積分による状態推定
    integrate_imu(dt);                    // 位置・速度・姿勢更新
    compute_process_noise(dt);            // プロセスノイズ計算  
    propagate_covariance(dt);             // 共分散伝播
    
    // 数値安定性保証
    normalize_quaternion();               // 四元数正規化
    symmetrize_covariance();              // 共分散対称化
}
```

**センサー更新** (Update Step):
```cpp  
void ESKFCore::update_sensor(const SensorMeasurement& z, const SensorConfig& config) {
    // Innovation計算
    VectorXf innovation = compute_innovation(z);
    MatrixXf H = compute_observation_jacobian();
    MatrixXf S = H * P_ * H.transpose() + config.R;
    
    // 外れ値検出
    float mahalanobis_dist = compute_mahalanobis_distance(innovation, S);
    if (mahalanobis_dist > config.outlier_threshold) {
        return;  // 外れ値として棄却
    }
    
    // Kalman更新
    MatrixXf K = P_ * H.transpose() * S.inverse();
    VectorXf dx = K * innovation;
    P_ = (MatrixXf::Identity(15,15) - K * H) * P_;
    
    // Error-state reset  
    apply_error_state_reset(dx);
}
```

### `eskf_sensor_updates.cpp` — センサー統合

**IMU更新**:
```cpp
void update_imu(ESKFState& state, const IMUMeasurement& imu, double dt) {
    // 加速度バイアス補正
    Vector3f accel_corrected = imu.accel - state.ba;
    
    // 重力補正 (world frame)  
    Vector3f gravity_world = {0.0f, 0.0f, -9.81f};
    Vector3f accel_world = quat_rotate(state.q, accel_corrected) + gravity_world;
    
    // 速度・位置積分
    state.v += accel_world * dt;
    state.p += state.v * dt + 0.5f * accel_world * dt * dt;
    
    // 角速度バイアス補正・姿勢更新
    Vector3f omega_corrected = imu.gyro - state.bg;
    Quaternion dq = axis_angle_to_quat(omega_corrected * dt);
    state.q = quat_multiply(state.q, dq);
    normalize_quaternion(state.q);
}
```

**GPS更新** (高精度座標処理):
```cpp
void update_gps(ESKFState& state, const GPSMeasurement& gps) {
    // WGS84 → UTM座標変換 (double精度維持)
    double utm_x, utm_y;
    wgs84_to_utm(gps.latitude, gps.longitude, utm_x, utm_y);
    
    // 位置Innovation (最後にfloatキャスト)
    Vector3f gps_pos = {(float)utm_x, (float)utm_y, (float)gps.altitude};
    Vector3f innovation = gps_pos - state.p;
    
    // GPS精度による重み調整
    float gps_accuracy = estimate_gps_accuracy(gps);  // HDOP/衛星数ベース
    Matrix3f R_gps = Matrix3f::Identity() * (gps_accuracy * gps_accuracy);
    
    apply_linear_update(innovation, get_position_jacobian(), R_gps);
}
```

### `eskf_math.cpp` — 数学関数

**四元数演算**:
```cpp  
// [w, x, y, z] 順序厳守 (スカラー先頭)
Quaternion quat_multiply(const Quaternion& q1, const Quaternion& q2) {
    return {
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,  // w
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,  // x
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,  // y
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w   // z
    };
}

void normalize_quaternion(Quaternion& q) {
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm > 1e-6f) {
        q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm;
    }
}
```

**行列演算** (固定サイズ最適化):
```cpp
// 15x15 共分散行列専用最適化
void symmetrize_covariance_15x15(Matrix15x15& P) {
    for (int i = 0; i < 15; ++i) {
        for (int j = i + 1; j < 15; ++j) {
            float avg = 0.5f * (P(i,j) + P(j,i));
            P(i,j) = avg;
            P(j,i) = avg;
        }
    }
}

// Cholesky分解による数値安定性向上
bool cholesky_decomposition_15x15(const Matrix15x15& A, Matrix15x15& L) {
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j <= i; ++j) {
            if (i == j) {
                float sum = 0.0f;
                for (int k = 0; k < j; ++k) sum += L(j,k) * L(j,k);
                float val = A(j,j) - sum;
                if (val <= 0.0f) return false;  // 非正定値
                L(j,j) = sqrtf(val);
            } else {
                float sum = 0.0f;
                for (int k = 0; k < j; ++k) sum += L(i,k) * L(j,k);
                L(i,j) = (A(i,j) - sum) / L(j,j);
            }
        }
    }
    return true;
}
```

---

## 📊 Common Libraries

### `Lib/Common/Math/math_utils.hpp` — ロバスト統計

**外れ値検出**:
```cpp
class OutlierDetector {
public:
    static bool is_outlier_mahalanobis(const VectorXf& residual, 
                                     const MatrixXf& covariance,
                                     float threshold = 5.991f) {  // χ²(3,0.05)
        float dist = residual.transpose() * covariance.inverse() * residual;
        return dist > threshold;
    }
    
    static bool is_outlier_robust_zscore(float value, float median, float mad,
                                        float threshold = 3.5f) {
        float robust_zscore = 0.6745f * (value - median) / mad;
        return fabsf(robust_zscore) > threshold;
    }
};
```

**ロバスト推定**:
```cpp
struct RobustStatistics {
    static float median(const std::vector<float>& data) {
        std::vector<float> sorted = data;
        std::sort(sorted.begin(), sorted.end());
        size_t n = sorted.size();
        return (n % 2 == 0) ? 0.5f * (sorted[n/2-1] + sorted[n/2]) : sorted[n/2];
    }
    
    static float mad(const std::vector<float>& data) {  // Median Absolute Deviation
        float med = median(data);
        std::vector<float> abs_dev;
        for (float x : data) abs_dev.push_back(fabsf(x - med));
        return median(abs_dev);
    }
    
    static void robust_mean_std(const std::vector<float>& data,
                               float& mean, float& std_dev,
                               float outlier_threshold = 3.0f) {
        float med = median(data);
        float mad_val = mad(data);
        
        // 外れ値除外後の平均・標準偏差
        float sum = 0.0f, sum_sq = 0.0f;
        int count = 0;
        for (float x : data) {
            float z_score = fabsf(x - med) / (1.4826f * mad_val + 1e-6f);
            if (z_score < outlier_threshold) {
                sum += x; sum_sq += x * x; count++;
            }
        }
        
        if (count > 0) {
            mean = sum / count;
            std_dev = sqrtf(sum_sq / count - mean * mean);
        } else {
            mean = med; std_dev = 1.4826f * mad_val;
        }
    }
};
```

### `Lib/Common/Sensor/sensor_filter.hpp` — センサー前処理

**適応型フィルタ**:
```cpp
class AdaptiveSensorFilter {
private:
    std::deque<float> window_;
    size_t window_size_;
    float outlier_threshold_;
    
public:
    AdaptiveSensorFilter(size_t window_size = 10, float threshold = 3.0f) 
        : window_size_(window_size), outlier_threshold_(threshold) {}
    
    float filter(float new_value) {
        window_.push_back(new_value);
        if (window_.size() > window_size_) window_.pop_front();
        
        // 窓内でのロバスト推定
        std::vector<float> data(window_.begin(), window_.end());
        float median_val = RobustStatistics::median(data);
        float mad_val = RobustStatistics::mad(data);
        
        // 現在値が外れ値か判定
        float z_score = fabsf(new_value - median_val) / (1.4826f * mad_val + 1e-6f);
        
        if (z_score > outlier_threshold_) {
            return median_val;  // 外れ値の場合、median で置換
        } else {
            return new_value;   // 正常値はそのまま使用
        }
    }
};
```

---

## ⚡ パフォーマンス最適化

### メモリ管理
```cpp
// 動的メモリ確保を排除 (リアルタイム性確保)
class FixedSizeMatrix15x15 {
private:
    float data_[15 * 15];  // スタック確保
    
public:
    inline float& operator()(int r, int c) {
        return data_[r * 15 + c];  // ループ展開可能な単純アクセス
    }
    
    void multiply(const FixedSizeMatrix15x15& other, FixedSizeMatrix15x15& result) const {
        // 15x15 専用の展開ループ（コンパイラ最適化促進）
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                result(i,j) = 0.0f;
                for (int k = 0; k < 15; ++k) {
                    result(i,j) += (*this)(i,k) * other(k,j);
                }
            }
        }
    }
};
```

### SIMD最適化準備
```cpp
// 四元数演算のベクトル化対応
#ifdef __AVX2__
void quat_multiply_avx2(const float* q1, const float* q2, float* result) {
    __m128 q1_vec = _mm_load_ps(q1);  
    __m128 q2_vec = _mm_load_ps(q2);
    // ... AVX2命令による並列演算 ...
    _mm_store_ps(result, result_vec);
}
#endif
```

### キャッシュ効率性
```cpp
// データ配置最適化 (構造体アライメント)
struct alignas(32) ESKFState {
    float p[3];      // 位置     : 12 bytes
    float v[3];      // 速度     : 12 bytes  
    float q[4];      // 四元数   : 16 bytes
    float ba[3];     // バイアス : 12 bytes
    float bg[3];     // バイアス : 12 bytes
    
    // 共分散は別配置 (メモリ局所性向上)
    float P[15*15];  // 900 bytes (32-byte aligned)
};
```

---

## 🔍 デバッグ・診断機能

### アサーション・バリデーション
```cpp
#ifdef DEBUG
#define ESKF_ASSERT(cond, msg) \
    if (!(cond)) mexErrMsgIdAndTxt("ESKF:Assertion", "Assert failed: %s", msg)
#else
#define ESKF_ASSERT(cond, msg) ((void)0)
#endif

void validate_state(const ESKFState& state) {
    // 四元数正規性チェック
    float qnorm = sqrtf(state.q[0]*state.q[0] + state.q[1]*state.q[1] + 
                       state.q[2]*state.q[2] + state.q[3]*state.q[3]);
    ESKF_ASSERT(fabsf(qnorm - 1.0f) < 1e-5f, "Quaternion not normalized");
    
    // 共分散正定値性チェック  
    for (int i = 0; i < 15; ++i) {
        ESKF_ASSERT(state.P[i*15 + i] > 0.0f, "Covariance diagonal not positive");
    }
    
    // NaN/Inf チェック
    for (int i = 0; i < 3; ++i) {
        ESKF_ASSERT(std::isfinite(state.p[i]), "Position contains NaN/Inf");
        ESKF_ASSERT(std::isfinite(state.v[i]), "Velocity contains NaN/Inf");
    }
}
```

### プロファイリング
```cpp
class ScopedTimer {
    std::chrono::high_resolution_clock::time_point start_;
    const char* name_;
public:
    ScopedTimer(const char* name) : name_(name), start_(std::chrono::high_resolution_clock::now()) {}
    ~ScopedTimer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
        mexPrintf("Timer [%s]: %ld μs\n", name_, duration.count());
    }
};

#define PROFILE_SCOPE(name) ScopedTimer timer(name)
```