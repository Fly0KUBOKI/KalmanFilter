#ifndef UNIFIED_FILTER_HPP
#define UNIFIED_FILTER_HPP

#include "meukf_types.hpp"  // MEUKFの型定義を使用

namespace unified {

// シンプルな固定配列型を使用
typedef float Vector3[3];
typedef float Quaternion[4];  // [qw, qx, qy, qz]
using Vector15 = cmath_fx::Vector<15, float>;
using Matrix15x15 = cmath_fx::Matrix<15, 15, float>;

// ========== 統合入力構造体 ==========
struct FilterInput {
    // タイムスタンプ
    float dt;  // サンプリング時間 [s]
    
    // IMUデータ (毎回必須)
    Vector3 accel;     // 加速度 [m/s^2] (Body frame)
    Vector3 gyro;      // 角速度 [rad/s] (Body frame)
    
    // 磁気計データ (更新時のみ有効)
    Vector3 mag;       // 磁場 [nT or normalized]
    bool mag_valid;    // 磁気計データ有効フラグ
    
    // GPSデータ (更新時のみ有効)
    Vector3 gps_pos;   // GPS位置 [m] (NED frame)
    bool gps_valid;    // GPSデータ有効フラグ
    
    // 気圧計データ (更新時のみ有効)
    float baro_alt;    // 気圧高度 [m]
    bool baro_valid;   // 気圧計データ有効フラグ
    
    // 前回値 (変更検知用)
    Vector3 prev_mag;
    Vector3 prev_gps_pos;
    float prev_baro_alt;
    
    // フィルタパラメータ
    Vector3 g;         // 重力ベクトル [m/s^2] (NED frame)
    Vector3 mag_ref;   // 磁場基準ベクトル (NED frame)
    
    // ノイズパラメータ (動的に更新される)
    float noise_accel;
    float noise_gyro;
    float noise_mag;
    float noise_gps;
    float noise_baro;
    
    // UKFパラメータ
    float alpha;       // シグマ点スプレッド
    float beta;        // 分布パラメータ
    float kappa;       // スケーリングパラメータ
    
    // コンストラクタ
    FilterInput() : dt(0.01f), mag_valid(false), gps_valid(false), baro_valid(false),
                    prev_baro_alt(0.0f),
                    noise_accel(0.01f), noise_gyro(1.74e-4f), noise_mag(0.1f),
                    noise_gps(1.0f), noise_baro(1.0f),
                    alpha(0.001f), beta(2.0f), kappa(0.0f) {
        for (int i = 0; i < 3; ++i) {
            accel[i] = 0.0f; gyro[i] = 0.0f; mag[i] = 0.0f;
            gps_pos[i] = 0.0f; prev_mag[i] = 0.0f; prev_gps_pos[i] = 0.0f;
            mag_ref[i] = (i == 0) ? 1.0f : 0.0f;
        }
        g[0] = 0.0f; g[1] = 0.0f; g[2] = 9.80665f;
    }
};

// ========== 統合出力構造体 ==========
struct FilterOutput {
    // 推定状態
    Vector3 position;      // 位置 [m] (NED frame)
    Vector3 velocity;      // 速度 [m/s] (NED frame)
    Quaternion quaternion; // 姿勢クォータニオン [qw,qx,qy,qz] (Body -> NED)
    Vector3 accel_bias;    // 加速度バイアス [m/s^2]
    Vector3 gyro_bias;     // ジャイロバイアス [rad/s]
    
    // 共分散行列
    Matrix15x15 covariance;  // 15x15 誤差共分散
    
    // オイラー角 (可視化用)
    float roll;    // [deg]
    float pitch;   // [deg]
    float yaw;     // [deg]
    
    // 診断情報
    float innovation_norm_accel;
    float innovation_norm_mag;
    float innovation_norm_gps;
    float innovation_norm_baro;
    
    bool divergence_detected;
    bool reset_occurred;
    
    // コンストラクタ
    FilterOutput() : roll(0.0f), pitch(0.0f), yaw(0.0f),
                     innovation_norm_accel(0.0f), innovation_norm_mag(0.0f),
                     innovation_norm_gps(0.0f), innovation_norm_baro(0.0f),
                     divergence_detected(false), reset_occurred(false) {
        for (int i = 0; i < 3; ++i) {
            position[i] = 0.0f;
            velocity[i] = 0.0f;
            accel_bias[i] = 0.0f;
            gyro_bias[i] = 0.0f;
        }
        quaternion[0] = 1.0f; quaternion[1] = 0.0f; quaternion[2] = 0.0f; quaternion[3] = 0.0f;
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                covariance(i, j) = (i == j) ? ((i < 3) ? 1.0f : (i < 6) ? 0.1f : (i < 9) ? 0.01f : 1e-3f) : 0.0f;
            }
        }
    }
};

// ========== 統合フィルタクラス (前方宣言) ==========
class UnifiedFilter {
public:
    UnifiedFilter();
    ~UnifiedFilter() = default;
    
    // メインインターフェース
    void initialize(const FilterOutput& initial_state);
    void update(const FilterInput& input, FilterOutput& output);
    
    // リセット機能
    void reset();
    
private:
    // 内部状態
    FilterOutput current_state_;
    
    // センサーフィルタ・推定器 (sensor_filter.hpp から)
    // 実装は .cpp ファイルで
};

} // namespace unified

#endif // UNIFIED_FILTER_HPP
