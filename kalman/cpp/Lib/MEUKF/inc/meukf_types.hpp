#pragma once
#include <cstdint>
#include "../../Matrix/fixed_matrix.hpp"

namespace meukf {

// 状態量と共分散
struct State {
    float p[3];         // 位置 (x, y, z)
    float v[3];         // 速度 (vx, vy, vz)
    float q[4];         // 姿勢クォータニオン (w, x, y, z)
    float ba[3];        // 加速度バイアス
    float bg[3];        // ジャイロバイアス
    float P[15 * 15];   // 誤差共分散行列 (15x15, Row-major)
};

// センサー入力データ
struct SensorData {
    float accel[3];     // 加速度計 [m/s^2]
    float gyro[3];      // ジャイロ [rad/s]
    float mag[3];       // 磁気計 [uT]
    float gps_pos[3];   // GPS位置 (LLA または NED)
    float alt_baro;     // 気圧高度 [m]
    
    // 前回のセンサー値（変更検知用）
    float prev_mag[3];       // 前回の磁気計値
    float prev_gps_pos[3];   // 前回のGPS位置
    float prev_baro_alt;     // 前回の気圧高度
    
    // 更新フラグ (1: 更新あり, 0: 更新なし)
    uint8_t update_accel;
    uint8_t update_gyro; // ジャイロは予測に必須だが、更新フラグとして一応用意
    uint8_t update_mag;
    uint8_t update_gps;
    uint8_t update_baro;
    uint8_t update_zupt; // ZUPT更新フラグ
    
    float dt;           // 前回からの経過時間 [s]
};

// パラメータ（定数設定）
struct Params {
    float g[3];                 // 重力ベクトル
    float mag_ref[3];           // 基準磁気ベクトル
    
    // ノイズパラメータ (標準偏差 または 分散)
    float noise_accel[3];       // 加速度ノイズ
    float noise_gyro[3];        // ジャイロノイズ
    float noise_ba[3];          // バイアスノイズ
    float noise_bg[3];          // バイアスノイズ
    float noise_mag[3];         // 磁気ノイズ
    float noise_gps[3];         // GPSノイズ
    float noise_baro;           // 気圧高度ノイズ
    float noise_zupt[3];        // ZUPTノイズ
    
    // UKFパラメータ
    float alpha;
    float beta;
    float kappa;
};

// 計算機への入力まとめ
struct MEUKFInput {
    State prev_state;       // 前回の状態
    SensorData sensor;      // 今回のセンサー値
    Params params;          // パラメータ
};

// 計算機からの出力まとめ
struct MEUKFOutput {
    State new_state;        // 更新後の状態
    float debug_info[10];   // デバッグ用（イノベーション等）
    uint8_t status;         // 計算ステータス (0: 正常, 1: エラー)
    // Optional debug outputs for external inspection
    // last_K: stored as row-major 15 x 3 matrix (unused columns zeroed if smaller)
    float last_K[15 * 3];
    // last_S: innovation covariance (3 x 3 for GPS, 1x1 for scalar updates, stored row-major)
    float last_S[3 * 3];
    // last_S_inv: inverse of innovation covariance (3 x 3), row-major
    float last_S_inv[3 * 3];
    // last_H: measurement matrix H stored row-major (3 x 15 for GPS)
    float last_H[3 * 15];
    // last_y: innovation vector (length <= 3)
    float last_y[3];
    // actual length of last_y (1..3, 0 if none)
    uint8_t last_y_len;
    // sensor type code for last update: 0=none,1=accel,2=mag,3=gps,4=baro,5=zupt
    uint8_t last_sensor_type;
    // predicted covariance (P) immediately after predict() and before update
    // stored row-major 15 x 15
    float pred_P[15 * 15];
};

} // namespace meukf
