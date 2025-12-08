#pragma once
#include <cstdint>
#include "../Common/Math/fixed_matrix.hpp"

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
    
    // 更新フラグ (1: 更新あり, 0: 更新なし)
    uint8_t update_accel;
    uint8_t update_gyro; // ジャイロは予測に必須だが、更新フラグとして一応用意
    uint8_t update_mag;
    uint8_t update_gps;
    uint8_t update_baro;
    
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
};

} // namespace meukf
