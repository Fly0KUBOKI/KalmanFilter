#pragma once

#include "Common/Math/fixed_matrix.hpp"
#include "Common/Math/quaternion.hpp"

namespace meukf {

using Vec3 = cmath_fx::Matrix<3, 1, float>;
using Vec4 = cmath_fx::Matrix<4, 1, float>;
using Mat3 = cmath_fx::Matrix<3, 3, float>;
using Mat15 = cmath_fx::Matrix<15, 15, float>;

// 入力構造体 - センサーデータ + パラメータ
struct FilterInput {
    // タイムステップ
    float dt;
    
    // センサーデータ（全て毎回送信）
    Vec3 accel;          // 加速度 [m/s^2] (100 Hz)
    Vec3 gyro;           // 角速度 [rad/s] (100 Hz)
    Vec3 mag;            // 磁場 (25 Hz → 4回重複)
    Vec3 gps_pos;        // GPS位置 [m] NED座標系 (4 Hz → 25回重複)
    float baro_alt;      // 気圧高度 [m] (2 Hz → 50回重複)
    
    // 有効フラグ
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
    float noise_baro;
    
    // UKFパラメータ
    float alpha;
    float beta;
    float kappa;
    
    // デフォルトコンストラクタ
    FilterInput() 
        : dt(0.01f), 
          baro_alt(0.0f),
          mag_valid(true), 
          gps_valid(true), 
          baro_valid(true),
          noise_baro(1.0f),
          alpha(1e-3f), 
          beta(2.0f), 
          kappa(0.0f) {
        // 手動でゼロ初期化
        for (int i = 0; i < 3; ++i) {
            accel(i, 0) = 0.0f;
            gyro(i, 0) = 0.0f;
            mag(i, 0) = 0.0f;
            gps_pos(i, 0) = 0.0f;
        }
        g(0, 0) = 0.0f; g(1, 0) = 0.0f; g(2, 0) = 9.80665f;
        mag_ref(0, 0) = 1.0f; mag_ref(1, 0) = 0.0f; mag_ref(2, 0) = 0.0f;
        // ノイズパラメータを定数で初期化
        for (int i = 0; i < 3; ++i) {
            noise_accel(i, 0) = 0.01f;
            noise_gyro(i, 0) = 0.001f;
            noise_mag(i, 0) = 0.1f;
            noise_gps(i, 0) = 5.0f;
        }
    }
};

// 出力構造体 - 推定状態 + 診断情報
struct FilterOutput {
    // 推定状態
    Vec3 position;       // 位置 [m] (NED)
    Vec3 velocity;       // 速度 [m/s] (NED)
    Vec4 quaternion;     // 姿勢 [qw, qx, qy, qz]
    Vec3 accel_bias;     // 加速度バイアス
    Vec3 gyro_bias;      // ジャイロバイアス
    Mat15 covariance;    // 共分散行列 (15x15)
    
    // オイラー角（可視化用、degrees）
    float roll;
    float pitch;
    float yaw;
    
    // 診断情報
    float innovation_norm_accel;
    float innovation_norm_mag;
    float innovation_norm_gps;
    float innovation_norm_baro;
    bool divergence_detected;
    bool reset_occurred;
    int num_updates_applied;
    
    // デフォルトコンストラクタ
    FilterOutput() 
        : roll(0.0f), pitch(0.0f), yaw(0.0f),
          innovation_norm_accel(0.0f),
          innovation_norm_mag(0.0f),
          innovation_norm_gps(0.0f),
          innovation_norm_baro(0.0f),
          divergence_detected(false),
          reset_occurred(false),
          num_updates_applied(0) {
        // 手動でゼロ初期化
        for (int i = 0; i < 3; ++i) {
            position(i, 0) = 0.0f;
            velocity(i, 0) = 0.0f;
            accel_bias(i, 0) = 0.0f;
            gyro_bias(i, 0) = 0.0f;
        }
        quaternion(0, 0) = 1.0f; quaternion(1, 0) = 0.0f; 
        quaternion(2, 0) = 0.0f; quaternion(3, 0) = 0.0f;
        // 共分散行列を単位行列 * 0.01 で初期化
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                covariance(i, j) = (i == j) ? 0.01f : 0.0f;
            }
        }
    }
};

// 内部状態構造体
struct FilterState {
    Vec3 p;              // 位置
    Vec3 v;              // 速度
    Vec4 q;              // 姿勢
    Vec3 ba;             // 加速度バイアス
    Vec3 bg;             // ジャイロバイアス
    Mat15 P;             // 共分散
    
    FilterState() {
        // 手動でゼロ初期化
        for (int i = 0; i < 3; ++i) {
            p(i, 0) = 0.0f;
            v(i, 0) = 0.0f;
            ba(i, 0) = 0.0f;
            bg(i, 0) = 0.0f;
        }
        q(0, 0) = 1.0f; q(1, 0) = 0.0f; q(2, 0) = 0.0f; q(3, 0) = 0.0f;
        // 共分散行列を単位行列 * 0.01 で初期化
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P(i, j) = (i == j) ? 0.01f : 0.0f;
            }
        }
    }
};

} // namespace meukf
