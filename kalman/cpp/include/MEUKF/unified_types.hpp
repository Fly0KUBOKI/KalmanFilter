#pragma once

#include "../Common/Math/fixed_matrix.hpp"
#include "../Common/Math/quaternion.hpp"

namespace meukf {

using Vec3 = FixedMatrix<3, 1>;
using Vec4 = FixedMatrix<4, 1>;
using Mat3 = FixedMatrix<3, 3>;
using Mat15 = FixedMatrix<15, 15>;

// 入力構造体 - センサーデータ + パラメータ
struct FilterInput {
    // タイムステップ
    double dt;
    
    // センサーデータ（全て毎回送信）
    Vec3 accel;          // 加速度 [m/s^2] (100 Hz)
    Vec3 gyro;           // 角速度 [rad/s] (100 Hz)
    Vec3 mag;            // 磁場 (25 Hz → 4回重複)
    Vec3 gps_pos;        // GPS位置 [m] NED座標系 (4 Hz → 25回重複)
    double baro_alt;     // 気圧高度 [m] (2 Hz → 50回重複)
    
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
    double noise_baro;
    
    // UKFパラメータ
    double alpha;
    double beta;
    double kappa;
    
    // デフォルトコンストラクタ
    FilterInput() 
        : dt(0.01), 
          baro_alt(0.0),
          mag_valid(true), 
          gps_valid(true), 
          baro_valid(true),
          noise_baro(1.0),
          alpha(1e-3), 
          beta(2.0), 
          kappa(0.0) {
        accel.setZero();
        gyro.setZero();
        mag.setZero();
        gps_pos.setZero();
        g(0) = 0; g(1) = 0; g(2) = 9.80665;
        mag_ref(0) = 1; mag_ref(1) = 0; mag_ref(2) = 0;
        noise_accel.setConstant(0.01);
        noise_gyro.setConstant(0.001);
        noise_mag.setConstant(0.1);
        noise_gps.setConstant(5.0);
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
    double roll;
    double pitch;
    double yaw;
    
    // 診断情報
    double innovation_norm_accel;
    double innovation_norm_mag;
    double innovation_norm_gps;
    double innovation_norm_baro;
    bool divergence_detected;
    bool reset_occurred;
    int num_updates_applied;
    
    // デフォルトコンストラクタ
    FilterOutput() 
        : roll(0.0), pitch(0.0), yaw(0.0),
          innovation_norm_accel(0.0),
          innovation_norm_mag(0.0),
          innovation_norm_gps(0.0),
          innovation_norm_baro(0.0),
          divergence_detected(false),
          reset_occurred(false),
          num_updates_applied(0) {
        position.setZero();
        velocity.setZero();
        quaternion(0) = 1; quaternion(1) = 0; 
        quaternion(2) = 0; quaternion(3) = 0;
        accel_bias.setZero();
        gyro_bias.setZero();
        covariance.setIdentity();
        covariance *= 0.01;
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
        p.setZero();
        v.setZero();
        q(0) = 1; q(1) = 0; q(2) = 0; q(3) = 0;
        ba.setZero();
        bg.setZero();
        P.setIdentity();
        P *= 0.01;
    }
};

} // namespace meukf
