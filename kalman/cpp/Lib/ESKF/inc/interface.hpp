#pragma once
#ifndef LIB_ESKF_INC_INTERFACE_HPP
#define LIB_ESKF_INC_INTERFACE_HPP

#include "../../Matrix/fixed_matrix.hpp"
#include <cstdint>

namespace kalman {

struct SensorData {
  float accel[3];      // m/s^2
  float gyro[3];       // rad/s
  float mag[3];        // uT
  float baro_alt;      // m
  double gps_lat;      // deg
  double gps_lon;      // deg
  double gps_alt;      // m
};

struct State {
  float p[3];
  float v[3];
  float q[4];   // [w,x,y,z]
  float euler[3];
  float ba[3];
  float bg[3];
  float P[15*15];
};

struct Params {
  float g[3];
  float mag_ref[3];
  float dt;
  float noise_accel[3];
  float noise_gyro[3];
  float noise_ba[3];
  float noise_bg[3];
  float noise_mag[3];
  float noise_baro;
  double noise_gps[3];
};

class Filter {
public:
  virtual ~Filter() {}
  virtual uint8_t init(const SensorData& obs, float static_time) = 0;
  virtual uint8_t update(const SensorData& obs) = 0;
  virtual uint8_t getState(State& out) = 0;
  virtual uint8_t setParams(const Params& p) = 0;
  virtual uint8_t reset() = 0;
};

} // namespace kalman

// 基本型定義
using Scalar = float;
using Vector3 = cmath_fx::Vector<3, Scalar>;
using Vector4 = cmath_fx::Vector<4, Scalar>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, Scalar>;

// センサーデータ入力構造体
struct SensorInput {
    // IMU
    Vector3 accel;              // 加速度 [m/s^2]
    Vector3 gyro;               // ジャイロ [rad/s]
    bool accel_updated;         // 加速度が更新されたか
    bool gyro_updated;          // ジャイロが更新されたか
    
    // 磁気計
    Vector3 mag;                // 磁気 [任意単位]
    bool mag_updated;           // 磁気計が更新されたか
    
    // GPS
    Vector3 gps_pos;            // GPS位置 [m] (LLA→ENU変換済み)
    Vector3 gps_vel;            // GPS速度 [m/s]
    bool gps_updated;           // GPSが更新されたか
    
    // 気圧計
    Scalar pressure;            // 気圧 [Pa]
    bool baro_updated;          // 気圧計が更新されたか
    
    // タイムスタンプ
    Scalar dt;                  // サンプリング時間 [s]
    
    // デフォルトコンストラクタ：全て未更新に初期化
    SensorInput() 
        : accel{0, 0, 0}
        , gyro{0, 0, 0}
        , accel_updated(false)
        , gyro_updated(false)
        , mag{0, 0, 0}
        , mag_updated(false)
        , gps_pos{0, 0, 0}
        , gps_vel{0, 0, 0}
        , gps_updated(false)
        , pressure(0)
        , baro_updated(false)
        , dt(0)
    {}
};

// フィルタ出力構造体
struct FilterOutput {
    // 状態推定値
    Vector3 position;           // 位置 [m]
    Vector3 velocity;           // 速度 [m/s]
    Vector4 quaternion;         // クォータニオン [w, x, y, z]
    Vector3 euler;              // オイラー角 [roll, pitch, yaw] [rad]
    Vector3 accel_bias;         // 加速度バイアス [m/s^2]
    Vector3 gyro_bias;          // ジャイロバイアス [rad/s]
    
    // 共分散（オプション）
    Scalar* P_data;             // 共分散行列のポインタ（必要に応じて）
    uint16_t P_size;            // 共分散行列のサイズ
    
    // デフォルトコンストラクタ
    FilterOutput()
        : position{0, 0, 0}
        , velocity{0, 0, 0}
        , quaternion{1, 0, 0, 0}  // 単位クォータニオン
        , euler{0, 0, 0}
        , accel_bias{0, 0, 0}
        , gyro_bias{0, 0, 0}
        , P_data(nullptr)
        , P_size(0)
    {}
};

// フィルタパラメータ構造体
struct FilterParams {
    // ノイズ共分散
    Matrix3x3 Q_gyro;           // ジャイロノイズ
    Matrix3x3 Q_accel;          // 加速度ノイズ
    Matrix3x3 Q_gyro_bias;      // ジャイロバイアスノイズ
    Matrix3x3 Q_accel_bias;     // 加速度バイアスノイズ
    
    Matrix3x3 R_mag;            // 磁気計観測ノイズ
    Matrix3x3 R_gps_pos;        // GPS位置観測ノイズ
    Matrix3x3 R_gps_vel;        // GPS速度観測ノイズ
    Scalar R_baro;              // 気圧計観測ノイズ
    
    // 環境パラメータ
    Vector3 gravity;            // 重力ベクトル [m/s^2] (Z-up: [0, 0, -9.81])
    Vector3 mag_world;          // 地磁気ベクトル（ワールド座標系）
    Vector3 gps_origin;         // GPS原点 [m]
    
    // しきい値
    Vector3 gyro_noise_threshold;   // ジャイロノイズしきい値
    Vector3 accel_noise_threshold;  // 加速度ノイズしきい値
    
    // デフォルトコンストラクタ
    FilterParams() 
        : gravity{0, 0, -9.81f}  // Z-up convention: gravity points down
        , mag_world{1, 0, 0}
        , gps_origin{0, 0, 0}
        , gyro_noise_threshold{0.001f, 0.001f, 0.001f}
        , accel_noise_threshold{0.01f, 0.01f, 0.01f}
        , R_baro(1.0f)
    {
        // 単位行列で初期化
        Q_gyro = Matrix3x3::identity() * 0.01f;
        Q_accel = Matrix3x3::identity() * 0.01f;
        Q_gyro_bias = Matrix3x3::identity() * 1e-6f;
        Q_accel_bias = Matrix3x3::identity() * 1e-6f;
        R_mag = Matrix3x3::identity() * 0.1f;
        R_gps_pos = Matrix3x3::identity() * 1.0f;
        R_gps_vel = Matrix3x3::identity() * 0.1f;
    }
};

// センサーデータ管理クラス
class SensorDataManager {
public:
    SensorDataManager() {
        reset();
    }
    
    // 前回のセンサーデータをリセット
    void reset() {
        prev_accel = Vector3{0, 0, 0};
        prev_gyro = Vector3{0, 0, 0};
        prev_mag = Vector3{0, 0, 0};
        prev_gps_pos = Vector3{0, 0, 0};
        prev_pressure = 0;
    }
    
    // センサーデータが更新されたか判定
    void check_updates(SensorInput& input) {
        // 加速度の更新判定
        input.accel_updated = !is_equal(input.accel, prev_accel) && !is_zero(input.accel);
        
        // ジャイロの更新判定
        input.gyro_updated = !is_equal(input.gyro, prev_gyro) && !is_zero(input.gyro);
        
        // 磁気計の更新判定
        input.mag_updated = !is_equal(input.mag, prev_mag) && !is_zero(input.mag);
        
        // GPSの更新判定
        input.gps_updated = !is_equal(input.gps_pos, prev_gps_pos) && !is_zero(input.gps_pos);
        
        // 気圧計の更新判定
        input.baro_updated = (input.pressure != prev_pressure) && (input.pressure != 0);
        
        // 更新があった場合、前回の値を保存
        if (input.accel_updated) prev_accel = input.accel;
        if (input.gyro_updated) prev_gyro = input.gyro;
        if (input.mag_updated) prev_mag = input.mag;
        if (input.gps_updated) prev_gps_pos = input.gps_pos;
        if (input.baro_updated) prev_pressure = input.pressure;
    }
    
private:
    Vector3 prev_accel;
    Vector3 prev_gyro;
    Vector3 prev_mag;
    Vector3 prev_gps_pos;
    Scalar prev_pressure;
    
    // ベクトル比較（許容誤差あり）
    static bool is_equal(const Vector3& a, const Vector3& b, Scalar tol = 1e-9f) {
        for (int i = 0; i < 3; ++i) {
            if (std::abs(a[i] - b[i]) > tol) return false;
        }
        return true;
    }
    
    // ゼロベクトル判定
    static bool is_zero(const Vector3& v, Scalar tol = 1e-9f) {
        for (int i = 0; i < 3; ++i) {
            if (std::abs(v[i]) > tol) return false;
        }
        return true;
    }
};

} // namespace kalman

#endif // LIB_COMMON_INC_INTERFACE_HPP
