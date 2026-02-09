#pragma once
#ifndef LIB_SENSOR_SENSOR_PROCESSING_HPP
#define LIB_SENSOR_SENSOR_PROCESSING_HPP

#include "../Matrix/fixed_matrix.hpp"
#include "../Quaternion/quaternion_functions.hpp"
#include <cmath>

namespace sensor {
namespace processing {

using Vector3 = cmath_fx::Vector<3, float>;
using Vector4 = cmath_fx::Vector<4, float>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, float>;

/**
 * 加速度計測値からRoll/Pitch四元数を計算
 * @param a_meas 加速度計測値 [m/s^2]
 * @param scale_factor スケールファクター（通常1.0）
 * @param q_out 出力：四元数 [w, x, y, z]
 */
inline void accel_to_quaternion(const Vector3& a_meas, float scale_factor, Vector4& q_out) {
    // 加速度ノルム計算
    float a_norm = 0.0f;
    for (int i = 0; i < 3; ++i) {
        a_norm += a_meas(i, 0) * a_meas(i, 0);
    }
    a_norm = std::sqrt(a_norm);
    
    // ゼロ除算防止
    if (a_norm < 1e-6f) {
        q_out(0, 0) = 1.0f;
        q_out(1, 0) = 0.0f;
        q_out(2, 0) = 0.0f;
        q_out(3, 0) = 0.0f;
        return;
    }
    
    // 正規化された加速度
    Vector3 a_norm_vec = a_meas;
    for (int i = 0; i < 3; ++i) {
        a_norm_vec(i, 0) /= a_norm;
        a_norm_vec(i, 0) *= scale_factor;
    }
    
    // Roll/Pitch計算（Yaw=0と仮定）
    float roll = std::atan2(a_norm_vec(1, 0), a_norm_vec(2, 0));
    float pitch = std::asin(-a_norm_vec(0, 0));
    
    // 四元数変換
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    
    q_out(0, 0) = cr * cp;
    q_out(1, 0) = sr * cp;
    q_out(2, 0) = cr * sp;
    q_out(3, 0) = -sr * sp;
    
    // 正規化
    cquat::normalize_quat(q_out);
}

/**
 * 磁気センサーの観測予測（ワールド→ボディ座標変換）
 * @param q 姿勢四元数 [w, x, y, z]
 * @param m_world ワールド座標系の磁場ベクトル [μT]
 * @param m_body_expected 出力：ボディ座標系の期待磁場 [μT]
 */
inline void mag_observation_prediction(
    const Vector4& q,
    const Vector3& m_world,
    Vector3& m_body_expected
) {
    // 四元数から回転行列へ変換
    Matrix3x3 R;
    cquat::quat_to_rotm(q, R);
    
    // ワールド→ボディ座標変換
    for (int i = 0; i < 3; ++i) {
        m_body_expected(i, 0) = 0.0f;
        for (int j = 0; j < 3; ++j) {
            m_body_expected(i, 0) += R(i, j) * m_world(j, 0);
        }
    }
}

/**
 * ベクトルをボディ座標からワールド座標へ変換
 * @param q 姿勢四元数 [w, x, y, z]
 * @param v_body ボディ座標系のベクトル
 * @param v_world 出力：ワールド座標系のベクトル
 */
inline void body_to_world(
    const Vector4& q,
    const Vector3& v_body,
    Vector3& v_world
) {
    Matrix3x3 R;
    cquat::quat_to_rotm(q, R);
    
    // R^T * v_body (ボディ→ワールドは逆変換)
    for (int i = 0; i < 3; ++i) {
        v_world(i, 0) = 0.0f;
        for (int j = 0; j < 3; ++j) {
            v_world(i, 0) += R(j, i) * v_body(j, 0);
        }
    }
}

/**
 * ベクトルをワールド座標からボディ座標へ変換
 * @param q 姿勢四元数 [w, x, y, z]
 * @param v_world ワールド座標系のベクトル
 * @param v_body 出力：ボディ座標系のベクトル
 */
inline void world_to_body(
    const Vector4& q,
    const Vector3& v_world,
    Vector3& v_body
) {
    Matrix3x3 R;
    cquat::quat_to_rotm(q, R);
    
    // R * v_world
    for (int i = 0; i < 3; ++i) {
        v_body(i, 0) = 0.0f;
        for (int j = 0; j < 3; ++j) {
            v_body(i, 0) += R(i, j) * v_world(j, 0);
        }
    }
}

/**
 * ベクトルのノルム計算
 * @param v 3Dベクトル
 * @return ノルム
 */
inline float vector3_norm(const Vector3& v) {
    float sum = v(0, 0) * v(0, 0) + v(1, 0) * v(1, 0) + v(2, 0) * v(2, 0);
    return std::sqrt(sum);
}

/**
 * ベクトルのノルム計算（double精度）
 * @param v 3Dベクトル
 * @return ノルム
 */
inline double vector3_norm_d(const Vector3& v) {
    double sum = static_cast<double>(v(0, 0)) * v(0, 0) 
               + static_cast<double>(v(1, 0)) * v(1, 0)
               + static_cast<double>(v(2, 0)) * v(2, 0);
    return std::sqrt(sum);
}

/**
 * Vector3ヘルパー関数（double→float変換）
 * @param x, y, z 成分
 * @return Vector3
 */
inline Vector3 make_vector3(double x, double y, double z) {
    Vector3 v;
    v(0, 0) = static_cast<float>(x);
    v(1, 0) = static_cast<float>(y);
    v(2, 0) = static_cast<float>(z);
    return v;
}

/**
 * Vector3ヘルパー関数（float版）
 * @param x, y, z 成分
 * @return Vector3
 */
inline Vector3 make_vector3_f(float x, float y, float z) {
    Vector3 v;
    v(0, 0) = x;
    v(1, 0) = y;
    v(2, 0) = z;
    return v;
}

/**
 * Vector4ヘルパー関数（四元数用、double→float変換）
 * @param w, x, y, z 成分
 * @return Vector4
 */
inline Vector4 make_vector4(double w, double x, double y, double z) {
    Vector4 v;
    v(0, 0) = static_cast<float>(w);
    v(1, 0) = static_cast<float>(x);
    v(2, 0) = static_cast<float>(y);
    v(3, 0) = static_cast<float>(z);
    return v;
}

/**
 * Vector4ヘルパー関数（float版）
 * @param w, x, y, z 成分
 * @return Vector4
 */
inline Vector4 make_vector4_f(float w, float x, float y, float z) {
    Vector4 v;
    v(0, 0) = w;
    v(1, 0) = x;
    v(2, 0) = y;
    v(3, 0) = z;
    return v;
}

} // namespace processing
} // namespace sensor

#endif // LIB_SENSOR_SENSOR_PROCESSING_HPP
