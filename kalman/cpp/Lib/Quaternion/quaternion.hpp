#pragma once

#include "../Matrix/matrix.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace lib {
namespace quat {

using namespace matrix;

// クォータニオン [w, x, y, z]
using Quat = Vec4;

// 正規化
template<typename T = Scalar>
void normalize(Quat& q) {
    T n = static_cast<T>(0);
    for (Index i = 0; i < 4; ++i) {
        n += q(i, 0) * q(i, 0);
    }
    n = std::sqrt(n);
    if (n < static_cast<T>(1e-12f)) {
        q(0, 0) = static_cast<T>(1.0f);
        q(1, 0) = static_cast<T>(0.0f);
        q(2, 0) = static_cast<T>(0.0f);
        q(3, 0) = static_cast<T>(0.0f);
        return;
    }
    for (Index i = 0; i < 4; ++i) {
        q(i, 0) /= n;
    }
}

// 共役
template<typename T = Scalar>
Quat conjugate(const Quat& q) {
    Quat result;
    result(0, 0) = q(0, 0);
    result(1, 0) = -q(1, 0);
    result(2, 0) = -q(2, 0);
    result(3, 0) = -q(3, 0);
    return result;
}

// 逆元
template<typename T = Scalar>
Quat inverse(const Quat& q) {
    T norm_sq = static_cast<T>(0);
    for (Index i = 0; i < 4; ++i) {
        norm_sq += q(i, 0) * q(i, 0);
    }
    T inv = static_cast<T>(1.0f) / norm_sq;
    Quat result;
    result(0, 0) = q(0, 0) * inv;
    result(1, 0) = -q(1, 0) * inv;
    result(2, 0) = -q(2, 0) * inv;
    result(3, 0) = -q(3, 0) * inv;
    return result;
}

// クォータニオン積 q1 * q2
template<typename T = Scalar>
Quat multiply(const Quat& q1, const Quat& q2) {
    T aw = q1(0, 0), ax = q1(1, 0), ay = q1(2, 0), az = q1(3, 0);
    T bw = q2(0, 0), bx = q2(1, 0), by = q2(2, 0), bz = q2(3, 0);
    
    Quat result;
    result(0, 0) = aw * bw - ax * bx - ay * by - az * bz;
    result(1, 0) = aw * bx + ax * bw + ay * bz - az * by;
    result(2, 0) = aw * by - ax * bz + ay * bw + az * bx;
    result(3, 0) = aw * bz + ax * by - ay * bx + az * bw;
    return result;
}

// 回転行列へ変換
template<typename T = Scalar>
Mat3 to_rotation_matrix(const Quat& q) {
    T qw = q(0, 0), qx = q(1, 0), qy = q(2, 0), qz = q(3, 0);
    Mat3 R;
    
    R(0, 0) = static_cast<T>(1.0f) - static_cast<T>(2.0f) * (qy * qy + qz * qz);
    R(0, 1) = static_cast<T>(2.0f) * (qx * qy - qz * qw);
    R(0, 2) = static_cast<T>(2.0f) * (qx * qz + qy * qw);
    R(1, 0) = static_cast<T>(2.0f) * (qx * qy + qz * qw);
    R(1, 1) = static_cast<T>(1.0f) - static_cast<T>(2.0f) * (qx * qx + qz * qz);
    R(1, 2) = static_cast<T>(2.0f) * (qy * qz - qx * qw);
    R(2, 0) = static_cast<T>(2.0f) * (qx * qz - qy * qw);
    R(2, 1) = static_cast<T>(2.0f) * (qy * qz + qx * qw);
    R(2, 2) = static_cast<T>(1.0f) - static_cast<T>(2.0f) * (qx * qx + qy * qy);
    
    return R;
}

// オイラー角へ変換 (度数法) [roll, pitch, yaw]
template<typename T = Scalar>
Vec3 to_euler_deg(const Quat& q) {
    T qw = q(0, 0), qx = q(1, 0), qy = q(2, 0), qz = q(3, 0);
    Vec3 euler;
    
    T sinr_cosp = static_cast<T>(2.0f) * (qw * qx + qy * qz);
    T cosr_cosp = static_cast<T>(1.0f) - static_cast<T>(2.0f) * (qx * qx + qy * qy);
    T roll = std::atan2(sinr_cosp, cosr_cosp);
    
    T sinp = static_cast<T>(2.0f) * (qw * qy - qz * qx);
    T pitch;
    if (std::abs(sinp) >= static_cast<T>(1.0f)) {
        pitch = std::copysign(static_cast<T>(M_PI) / static_cast<T>(2.0f), sinp);
    } else {
        pitch = std::asin(sinp);
    }
    
    T siny_cosp = static_cast<T>(2.0f) * (qw * qz + qx * qy);
    T cosy_cosp = static_cast<T>(1.0f) - static_cast<T>(2.0f) * (qy * qy + qz * qz);
    T yaw = std::atan2(siny_cosp, cosy_cosp);
    
    euler(0, 0) = roll * static_cast<T>(180.0f) / static_cast<T>(M_PI);
    euler(1, 0) = pitch * static_cast<T>(180.0f) / static_cast<T>(M_PI);
    euler(2, 0) = yaw * static_cast<T>(180.0f) / static_cast<T>(M_PI);
    
    return euler;
}

// オイラー角から生成 (度数法)
template<typename T = Scalar>
Quat from_euler_deg(T roll_deg, T pitch_deg, T yaw_deg) {
    T roll = roll_deg * static_cast<T>(M_PI) / static_cast<T>(180.0f);
    T pitch = pitch_deg * static_cast<T>(M_PI) / static_cast<T>(180.0f);
    T yaw = yaw_deg * static_cast<T>(M_PI) / static_cast<T>(180.0f);
    
    T cy = std::cos(yaw * static_cast<T>(0.5f));
    T sy = std::sin(yaw * static_cast<T>(0.5f));
    T cp = std::cos(pitch * static_cast<T>(0.5f));
    T sp = std::sin(pitch * static_cast<T>(0.5f));
    T cr = std::cos(roll * static_cast<T>(0.5f));
    T sr = std::sin(roll * static_cast<T>(0.5f));
    
    Quat q;
    q(0, 0) = cr * cp * cy + sr * sp * sy;
    q(1, 0) = sr * cp * cy - cr * sp * sy;
    q(2, 0) = cr * sp * cy + sr * cp * sy;
    q(3, 0) = cr * cp * sy - sr * sp * cy;
    
    normalize(q);
    return q;
}

// 小角度回転ベクトルから生成 (ラジアン)
template<typename T = Scalar>
Quat from_small_angle(const Vec3& theta) {
    T th2 = theta(0, 0) * theta(0, 0) + theta(1, 0) * theta(1, 0) + theta(2, 0) * theta(2, 0);
    
    Quat q;
    if (th2 < static_cast<T>(1e-18f)) {
        q(0, 0) = static_cast<T>(1.0f);
        q(1, 0) = static_cast<T>(0.5f) * theta(0, 0);
        q(2, 0) = static_cast<T>(0.5f) * theta(1, 0);
        q(3, 0) = static_cast<T>(0.5f) * theta(2, 0);
    } else {
        T angle = std::sqrt(th2);
        T half_angle = angle * static_cast<T>(0.5f);
        T s = std::sin(half_angle) / angle;
        
        q(0, 0) = std::cos(half_angle);
        q(1, 0) = theta(0, 0) * s;
        q(2, 0) = theta(1, 0) * s;
        q(3, 0) = theta(2, 0) * s;
    }
    
    normalize(q);
    return q;
}

// 角速度による積分
template<typename T = Scalar>
Quat integrate(const Quat& q, const Vec3& omega, T dt) {
    T w_dt_x = omega(0, 0) * dt;
    T w_dt_y = omega(1, 0) * dt;
    T w_dt_z = omega(2, 0) * dt;
    T w_dt_norm = std::sqrt(w_dt_x * w_dt_x + w_dt_y * w_dt_y + w_dt_z * w_dt_z);
    
    constexpr T threshold = static_cast<T>(1e-15f);
    if (w_dt_norm > threshold) {
        T half_angle = w_dt_norm * static_cast<T>(0.5f);
        
        Quat delta_q;
        if (half_angle > static_cast<T>(1e-6f)) {
            T sin_half = std::sin(half_angle);
            T cos_half = std::cos(half_angle);
            T inv_norm = sin_half / w_dt_norm;
            
            delta_q(0, 0) = cos_half;
            delta_q(1, 0) = w_dt_x * inv_norm;
            delta_q(2, 0) = w_dt_y * inv_norm;
            delta_q(3, 0) = w_dt_z * inv_norm;
        } else {
            // Taylor展開
            T w_norm_sq = w_dt_norm * w_dt_norm;
            delta_q(0, 0) = static_cast<T>(1.0f) - w_norm_sq / static_cast<T>(8.0f);
            T coeff = static_cast<T>(0.5f) * (static_cast<T>(1.0f) - w_norm_sq / static_cast<T>(24.0f));
            delta_q(1, 0) = w_dt_x * coeff;
            delta_q(2, 0) = w_dt_y * coeff;
            delta_q(3, 0) = w_dt_z * coeff;
        }
        
        Quat result = multiply(q, delta_q);
        normalize(result);
        return result;
    } else {
        return q;
    }
}

// 球面線形補間
template<typename T = Scalar>
Quat slerp(const Quat& q1, const Quat& q2, T t) {
    Quat a = q1;
    Quat b = q2;
    normalize(a);
    normalize(b);
    
    T dot_product = a(0, 0) * b(0, 0) + a(1, 0) * b(1, 0) + a(2, 0) * b(2, 0) + a(3, 0) * b(3, 0);
    
    // 最短経路選択
    if (dot_product < static_cast<T>(0)) {
        b(0, 0) = -b(0, 0);
        b(1, 0) = -b(1, 0);
        b(2, 0) = -b(2, 0);
        b(3, 0) = -b(3, 0);
        dot_product = -dot_product;
    }
    
    dot_product = std::min(dot_product, static_cast<T>(1.0f));
    T theta = std::acos(dot_product);
    
    constexpr T EPS = static_cast<T>(1e-9f);
    if (std::abs(theta) < EPS) {
        return a;
    }
    
    T sin_theta = std::sin(theta);
    T w1 = std::sin((static_cast<T>(1.0f) - t) * theta) / sin_theta;
    T w2 = std::sin(t * theta) / sin_theta;
    
    Quat result;
    result(0, 0) = w1 * a(0, 0) + w2 * b(0, 0);
    result(1, 0) = w1 * a(1, 0) + w2 * b(1, 0);
    result(2, 0) = w1 * a(2, 0) + w2 * b(2, 0);
    result(3, 0) = w1 * a(3, 0) + w2 * b(3, 0);
    
    normalize(result);
    return result;
}

// 2クォータニオン間の角度 (度数法)
template<typename T = Scalar>
T angle_between(const Quat& q1, const Quat& q2) {
    Quat a = q1;
    Quat b = q2;
    normalize(a);
    normalize(b);
    
    T dot_product = std::abs(a(0, 0) * b(0, 0) + a(1, 0) * b(1, 0) + a(2, 0) * b(2, 0) + a(3, 0) * b(3, 0));
    dot_product = std::min(dot_product, static_cast<T>(1.0f));
    
    return static_cast<T>(2.0f) * std::acos(dot_product) * static_cast<T>(180.0f) / static_cast<T>(M_PI);
}

} // namespace quat
} // namespace lib



