#pragma once
#ifndef LIB_QUATERNION_QUATERNION_FUNCTIONS_HPP
#define LIB_QUATERNION_QUATERNION_FUNCTIONS_HPP
// Moved from Inc/Common/Math/quaternion.hpp to Lib/Quaternion/quaternion_functions.hpp
#include <cmath>
#include "../Matrix/fixed_matrix.hpp"
#include "../Matrix/Math/statistics.hpp"

namespace cquat {

// Small epsilon provider for quaternion utilities
template<typename T>
inline T quat_eps() { return static_cast<T>(1e-10); }

// Quat as [w, x, y, z]
template <typename T>
inline void normalize_quat(cmath_fx::Vector<4, T>& q) {
    T n = 0.0;
    for (int i = 0; i < 4; ++i) n += q(i,0) * q(i,0);
    n = std::sqrt(n);
    if (n < quat_eps<T>()) { 
        q(0,0)=static_cast<T>(1.0); 
        q(1,0)=static_cast<T>(0.0); 
        q(2,0)=static_cast<T>(0.0); 
        q(3,0)=static_cast<T>(0.0); 
        return; 
    }
    for (int i = 0; i < 4; ++i) q(i,0) /= n;
}

// NOTE: value-return wrapper `normalize_quaternion` removed — use `cquat::normalize_quat` instead.
template <typename T>
inline void multiply_quat(const cmath_fx::Vector<4, T>& a, const cmath_fx::Vector<4, T>& b, cmath_fx::Vector<4, T>& out) {
    T aw=a(0,0), ax=a(1,0), ay=a(2,0), az=a(3,0);
    T bw=b(0,0), bx=b(1,0), by=b(2,0), bz=b(3,0);
    out(0,0) = aw*bw - ax*bx - ay*by - az*bz;
    out(1,0) = aw*bx + ax*bw + ay*bz - az*by;
    out(2,0) = aw*by - ax*bz + ay*bw + az*bx;
    out(3,0) = aw*bz + ax*by - ay*bx + az*bw;
}

template <typename T>
inline void quat_to_rotm(const cmath_fx::Vector<4, T>& q, cmath_fx::Matrix<3, 3, T>& R) {
    T qw=q(0,0), qx=q(1,0), qy=q(2,0), qz=q(3,0);
    R(0,0) = static_cast<T>(1.0) - static_cast<T>(2.0)*(qy*qy + qz*qz);
    R(0,1) = static_cast<T>(2.0)*(qx*qy - qz*qw);
    R(0,2) = static_cast<T>(2.0)*(qx*qz + qy*qw);
    R(1,0) = static_cast<T>(2.0)*(qx*qy + qz*qw);
    R(1,1) = static_cast<T>(1.0) - static_cast<T>(2.0)*(qx*qx + qz*qz);
    R(1,2) = static_cast<T>(2.0)*(qy*qz - qx*qw);
    R(2,0) = static_cast<T>(2.0)*(qx*qz - qy*qw);
    R(2,1) = static_cast<T>(2.0)*(qy*qz + qx*qw);
    R(2,2) = static_cast<T>(1.0) - static_cast<T>(2.0)*(qx*qx + qy*qy);
}

template <typename T>
inline void from_euler_deg(T roll_deg, T pitch_deg, T yaw_deg, cmath_fx::Vector<4, T>& q_out) {
    T roll = roll_deg * static_cast<T>(common::math::PI) / static_cast<T>(180.0);
    T pitch = pitch_deg * static_cast<T>(common::math::PI) / static_cast<T>(180.0);
    T yaw = yaw_deg * static_cast<T>(common::math::PI) / static_cast<T>(180.0);
    T cy = std::cos(yaw * static_cast<T>(0.5));
    T sy = std::sin(yaw * static_cast<T>(0.5));
    T cp = std::cos(pitch * static_cast<T>(0.5));
    T sp = std::sin(pitch * static_cast<T>(0.5));
    T cr = std::cos(roll * static_cast<T>(0.5));
    T sr = std::sin(roll * static_cast<T>(0.5));
    T qw = cr*cp*cy + sr*sp*sy;
    T qx = sr*cp*cy - cr*sp*sy;
    T qy = cr*sp*cy + sr*cp*sy;
    T qz = cr*cp*sy - sr*sp*cy;
    q_out(0,0)=qw; q_out(1,0)=qx; q_out(2,0)=qy; q_out(3,0)=qz;
    normalize_quat(q_out);
}

template <typename T>
inline void to_euler_deg(const cmath_fx::Vector<4, T>& q, cmath_fx::Vector<3, T>& euler_deg) {
    T qw=q(0,0), qx=q(1,0), qy=q(2,0), qz=q(3,0);
    T sinr_cosp = static_cast<T>(2.0) * (qw * qx + qy * qz);
    T cosr_cosp = static_cast<T>(1.0) - static_cast<T>(2.0) * (qx*qx + qy*qy);
    T roll = std::atan2(sinr_cosp, cosr_cosp);
    T sinp = static_cast<T>(2.0) * (qw * qy - qz * qx);
    T pitch;
    if (std::abs(sinp) >= static_cast<T>(1.0))
        pitch = std::copysign(static_cast<T>(common::math::PI)/static_cast<T>(2.0), sinp);
    else
        pitch = std::asin(sinp);
    T siny_cosp = static_cast<T>(2.0) * (qw * qz + qx * qy);
    T cosy_cosp = static_cast<T>(1.0) - static_cast<T>(2.0) * (qy*qy + qz*qz);
    T yaw = std::atan2(siny_cosp, cosy_cosp);
    euler_deg(0,0) = roll * static_cast<T>(180.0) / static_cast<T>(common::math::PI);
    euler_deg(1,0) = pitch * static_cast<T>(180.0) / static_cast<T>(common::math::PI);
    euler_deg(2,0) = yaw * static_cast<T>(180.0) / static_cast<T>(common::math::PI);
}

// Convert quaternion to Euler angles in degrees (value API)
template <typename T>
inline void to_euler_deg(const cmath_fx::Vector<4, T>& q, T& roll_deg, T& pitch_deg, T& yaw_deg) {
    T qw=q(0,0), qx=q(1,0), qy=q(2,0), qz=q(3,0);
    T sinr_cosp = static_cast<T>(2.0) * (qw * qx + qy * qz);
    T cosr_cosp = static_cast<T>(1.0) - static_cast<T>(2.0) * (qx*qx + qy*qy);
    T roll = std::atan2(sinr_cosp, cosr_cosp);
    T sinp = static_cast<T>(2.0) * (qw * qy - qz * qx);
    T pitch;
    if (std::abs(sinp) >= static_cast<T>(1.0))
        pitch = std::copysign(static_cast<T>(common::math::PI)/static_cast<T>(2.0), sinp);
    else
        pitch = std::asin(sinp);
    T siny_cosp = static_cast<T>(2.0) * (qw * qz + qx * qy);
    T cosy_cosp = static_cast<T>(1.0) - static_cast<T>(2.0) * (qy*qy + qz*qz);
    T yaw = std::atan2(siny_cosp, cosy_cosp);
    roll_deg = roll * static_cast<T>(180.0) / static_cast<T>(common::math::PI);
    pitch_deg = pitch * static_cast<T>(180.0) / static_cast<T>(common::math::PI);
    yaw_deg = yaw * static_cast<T>(180.0) / static_cast<T>(common::math::PI);
}

// Quaternion from small-angle approximation
template <typename T>
inline void from_small_angle(T theta_x, T theta_y, T theta_z, cmath_fx::Vector<4, T>& q_out) {
    T th2 = theta_x*theta_x + theta_y*theta_y + theta_z*theta_z;
    const T EPS = quat_eps<T>();
    
    if (th2 < EPS*EPS) {
        q_out(0,0) = static_cast<T>(1.0);
        q_out(1,0) = static_cast<T>(0.5) * theta_x;
        q_out(2,0) = static_cast<T>(0.5) * theta_y;
        q_out(3,0) = static_cast<T>(0.5) * theta_z;
    } else {
        T angle = std::sqrt(th2);
        T half_angle = angle * static_cast<T>(0.5);
        T s = std::sin(half_angle) / angle;
        q_out(0,0) = std::cos(half_angle);
        q_out(1,0) = theta_x * s;
        q_out(2,0) = theta_y * s;
        q_out(3,0) = theta_z * s;
    }
    normalize_quat(q_out);
}

// Convert quaternion to rotation matrix (row-major array)
template <typename T>
inline void quat_to_rotm_array(const cmath_fx::Vector<4, T>& q, T R[9]) {
    T qw=q(0,0), qx=q(1,0), qy=q(2,0), qz=q(3,0);
    
    R[0] = static_cast<T>(1.0) - static_cast<T>(2.0)*(qy*qy + qz*qz);
    R[1] = static_cast<T>(2.0)*(qx*qy - qz*qw);
    R[2] = static_cast<T>(2.0)*(qx*qz + qy*qw);
    
    R[3] = static_cast<T>(2.0)*(qx*qy + qz*qw);
    R[4] = static_cast<T>(1.0) - static_cast<T>(2.0)*(qx*qx + qz*qz);
    R[5] = static_cast<T>(2.0)*(qy*qz - qx*qw);
    
    R[6] = static_cast<T>(2.0)*(qx*qz - qy*qw);
    R[7] = static_cast<T>(2.0)*(qy*qz + qx*qw);
    R[8] = static_cast<T>(1.0) - static_cast<T>(2.0)*(qx*qx + qy*qy);
    
    // Cleanup small numerical noise: snap near-zero to 0 and near-one to ±1
    const T EPS = quat_eps<T>();
    for (int i = 0; i < 9; ++i) {
        if (std::abs(R[i]) < EPS) {
            R[i] = static_cast<T>(0.0);
        } else if (std::abs(R[i] - static_cast<T>(1.0)) < EPS) {
            R[i] = (R[i] < static_cast<T>(0.0)) ? static_cast<T>(-1.0) : static_cast<T>(1.0);
        }
    }
}

/**
 * 四元数の時間積分（角速度ベクトルによる更新）
 * @param q_in 入力四元数 [w, x, y, z]
 * @param w 角速度ベクトル [rad/s]
 * @param dt 時間ステップ [s]
 * @param q_out 出力四元数（正規化済み）
 */
template <typename T>
inline void quaternion_integration(
    const cmath_fx::Vector<4, T>& q_in,
    const cmath_fx::Vector<3, T>& w,
    T dt,
    cmath_fx::Vector<4, T>& q_out
) {
    // w_half = w * dt/2
    cmath_fx::Vector<3, T> w_half = w;
    for (int i = 0; i < 3; ++i) {
        w_half(i, 0) *= static_cast<T>(0.5) * dt;
    }
    
    // dq = 0.5 * q * [0; w_half]
    cmath_fx::Vector<4, T> dq;
    dq(0, 0) = static_cast<T>(-0.5) * (q_in(1, 0) * w_half(0, 0) + 
                                        q_in(2, 0) * w_half(1, 0) + 
                                        q_in(3, 0) * w_half(2, 0));
    dq(1, 0) = static_cast<T>(0.5) * (q_in(0, 0) * w_half(0, 0) + 
                                       q_in(2, 0) * w_half(2, 0) - 
                                       q_in(3, 0) * w_half(1, 0));
    dq(2, 0) = static_cast<T>(0.5) * (q_in(0, 0) * w_half(1, 0) - 
                                       q_in(1, 0) * w_half(2, 0) + 
                                       q_in(3, 0) * w_half(0, 0));
    dq(3, 0) = static_cast<T>(0.5) * (q_in(0, 0) * w_half(2, 0) + 
                                       q_in(1, 0) * w_half(1, 0) - 
                                       q_in(2, 0) * w_half(0, 0));
    
    // q_out = q_in + dq
    for (int i = 0; i < 4; ++i) {
        q_out(i, 0) = q_in(i, 0) + dq(i, 0);
    }
    
    // 正規化
    normalize_quat(q_out);
}

/**
 * 四元数の共役（逆回転）
 * @param q 入力四元数 [w, x, y, z]
 * @param q_conj 出力：共役四元数 [w, -x, -y, -z]
 */
template <typename T>
inline void conjugate_quat(
    const cmath_fx::Vector<4, T>& q,
    cmath_fx::Vector<4, T>& q_conj
) {
    q_conj(0, 0) = q(0, 0);
    q_conj(1, 0) = -q(1, 0);
    q_conj(2, 0) = -q(2, 0);
    q_conj(3, 0) = -q(3, 0);
}

/**
 * 四元数による3Dベクトルの回転
 * @param q 回転四元数 [w, x, y, z]
 * @param v 入力ベクトル
 * @param v_rot 出力：回転後のベクトル
 */
template <typename T>
inline void rotate_vector_by_quat(
    const cmath_fx::Vector<4, T>& q,
    const cmath_fx::Vector<3, T>& v,
    cmath_fx::Vector<3, T>& v_rot
) {
    // v_rot = q * [0; v] * q^-1
    cmath_fx::Vector<4, T> v_quat;
    v_quat(0, 0) = static_cast<T>(0.0);
    v_quat(1, 0) = v(0, 0);
    v_quat(2, 0) = v(1, 0);
    v_quat(3, 0) = v(2, 0);
    
    cmath_fx::Vector<4, T> q_conj;
    conjugate_quat(q, q_conj);
    
    cmath_fx::Vector<4, T> temp;
    multiply_quat(q, v_quat, temp);
    
    cmath_fx::Vector<4, T> result;
    multiply_quat(temp, q_conj, result);
    
    v_rot(0, 0) = result(1, 0);
    v_rot(1, 0) = result(2, 0);
    v_rot(2, 0) = result(3, 0);
}

} // namespace cquat



#endif // LIB_QUATERNION_QUATERNION_FUNCTIONS_HPP
