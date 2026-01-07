#pragma once
// Moved from Inc/Common/Math/quaternion.hpp to Lib/Quaternion/quaternion_functions.hpp
#include "../Matrix/fixed_matrix.hpp"
#include <cmath>
#include "../Common/inc/Math/matrix_operations.hpp"
#include "../Common/inc/Math/statistics.hpp"
#include "../Common/inc/Math/geometry.hpp"
#include "../Common/inc/Math/numerical.hpp"
#include "../Common/inc/Math/math_utils.hpp"

namespace cquat {

// Quat as [w, x, y, z]
template <typename T>
inline void normalize_quat(cmath_fx::Vector<4, T>& q) {
    T n = 0.0;
    for (int i = 0; i < 4; ++i) n += q(i,0) * q(i,0);
    n = std::sqrt(n);
    if (n < static_cast<T>(1e-12)) { 
        q(0,0)=static_cast<T>(1.0); 
        q(1,0)=static_cast<T>(0.0); 
        q(2,0)=static_cast<T>(0.0); 
        q(3,0)=static_cast<T>(0.0); 
        return; 
    }
    for (int i = 0; i < 4; ++i) q(i,0) /= n;
}

// Wrapper returning a normalized quaternion (value API) for callers using Vec4 return
template <typename T>
inline cmath_fx::Vector<4, T> normalize_quaternion(const cmath_fx::Vector<4, T>& q_in) {
    cmath_fx::Vector<4, T> q = q_in;
    normalize_quat(q);
    return q;
}
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
    T roll = roll_deg * static_cast<T>(common::math::PI_CONST) / static_cast<T>(180.0);
    T pitch = pitch_deg * static_cast<T>(common::math::PI_CONST) / static_cast<T>(180.0);
    T yaw = yaw_deg * static_cast<T>(common::math::PI_CONST) / static_cast<T>(180.0);
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
        pitch = std::copysign(static_cast<T>(common::math::PI_CONST)/static_cast<T>(2.0), sinp);
    else
        pitch = std::asin(sinp);
    T siny_cosp = static_cast<T>(2.0) * (qw * qz + qx * qy);
    T cosy_cosp = static_cast<T>(1.0) - static_cast<T>(2.0) * (qy*qy + qz*qz);
    T yaw = std::atan2(siny_cosp, cosy_cosp);
    euler_deg(0,0) = roll * static_cast<T>(180.0) / static_cast<T>(common::math::PI_CONST);
    euler_deg(1,0) = pitch * static_cast<T>(180.0) / static_cast<T>(common::math::PI_CONST);
    euler_deg(2,0) = yaw * static_cast<T>(180.0) / static_cast<T>(common::math::PI_CONST);
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
        pitch = std::copysign(static_cast<T>(common::math::PI_CONST)/static_cast<T>(2.0), sinp);
    else
        pitch = std::asin(sinp);
    T siny_cosp = static_cast<T>(2.0) * (qw * qz + qx * qy);
    T cosy_cosp = static_cast<T>(1.0) - static_cast<T>(2.0) * (qy*qy + qz*qz);
    T yaw = std::atan2(siny_cosp, cosy_cosp);
    roll_deg = roll * static_cast<T>(180.0) / static_cast<T>(common::math::PI_CONST);
    pitch_deg = pitch * static_cast<T>(180.0) / static_cast<T>(common::math::PI_CONST);
    yaw_deg = yaw * static_cast<T>(180.0) / static_cast<T>(common::math::PI_CONST);
}

// Quaternion from small-angle approximation
template <typename T>
inline void from_small_angle(T theta_x, T theta_y, T theta_z, cmath_fx::Vector<4, T>& q_out) {
    T th2 = theta_x*theta_x + theta_y*theta_y + theta_z*theta_z;
    constexpr T EPS = static_cast<T>(1e-9);
    
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
    constexpr T EPS = static_cast<T>(1e-9);
    for (int i = 0; i < 9; ++i) {
        if (std::abs(R[i]) < EPS) {
            R[i] = static_cast<T>(0.0);
        } else if (std::abs(R[i] - static_cast<T>(1.0)) < EPS) {
            R[i] = (R[i] < static_cast<T>(0.0)) ? static_cast<T>(-1.0) : static_cast<T>(1.0);
        }
    }
}

} // namespace cquat


