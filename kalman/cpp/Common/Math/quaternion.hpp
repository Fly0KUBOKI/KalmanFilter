#pragma once
#include "fixed_matrix.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace cquat {

using Vector4 = cmath_fx::Vector<4, float>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, float>;
using Vector3 = cmath_fx::Vector<3, float>;

// Quat as [w, x, y, z]
inline void normalize_quat(Vector4& q) {
    float n = 0.0f;
    for (int i = 0; i < 4; ++i) n += q(i,0) * q(i,0);
    n = std::sqrt(n);
    if (n < 1e-12f) { q(0,0)=1.0f; q(1,0)=0.0f; q(2,0)=0.0f; q(3,0)=0.0f; return; }
    for (int i = 0; i < 4; ++i) q(i,0) /= n;
}

inline void multiply_quat(const Vector4& a, const Vector4& b, Vector4& out) {
    float aw=a(0,0), ax=a(1,0), ay=a(2,0), az=a(3,0);
    float bw=b(0,0), bx=b(1,0), by=b(2,0), bz=b(3,0);
    out(0,0) = aw*bw - ax*bx - ay*by - az*bz;
    out(1,0) = aw*bx + ax*bw + ay*bz - az*by;
    out(2,0) = aw*by - ax*bz + ay*bw + az*bx;
    out(3,0) = aw*bz + ax*by - ay*bx + az*bw;
}

inline void quat_to_rotm(const Vector4& q, Matrix3x3& R) {
    float qw=q(0,0), qx=q(1,0), qy=q(2,0), qz=q(3,0);
    R(0,0) = 1.0f - 2.0f*(qy*qy + qz*qz);
    R(0,1) = 2.0f*(qx*qy - qz*qw);
    R(0,2) = 2.0f*(qx*qz + qy*qw);
    R(1,0) = 2.0f*(qx*qy + qz*qw);
    R(1,1) = 1.0f - 2.0f*(qx*qx + qz*qz);
    R(1,2) = 2.0f*(qy*qz - qx*qw);
    R(2,0) = 2.0f*(qx*qz - qy*qw);
    R(2,1) = 2.0f*(qy*qz + qx*qw);
    R(2,2) = 1.0f - 2.0f*(qx*qx + qy*qy);
}

inline void from_euler_deg(float roll_deg, float pitch_deg, float yaw_deg, Vector4& q_out) {
    float roll = roll_deg * static_cast<float>(M_PI) / 180.0f;
    float pitch = pitch_deg * static_cast<float>(M_PI) / 180.0f;
    float yaw = yaw_deg * static_cast<float>(M_PI) / 180.0f;
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    float qw = cr*cp*cy + sr*sp*sy;
    float qx = sr*cp*cy - cr*sp*sy;
    float qy = cr*sp*cy + sr*cp*sy;
    float qz = cr*cp*sy - sr*sp*cy;
    q_out(0,0)=qw; q_out(1,0)=qx; q_out(2,0)=qy; q_out(3,0)=qz;
    normalize_quat(q_out);
}

inline void to_euler_deg(const Vector4& q, Vector3& euler_deg) {
    float qw=q(0,0), qx=q(1,0), qy=q(2,0), qz=q(3,0);
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx*qx + qy*qy);
    float roll = std::atan2(sinr_cosp, cosr_cosp);
    float sinp = 2.0f * (qw * qy - qz * qx);
    float pitch;
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(static_cast<float>(M_PI)/2.0f, sinp);
    else
        pitch = std::asin(sinp);
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz);
    float yaw = std::atan2(siny_cosp, cosy_cosp);
    euler_deg(0,0) = roll * 180.0f / static_cast<float>(M_PI);
    euler_deg(1,0) = pitch * 180.0f / static_cast<float>(M_PI);
    euler_deg(2,0) = yaw * 180.0f / static_cast<float>(M_PI);
}

// Template overloads for double precision (used by MEUKF with double types)
template <typename T>
inline void normalize_quat(cmath_fx::Matrix<4, 1, T>& q) {
    T n = 0;
    for (int i = 0; i < 4; ++i) n += q(i,0) * q(i,0);
    n = std::sqrt(n);
    if (n < static_cast<T>(1e-12)) { 
        q(0,0)=static_cast<T>(1); q(1,0)=static_cast<T>(0); 
        q(2,0)=static_cast<T>(0); q(3,0)=static_cast<T>(0); 
        return; 
    }
    for (int i = 0; i < 4; ++i) q(i,0) /= n;
}

template <typename T>
inline void multiply_quat(const cmath_fx::Matrix<4, 1, T>& a, const cmath_fx::Matrix<4, 1, T>& b, cmath_fx::Matrix<4, 1, T>& out) {
    T aw=a(0,0), ax=a(1,0), ay=a(2,0), az=a(3,0);
    T bw=b(0,0), bx=b(1,0), by=b(2,0), bz=b(3,0);
    out(0,0) = aw*bw - ax*bx - ay*by - az*bz;
    out(1,0) = aw*bx + ax*bw + ay*bz - az*by;
    out(2,0) = aw*by - ax*bz + ay*bw + az*bx;
    out(3,0) = aw*bz + ax*by - ay*bx + az*bw;
}

template <typename T>
inline void quat_to_rotm(const cmath_fx::Matrix<4, 1, T>& q, cmath_fx::Matrix<3, 3, T>& R) {
    T qw=q(0,0), qx=q(1,0), qy=q(2,0), qz=q(3,0);
    R(0,0) = static_cast<T>(1) - static_cast<T>(2)*(qy*qy + qz*qz);
    R(0,1) = static_cast<T>(2)*(qx*qy - qz*qw);
    R(0,2) = static_cast<T>(2)*(qx*qz + qy*qw);
    R(1,0) = static_cast<T>(2)*(qx*qy + qz*qw);
    R(1,1) = static_cast<T>(1) - static_cast<T>(2)*(qx*qx + qz*qz);
    R(1,2) = static_cast<T>(2)*(qy*qz - qx*qw);
    R(2,0) = static_cast<T>(2)*(qx*qz - qy*qw);
    R(2,1) = static_cast<T>(2)*(qy*qz + qx*qw);
    R(2,2) = static_cast<T>(1) - static_cast<T>(2)*(qx*qx + qy*qy);
}

} // namespace cquat
