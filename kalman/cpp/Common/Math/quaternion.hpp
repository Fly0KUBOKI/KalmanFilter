#pragma once
#include "fixed_matrix.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace cquat {

using cm = cmath_fx::FixedMatrix;

// Quat as [w, x, y, z] stored in cm (4x1)
inline void normalize_quat(cm& q) {
    float n = 0.0f;
    for (int i = 0; i < 4; ++i) n += q(i,0) * q(i,0);
    n = std::sqrt(n);
    if (n < 1e-12f) { q(0,0)=1; q(1,0)=0; q(2,0)=0; q(3,0)=0; return; }
    for (int i = 0; i < 4; ++i) q(i,0) /= n;
}

inline void multiply_quat(const cm& a, const cm& b, cm& out) {
    // a,b: 4x1
    out.resize(4,1);
    float aw=a(0,0), ax=a(1,0), ay=a(2,0), az=a(3,0);
    float bw=b(0,0), bx=b(1,0), by=b(2,0), bz=b(3,0);
    out(0,0) = aw*bw - ax*bx - ay*by - az*bz;
    out(1,0) = aw*bx + ax*bw + ay*bz - az*by;
    out(2,0) = aw*by - ax*bz + ay*bw + az*bx;
    out(3,0) = aw*bz + ax*by - ay*bx + az*bw;
}

inline void quat_to_rotm(const cm& q, cmath_fx::FixedMatrix& R) {
    // R 3x3
    R.resize(3,3);
    float qw=q(0,0), qx=q(1,0), qy=q(2,0), qz=q(3,0);
    R(0,0) = 1 - 2*(qy*qy + qz*qz);
    R(0,1) = 2*(qx*qy - qz*qw);
    R(0,2) = 2*(qx*qz + qy*qw);
    R(1,0) = 2*(qx*qy + qz*qw);
    R(1,1) = 1 - 2*(qx*qx + qz*qz);
    R(1,2) = 2*(qy*qz - qx*qw);
    R(2,0) = 2*(qx*qz - qy*qw);
    R(2,1) = 2*(qy*qz + qx*qw);
    R(2,2) = 1 - 2*(qx*qx + qy*qy);
}

inline void from_euler_deg(float roll_deg, float pitch_deg, float yaw_deg, cm& q_out) {
    float roll = roll_deg * static_cast<float>(M_PI) / 180.0f;
    float pitch = pitch_deg * static_cast<float>(M_PI) / 180.0f;
    float yaw = yaw_deg * static_cast<float>(M_PI) / 180.0f;
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float qw = cr*cp*cy + sr*sp*sy;
    float qx = sr*cp*cy - cr*sp*sy;
    float qy = cr*sp*cy + sr*cp*sy;
    float qz = cr*cp*sy - sr*sp*cy;
    q_out.resize(4,1);
    q_out(0,0)=qw; q_out(1,0)=qx; q_out(2,0)=qy; q_out(3,0)=qz;
    normalize_quat(q_out);
}

inline void to_euler_deg(const cm& q, cmath_fx::FixedMatrix& euler_deg) {
    // returns roll,pitch,yaw in degrees as 3x1
    float qw=q(0,0), qx=q(1,0), qy=q(2,0), qz=q(3,0);
    float sinr_cosp = 2.f * (qw * qx + qy * qz);
    float cosr_cosp = 1.f - 2.f * (qx*qx + qy*qy);
    float roll = std::atan2f(sinr_cosp, cosr_cosp);
    float sinp = 2.f * (qw * qy - qz * qx);
    float pitch;
    if (std::abs(sinp) >= 1.f)
        pitch = std::copysignf(static_cast<float>(M_PI)/2.0f, sinp);
    else
        pitch = asinf(sinp);
    float siny_cosp = 2.f * (qw * qz + qx * qy);
    float cosy_cosp = 1.f - 2.f * (qy*qy + qz*qz);
    float yaw = std::atan2f(siny_cosp, cosy_cosp);
    euler_deg.resize(3,1);
    euler_deg(0,0) = roll * 180.0f / static_cast<float>(M_PI);
    euler_deg(1,0) = pitch * 180.0f / static_cast<float>(M_PI);
    euler_deg(2,0) = yaw * 180.0f / static_cast<float>(M_PI);
}

} // namespace cquat
