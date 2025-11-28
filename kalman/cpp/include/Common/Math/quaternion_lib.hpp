#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace quat_lib {

template<typename T = float>
class Quaternion {
public:
    static constexpr T EPS = static_cast<T>(1e-9);
    
    T w, x, y, z;
    
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(T w_, T x_, T y_, T z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    // 正規化
    Quaternion& normalize() {
        T n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n < EPS) {
            w = 1; x = 0; y = 0; z = 0;
        } else {
            T inv_n = T(1) / n;
            w *= inv_n;
            x *= inv_n;
            y *= inv_n;
            z *= inv_n;
            
            // ゼロに近い値をクリア
            if (std::abs(w) < EPS) w = 0;
            if (std::abs(x) < EPS) x = 0;
            if (std::abs(y) < EPS) y = 0;
            if (std::abs(z) < EPS) z = 0;
        }
        return *this;
    }
    
    // 共役
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    // 逆元
    Quaternion inverse() const {
        T norm_sq = w*w + x*x + y*y + z*z;
        T inv = T(1) / norm_sq;
        return Quaternion(w*inv, -x*inv, -y*inv, -z*inv);
    }
    
    // 回転行列へ変換 (3x3, row-major order)
    void to_rotation_matrix(T R[9]) const {
        T qw = w, qx = x, qy = y, qz = z;
        
        R[0] = 1 - 2*(qy*qy + qz*qz);
        R[1] = 2*(qx*qy - qz*qw);
        R[2] = 2*(qx*qz + qy*qw);
        
        R[3] = 2*(qx*qy + qz*qw);
        R[4] = 1 - 2*(qx*qx + qz*qz);
        R[5] = 2*(qy*qz - qx*qw);
        
        R[6] = 2*(qx*qz - qy*qw);
        R[7] = 2*(qy*qz + qx*qw);
        R[8] = 1 - 2*(qx*qx + qy*qy);
        
        // 小さい値をゼロに、1に近い値を±1に
        for (int i = 0; i < 9; ++i) {
            if (std::abs(R[i]) < EPS) {
                R[i] = 0;
            } else if (std::abs(R[i] - 1) < EPS) {
                R[i] = (R[i] < 0) ? -1 : 1;
            }
        }
    }
    
    // オイラー角へ変換 (ZYX順, 度数法)
    void to_euler(T& roll_deg, T& pitch_deg, T& yaw_deg) const {
        T qw = w, qx = x, qy = y, qz = z;
        
        // Roll (X軸)
        T sinr_cosp = 2 * (qw * qx + qy * qz);
        T cosr_cosp = 1 - 2 * (qx*qx + qy*qy);
        T roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (Y軸)
        T sinp = 2 * (qw * qy - qz * qx);
        T pitch;
        if (std::abs(sinp) >= 1) {
            pitch = (sinp < 0 ? -1 : 1) * static_cast<T>(M_PI / 2.0);
        } else {
            pitch = std::asin(sinp);
        }
        
        // Yaw (Z軸)
        T siny_cosp = 2 * (qw * qz + qx * qy);
        T cosy_cosp = 1 - 2 * (qy*qy + qz*qz);
        T yaw = std::atan2(siny_cosp, cosy_cosp);
        
        roll_deg = roll * static_cast<T>(180.0 / M_PI);
        pitch_deg = pitch * static_cast<T>(180.0 / M_PI);
        yaw_deg = yaw * static_cast<T>(180.0 / M_PI);
    }
    
    // オイラー角から生成 (度数法)
    static Quaternion from_euler(T roll_deg, T pitch_deg, T yaw_deg) {
        T roll = roll_deg * static_cast<T>(M_PI / 180.0);
        T pitch = pitch_deg * static_cast<T>(M_PI / 180.0);
        T yaw = yaw_deg * static_cast<T>(M_PI / 180.0);
        
        T cy = std::cos(yaw * static_cast<T>(0.5));
        T sy = std::sin(yaw * static_cast<T>(0.5));
        T cp = std::cos(pitch * static_cast<T>(0.5));
        T sp = std::sin(pitch * static_cast<T>(0.5));
        T cr = std::cos(roll * static_cast<T>(0.5));
        T sr = std::sin(roll * static_cast<T>(0.5));
        
        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        
        q.normalize();
        return q;
    }
    
    // 小角度回転ベクトルから生成
    static Quaternion from_small_angle(T theta_x, T theta_y, T theta_z) {
        T th2 = theta_x*theta_x + theta_y*theta_y + theta_z*theta_z;
        
        Quaternion q;
        if (th2 < EPS*EPS) {
            q.w = static_cast<T>(1);
            q.x = static_cast<T>(0.5) * theta_x;
            q.y = static_cast<T>(0.5) * theta_y;
            q.z = static_cast<T>(0.5) * theta_z;
        } else {
            T angle = std::sqrt(th2);
            T half_angle = angle * static_cast<T>(0.5);
            T s = std::sin(half_angle) / angle;
            
            q.w = std::cos(half_angle);
            q.x = theta_x * s;
            q.y = theta_y * s;
            q.z = theta_z * s;
        }
        
        q.normalize();
        return q;
    }
    
    // クォータニオン積
    static Quaternion multiply(const Quaternion& a, const Quaternion& b) {
        Quaternion result;
        
        result.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
        result.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
        result.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
        result.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
        
        return result;
    }
    
    // 角速度による積分
    static Quaternion integrate(const Quaternion& q, T omega_x, T omega_y, T omega_z, T dt) {
        T w_dt_x = omega_x * dt;
        T w_dt_y = omega_y * dt;
        T w_dt_z = omega_z * dt;
        T w_dt_norm = std::sqrt(w_dt_x*w_dt_x + w_dt_y*w_dt_y + w_dt_z*w_dt_z);
        
        constexpr T threshold = static_cast<T>(1e-15);
        if (w_dt_norm > threshold) {
            T half_angle = w_dt_norm * static_cast<T>(0.5);
            
            Quaternion delta_q;
            if (half_angle > static_cast<T>(1e-6)) {
                T sin_half = std::sin(half_angle);
                T cos_half = std::cos(half_angle);
                T inv_norm = sin_half / w_dt_norm;
                
                delta_q.w = cos_half;
                delta_q.x = w_dt_x * inv_norm;
                delta_q.y = w_dt_y * inv_norm;
                delta_q.z = w_dt_z * inv_norm;
            } else {
                // Taylor展開
                T w_norm_sq = w_dt_norm * w_dt_norm;
                delta_q.w = static_cast<T>(1.0) - w_norm_sq / static_cast<T>(8.0);
                T coeff = static_cast<T>(0.5) * (static_cast<T>(1.0) - w_norm_sq / static_cast<T>(24.0));
                delta_q.x = w_dt_x * coeff;
                delta_q.y = w_dt_y * coeff;
                delta_q.z = w_dt_z * coeff;
            }
            
            Quaternion result = multiply(q, delta_q);
            result.normalize();
            return result;
        } else {
            return q;
        }
    }
    
    // 2ベクトル間の回転
    static Quaternion from_two_vectors(T v1_x, T v1_y, T v1_z, T v2_x, T v2_y, T v2_z) {
        // 正規化
        T n1 = std::sqrt(v1_x*v1_x + v1_y*v1_y + v1_z*v1_z);
        T n2 = std::sqrt(v2_x*v2_x + v2_y*v2_y + v2_z*v2_z);
        
        if (n1 < EPS) {
            v1_x = static_cast<T>(1); v1_y = static_cast<T>(0); v1_z = static_cast<T>(0);
        } else {
            T inv = static_cast<T>(1) / n1;
            v1_x *= inv; v1_y *= inv; v1_z *= inv;
        }
        
        if (n2 < EPS) {
            v2_x = static_cast<T>(1); v2_y = static_cast<T>(0); v2_z = static_cast<T>(0);
        } else {
            T inv = static_cast<T>(1) / n2;
            v2_x *= inv; v2_y *= inv; v2_z *= inv;
        }
        
        // 内積と外積
        T dot_product = v1_x*v2_x + v1_y*v2_y + v1_z*v2_z;
        T cross_x = v1_y*v2_z - v1_z*v2_y;
        T cross_y = v1_z*v2_x - v1_x*v2_z;
        T cross_z = v1_x*v2_y - v1_y*v2_x;
        
        Quaternion q;
        q.w = static_cast<T>(1) + dot_product;
        q.x = cross_x;
        q.y = cross_y;
        q.z = cross_z;
        
        q.normalize();
        return q;
    }
    
    // 角度距離
    static T distance(const Quaternion& q1, const Quaternion& q2) {
        Quaternion a = q1; a.normalize();
        Quaternion b = q2; b.normalize();
        
        T dot_product = std::abs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z);
        dot_product = std::min(dot_product, T(1));
        
        return 2 * std::acos(dot_product);
    }
    
    // 球面線形補間
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, T t) {
        Quaternion a = q1; a.normalize();
        Quaternion b = q2; b.normalize();
        
        T dot_product = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
        
        // 最短経路選択
        if (dot_product < 0) {
            b.w = -b.w; b.x = -b.x; b.y = -b.y; b.z = -b.z;
            dot_product = -dot_product;
        }
        
        dot_product = std::min(dot_product, T(1));
        T theta = std::acos(dot_product);
        
        if (std::abs(theta) < EPS) {
            return a;
        }
        
        T sin_theta = std::sin(theta);
        T w1 = std::sin((1 - t) * theta) / sin_theta;
        T w2 = std::sin(t * theta) / sin_theta;
        
        Quaternion result;
        result.w = w1 * a.w + w2 * b.w;
        result.x = w1 * a.x + w2 * b.x;
        result.y = w1 * a.y + w2 * b.y;
        result.z = w1 * a.z + w2 * b.z;
        
        result.normalize();
        return result;
    }
    
    // 歪対称行列 (3x3, row-major)
    static void skew_symmetric(T vx, T vy, T vz, T S[9]) {
        S[0] = 0;    S[1] = -vz;  S[2] = vy;
        S[3] = vz;   S[4] = 0;    S[5] = -vx;
        S[6] = -vy;  S[7] = vx;   S[8] = 0;
    }
};

} // namespace quat_lib
