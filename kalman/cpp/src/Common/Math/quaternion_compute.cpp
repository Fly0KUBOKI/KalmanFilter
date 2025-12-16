#include "Common/Math/quaternion_compute.hpp"

namespace kalman_compute {

void QuaternionCompute::multiply(const Scalar* input, Scalar* output) {
    // input: [q1(4); q2(4)]
    // output: q_result(4)
    
    const Scalar q1w = input[0];
    const Scalar q1x = input[1];
    const Scalar q1y = input[2];
    const Scalar q1z = input[3];
    
    const Scalar q2w = input[4];
    const Scalar q2x = input[5];
    const Scalar q2y = input[6];
    const Scalar q2z = input[7];
    
    // Hamilton product: q1 * q2
    output[0] = q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z;  // w
    output[1] = q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y;  // x
    output[2] = q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x;  // y
    output[3] = q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w;  // z
}

void QuaternionCompute::normalize(const Scalar* input, Scalar* output) {
    // input: q(4)
    // output: q_normalized(4)
    
    const Scalar w = input[0];
    const Scalar x = input[1];
    const Scalar y = input[2];
    const Scalar z = input[3];
    
    const Scalar norm = safe_sqrt(w*w + x*x + y*y + z*z);
    
    if (norm < EPS) {
        // Identity quaternion
        output[0] = 1.0f;
        output[1] = 0.0f;
        output[2] = 0.0f;
        output[3] = 0.0f;
    } else {
        const Scalar inv_norm = 1.0f / norm;
        output[0] = w * inv_norm;
        output[1] = x * inv_norm;
        output[2] = y * inv_norm;
        output[3] = z * inv_norm;
        
        // ゼロに近い値をクリア
        for (uint8_t i = 0; i < 4; ++i) {
            if (std::abs(output[i]) < EPS) {
                output[i] = 0.0f;
            }
        }
    }
}

void QuaternionCompute::conjugate(const Scalar* input, Scalar* output) {
    // input: q(4)
    // output: q_conjugate(4) = [w, -x, -y, -z]
    
    output[0] =  input[0];
    output[1] = -input[1];
    output[2] = -input[2];
    output[3] = -input[3];
}

void QuaternionCompute::inverse(const Scalar* input, Scalar* output) {
    // input: q(4)
    // output: q_inverse(4)
    
    const Scalar w = input[0];
    const Scalar x = input[1];
    const Scalar y = input[2];
    const Scalar z = input[3];
    
    const Scalar norm_sq = w*w + x*x + y*y + z*z;
    
    if (norm_sq < EPS) {
        // Return identity
        output[0] = 1.0f;
        output[1] = 0.0f;
        output[2] = 0.0f;
        output[3] = 0.0f;
    } else {
        const Scalar inv = 1.0f / norm_sq;
        output[0] =  w * inv;
        output[1] = -x * inv;
        output[2] = -y * inv;
        output[3] = -z * inv;
    }
}

void QuaternionCompute::to_rotation_matrix(const Scalar* input, Scalar* output) {
    // input: q(4)
    // output: R(9) - row-major
    
    // Normalize first
    Scalar q[4];
    normalize(input, q);
    
    const Scalar qw = q[0];
    const Scalar qx = q[1];
    const Scalar qy = q[2];
    const Scalar qz = q[3];
    
    // Row 1
    output[0] = 1.0f - 2.0f*(qy*qy + qz*qz);
    output[1] = 2.0f*(qx*qy - qz*qw);
    output[2] = 2.0f*(qx*qz + qy*qw);
    
    // Row 2
    output[3] = 2.0f*(qx*qy + qz*qw);
    output[4] = 1.0f - 2.0f*(qx*qx + qz*qz);
    output[5] = 2.0f*(qy*qz - qx*qw);
    
    // Row 3
    output[6] = 2.0f*(qx*qz - qy*qw);
    output[7] = 2.0f*(qy*qz + qx*qw);
    output[8] = 1.0f - 2.0f*(qx*qx + qy*qy);
    
    // Clean up small values
    for (uint8_t i = 0; i < 9; ++i) {
        if (std::abs(output[i]) < EPS) {
            output[i] = 0.0f;
        } else if (std::abs(output[i] - 1.0f) < EPS) {
            output[i] = 1.0f;
        } else if (std::abs(output[i] + 1.0f) < EPS) {
            output[i] = -1.0f;
        }
    }
}

void QuaternionCompute::to_euler(const Scalar* input, Scalar* output) {
    // input: q(4)
    // output: euler(3) - [roll, pitch, yaw] in degrees
    
    // Normalize first
    Scalar q[4];
    normalize(input, q);
    
    const Scalar qw = q[0];
    const Scalar qx = q[1];
    const Scalar qy = q[2];
    const Scalar qz = q[3];
    
    // Roll (x-axis rotation)
    const Scalar sinr_cosp = 2.0f * (qw*qx + qy*qz);
    const Scalar cosr_cosp = 1.0f - 2.0f * (qx*qx + qy*qy);
    const Scalar roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    const Scalar sinp = 2.0f * (qw*qy - qz*qx);
    Scalar pitch;
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    } else {
        pitch = safe_asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    const Scalar siny_cosp = 2.0f * (qw*qz + qx*qy);
    const Scalar cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz);
    const Scalar yaw = std::atan2(siny_cosp, cosy_cosp);
    
    // Convert to degrees
    output[0] = roll  * 180.0f / M_PI;
    output[1] = pitch * 180.0f / M_PI;
    output[2] = yaw   * 180.0f / M_PI;
}

void QuaternionCompute::from_euler(const Scalar* input, Scalar* output) {
    // input: euler(3) - [roll, pitch, yaw] in degrees
    // output: q(4)
    
    // Convert to radians
    const Scalar roll  = input[0] * M_PI / 180.0f;
    const Scalar pitch = input[1] * M_PI / 180.0f;
    const Scalar yaw   = input[2] * M_PI / 180.0f;
    
    const Scalar cy = std::cos(yaw   * 0.5f);
    const Scalar sy = std::sin(yaw   * 0.5f);
    const Scalar cp = std::cos(pitch * 0.5f);
    const Scalar sp = std::sin(pitch * 0.5f);
    const Scalar cr = std::cos(roll  * 0.5f);
    const Scalar sr = std::sin(roll  * 0.5f);
    
    output[0] = cr*cp*cy + sr*sp*sy;  // w
    output[1] = sr*cp*cy - cr*sp*sy;  // x
    output[2] = cr*sp*cy + sr*cp*sy;  // y
    output[3] = cr*cp*sy - sr*sp*cy;  // z
    
    // Normalize
    normalize(output, output);
}

void QuaternionCompute::from_small_angle(const Scalar* input, Scalar* output) {
    // input: theta(3) - [theta_x, theta_y, theta_z] in radians
    // output: q(4)
    
    const Scalar theta_x = input[0];
    const Scalar theta_y = input[1];
    const Scalar theta_z = input[2];
    
    const Scalar theta_sq = theta_x*theta_x + theta_y*theta_y + theta_z*theta_z;
    
    if (theta_sq < EPS*EPS) {
        // Small angle approximation
        output[0] = 1.0f;
        output[1] = 0.5f * theta_x;
        output[2] = 0.5f * theta_y;
        output[3] = 0.5f * theta_z;
    } else {
        const Scalar theta = safe_sqrt(theta_sq);
        const Scalar half_theta = theta * 0.5f;
        const Scalar s = std::sin(half_theta) / theta;
        
        output[0] = std::cos(half_theta);
        output[1] = theta_x * s;
        output[2] = theta_y * s;
        output[3] = theta_z * s;
    }
    
    // Normalize
    normalize(output, output);
}

void QuaternionCompute::integrate(const Scalar* input, Scalar* output) {
    // input: [q(4); omega(3); dt(1)] -> 8x1
    // output: q_new(4)
    
    const Scalar dt = input[7];
    
    // omega * dt
    const Scalar w_dt[3] = {
        input[4] * dt,
        input[5] * dt,
        input[6] * dt
    };
    
    const Scalar w_dt_norm_sq = w_dt[0]*w_dt[0] + w_dt[1]*w_dt[1] + w_dt[2]*w_dt[2];
    
    if (w_dt_norm_sq < EPS*EPS) {
        // No rotation, copy input quaternion
        output[0] = input[0];
        output[1] = input[1];
        output[2] = input[2];
        output[3] = input[3];
        return;
    }
    
    const Scalar w_dt_norm = safe_sqrt(w_dt_norm_sq);
    const Scalar half_angle = w_dt_norm * 0.5f;
    
    // delta_q from angle-axis
    Scalar delta_q[4];
    if (half_angle > 1e-6f) {
        const Scalar sin_half = std::sin(half_angle);
        const Scalar cos_half = std::cos(half_angle);
        const Scalar inv_norm = 1.0f / w_dt_norm;
        
        delta_q[0] = cos_half;
        delta_q[1] = w_dt[0] * inv_norm * sin_half;
        delta_q[2] = w_dt[1] * inv_norm * sin_half;
        delta_q[3] = w_dt[2] * inv_norm * sin_half;
    } else {
        // Taylor expansion for small angles
        const Scalar w_sq = w_dt_norm_sq;
        delta_q[0] = 1.0f - w_sq / 8.0f;
        delta_q[1] = w_dt[0] * 0.5f * (1.0f - w_sq / 24.0f);
        delta_q[2] = w_dt[1] * 0.5f * (1.0f - w_sq / 24.0f);
        delta_q[3] = w_dt[2] * 0.5f * (1.0f - w_sq / 24.0f);
    }
    
    // q_new = q * delta_q
    Scalar temp_input[8];
    for (uint8_t i = 0; i < 4; ++i) {
        temp_input[i] = input[i];      // q
        temp_input[i+4] = delta_q[i];  // delta_q
    }
    
    multiply(temp_input, output);
    normalize(output, output);
}

void QuaternionCompute::angle_between(const Scalar* input, Scalar* output) {
    // input: [q1(4); q2(4)] -> 8x1
    // output: angle(1) in degrees
    
    // Normalize both quaternions
    Scalar q1[4], q2[4];
    normalize(input, q1);
    normalize(input + 4, q2);
    
    // Compute dot product
    Scalar dot_prod = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    
    // Clamp to [-1, 1]
    dot_prod = std::max(-1.0f, std::min(1.0f, dot_prod));
    
    // angle = 2 * acos(|dot|)
    const Scalar angle_rad = 2.0f * std::acos(std::abs(dot_prod));
    
    output[0] = angle_rad * 180.0f / M_PI;
}

void QuaternionCompute::slerp(const Scalar* input, Scalar* output) {
    // input: [q1(4); q2(4); t(1)] -> 9x1
    // output: q_interp(4)
    
    Scalar q1[4], q2[4];
    normalize(input, q1);
    normalize(input + 4, q2);
    
    const Scalar t = input[8];
    
    // Compute dot product
    Scalar dot_prod = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    
    // If negative, negate q2 to take shorter path
    if (dot_prod < 0.0f) {
        for (uint8_t i = 0; i < 4; ++i) {
            q2[i] = -q2[i];
        }
        dot_prod = -dot_prod;
    }
    
    // Clamp
    dot_prod = std::min(1.0f, dot_prod);
    
    const Scalar theta = std::acos(dot_prod);
    
    if (theta < EPS) {
        // Quaternions are very close, use linear interpolation
        for (uint8_t i = 0; i < 4; ++i) {
            output[i] = (1.0f - t) * q1[i] + t * q2[i];
        }
    } else {
        const Scalar sin_theta = std::sin(theta);
        const Scalar w1 = std::sin((1.0f - t) * theta) / sin_theta;
        const Scalar w2 = std::sin(t * theta) / sin_theta;
        
        for (uint8_t i = 0; i < 4; ++i) {
            output[i] = w1 * q1[i] + w2 * q2[i];
        }
    }
    
    normalize(output, output);
}

void QuaternionCompute::from_axis_angle(const Scalar* input, Scalar* output) {
    // input: [axis(3); angle(1)] -> 4x1, angle in radians
    // output: q(4)
    
    const Scalar angle = input[3];
    const Scalar half_angle = angle * 0.5f;
    
    // Normalize axis
    const Scalar axis_norm = safe_sqrt(input[0]*input[0] + input[1]*input[1] + input[2]*input[2]);
    
    if (axis_norm < EPS || std::abs(angle) < EPS) {
        // Identity quaternion
        output[0] = 1.0f;
        output[1] = 0.0f;
        output[2] = 0.0f;
        output[3] = 0.0f;
        return;
    }
    
    const Scalar inv_norm = 1.0f / axis_norm;
    const Scalar sin_half = std::sin(half_angle);
    
    output[0] = std::cos(half_angle);
    output[1] = input[0] * inv_norm * sin_half;
    output[2] = input[1] * inv_norm * sin_half;
    output[3] = input[2] * inv_norm * sin_half;
    
    normalize(output, output);
}

void QuaternionCompute::to_axis_angle(const Scalar* input, Scalar* output) {
    // input: q(4)
    // output: [axis(3); angle(1)] -> 4x1, angle in radians
    
    Scalar q[4];
    normalize(input, q);
    
    const Scalar qw = q[0];
    const Scalar qx = q[1];
    const Scalar qy = q[2];
    const Scalar qz = q[3];
    
    const Scalar angle = 2.0f * std::acos(std::max(-1.0f, std::min(1.0f, qw)));
    
    if (std::abs(angle) < EPS) {
        // No rotation
        output[0] = 1.0f;
        output[1] = 0.0f;
        output[2] = 0.0f;
        output[3] = 0.0f;
    } else {
        const Scalar sin_half = std::sin(angle * 0.5f);
        const Scalar inv_sin = 1.0f / sin_half;
        
        output[0] = qx * inv_sin;
        output[1] = qy * inv_sin;
        output[2] = qz * inv_sin;
        output[3] = angle;
    }
}

void QuaternionCompute::dot(const Scalar* input, Scalar* output) {
    // input: [q1(4); q2(4)] -> 8x1
    // output: dot(1)
    
    output[0] = input[0]*input[4] + input[1]*input[5] + input[2]*input[6] + input[3]*input[7];
}

} // namespace kalman_compute
