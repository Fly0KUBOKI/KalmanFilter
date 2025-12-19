#include "Common/Math/rotation_compute.hpp"

namespace kalman_compute {

void RotationCompute::skew_symmetric(const Scalar* input, Scalar* output) {
    // input: v(3)
    // output: skew(9) - row-major
    // [  0  -vz   vy ]
    // [ vz    0  -vx ]
    // [-vy   vx    0 ]
    
    const Scalar vx = input[0];
    const Scalar vy = input[1];
    const Scalar vz = input[2];
    
    output[0] =  0.0f;  output[1] = -vz;    output[2] =  vy;
    output[3] =  vz;    output[4] =  0.0f;  output[5] = -vx;
    output[6] = -vy;    output[7] =  vx;    output[8] =  0.0f;
}

void RotationCompute::rotation_x(const Scalar* input, Scalar* output) {
    // input: angle(1) - radians
    // output: R(9) - row-major
    
    const Scalar angle = input[0];
    const Scalar c = std::cos(angle);
    const Scalar s = std::sin(angle);
    
    output[0] = 1.0f;  output[1] = 0.0f;  output[2] = 0.0f;
    output[3] = 0.0f;  output[4] = c;     output[5] = -s;
    output[6] = 0.0f;  output[7] = s;     output[8] = c;
}

void RotationCompute::rotation_y(const Scalar* input, Scalar* output) {
    // input: angle(1) - radians
    // output: R(9) - row-major
    
    const Scalar angle = input[0];
    const Scalar c = std::cos(angle);
    const Scalar s = std::sin(angle);
    
    output[0] = c;     output[1] = 0.0f;  output[2] = s;
    output[3] = 0.0f;  output[4] = 1.0f;  output[5] = 0.0f;
    output[6] = -s;    output[7] = 0.0f;  output[8] = c;
}

void RotationCompute::rotation_z(const Scalar* input, Scalar* output) {
    // input: angle(1) - radians
    // output: R(9) - row-major
    
    const Scalar angle = input[0];
    const Scalar c = std::cos(angle);
    const Scalar s = std::sin(angle);
    
    output[0] = c;     output[1] = -s;    output[2] = 0.0f;
    output[3] = s;     output[4] = c;     output[5] = 0.0f;
    output[6] = 0.0f;  output[7] = 0.0f;  output[8] = 1.0f;
}

void RotationCompute::from_euler(const Scalar* input, Scalar* output) {
    // input: euler(3) - [roll, pitch, yaw] in degrees
    // output: R(9) - row-major
    // ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    
    const Scalar roll  = input[0] * M_PI / 180.0f;
    const Scalar pitch = input[1] * M_PI / 180.0f;
    const Scalar yaw   = input[2] * M_PI / 180.0f;
    
    const Scalar cr = std::cos(roll);
    const Scalar sr = std::sin(roll);
    const Scalar cp = std::cos(pitch);
    const Scalar sp = std::sin(pitch);
    const Scalar cy = std::cos(yaw);
    const Scalar sy = std::sin(yaw);
    
    // Row 1
    output[0] = cy*cp;
    output[1] = cy*sp*sr - sy*cr;
    output[2] = cy*sp*cr + sy*sr;
    
    // Row 2
    output[3] = sy*cp;
    output[4] = sy*sp*sr + cy*cr;
    output[5] = sy*sp*cr - cy*sr;
    
    // Row 3
    output[6] = -sp;
    output[7] = cp*sr;
    output[8] = cp*cr;
}

void RotationCompute::to_euler(const Scalar* input, Scalar* output) {
    // input: R(9) - row-major
    // output: euler(3) - [roll, pitch, yaw] in degrees
    // ZYX convention
    
    const Scalar r11 = input[0];
    const Scalar r12 = input[1];
    const Scalar r13 = input[2];
    const Scalar r21 = input[3];
    const Scalar r22 = input[4];
    const Scalar r23 = input[5];
    const Scalar r31 = input[6];
    const Scalar r32 = input[7];
    const Scalar r33 = input[8];
    
    // Pitch
    Scalar pitch;
    const Scalar sp = -r31;
    if (std::abs(sp) >= 1.0f) {
        pitch = std::copysign(M_PI / 2.0f, sp);
    } else {
        pitch = std::asin(sp);
    }
    
    // Roll and Yaw
    Scalar roll, yaw;
    if (std::abs(sp) > 0.99999f) {
        // Gimbal lock
        roll = 0.0f;
        yaw = std::atan2(-r12, r22);
    } else {
        roll = std::atan2(r32, r33);
        yaw = std::atan2(r21, r11);
    }
    
    // Convert to degrees
    output[0] = roll  * 180.0f / M_PI;
    output[1] = pitch * 180.0f / M_PI;
    output[2] = yaw   * 180.0f / M_PI;
}

void RotationCompute::rodrigues(const Scalar* input, Scalar* output) {
    // input: [axis(3); angle(1)] - angle in radians
    // output: R(9) - row-major
    // Rodrigues formula: R = I + sin(θ)*K + (1-cos(θ))*K^2
    
    const Scalar angle = input[3];
    
    if (std::abs(angle) < EPS) {
        // Identity matrix
        output[0] = 1.0f; output[1] = 0.0f; output[2] = 0.0f;
        output[3] = 0.0f; output[4] = 1.0f; output[5] = 0.0f;
        output[6] = 0.0f; output[7] = 0.0f; output[8] = 1.0f;
        return;
    }
    
    // Normalize axis
    Scalar axis[3] = {input[0], input[1], input[2]};
    normalize_vector_3(axis);
    
    const Scalar c = std::cos(angle);
    const Scalar s = std::sin(angle);
    const Scalar t = 1.0f - c;
    
    const Scalar x = axis[0];
    const Scalar y = axis[1];
    const Scalar z = axis[2];
    
    // R = I + sin(θ)*K + (1-cos(θ))*K^2
    output[0] = t*x*x + c;    output[1] = t*x*y - s*z;  output[2] = t*x*z + s*y;
    output[3] = t*x*y + s*z;  output[4] = t*y*y + c;    output[5] = t*y*z - s*x;
    output[6] = t*x*z - s*y;  output[7] = t*y*z + s*x;  output[8] = t*z*z + c;
}

void RotationCompute::from_axis_angle(const Scalar* input, Scalar* output) {
    // Same as rodrigues
    rodrigues(input, output);
}

void RotationCompute::to_axis_angle(const Scalar* input, Scalar* output) {
    // input: R(9) - row-major
    // output: [axis(3); angle(1)] - angle in radians
    
    const Scalar r11 = input[0];
    const Scalar r12 = input[1];
    const Scalar r13 = input[2];
    const Scalar r21 = input[3];
    const Scalar r22 = input[4];
    const Scalar r23 = input[5];
    const Scalar r31 = input[6];
    const Scalar r32 = input[7];
    const Scalar r33 = input[8];
    
    const Scalar trace = r11 + r22 + r33;
    const Scalar angle = safe_acos((trace - 1.0f) * 0.5f);
    
    if (std::abs(angle) < EPS) {
        // Identity rotation
        output[0] = 1.0f;
        output[1] = 0.0f;
        output[2] = 0.0f;
        output[3] = 0.0f;
    } else if (std::abs(angle - M_PI) < EPS) {
        // 180 degree rotation - special case
        const Scalar xx = (r11 + 1.0f) * 0.5f;
        const Scalar yy = (r22 + 1.0f) * 0.5f;
        const Scalar zz = (r33 + 1.0f) * 0.5f;
        const Scalar xy = (r12 + r21) * 0.25f;
        const Scalar xz = (r13 + r31) * 0.25f;
        const Scalar yz = (r23 + r32) * 0.25f;
        
        if (xx > yy && xx > zz) {
            output[0] = safe_sqrt(xx);
            output[1] = xy / output[0];
            output[2] = xz / output[0];
        } else if (yy > zz) {
            output[1] = safe_sqrt(yy);
            output[0] = xy / output[1];
            output[2] = yz / output[1];
        } else {
            output[2] = safe_sqrt(zz);
            output[0] = xz / output[2];
            output[1] = yz / output[2];
        }
        output[3] = angle;
    } else {
        // General case
        const Scalar s = 1.0f / (2.0f * std::sin(angle));
        output[0] = (r32 - r23) * s;
        output[1] = (r13 - r31) * s;
        output[2] = (r21 - r12) * s;
        output[3] = angle;
    }
}

void RotationCompute::orthonormalize(const Scalar* input, Scalar* output) {
    // input: R(9) - row-major
    // output: R_ortho(9) - row-major
    // Gram-Schmidt orthonormalization
    
    // Extract rows
    Scalar r1[3] = {input[0], input[1], input[2]};
    Scalar r2[3] = {input[3], input[4], input[5]};
    Scalar r3[3] = {input[6], input[7], input[8]};
    
    // Normalize r1
    normalize_vector_3(r1);
    
    // r2 = r2 - (r2·r1)*r1
    const Scalar dot_r2_r1 = r2[0]*r1[0] + r2[1]*r1[1] + r2[2]*r1[2];
    r2[0] -= dot_r2_r1 * r1[0];
    r2[1] -= dot_r2_r1 * r1[1];
    r2[2] -= dot_r2_r1 * r1[2];
    normalize_vector_3(r2);
    
    // r3 = r1 × r2 (cross product)
    r3[0] = r1[1]*r2[2] - r1[2]*r2[1];
    r3[1] = r1[2]*r2[0] - r1[0]*r2[2];
    r3[2] = r1[0]*r2[1] - r1[1]*r2[0];
    normalize_vector_3(r3);
    
    // Store result
    output[0] = r1[0]; output[1] = r1[1]; output[2] = r1[2];
    output[3] = r2[0]; output[4] = r2[1]; output[5] = r2[2];
    output[6] = r3[0]; output[7] = r3[1]; output[8] = r3[2];
}

void RotationCompute::inverse(const Scalar* input, Scalar* output) {
    // input: R(9) - row-major
    // output: R_inv(9) - row-major (transpose)
    
    output[0] = input[0];  // R11
    output[1] = input[3];  // R21
    output[2] = input[6];  // R31
    output[3] = input[1];  // R12
    output[4] = input[4];  // R22
    output[5] = input[7];  // R32
    output[6] = input[2];  // R13
    output[7] = input[5];  // R23
    output[8] = input[8];  // R33
}

void RotationCompute::is_valid(const Scalar* input, Scalar* output) {
    // input: R(9) - row-major
    // output: is_valid(1) - 1.0 if valid, 0.0 if invalid
    
    // Check orthogonality: R * R^T = I
    Scalar R_inv[9];
    inverse(input, R_inv);
    
    Scalar I[9];
    matrix_multiply_3x3(input, R_inv, I);
    
    // Check if result is close to identity
    const Scalar tolerance = 1e-3f;
    bool is_identity = true;
    
    for (uint8_t i = 0; i < 3; ++i) {
        for (uint8_t j = 0; j < 3; ++j) {
            const Scalar expected = (i == j) ? 1.0f : 0.0f;
            const Scalar actual = I[i*3 + j];
            if (std::abs(actual - expected) > tolerance) {
                is_identity = false;
                break;
            }
        }
        if (!is_identity) break;
    }
    
    // Check determinant ≈ 1
    const Scalar det = input[0]*(input[4]*input[8] - input[5]*input[7])
                     - input[1]*(input[3]*input[8] - input[5]*input[6])
                     + input[2]*(input[3]*input[7] - input[4]*input[6]);
    
    const bool det_valid = std::abs(det - 1.0f) < tolerance;
    
    output[0] = (is_identity && det_valid) ? 1.0f : 0.0f;
}

void RotationCompute::apply_rotation(const Scalar* input, Scalar* output) {
    // input: [R(9); v(3)] - R is row-major
    // output: v_rotated(3) = R * v
    
    const Scalar* R = input;
    const Scalar* v = input + 9;
    
    output[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
    output[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
    output[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

void RotationCompute::compose(const Scalar* input, Scalar* output) {
    // input: [R1(9); R2(9)] - row-major
    // output: R_result(9) = R1 * R2
    
    const Scalar* R1 = input;
    const Scalar* R2 = input + 9;
    
    matrix_multiply_3x3(R1, R2, output);
}

// ===== Helper Functions =====

void RotationCompute::matrix_multiply_3x3(const Scalar* A, const Scalar* B, Scalar* C) {
    // C = A * B (all row-major 3x3)
    
    for (uint8_t i = 0; i < 3; ++i) {
        for (uint8_t j = 0; j < 3; ++j) {
            C[i*3 + j] = 0.0f;
            for (uint8_t k = 0; k < 3; ++k) {
                C[i*3 + j] += A[i*3 + k] * B[k*3 + j];
            }
        }
    }
}

void RotationCompute::normalize_vector_3(Scalar* v) {
    const Scalar norm = safe_sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    
    if (norm < EPS) {
        v[0] = 1.0f;
        v[1] = 0.0f;
        v[2] = 0.0f;
    } else {
        const Scalar inv_norm = 1.0f / norm;
        v[0] *= inv_norm;
        v[1] *= inv_norm;
        v[2] *= inv_norm;
    }
}

} // namespace kalman_compute
