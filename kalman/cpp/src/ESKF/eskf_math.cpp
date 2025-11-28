#include "../../include/ESKF/eskf_math.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace eskf_math {

// ===== Quaternion Integration =====

void ESKFMath::quaternion_integration(
    const Vector4& q_in,
    const Vector3& w,
    Scalar dt,
    Vector4& q_out
) {
    // Angular velocity integration: q_new = q * delta_q
    Vector3 w_dt = w * dt;
    
    Scalar w_dt_norm = 0.0f;
    for (int i = 0; i < 3; ++i) {
        w_dt_norm += w_dt(i,0) * w_dt(i,0);
    }
    w_dt_norm = std::sqrt(w_dt_norm);
    
    Scalar threshold = 1e-15f;
    if (w_dt_norm > threshold) {
        Scalar half_angle = w_dt_norm / 2.0f;
        Vector4 delta_q;
        
        if (half_angle > 1e-6f) {
            Scalar sin_half = std::sin(half_angle);
            Scalar cos_half = std::cos(half_angle);
            Vector3 w_unit = w_dt * (1.0f / w_dt_norm);
            
            delta_q(0,0) = cos_half;
            for (int i = 0; i < 3; ++i) {
                delta_q(i+1,0) = w_unit(i,0) * sin_half;
            }
        } else {
            // Taylor expansion for small angles
            Scalar w_norm_sq = w_dt_norm * w_dt_norm;
            delta_q(0,0) = 1.0f - w_norm_sq / 8.0f;
            for (int i = 0; i < 3; ++i) {
                delta_q(i+1,0) = w_dt(i,0) * 0.5f * (1.0f - w_norm_sq / 24.0f);
            }
        }
        
        // q_new = q * delta_q
        cquat::multiply_quat(q_in, delta_q, q_out);
        cquat::normalize_quat(q_out);
    } else {
        // No rotation
        q_out = q_in;
    }
}

// ===== Accel to Quaternion =====

void ESKFMath::accel_to_quaternion(
    const Vector3& a_meas,
    Scalar scale_factor,
    Vector4& q_out
) {
    // Compute roll and pitch from acceleration
    // Assumes a_meas points roughly downward (gravity direction)
    
    Scalar ax = a_meas(0,0);
    Scalar ay = a_meas(1,0);
    Scalar az = a_meas(2,0);
    
    // Normalize acceleration
    Scalar a_norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (a_norm < 1e-6f) {
        // No acceleration, return identity
        q_out(0,0) = 1.0f;
        q_out(1,0) = 0.0f;
        q_out(2,0) = 0.0f;
        q_out(3,0) = 0.0f;
        return;
    }
    
    ax /= a_norm;
    ay /= a_norm;
    az /= a_norm;
    
    // Roll: rotation around x-axis
    Scalar roll = std::atan2(ay, az);
    
    // Pitch: rotation around y-axis
    Scalar pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
    
    // Apply scale factor (for smoothing)
    roll *= scale_factor;
    pitch *= scale_factor;
    
    // Convert to quaternion (yaw = 0)
    Scalar cr = std::cos(roll * 0.5f);
    Scalar sr = std::sin(roll * 0.5f);
    Scalar cp = std::cos(pitch * 0.5f);
    Scalar sp = std::sin(pitch * 0.5f);
    
    q_out(0,0) = cr * cp;  // w
    q_out(1,0) = sr * cp;  // x
    q_out(2,0) = cr * sp;  // y
    q_out(3,0) = 0.0f;     // z (no yaw)
    
    cquat::normalize_quat(q_out);
}

// ===== Position/Velocity Integration =====

void ESKFMath::pv_integration(
    const PVIntegrationInput& input,
    PVIntegrationOutput& output
) {
    if (input.use_ab2) {
        // Adams-Bashforth 2nd order
        // v_new = v + dt * (1.5*a_world + g - 0.5*(prev_a + g))
        Vector3 accel_current = input.a_world + input.g;
        Vector3 accel_prev = input.prev_a + input.g;
        Vector3 dv = (accel_current * 1.5f - accel_prev * 0.5f) * input.dt;
        
        // Saturation
        Scalar dv_norm = 0.0f;
        for (int i = 0; i < 3; ++i) {
            dv_norm += dv(i,0) * dv(i,0);
        }
        dv_norm = std::sqrt(dv_norm);
        
        Scalar max_dv = input.max_accel * input.dt;
        if (dv_norm > max_dv && dv_norm > 1e-9f) {
            Scalar scale = max_dv / dv_norm;
            dv = dv * scale;
        }
        
        output.v_new = input.v + dv;
        
        // Position: p_new = p + dt * (1.5*v_new - 0.5*prev_v)
        Vector3 dp = (output.v_new * 1.5f - input.prev_v * 0.5f) * input.dt;
        output.p_new = input.p + dp;
        
    } else {
        // Forward Euler
        Vector3 accel_total = input.a_world + input.g;
        Vector3 dv = accel_total * input.dt;
        
        // Saturation
        Scalar dv_norm = 0.0f;
        for (int i = 0; i < 3; ++i) {
            dv_norm += dv(i,0) * dv(i,0);
        }
        dv_norm = std::sqrt(dv_norm);
        
        Scalar max_dv = input.max_accel * input.dt;
        if (dv_norm > max_dv && dv_norm > 1e-9f) {
            Scalar scale = max_dv / dv_norm;
            dv = dv * scale;
        }
        
        output.v_new = input.v + dv;
        output.p_new = input.p + output.v_new * input.dt;
    }
    
    // Velocity clipping
    Scalar v_norm = 0.0f;
    for (int i = 0; i < 3; ++i) {
        v_norm += output.v_new(i,0) * output.v_new(i,0);
    }
    v_norm = std::sqrt(v_norm);
    
    if (v_norm > input.max_velocity && v_norm > 1e-9f) {
        Scalar scale = input.max_velocity / v_norm;
        output.v_new = output.v_new * scale;
    }
    
    // Output for next iteration
    output.a_out = input.a_world;
    output.v_out = output.v_new;
}

// ===== State Transition Matrix F =====

void ESKFMath::compute_F_matrix(
    const Vector4& q,
    const Vector3& a_meas,
    const Vector3& ba,
    const Vector3& w_meas,
    const Vector3& bg,
    Scalar dt,
    Matrix15x15& F
) {
    // Bias-corrected measurements
    Vector3 a = a_meas - ba;
    Vector3 w = w_meas - bg;
    
    // Rotation matrix from body to world
    Matrix3x3 R;
    cquat::quat_to_rotm(q, R);
    
    // Skew-symmetric matrix of acceleration
    Matrix3x3 a_skew;
    a_skew(0,0) = 0.0f;      a_skew(0,1) = -a(2,0);  a_skew(0,2) = a(1,0);
    a_skew(1,0) = a(2,0);    a_skew(1,1) = 0.0f;     a_skew(1,2) = -a(0,0);
    a_skew(2,0) = -a(1,0);   a_skew(2,1) = a(0,0);   a_skew(2,2) = 0.0f;
    
    // Skew-symmetric matrix of angular velocity
    Matrix3x3 w_skew;
    w_skew(0,0) = 0.0f;      w_skew(0,1) = -w(2,0);  w_skew(0,2) = w(1,0);
    w_skew(1,0) = w(2,0);    w_skew(1,1) = 0.0f;     w_skew(1,2) = -w(0,0);
    w_skew(2,0) = -w(1,0);   w_skew(2,1) = w(0,0);   w_skew(2,2) = 0.0f;
    
    // Initialize F as identity
    F = Matrix15x15::Identity();
    
    // F = I + Fc * dt (first-order discretization)
    // Continuous-time state transition matrix Fc blocks:
    
    // dp/dt = v
    for (int i = 0; i < 3; ++i) {
        F(i, 3+i) = dt;  // dp/dv
    }
    
    // dv/dt = R * a - skew(a) * R * dtheta
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F(3+i, 6+j) += -dt * (R * a_skew)(i, j);  // dv/dtheta
            F(3+i, 9+j) += -dt * R(i, j);              // dv/dba
        }
    }
    
    // dtheta/dt = -skew(w)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            F(6+i, 6+j) += -dt * w_skew(i, j);  // dtheta/dtheta
            F(6+i, 12+j) += -dt * (i == j ? 1.0f : 0.0f);  // dtheta/dbg
        }
    }
    
    // Bias dynamics: dba/dt = 0, dbg/dt = 0 (already identity)
}

// ===== Covariance Prediction =====

void ESKFMath::covariance_prediction(
    const Matrix15x15& P,
    const Matrix15x15& F,
    const Matrix15x15& Q,
    Matrix15x15& P_new
) {
    // P_new = F * P * F' + Q
    P_new = F * P * F.transpose() + Q;
    
    // Symmetrize
    P_new = (P_new + P_new.transpose()) * 0.5f;
}

// ===== Error State Injection =====

void ESKFMath::inject_error_state(
    const Vector3& p_in,
    const Vector3& v_in,
    const Vector4& q_in,
    const Vector3& ba_in,
    const Vector3& bg_in,
    const Vector15& dx,
    Vector3& p_out,
    Vector3& v_out,
    Vector4& q_out,
    Vector3& ba_out,
    Vector3& bg_out
) {
    // Extract error states
    Vector3 dp, dv, dtheta, dba, dbg;
    for (int i = 0; i < 3; ++i) {
        dp(i,0) = dx(i,0);
        dv(i,0) = dx(3+i,0);
        dtheta(i,0) = dx(6+i,0);
        dba(i,0) = dx(9+i,0);
        dbg(i,0) = dx(12+i,0);
    }
    
    // Inject position and velocity
    p_out = p_in + dp;
    v_out = v_in + dv;
    
    // Inject attitude: q_new = q * delta_q
    // delta_q from small angle approximation
    Scalar theta_norm = 0.0f;
    for (int i = 0; i < 3; ++i) {
        theta_norm += dtheta(i,0) * dtheta(i,0);
    }
    theta_norm = std::sqrt(theta_norm);
    
    Vector4 delta_q;
    if (theta_norm > 1e-8f) {
        Scalar half_angle = theta_norm / 2.0f;
        Scalar sin_half = std::sin(half_angle);
        Scalar cos_half = std::cos(half_angle);
        
        delta_q(0,0) = cos_half;
        for (int i = 0; i < 3; ++i) {
            delta_q(i+1,0) = dtheta(i,0) / theta_norm * sin_half;
        }
    } else {
        // Small angle: delta_q ≈ [1, dtheta/2]
        delta_q(0,0) = 1.0f;
        for (int i = 0; i < 3; ++i) {
            delta_q(i+1,0) = dtheta(i,0) / 2.0f;
        }
    }
    
    cquat::multiply_quat(q_in, delta_q, q_out);
    cquat::normalize_quat(q_out);
    
    // Inject biases
    ba_out = ba_in + dba;
    bg_out = bg_in + dbg;
}

// ===== Observation Functions =====

void ESKFMath::mag_observation_prediction(
    const Vector4& q,
    const Vector3& m_world,
    Vector3& m_body_expected
) {
    // Rotate world magnetic field to body frame
    // m_body = R' * m_world (R is body-to-world, so R' is world-to-body)
    Matrix3x3 R;
    cquat::quat_to_rotm(q, R);
    
    m_body_expected = R.transpose() * m_world;
}

void ESKFMath::gps_to_local(
    const Vector3& gps_pos,
    const Vector3& origin_pos,
    Vector3& local_pos
) {
    // Simple flat-earth approximation
    // Assumes input is [lat, lon, alt] in degrees and meters
    Scalar lat_diff = (gps_pos(0,0) - origin_pos(0,0)) * M_PI / 180.0f;
    Scalar lon_diff = (gps_pos(1,0) - origin_pos(1,0)) * M_PI / 180.0f;
    Scalar alt_diff = gps_pos(2,0) - origin_pos(2,0);
    
    // Earth radius (m)
    Scalar R_earth = 6371000.0f;
    
    // North, East, Down
    Scalar north = lat_diff * R_earth;
    Scalar east = lon_diff * R_earth * std::cos(origin_pos(0,0) * M_PI / 180.0f);
    Scalar down = -alt_diff;  // NED convention
    
    local_pos(0,0) = north;
    local_pos(1,0) = east;
    local_pos(2,0) = down;
}

Scalar ESKFMath::pressure_to_altitude(Scalar pressure) {
    // Standard atmosphere model
    // p = p0 * (1 - L*h/T0)^(g*M/(R*L))
    // Solving for h: h = (T0/L) * (1 - (p/p0)^(R*L/(g*M)))
    
    Scalar p0 = 101325.0f;  // Sea level pressure (Pa)
    Scalar T0 = 288.15f;    // Sea level temperature (K)
    Scalar L = 0.0065f;     // Temperature lapse rate (K/m)
    Scalar g = 9.80665f;    // Gravity (m/s^2)
    Scalar M = 0.0289644f;  // Molar mass of air (kg/mol)
    Scalar R = 8.31447f;    // Universal gas constant (J/(mol*K))
    
    Scalar exponent = (R * L) / (g * M);
    Scalar altitude = (T0 / L) * (1.0f - std::pow(pressure / p0, exponent));
    
    return altitude;
}

} // namespace eskf_math
