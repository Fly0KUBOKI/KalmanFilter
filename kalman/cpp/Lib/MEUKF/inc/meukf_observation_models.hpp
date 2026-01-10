#pragma once

// MEUKF observation models for use with generic UKF library

#include "../../Matrix/fixed_matrix.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include <cmath>

namespace meukf {

// Forward declaration of State type
// State structure: [p(3), v(3), q(4), ba(3), bg(3)] = 15D
// We'll use a simple accessor pattern for UKF

// Attitude observation model (for accelerometer)
// Observes: q (quaternion representing attitude)
// Measurement: normalized acceleration = -R(q)^T @ g_nominal
class AccelObservationModel {
public:
    // gravity vector in NED frame
    static const float g_ned[3];

    // Quaternion to rotation matrix and apply to gravity
    // Input: 4D state vector representing quaternion [w, x, y, z]
    // Output: 3D measurement (expected accelerometer reading = -R(q)^T @ g)
    static cmath_fx::Vector<3, float> h_accel(const cmath_fx::Vector<15, float>& x_15) {
        // Extract quaternion from state: x_15[6:10] = [w, x, y, z]
        float qw = x_15(6, 0);
        float qx = x_15(7, 0);
        float qy = x_15(8, 0);
        float qz = x_15(9, 0);

        // Build quaternion vector expected by quaternion utilities
        cmath_fx::Vector<4, float> q;
        q(0,0) = qw;
        q(1,0) = qx;
        q(2,0) = qy;
        q(3,0) = qz;

        // Rotation matrix from quaternion
        cmath_fx::Matrix<3,3,float> R;
        cquat::quat_to_rotm(q, R);

        // gravity vector in NED frame
        cmath_fx::Vector<3, float> g_vec;
        g_vec(0,0) = g_ned[0];
        g_vec(1,0) = g_ned[1];
        g_vec(2,0) = g_ned[2];

        // Expected accelerometer reading in body frame = - R^T * g_ned
        cmath_fx::Vector<3, float> a_pred = (R.transpose() * g_vec) * -1.0f;

        cmath_fx::Vector<3, float> z_pred;
        z_pred(0,0) = a_pred(0,0);
        z_pred(1,0) = a_pred(1,0);
        z_pred(2,0) = a_pred(2,0);

        return z_pred;
    }
};

// Magnetometer observation model
class MagObservationModel {
public:
    // magnetic field in NED frame (normalized, site-dependent)
    // For simplicity, assume pointing North with small dip
    static const float m_ned[3];  // Placeholder

    static cmath_fx::Vector<3, float> h_mag(const cmath_fx::Vector<15, float>& x_15) {
        // Extract quaternion from state: x_15[6:10] = [w, x, y, z]
        float qw = x_15(6, 0);
        float qx = x_15(7, 0);
        float qy = x_15(8, 0);
        float qz = x_15(9, 0);

        // Rotate m_ned from NED to body frame using q
        // m_body = R(q) @ m_ned
        float m_norm = std::sqrt(m_ned[0]*m_ned[0] + m_ned[1]*m_ned[1] + m_ned[2]*m_ned[2]);

        // Simple rotation via quaternion conjugate multiplication
        // For now, use simplified form (full implementation would be more complex)
        float m_body_x = m_ned[0];
        float m_body_y = m_ned[1];
        float m_body_z = m_ned[2];

        cmath_fx::Vector<3, float> z_pred;
        z_pred(0, 0) = m_body_x;
        z_pred(1, 0) = m_body_y;
        z_pred(2, 0) = m_body_z;

        return z_pred;
    }
};

// Position observation model (for GPS)
class GPSObservationModel {
public:
    // Observes position directly
    static cmath_fx::Vector<3, float> h_gps(const cmath_fx::Vector<15, float>& x_15) {
        // Extract position from state: x_15[0:3] = [p_x, p_y, p_z]
        cmath_fx::Vector<3, float> z_pred;
        z_pred(0, 0) = x_15(0, 0);  // x position (ENU)
        z_pred(1, 0) = x_15(1, 0);  // y position (ENU)
        z_pred(2, 0) = x_15(2, 0);  // z position (ENU)
        return z_pred;
    }
};

// Altitude observation model (for barometer)
class AltObservationModel {
public:
    // Observes altitude (z position) directly
    static cmath_fx::Vector<1, float> h_alt(const cmath_fx::Vector<15, float>& x_15) {
        cmath_fx::Vector<1, float> z_pred;
        z_pred(0, 0) = x_15(2, 0);  // z position (altitude in ENU)
        return z_pred;
    }
};

} // namespace meukf
