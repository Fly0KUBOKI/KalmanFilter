#pragma once

#include "../Common/Math/fixed_matrix.hpp"
#include "../Common/Math/quaternion.hpp"

namespace eskf_math {

// Use float for performance
using Scalar = float;
using Vector3 = cmath_fx::Vector<3, Scalar>;
using Vector4 = cmath_fx::Vector<4, Scalar>;
using Vector15 = cmath_fx::Vector<15, Scalar>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, Scalar>;
using Matrix15x15 = cmath_fx::Matrix<15, 15, Scalar>;

/**
 * Pure computation functions for ESKF - stateless, math-library style
 * All functions follow the pattern: compute_xxx(input_matrix) -> output_matrix
 * State management is handled by MATLAB side
 */

class ESKFMath {
public:
    // ===== Quaternion Operations =====
    
    /**
     * Quaternion integration from angular velocity
     * Input: [q(4x1); w(3x1); dt(scalar)]
     * Output: q_new(4x1)
     */
    static void quaternion_integration(
        const Vector4& q_in,
        const Vector3& w,
        Scalar dt,
        Vector4& q_out
    );
    
    /**
     * Convert acceleration vector to roll/pitch quaternion
     * Input: [a_meas(3x1); scale_factor(scalar)]
     * Output: q_rp(4x1) - quaternion with only roll/pitch (yaw=0)
     */
    static void accel_to_quaternion(
        const Vector3& a_meas,
        Scalar scale_factor,
        Vector4& q_out
    );
    
    // ===== Position/Velocity Integration =====
    
    /**
     * Position and velocity integration (RK2/AB2)
     * Input struct to avoid too many parameters
     */
    struct PVIntegrationInput {
        Vector3 p;           // Current position
        Vector3 v;           // Current velocity
        Vector3 a_world;     // World-frame acceleration
        Vector3 g;           // Gravity vector
        Scalar dt;           // Time step
        Vector3 prev_a;      // Previous acceleration (for AB2)
        Vector3 prev_v;      // Previous velocity (for AB2)
        bool use_ab2;        // true: AB2, false: Euler
        Scalar max_accel;    // Maximum acceleration for saturation
        Scalar max_velocity; // Maximum velocity for clipping
    };
    
    struct PVIntegrationOutput {
        Vector3 p_new;       // New position
        Vector3 v_new;       // New velocity
        Vector3 a_out;       // Output acceleration (for next AB2)
        Vector3 v_out;       // Output velocity (for next AB2)
    };
    
    static void pv_integration(
        const PVIntegrationInput& input,
        PVIntegrationOutput& output
    );
    
    // ===== State Transition and Covariance =====
    
    /**
     * Compute state transition matrix F for ESKF
     * Input: [q(4x1); a_meas(3x1); ba(3x1); w_meas(3x1); bg(3x1); dt(scalar)]
     * Output: F(15x15)
     */
    static void compute_F_matrix(
        const Vector4& q,
        const Vector3& a_meas,
        const Vector3& ba,
        const Vector3& w_meas,
        const Vector3& bg,
        Scalar dt,
        Matrix15x15& F
    );
    
    /**
     * Covariance prediction: P_new = F * P * F' + Q
     * Input: [P(15x15); F(15x15); Q(15x15)]
     * Output: P_new(15x15)
     */
    static void covariance_prediction(
        const Matrix15x15& P,
        const Matrix15x15& F,
        const Matrix15x15& Q,
        Matrix15x15& P_new
    );
    
    // ===== Error State Injection =====
    
    /**
     * Inject error state into nominal state
     * Input: [p(3x1); v(3x1); q(4x1); ba(3x1); bg(3x1); dx(15x1)]
     * Output: [p_new(3x1); v_new(3x1); q_new(4x1); ba_new(3x1); bg_new(3x1)]
     */
    static void inject_error_state(
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
    );
    
    // ===== Observation Functions (Sensor-Independent) =====
    
    /**
     * Predict magnetic field in body frame from world frame
     * Input: [q(4x1); m_world(3x1)]
     * Output: m_body_expected(3x1)
     */
    static void mag_observation_prediction(
        const Vector4& q,
        const Vector3& m_world,
        Vector3& m_body_expected
    );
    
    /**
     * GPS coordinates to local position
     * Input: [gps_pos(3x1); origin_pos(3x1)]
     * Output: local_pos(3x1)
     */
    static void gps_to_local(
        const Vector3& gps_pos,
        const Vector3& origin_pos,
        Vector3& local_pos
    );
    
    /**
     * Barometric pressure to altitude
     * Input: pressure(scalar)
     * Output: altitude(scalar)
     */
    static Scalar pressure_to_altitude(Scalar pressure);
    
    // ===== Kalman Update (Generic) =====
    
    /**
     * Generic Kalman update computation
     * Input: x(Nx1), P(NxN), y(Mx1), H(MxN), R(MxM)
     * Output: x_new(Nx1), P_new(NxN), K(NxM), S(MxM)
     */
    template<int N, int M>
    static void kalman_update(
        const cmath_fx::Vector<N, Scalar>& x_in,
        const cmath_fx::Matrix<N, N, Scalar>& P_in,
        const cmath_fx::Vector<M, Scalar>& y,  // innovation
        const cmath_fx::Matrix<M, N, Scalar>& H,
        const cmath_fx::Matrix<M, M, Scalar>& R,
        cmath_fx::Vector<N, Scalar>& x_out,
        cmath_fx::Matrix<N, N, Scalar>& P_out,
        cmath_fx::Matrix<N, M, Scalar>& K_out,
        cmath_fx::Matrix<M, M, Scalar>& S_out
    );
};

// Template implementation for kalman_update
template<int N, int M>
void ESKFMath::kalman_update(
    const cmath_fx::Vector<N, Scalar>& x_in,
    const cmath_fx::Matrix<N, N, Scalar>& P_in,
    const cmath_fx::Vector<M, Scalar>& y,
    const cmath_fx::Matrix<M, N, Scalar>& H,
    const cmath_fx::Matrix<M, M, Scalar>& R,
    cmath_fx::Vector<N, Scalar>& x_out,
    cmath_fx::Matrix<N, N, Scalar>& P_out,
    cmath_fx::Matrix<N, M, Scalar>& K_out,
    cmath_fx::Matrix<M, M, Scalar>& S_out
) {
    // S = H * P * H' + R
    S_out = H * P_in * H.transpose() + R;
    
    // Symmetrize S
    S_out = (S_out + S_out.transpose()) * static_cast<Scalar>(0.5);
    
    // K = P * H' * S^-1
    cmath_fx::Matrix<M, M, Scalar> S_inv;
    if (!S_out.inverse(S_inv)) {
        // Fallback: use pseudoinverse or return zero gain
        K_out = cmath_fx::Matrix<N, M, Scalar>::Zero();
        x_out = x_in;
        P_out = P_in;
        return;
    }
    
    K_out = P_in * H.transpose() * S_inv;
    
    // x_new = x + K * y
    x_out = x_in + K_out * y;
    
    // P_new = (I - K*H) * P * (I - K*H)' + K*R*K' (Joseph form)
    auto I = cmath_fx::Matrix<N, N, Scalar>::Identity();
    auto I_KH = I - K_out * H;
    P_out = I_KH * P_in * I_KH.transpose() + K_out * R * K_out.transpose();
    
    // Symmetrize P
    P_out = (P_out + P_out.transpose()) * static_cast<Scalar>(0.5);
}

} // namespace eskf_math
