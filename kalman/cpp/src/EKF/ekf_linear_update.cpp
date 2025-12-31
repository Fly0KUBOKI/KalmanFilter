// ekf_linear_update.cpp
// Implementation file for EKF linear update
// Standard Kalman filter update step for linear observation model

#include "../../Inc/EKF/ekf_linear_update.hpp"
#include "../../Inc/Common/Math/fixed_matrix.hpp"
#include "../../Inc/KF/kalman_filter_core.hpp"
#include <cmath>

namespace ekf {
namespace linear {

void ekf_linear_update(
    const cmath_fx::FixedMatrix& x,
    const cmath_fx::FixedMatrix& P,
    const cmath_fx::FixedMatrix& z,
    const cmath_fx::FixedMatrix& H,
    const cmath_fx::FixedMatrix& R,
    cmath_fx::FixedMatrix& x_upd,
    cmath_fx::FixedMatrix& P_upd
) {
    // Get dimensions
    int n = x.rows;  // State dimension
    int m = z.rows;  // Measurement dimension
    
    // Resize output matrices
    x_upd.resize(n, 1);
    P_upd.resize(n, n);
    
    // Step 1: Innovation: y = z - H*x
    cmath_fx::FixedMatrix Hx = H * x;
    
    cmath_fx::FixedMatrix y;
    y.resize(m, 1);
    for (int i = 0; i < m; ++i) {
        y(i, 0) = z(i, 0) - Hx(i, 0);
    }
    
    // Step 2: Innovation covariance: S = H*P*H' + R
    cmath_fx::FixedMatrix Ht = H.transpose();
    cmath_fx::FixedMatrix HP = H * P;
    cmath_fx::FixedMatrix HPHt = HP * Ht;
    
    cmath_fx::FixedMatrix S;
    S.resize(m, m);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) {
            S(i, j) = HPHt(i, j) + R(i, j);
        }
    }
    
    // Symmetrize S
    for (int i = 0; i < m; ++i) {
        for (int j = i + 1; j < m; ++j) {
            float avg = 0.5f * (S(i, j) + S(j, i));
            S(i, j) = avg;
            S(j, i) = avg;
        }
    }
    
    // Step 3: Kalman gain: K = P*H'*S^-1
    cmath_fx::FixedMatrix PHt = P * Ht;
    
    cmath_fx::FixedMatrix S_inv;
    S_inv.resize(m, m);
    if (!S.inverse(S_inv)) {
        // If inverse fails, return unchanged state
        x_upd = x;
        P_upd = P;
        return;
    }
    
    cmath_fx::FixedMatrix K = PHt * S_inv;
    
    // Step 4: State update: x_upd = x + K*y
    cmath_fx::FixedMatrix Ky = K * y;
    
    for (int i = 0; i < n; ++i) {
        x_upd(i, 0) = x(i, 0) + Ky(i, 0);
    }
    
    // Step 5: Covariance update: P_upd = (I - K*H)*P*(I - K*H)' + K*R*K' (Joseph form)
    cmath_fx::FixedMatrix KH = K * H;
    
    cmath_fx::FixedMatrix I;
    I.resize(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            I(i, j) = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    cmath_fx::FixedMatrix I_KH;
    I_KH.resize(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            I_KH(i, j) = I(i, j) - KH(i, j);
        }
    }
    
    cmath_fx::FixedMatrix I_KHt = I_KH.transpose();
    cmath_fx::FixedMatrix temp1 = I_KH * P;
    cmath_fx::FixedMatrix term1 = temp1 * I_KHt;
    
    cmath_fx::FixedMatrix KR = K * R;
    cmath_fx::FixedMatrix Kt = K.transpose();
    cmath_fx::FixedMatrix term2 = KR * Kt;
    
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            P_upd(i, j) = term1(i, j) + term2(i, j);
        }
    }
    
    // Symmetrize P_upd
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            float avg = 0.5f * (P_upd(i, j) + P_upd(j, i));
            P_upd(i, j) = avg;
            P_upd(j, i) = avg;
        }
    }
}

} // namespace linear
} // namespace ekf
