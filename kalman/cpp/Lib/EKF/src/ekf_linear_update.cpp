// ekf_linear_update.cpp
// Implementation file for EKF linear update (Lib copy)
// Standard Kalman filter update step for linear observation model

#include "../inc/ekf_linear_update.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../KF/inc/kalman_filter_core.hpp"
#include "../../Common/inc/Math/math_utils.hpp"
#include "../../Matrix/matrix_utils.hpp"
#include "../../KF/inc/kf_operations.hpp"
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
    
    // Step 1/2: Innovation and innovation covariance (unified)
    cmath_fx::FixedMatrix y; y.resize(m,1);
    cmath_fx::FixedMatrix S; S.resize(m,m);
    cmath_fx::FixedMatrix R_out; R_out.resize(m,m);
    common::math::MathUtils::compute_innovation_and_S(z, x, H, P, R, y, S, R_out);
    
    // Step 3: Kalman gain: K = P*H'*S^-1
    cmath_fx::FixedMatrix Ht = H.transpose();
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
    
    // Step 5: Covariance update (Joseph form) via centralized helper
    kf::ops::joseph_form_update(P, K, H, R, P_upd);
}

} // namespace linear
} // namespace ekf
