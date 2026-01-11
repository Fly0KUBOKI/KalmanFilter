// ekf_linear_update.cpp
// Implementation file for EKF linear update (Lib copy)
// Standard Kalman filter update step for linear observation model

#include "../inc/ekf_linear_update.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../KF/inc/kalman_filter_core.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Common/inc/Math/statistics.hpp"
#include "../../Common/inc/Math/geometry.hpp"
#include "../../Common/inc/Math/numerical.hpp"
#include "../../Matrix/fixed_matrix.hpp"
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
    {
        // predicted measurement
        cmath_fx::FixedMatrix h; h.resize(m, 1);
        h = H * x;

        // innovation y = z - h
        cmath_fx::FixedMatrix y; y.resize(m, 1);
        for (int i = 0; i < m; ++i) y(i,0) = z(i,0) - h(i,0);

        // innovation covariance S = H * P * H' + R
        cmath_fx::FixedMatrix S; S.resize(m, m);
        S = H * P * H.transpose() + R;
        S = cmath_fx::utils::symmetrize(S);

        // invert S
        cmath_fx::FixedMatrix S_inv; S_inv.resize(m, m);
        if (!S.inverse(S_inv)) {
            x_upd = x;
            P_upd = P;
            return;
        }

        // Kalman gain K = P * H' * S_inv
        cmath_fx::FixedMatrix K = P * H.transpose() * S_inv;

        // State update: x_upd = x + K * y
        cmath_fx::FixedMatrix Ky = K * y;
        for (int i = 0; i < n; ++i) x_upd(i,0) = x(i,0) + Ky(i,0);

        // Covariance update (Joseph form)
        kf::ops::joseph_form_update(P, K, H, R, P_upd);
    }
}

} // namespace linear
} // namespace ekf
