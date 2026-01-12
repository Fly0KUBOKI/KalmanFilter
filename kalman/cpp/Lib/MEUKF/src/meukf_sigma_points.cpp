#include "../inc/meukf_core.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../UKF/inc/ukf_sigma_points.hpp"
#include <cmath>

namespace meukf {

using SPGen = ukf::SigmaPointGenerator<15, float>;

// Compute sigma points for state x (15x1) and covariance P (15x15).
// Output `sigmas` is (15 x 31) matrix (2*n+1 columns).
static void compute_sigma_points(const Vector15& x, const Matrix15x15& P, cmath_fx::Matrix<15,31,float>& sigmas) {
    ukf::UKFParams params;
    params.alpha = 1e-3f;
    params.beta = 2.0f;
    params.kappa = 0.0f;

    SPGen::Weights wm, wc;
    SPGen::generate(x, P, params, sigmas, wm, wc);
}

// Reconstruct mean and covariance from sigma points using standard UKF weights.
static void reconstruct_mean_cov(const cmath_fx::Matrix<15,31,float>& sigmas, Vector15& x_out, Matrix15x15& P_out) {
    ukf::UKFParams params;
    params.alpha = 1e-3f;
    params.beta = 2.0f;
    params.kappa = 0.0f;

    SPGen::Weights wm, wc;
    // We need an input P for generate to compute weights; pass tiny diag
    Matrix15x15 P_tmp = Matrix15x15::Identity() * 1e-6f;
    SPGen::generate(x_out, P_tmp, params, const_cast<cmath_fx::Matrix<15,31,float>&>(sigmas), wm, wc);

    // Compute mean
    for (int i = 0; i < 15; ++i) {
        float acc = 0.0f;
        for (int k = 0; k < 31; ++k) acc += sigmas(i, k) * wm(k, 0);
        x_out(i, 0) = acc;
    }

    // Compute covariance
    P_out = Matrix15x15::Zero();
    for (int k = 0; k < 31; ++k) {
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                float di = sigmas(i, k) - x_out(i, 0);
                float dj = sigmas(j, k) - x_out(j, 0);
                P_out(i, j) += wc(k, 0) * di * dj;
            }
        }
    }
}

} // namespace meukf
