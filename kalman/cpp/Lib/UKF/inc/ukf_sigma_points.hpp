#pragma once

// Implementation: Src/UKF/ukf_sigma_points.cpp

#include "../../Matrix/fixed_matrix.hpp"

namespace ukf {

// UKF sigma point generation function (dynamic size version for MEX)
// This function generates sigma points for UKF with dynamic size
// Input: x (n x 1), P (n x n, row-major), alpha, beta, kappa, n (dimension)
// Output: sig (n x (2n+1), column-major), wm ((2n+1) x 1), wc ((2n+1) x 1)
// Note: This is a temporary function for MEX wrapper. Full implementation should use UKFCore template
void generate_sigma_points_dynamic(
    const float* x,           // Input: state vector (n x 1)
    const float* P,            // Input: covariance matrix (n x n, row-major)
    int n,                     // State dimension
    float alpha,
    float beta,
    float kappa,
    float* sig,                // Output: sigma points (n x (2n+1), column-major)
    float* wm,                 // Output: mean weights ((2n+1) x 1)
    float* wc                  // Output: covariance weights ((2n+1) x 1)
);

} // namespace ukf
