#pragma once

// UKF Shared Utilities
// Common operations for UKF-based filters (Cholesky, sigma point generation, weight calculation)
// These are extracted from MEUKF to be reusable across different attitude/position filters

#include "../../Matrix/fixed_matrix.hpp"
#include <cmath>
#include <algorithm>

namespace ukf_utils {

// UKF Parameters
struct UKFParams {
    float alpha;   // Spread of sigma points (typically 1e-3)
    float beta;    // Distribution scaling (typically 2.0 for Gaussian)
    float kappa;   // Secondary parameter (typically 0.0)
};

// Cholesky decomposition for 3x3 matrix
// Returns true if successful, false if not positive definite
template<typename T>
inline bool cholesky3x3(const cmath_fx::Matrix<3, 3, T>& A, cmath_fx::Matrix<3, 3, T>& L) {
    // Delegate to Matrix layer optimized 3x3 Cholesky
    return cmath_fx::decomp::cholesky_3x3_optimized<T>(A, L);
}

// Robust Cholesky with fallback to diagonal approximation
template<typename T>
inline bool cholesky3x3_robust(cmath_fx::Matrix<3, 3, T>& A, cmath_fx::Matrix<3, 3, T>& L) {
    return cmath_fx::decomp::cholesky_robust<3, T>(A, L);
}

// Ensure 3x3 covariance is positive definite
template<typename T>
inline void ensure_positive_definite_3x3(cmath_fx::Matrix<3, 3, T>& P) {
    cmath_fx::utils::ensure_positive_definite<3, T>(P);
}

// Calculate UKF weights for given dimension and parameters
template<int N, typename T>
inline void calculate_weights(
    const UKFParams& params,
    cmath_fx::Vector<2*N+1, T>& wm,  // Mean weights
    cmath_fx::Vector<2*N+1, T>& wc   // Covariance weights
) {
    T n = static_cast<T>(N);
    T alpha = static_cast<T>(params.alpha);
    T beta = static_cast<T>(params.beta);
    T kappa = static_cast<T>(params.kappa);

    T lambda = alpha * alpha * (n + kappa) - n;
    T c = n + lambda;

    wm(0, 0) = lambda / c;
    wc(0, 0) = lambda / c + (1 - alpha * alpha + beta);

    for (int i = 1; i < 2*N+1; ++i) {
        wm(i, 0) = 1 / (2 * c);
        wc(i, 0) = 1 / (2 * c);
    }
}

// Generate sigma points for attitude (3D rotation in tangent space)
// Input: q_nominal (quaternion), P_att (3x3 covariance of attitude error)
// Output: dtheta_sigma (7 sigma points of 3D attitude errors)
inline void generate_attitude_sigma_points_3d(
    const cmath_fx::Vector<4, float>& q_nominal,
    const cmath_fx::Matrix<3, 3, float>& P_att,
    const UKFParams& params,
    cmath_fx::Vector<3, float> dtheta_sigma[7]
) {
    // Cholesky decomposition
    cmath_fx::Matrix<3, 3, float> L;
    cmath_fx::Matrix<3, 3, float> P_att_copy = P_att;
    if (!cholesky3x3_robust(P_att_copy, L)) {
        // Fallback
        L = cmath_fx::Matrix<3, 3, float>::Zero();
        for(int i=0; i<3; ++i) L(i,i) = std::sqrt(std::max(0.0f, P_att(i,i)));
    }

    float alpha = params.alpha;
    float kappa = params.kappa;
    int n = 3;
    float lambda = alpha * alpha * (n + kappa) - n;
    float gamma = std::sqrt(n + lambda);

    // Scale L
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            L(i, j) *= gamma;
        }
    }

    // Central sigma point (zero error)
    dtheta_sigma[0] = cmath_fx::Vector<3, float>::Zero();

    // Positive/negative sigma points
    for (int i = 0; i < 3; ++i) {
        cmath_fx::Vector<3, float> col;
        col(0, 0) = L(0, i);
        col(1, 0) = L(1, i);
        col(2, 0) = L(2, i);
        dtheta_sigma[1 + i] = col;
        dtheta_sigma[4 + i] = col * -1.0f;
    }
}

// Generate sigma points for position (3D Gaussian)
// Input: p_nominal, P_pos (3x3 covariance)
// Output: dp_sigma (7 sigma points of 3D position errors)
inline void generate_position_sigma_points_3d(
    const cmath_fx::Vector<3, float>& p_nominal,
    const cmath_fx::Matrix<3, 3, float>& P_pos,
    const UKFParams& params,
    cmath_fx::Vector<3, float> dp_sigma[7]
) {
    // Similar to attitude but for position
    cmath_fx::Matrix<3, 3, float> L;
    cmath_fx::Matrix<3, 3, float> P_pos_copy = P_pos;
    if (!cholesky3x3_robust(P_pos_copy, L)) {
        L = cmath_fx::Matrix<3, 3, float>::Zero();
        for(int i=0; i<3; ++i) L(i,i) = std::sqrt(std::max(0.0f, P_pos(i,i)));
    }

    float alpha = params.alpha;
    float kappa = params.kappa;
    int n = 3;
    float lambda = alpha * alpha * (n + kappa) - n;
    float gamma = std::sqrt(n + lambda);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            L(i, j) *= gamma;
        }
    }

    dp_sigma[0] = cmath_fx::Vector<3, float>::Zero();

    for (int i = 0; i < 3; ++i) {
        cmath_fx::Vector<3, float> col;
        col(0, 0) = L(0, i);
        col(1, 0) = L(1, i);
        col(2, 0) = L(2, i);
        dp_sigma[1 + i] = col;
        dp_sigma[4 + i] = col * -1.0f;
    }
}

} // namespace ukf_utils
