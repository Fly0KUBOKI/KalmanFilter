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
    T a[3][3];
    for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
            a[i][j] = A(i,j);

    L = cmath_fx::Matrix<3, 3, T>::Zero();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j <= i; ++j) {
            T sum = 0;
            for (int k = 0; k < j; ++k) {
                sum += L(i,k) * L(j,k);
            }

            if (i == j) {
                T val = a[i][i] - sum;
                if (val <= static_cast<T>(1e-9)) return false;
                L(i,j) = std::sqrt(val);
            } else {
                if (L(j,j) < static_cast<T>(1e-12)) return false;
                L(i,j) = (a[i][j] - sum) / L(j,j);
            }
        }
    }
    
    return true;
}

// Robust Cholesky with fallback to diagonal approximation
template<typename T>
inline bool cholesky3x3_robust(cmath_fx::Matrix<3, 3, T>& A, cmath_fx::Matrix<3, 3, T>& L) {
    // 1. Symmetrize
    for(int i=0; i<3; ++i) {
        for(int j=i+1; j<3; ++j) {
            T avg = (A(i,j) + A(j,i)) / 2;
            A(i,j) = avg;
            A(j,i) = avg;
        }
    }
    
    // 2. Check minimum eigenvalue (approximated by min diagonal)
    T min_diag = A(0,0);
    for(int i=1; i<3; ++i) {
        if (A(i,i) < min_diag) min_diag = A(i,i);
    }
    
    // 3. Regularize if not positive definite
    if (min_diag <= 0) {
        T reg = std::abs(min_diag) + static_cast<T>(1e-6);
        for(int i=0; i<3; ++i) A(i,i) += reg;
    }
    
    // 4. Try Cholesky
    if (cholesky3x3(A, L)) {
        return true;
    }
    
    // 5. Stronger regularization
    for(int i=0; i<3; ++i) A(i,i) += static_cast<T>(1e-4);
    if (cholesky3x3(A, L)) {
        return true;
    }
    
    // 6. Fallback: diagonal approximation
    L = cmath_fx::Matrix<3, 3, T>::Zero();
    for(int i=0; i<3; ++i) {
        L(i,i) = std::sqrt(std::max(static_cast<T>(0), A(i,i)));
    }
    return true;
}

// Ensure 3x3 covariance is positive definite
template<typename T>
inline void ensure_positive_definite_3x3(cmath_fx::Matrix<3, 3, T>& P) {
    // Symmetrize
    for(int i=0; i<3; ++i) {
        for(int j=i+1; j<3; ++j) {
            T avg = (P(i,j) + P(j,i)) / 2;
            P(i,j) = avg;
            P(j,i) = avg;
        }
    }
    
    // Check and regularize diagonal
    T min_diag = P(0,0);
    for(int i=1; i<3; ++i) {
        if (P(i,i) < min_diag) min_diag = P(i,i);
    }
    
    if (min_diag <= 0) {
        T reg = std::abs(min_diag) + static_cast<T>(1e-8);
        for(int i=0; i<3; ++i) P(i,i) += reg;
        
        // Re-symmetrize
        for(int i=0; i<3; ++i) {
            for(int j=i+1; j<3; ++j) {
                T avg = (P(i,j) + P(j,i)) / 2;
                P(i,j) = avg;
                P(j,i) = avg;
            }
        }
    }
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
