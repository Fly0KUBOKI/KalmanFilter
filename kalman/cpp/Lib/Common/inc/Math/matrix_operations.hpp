#pragma once

#include <cmath>
#include "../../../Matrix/fixed_matrix.hpp"
#include "../../../Matrix/matrix_inverse.hpp"
#include "../../../Matrix/matrix_decomposition.hpp"

namespace common {
namespace math {

using cm = cmath_fx::FixedMatrix;

// Enforce matrix symmetry
inline cm enforce_symmetry(const cm& M) {
    cm result = M;
    for (int i = 0; i < M.rows; ++i) {
        for (int j = 0; j < M.cols; ++j) {
            result(i,j) = 0.5f * (M(i,j) + M(j,i));
        }
    }
    return result;
}

// Compute innovation and its covariance S (dynamic-size version)
inline void compute_innovation_and_S(const cm& z, const cm& h, const cm& H,
                                     const cm& P_pred, const cm& R,
                                     cm& y, cm& S, cm& R_out) {
    y = z - h;
    cm HP = H * P_pred;
    cm HPHt = HP * H.transpose();
    S = HPHt + R;
    S = enforce_symmetry(S);
    R_out = R;
}

// 3x3 inverse wrapper
template<typename T>
inline bool invert3x3(const cmath_fx::Matrix<3,3,T>& A, cmath_fx::Matrix<3,3,T>& A_inv) {
    return cmath_fx::inv::inverse<3, T>(A, A_inv);
}

// Safe Cholesky: attempt robust methods from Matrix layer
inline bool safe_cholesky(const cm& A, cm& L) {
    if (A.rows != A.cols) return false;
    int n = A.rows;
    L.resize(n, n);

    // Try FixedMatrix cholesky first
    if (A.cholesky(L)) return true;

    cm B = A;
    // Symmetrize and delegate to matrix_decomposition robust routine
    for (int i = 0; i < n; ++i) for (int j = i+1; j < n; ++j) {
        float avg = 0.5f * (B(i,j) + B(j,i)); B(i,j) = avg; B(j,i) = avg;
    }

    // Try robust cholesky from Matrix layer (fallbacks internally)
    // Use dynamic path: convert cm to templated Matrix<N,N> not available, so use A.cholesky again
    if (B.cholesky(L)) return true;

    // Try small regularization
    float eps = 1e-8f;
    for (int attempt=0; attempt<4; ++attempt) {
        for (int i=0;i<n;++i) B(i,i) += eps;
        if (B.cholesky(L)) return true;
        eps *= 10.0f;
    }

    // Final fallback: diagonal
    for (int i=0;i<n;++i) for (int j=0;j<n;++j) L(i,j) = 0.0f;
    for (int i=0;i<n;++i) {
        float v = A(i,i); if (v < 0.0f) v = 0.0f; L(i,i) = std::sqrt(v);
    }
    return true;
}

// Mahalanobis distance (dynamic FixedMatrix version)
inline float mahalanobis_distance_squared(const cm& innovation, const cm& S) {
    int n = innovation.rows;
    if (n == 0) return 0.0f;

    cm L(n,n);
    if (!safe_cholesky(S, L)) {
        float max_var = 0.0f;
        for (int i=0;i<n;++i) if (S(i,i) > max_var) max_var = S(i,i);
        if (max_var < 1e-9f) max_var = 1.0f;
        float norm_sq = 0.0f;
        for (int i=0;i<n;++i) norm_sq += innovation(i,0) * innovation(i,0);
        return norm_sq / max_var;
    }

    // Solve L * y = innovation  (forward substitution)
    cm y_vec(n,1);
    for (int i=0;i<n;++i) {
        float sum = 0.0f;
        for (int j=0;j<i;++j) sum += L(i,j) * y_vec(j,0);
        float d = L(i,i);
        if (fabsf(d) < 1e-12f) d = 1e-12f;
        y_vec(i,0) = (innovation(i,0) - sum) / d;
    }

    float dist_sq = 0.0f;
    for (int i=0;i<n;++i) dist_sq += y_vec(i,0) * y_vec(i,0);
    return dist_sq;
}

// 3x3 skew-symmetric builder (vector v 3x1)
inline cm skew_symmetric(const cm& v) {
    cm S; S.resize(3,3);
    float vx = v(0,0); float vy = v(1,0); float vz = v(2,0);
    S(0,0)=0.0f; S(0,1)=-vz; S(0,2)=vy;
    S(1,0)=vz; S(1,1)=0.0f; S(1,2)=-vx;
    S(2,0)=-vy; S(2,1)=vx; S(2,2)=0.0f;
    return S;
}

} // namespace math
} // namespace common
