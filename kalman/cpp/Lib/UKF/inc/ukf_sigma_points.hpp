#pragma once

// UKF Sigma Point Generation
// Provides templated sigma point generation for arbitrary state dimensions

#include "../../Matrix/fixed_matrix.hpp"
#include <cmath>

namespace ukf {

// UKF Parameters
struct UKFParams {
    float alpha;   // Spread of sigma points (typically 1e-3)
    float beta;    // Beta for distribution (typically 2.0 for Gaussian)
    float kappa;   // Secondary scaling parameter (typically 0.0)
};

// Generate sigma points and weights for UKF
// Template parameters:
//   N: State dimension
//   T: Scalar type (float or double)
template<int N, typename T = float>
class SigmaPointGenerator {
public:
    using VectorN = cmath_fx::Vector<N, T>;
    using MatrixNN = cmath_fx::Matrix<N, N, T>;
    using SigmaPoints = cmath_fx::Matrix<N, 2*N+1, T>;
    using Weights = cmath_fx::Vector<2*N+1, T>;

    // Generate sigma points from state and covariance
    static void generate(
        const VectorN& x,
        const MatrixNN& P,
        const UKFParams& params,
        SigmaPoints& sig,
        Weights& wm,
        Weights& wc
    ) {
        T n = static_cast<T>(N);
        T alpha = static_cast<T>(params.alpha);
        T beta = static_cast<T>(params.beta);
        T kappa = static_cast<T>(params.kappa);

        T lambda = alpha * alpha * (n + kappa) - n;
        T c = n + lambda;

        // Weights for mean and covariance
        wm(0, 0) = lambda / c;
        wc(0, 0) = lambda / c + (static_cast<T>(1) - alpha * alpha + beta);

        for (int i = 1; i < 2*N+1; ++i) {
            wm(i, 0) = static_cast<T>(0.5) / c;
            wc(i, 0) = static_cast<T>(0.5) / c;
        }

        // Cholesky decomposition of P
        MatrixNN L;
        if (!P.cholesky(L)) {
            // Fallback: diagonal approximation
            L = MatrixNN::Zero();
            for (int i = 0; i < N; ++i) {
                L(i, i) = std::sqrt(std::max(static_cast<T>(0), P(i, i)));
            }
        }

        // Scale by sqrt(c)
        T scale = std::sqrt(c);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                L(i, j) *= scale;
            }
        }

        // Generate sigma points
        // sig[:,0] = x
        for (int i = 0; i < N; ++i) {
            sig(i, 0) = x(i, 0);
        }

        // sig[:,1:N] = x + L[:,i]
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                sig(j, i+1) = x(j, 0) + L(j, i);
            }
        }

        // sig[:,N+1:2N] = x - L[:,i]
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                sig(j, N+i+1) = x(j, 0) - L(j, i);
            }
        }
    }
};

} // namespace ukf
