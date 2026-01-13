#pragma once
#ifndef LIB_UKF_INC_UKF_UPDATE_HPP
#define LIB_UKF_INC_UKF_UPDATE_HPP

// UKF Update Step
// Provides templated UKF measurement update for arbitrary state/measurement dimensions

#include "ukf_sigma_points.hpp"
#include <functional>

namespace ukf {

// Generic UKF update function
// 
// Template Parameters:
//   N:           State dimension
//   M:           Measurement dimension
//   T:           Scalar type (float or double)
//
// Inputs:
//   x:           Current state estimate (N x 1)
//   P:           State covariance (N x N)
//   z:           Measurement vector (M x 1)
//   h_func:      Observation function: VectorN -> VectorM
//   R:           Measurement noise covariance (M x M)
//   params:      UKF parameters (alpha, beta, kappa)
//
// Outputs (optional):
//   K_out:       Kalman gain (N x M)
//   S_out:       Innovation covariance (M x M)
//   y_out:       Innovation vector (M x 1)
//
// Returns: true if update succeeded, false if singular matrix encountered

template<int N, int M, typename T = float>
class UKFUpdate {
public:
    using VectorN = cmath_fx::Vector<N, T>;
    using VectorM = cmath_fx::Vector<M, T>;
    using MatrixNN = cmath_fx::Matrix<N, N, T>;
    using MatrixMM = cmath_fx::Matrix<M, M, T>;
    using MatrixNM = cmath_fx::Matrix<N, M, T>;
    using MatrixMN = cmath_fx::Matrix<M, N, T>;
    using SigmaPoints = cmath_fx::Matrix<N, 2*N+1, T>;
    using Weights = cmath_fx::Vector<2*N+1, T>;
    using MeasSigmaPoints = cmath_fx::Matrix<M, 2*N+1, T>;

    // Generic update with user-provided observation function
    template<typename ObsFunc>
    static bool update(
        VectorN& x,                    // State (in/out)
        MatrixNN& P,                   // Covariance (in/out)
        const VectorM& z,              // Measurement
        ObsFunc h_func,                // Observation function: VectorN -> VectorM
        const MatrixMM& R,             // Measurement noise covariance
        const UKFParams& params,
        MatrixNM* K_out = nullptr,
        MatrixMM* S_out = nullptr,
        VectorM* y_out = nullptr
    ) {
        // Generate sigma points and weights
        SigmaPoints sig;
        Weights wm, wc;
        SigmaPointGenerator<N, T>::generate(x, P, params, sig, wm, wc);

        // Transform sigma points through observation model
        MeasSigmaPoints z_sig;
        for (int i = 0; i < 2*N+1; ++i) {
            VectorN x_sig;
            for (int j = 0; j < N; ++j) {
                x_sig(j, 0) = sig(j, i);
            }
            VectorM z_i = h_func(x_sig);
            for (int j = 0; j < M; ++j) {
                z_sig(j, i) = z_i(j, 0);
            }
        }

        // Predicted measurement mean
        VectorM z_pred;
        for (int i = 0; i < M; ++i) {
            z_pred(i, 0) = 0.0;
            for (int j = 0; j < 2*N+1; ++j) {
                z_pred(i, 0) += z_sig(i, j) * wm(j, 0);
            }
        }

        // Innovation covariance S and cross-covariance Pxz
        MatrixMM S = MatrixMM::Zero();
        MatrixNM Pxz = MatrixNM::Zero();

        for (int k = 0; k < 2*N+1; ++k) {
            // Measurement residual
            VectorM dz;
            for (int i = 0; i < M; ++i) {
                dz(i, 0) = z_sig(i, k) - z_pred(i, 0);
            }

            // S += wc[k] * dz * dz'
            T wc_k = wc(k, 0);
            for (int i = 0; i < M; ++i) {
                for (int j = 0; j < M; ++j) {
                    S(i, j) += wc_k * dz(i, 0) * dz(j, 0);
                }
            }

            // State residual
            VectorN dx;
            for (int i = 0; i < N; ++i) {
                dx(i, 0) = sig(i, k) - x(i, 0);
            }

            // Pxz += wc[k] * dx * dz'
            for (int i = 0; i < N; ++i) {
                for (int j = 0; j < M; ++j) {
                    Pxz(i, j) += wc_k * dx(i, 0) * dz(j, 0);
                }
            }
        }

        // Add measurement noise: S += R
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < M; ++j) {
                S(i, j) += R(i, j);
            }
        }

        // Compute Kalman gain: K = Pxz * inv(S)
        MatrixNM K;
        {
            MatrixMM S_inv;
            if (!S.inverse(S_inv)) {
                // Singular matrix - cannot update
                return false;
            }
            K = Pxz * S_inv;
        }

        // Innovation: y = z - z_pred
        VectorM y = z - z_pred;

        // State update: x = x + K * y
        VectorN Ky = K * y;
        x = x + Ky;

        // Covariance update: P = P - K * S * K'
        MatrixNM KS = K * S;
        MatrixNN KSKt = KS * K.transpose();
        P = P - KSKt;

        // Symmetrize covariance
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                P(i, j) = static_cast<T>(0.5) * (P(i, j) + P(j, i));
            }
        }

        // Optional outputs
        if (K_out) *K_out = K;
        if (S_out) *S_out = S;
        if (y_out) *y_out = y;

        return true;
    }
};

} // namespace ukf

#endif // LIB_UKF_INC_UKF_UPDATE_HPP
