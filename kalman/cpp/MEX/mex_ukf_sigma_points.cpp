// mex_ukf_sigma_points.cpp
// MEX wrapper for UKF sigma point generation
//
// Usage:
//   [sig, wm, wc] = mex_ukf_sigma_points(x, P, alpha, beta, kappa)

#include "mex.h"
#include <cmath>
#include <algorithm>
#include "mex_type_conv.hpp"
#include <vector>

// Cholesky decomposition for lower triangular matrix
static bool chol_lower(const float* A, int n, float* L) {
    for (int i = 0; i < n * n; ++i) L[i] = 0.0f;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < j; ++k) {
                sum += L[i * n + k] * L[j * n + k];
            }

            if (i == j) {
                float val = A[i * n + i] - sum;
                if (val <= 0.0f) return false; // Not positive definite
                L[i * n + i] = sqrtf(val);
            } else {
                L[i * n + j] = (A[i * n + j] - sum) / L[j * n + j];
            }
        }
    }
    return true;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Input validation
    if (nrhs < 2 || nrhs > 5) {
        mexErrMsgIdAndTxt("mex_ukf_sigma_points:nrhs",
                          "Usage: [sig, wm, wc] = mex_ukf_sigma_points(x, P, [alpha, beta, kappa])");
    }
    if (nlhs > 3) {
        mexErrMsgIdAndTxt("mex_ukf_sigma_points:nlhs", "Too many output arguments");
    }
    
    // Get inputs
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0])) {
        mexErrMsgIdAndTxt("mex_ukf_sigma_points:notDouble", "x must be real double vector");
    }
    if (!mxIsDouble(prhs[1]) || mxIsComplex(prhs[1])) {
        mexErrMsgIdAndTxt("mex_ukf_sigma_points:notDouble", "P must be real double matrix");
    }
    
    int n = (int)mxGetM(prhs[0]);
    if (mxGetN(prhs[0]) != 1) {
        mexErrMsgIdAndTxt("mex_ukf_sigma_points:xNotVector", "x must be column vector");
    }
    
    if (mxGetM(prhs[1]) != n || mxGetN(prhs[1]) != n) {
        mexErrMsgIdAndTxt("mex_ukf_sigma_points:sizeMismatch", "P must be nxn matrix");
    }
    
    // Optional parameters
    float alpha = (nrhs >= 3) ? mex_conv::mxGetScalarAsFloat(prhs[2]) : 1e-3f;
    float beta = (nrhs >= 4) ? mex_conv::mxGetScalarAsFloat(prhs[3]) : 2.0f;
    float kappa = (nrhs >= 5) ? mex_conv::mxGetScalarAsFloat(prhs[4]) : 0.0f;
    
    // Calculate lambda
    float lambda = alpha * alpha * (n + kappa) - n;
    
    // Calculate weights
    int n_sig = 2 * n + 1;
    plhs[1] = mxCreateDoubleMatrix(n_sig, 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(n_sig, 1, mxREAL);
    double* wm = mxGetPr(plhs[1]);
    double* wc = mxGetPr(plhs[2]);
    
    wm[0] = static_cast<double>(lambda / (n + lambda));
    wc[0] = static_cast<double>(wm[0] + (1.0f - alpha * alpha + beta));

    double w_other = static_cast<double>(1.0f / (2.0f * (n + lambda)));
    for (int i = 1; i < n_sig; ++i) {
        wm[i] = w_other;
        wc[i] = w_other;
    }
    
    // Scale P and compute Cholesky
    // Convert inputs to float temporaries
    std::vector<float> x_tmp(static_cast<size_t>(n));
    std::vector<float> P_tmp(static_cast<size_t>(n) * static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(prhs[0], x_tmp.data(), static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(prhs[1], P_tmp.data(), static_cast<size_t>(n) * static_cast<size_t>(n));

    std::vector<float> P_scaled(static_cast<size_t>(n) * static_cast<size_t>(n));
    for (int i = 0; i < n * n; ++i) {
        P_scaled[static_cast<size_t>(i)] = (n + lambda) * P_tmp[static_cast<size_t>(i)];
    }

    // Add small regularization
    for (int i = 0; i < n; ++i) {
        P_scaled[static_cast<size_t>(i) * static_cast<size_t>(n) + static_cast<size_t>(i)] += 1e-9f;
    }

    std::vector<float> sqrtP(static_cast<size_t>(n) * static_cast<size_t>(n));
    bool chol_success = chol_lower(P_scaled.data(), n, sqrtP.data());

    if (!chol_success) {
        mexErrMsgIdAndTxt("mex_ukf_sigma_points:cholFailed",
                          "Cholesky decomposition failed - P is not positive definite");
    }
    
    // Generate sigma points
    plhs[0] = mxCreateDoubleMatrix(n, n_sig, mxREAL);
    double* sig = mxGetPr(plhs[0]);

    // First sigma point: x
    for (int i = 0; i < n; ++i) {
        sig[i] = static_cast<double>(x_tmp[i]);
    }

    // Next n sigma points: x + sqrtP columns
    for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
            sig[i + (j + 1) * n] = static_cast<double>(x_tmp[i] + sqrtP[i * n + j]);
        }
    }

    // Last n sigma points: x - sqrtP columns
    for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
            sig[i + (n + j + 1) * n] = static_cast<double>(x_tmp[i] - sqrtP[i * n + j]);
        }
    }
    
    // vectors auto-free
}
