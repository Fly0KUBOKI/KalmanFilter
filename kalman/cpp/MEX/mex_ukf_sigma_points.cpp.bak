// mex_ukf_sigma_points.cpp
// MEX wrapper for UKF sigma point generation
//
// Usage:
//   [sig, wm, wc] = mex_ukf_sigma_points(x, P, alpha, beta, kappa)

#include "mex.h"
#include <cmath>
#include <algorithm>
#include "mex_type_conv.hpp"
#include "../Inc/UKF/ukf_sigma_points.hpp"
#include <vector>

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
    
    // Convert inputs to float temporaries
    std::vector<float> x_tmp(static_cast<size_t>(n));
    std::vector<float> P_tmp(static_cast<size_t>(n) * static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(prhs[0], x_tmp.data(), static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(prhs[1], P_tmp.data(), static_cast<size_t>(n) * static_cast<size_t>(n));

    // Allocate output arrays
    int n_sig = 2 * n + 1;
    std::vector<float> sig_tmp(static_cast<size_t>(n) * static_cast<size_t>(n_sig));
    std::vector<float> wm_tmp(static_cast<size_t>(n_sig));
    std::vector<float> wc_tmp(static_cast<size_t>(n_sig));
    
    // Call implementation function (Src/UKF/ukf_sigma_points.cpp)
    ukf::generate_sigma_points_dynamic(
        x_tmp.data(),
        P_tmp.data(),
        n,
        alpha,
        beta,
        kappa,
        sig_tmp.data(),
        wm_tmp.data(),
        wc_tmp.data()
    );
    
    // Convert outputs to MATLAB arrays
    plhs[0] = mxCreateDoubleMatrix(n, n_sig, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(n_sig, 1, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(n_sig, 1, mxREAL);
    
    double* sig = mxGetPr(plhs[0]);
    double* wm = mxGetPr(plhs[1]);
    double* wc = mxGetPr(plhs[2]);
    
    for (int i = 0; i < n_sig; ++i) {
        wm[i] = static_cast<double>(wm_tmp[i]);
        wc[i] = static_cast<double>(wc_tmp[i]);
    }
    
    for (int j = 0; j < n_sig; ++j) {
        for (int i = 0; i < n; ++i) {
            sig[i + j * n] = static_cast<double>(sig_tmp[i + j * n]);
        }
    }
}
