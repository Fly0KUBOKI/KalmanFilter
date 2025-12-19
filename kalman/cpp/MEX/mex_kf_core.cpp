// mex_kf_core.cpp
// MATLAB MEX wrapper for kalman_filter_core::predict_step (nomalloc)

#include "mex.h"
#include "../KF/Core/kalman_filter_core.hpp"
#include "../Common/Math/fixed_matrix.hpp"

using cm = cmath_fx::FixedMatrix;

static bool matToFixed(const mxArray* arr, cm& out) {
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); mwSize cols = mxGetN(arr);
    if (rows > cmath_fx::MAX_N || cols > cmath_fx::MAX_N) return false;
    double* pr = mxGetPr(arr);
    out.resize((int)rows, (int)cols);
    for (mwSize j = 0; j < cols; ++j) {
        for (mwSize i = 0; i < rows; ++i) {
            out((int)i,(int)j) = static_cast<float>(pr[j*rows + i]);
        }
    }
    return true;
}

static mxArray* fixedToMat(const cm& M) {
    mwSize rows = (mwSize)M.rows;
    mwSize cols = (mwSize)M.cols;
    mxArray* out = mxCreateDoubleMatrix(rows, cols, mxREAL);
    double* pr = mxGetPr(out);
    for (mwSize j = 0; j < cols; ++j) for (mwSize i = 0; i < rows; ++i) pr[j*rows + i] = static_cast<double>(M((int)i,(int)j));
    return out;
}

// Usage from MATLAB:
// mex_kf_core(P, q, a_meas, ba, w_meas, bg, Q, dt)
// Returns P_out as single output
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs != 8) {
        mexErrMsgTxt("Usage: mex_kf_core(P, q, a_meas, ba, w_meas, bg, Q, dt)");
        return;
    }

    cm P, q, a_meas, ba, w_meas, bg, Q;
    if (!matToFixed(prhs[0], P)) mexErrMsgTxt("Failed to read P");
    if (!matToFixed(prhs[1], q)) mexErrMsgTxt("Failed to read q");
    if (!matToFixed(prhs[2], a_meas)) mexErrMsgTxt("Failed to read a_meas");
    if (!matToFixed(prhs[3], ba)) mexErrMsgTxt("Failed to read ba");
    if (!matToFixed(prhs[4], w_meas)) mexErrMsgTxt("Failed to read w_meas");
    if (!matToFixed(prhs[5], bg)) mexErrMsgTxt("Failed to read bg");
    if (!matToFixed(prhs[6], Q)) mexErrMsgTxt("Failed to read Q");
    if (!mxIsDouble(prhs[7]) || mxIsComplex(prhs[7]) || mxGetNumberOfElements(prhs[7])!=1) mexErrMsgTxt("dt must be scalar double");
    float dt = static_cast<float>(mxGetScalar(prhs[7]));

    // check sizes
    if (P.rows != P.cols) mexErrMsgTxt("P must be square");
    if (Q.rows != Q.cols) mexErrMsgTxt("Q must be square");
    if (P.rows != Q.rows) mexErrMsgTxt("P and Q size mismatch");

    cm P_out;
    try {
        P_out = kf::KalmanFilterCore::predict_step(P, q, a_meas, ba, w_meas, bg, Q, dt);
    } catch (...) {
        mexErrMsgTxt("Exception during predict_step");
    }

    plhs[0] = fixedToMat(P_out);
}
