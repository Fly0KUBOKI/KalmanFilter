// mex_ukf.cpp (nomalloc simplified)
// Simplified UKF update wrapper using FixedMatrix. For testing we use linear H.

#include "mex.h"
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include "../Inc/EKF/ekf_linear_update.hpp"
#include "mex_type_conv.hpp"
#include <vector>

using cm = cmath_fx::FixedMatrix;

static bool matToFixed(const mxArray* arr, cm& out) {
    if (!arr) return false;
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); mwSize cols = mxGetN(arr);
    if (rows > cmath_fx::MAX_N || cols > cmath_fx::MAX_N) return false;
    size_t ne = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    std::vector<float> tmp(ne);
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), ne);
    out.resize((int)rows, (int)cols);
    for (mwSize j = 0; j < cols; ++j) for (mwSize i = 0; i < rows; ++i) out((int)i,(int)j) = tmp[j*rows + i];
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

// Usage: [x_upd,P_upd] = mex_ukf(x,P,z,H,R)
// This simplified wrapper performs a UKF-like update assuming linear H (so reduces to KF update).
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs != 5) mexErrMsgTxt("Usage: [x_upd,P_upd] = mex_ukf(x,P,z,H,R)");
    cm x,P,z,H,R;
    if (!matToFixed(prhs[0], x)) mexErrMsgTxt("Failed to read x");
    if (!matToFixed(prhs[1], P)) mexErrMsgTxt("Failed to read P");
    if (!matToFixed(prhs[2], z)) mexErrMsgTxt("Failed to read z");
    if (!matToFixed(prhs[3], H)) mexErrMsgTxt("Failed to read H");
    if (!matToFixed(prhs[4], R)) mexErrMsgTxt("Failed to read R");

    // For linear observation the UKF update equals KF update — use EKF linear update
    // Implementation: Src/EKF/ekf_linear_update.cpp
    cm x_upd, P_upd;
    ekf::linear::ekf_linear_update(x, P, z, H, R, x_upd, P_upd);

    plhs[0] = fixedToMat(x_upd);
    plhs[1] = fixedToMat(P_upd);
}

