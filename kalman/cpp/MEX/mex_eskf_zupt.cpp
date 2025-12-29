// mex_eskf_zupt.cpp
// MEX wrapper for ZUPT (Zero Velocity Update)
// Implements: velocity zeroing and covariance update using Kalman filter update
// Based on meukf_core.cpp::update_zupt() implementation

#include "mex.h"
#include "mex_type_conv.hpp"
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include "../Inc/ESKF/eskf_core.hpp"
#include <string>
#include <cmath>
#include <vector>

using namespace cmath_fx;
using namespace eskf;

// Type aliases (use ESKF types)
using Vector3 = eskf::Vector3;
using Vector4 = eskf::Vector4;
using Vector15 = eskf::Vector15;
using Matrix3x3 = eskf::Matrix3x3;
using Matrix15x3 = cmath_fx::Matrix<15, 3, float>;
using Matrix15x15 = eskf::Matrix15x15;

// Helper: MATLAB array -> Vector
template<int R>
static bool matToVector(const mxArray* arr, Vector<R, float>& out) {
    if (!arr) return false;
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); mwSize cols = mxGetN(arr);
    if (rows != R || cols != 1) return false;
    std::vector<float> tmp(static_cast<size_t>(R));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R));
    for (int i = 0; i < R; ++i) out(i, 0) = tmp[i];
    return true;
}

// Helper: MATLAB array -> Matrix
template<int R, int C>
static bool matToMatrix(const mxArray* arr, Matrix<R, C, float>& out) {
    if (!arr) return false;
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); mwSize cols = mxGetN(arr);
    if (rows != R || cols != C) return false;
    std::vector<float> tmp(static_cast<size_t>(R) * static_cast<size_t>(C));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R) * static_cast<size_t>(C));
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            out(i, j) = tmp[static_cast<size_t>(j) * static_cast<size_t>(R) + static_cast<size_t>(i)];
        }
    }
    return true;
}

// Helper: Vector -> MATLAB array
template<int R>
static mxArray* vectorToMat(const Vector<R, float>& v) {
    mxArray* out = mxCreateDoubleMatrix(R, 1, mxREAL);
    double* pr = mxGetPr(out);
    for (int i = 0; i < R; ++i) pr[i] = static_cast<double>(v(i, 0));
    return out;
}

// Helper: Matrix -> MATLAB array
template<int R, int C>
static mxArray* matrixToMat(const Matrix<R, C, float>& M) {
    mxArray* out = mxCreateDoubleMatrix(R, C, mxREAL);
    double* pr = mxGetPr(out);
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            pr[j * R + i] = static_cast<double>(M(i, j));
        }
    }
    return out;
}

// invert3x3関数はSrc/Common/Math/math_utils.hppに移動済み

// Main ZUPT update function
// Input: v_in (3x1), P_in (15x15)
// Output: v_out (3x1), P_out (15x15)
// Based on meukf_core.cpp::update_zupt() but simplified to work with v and P only
static void handle_update(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 3) mexErrMsgTxt("update requires v_in and P_in");
    
    // Parse inputs
    Vector3 v_in;
    if (!matToVector<3>(prhs[1], v_in)) mexErrMsgTxt("v_in must be a 3x1 double vector");
    
    Matrix15x15 P_in;
    if (!matToMatrix<15, 15>(prhs[2], P_in)) mexErrMsgTxt("P_in must be a 15x15 double matrix");
    
    // ZUPT更新（実装はSrc/ESKF/eskf_core.cppに移動）
    Vector3 v_out;
    Matrix15x15 P_out;
    eskf::ESKFCore::update_zupt(v_in, P_in, v_out, P_out);
    
    // Output
    plhs[0] = vectorToMat<3>(v_out);
    plhs[1] = matrixToMat<15, 15>(P_out);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs < 1) mexErrMsgTxt("Usage: [v_out, P_out] = mex_eskf_zupt('update', v_in, P_in)");
    
    char cmd[128];
    if (mxGetString(prhs[0], cmd, sizeof(cmd))) mexErrMsgTxt("Command must be string");
    
    std::string cmdstr(cmd);
    
    if (cmdstr == "update") {
        if (nlhs < 2) mexErrMsgTxt("update requires 2 outputs: v_out, P_out");
        handle_update(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgTxt("Unknown command. Use 'update'");
    }
}

