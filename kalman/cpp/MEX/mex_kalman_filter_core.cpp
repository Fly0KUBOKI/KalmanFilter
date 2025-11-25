// mex_kalman_filter_core.cpp
// MATLAB MEX wrapper for kalman_filter_core functions
//
// Usage:
//   K = mex_kalman_filter_core('compute_kalman_gain', P, H, S)

#include "mex.h"
#include "../KF/Core/kalman_filter_core.hpp"
#include <string>
#include <cstring>

// Helper: MATLAB array -> Matrix
template<int R, int C, typename T = float>
static bool matToMatrix(const mxArray* arr, cmath_fx::Matrix<R, C, T>& out) {
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr);
    mwSize cols = mxGetN(arr);
    if (rows != R || cols != C) return false;
    double* pr = mxGetPr(arr);
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            out(i, j) = static_cast<T>(pr[j * rows + i]);
        }
    }
    return true;
}

// Helper: Matrix -> MATLAB array
template<int R, int C, typename T>
static mxArray* matrixToMat(const cmath_fx::Matrix<R, C, T>& M) {
    mxArray* out = mxCreateDoubleMatrix(R, C, mxREAL);
    double* pr = mxGetPr(out);
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            pr[j * R + i] = static_cast<double>(M(i, j));
        }
    }
    return out;
}

// Dynamic size helper
static bool matToMatrixDynamic(const mxArray* arr, int& rows, int& cols, float* data, int max_size) {
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    rows = (int)mxGetM(arr);
    cols = (int)mxGetN(arr);
    if (rows * cols > max_size) return false;
    
    double* pr = mxGetPr(arr);
    for (int j = 0; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            data[i * cols + j] = static_cast<float>(pr[j * rows + i]);
        }
    }
    return true;
}

static mxArray* matrixDynamicToMat(int rows, int cols, const float* data) {
    mxArray* out = mxCreateDoubleMatrix(rows, cols, mxREAL);
    double* pr = mxGetPr(out);
    for (int j = 0; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            pr[j * rows + i] = static_cast<double>(data[i * cols + j]);
        }
    }
    return out;
}

// Handler: compute_kalman_gain
// K = P * H' * inv(S)
// P: NxN, H: MxN, S: MxM -> K: NxM
static void handle_compute_kalman_gain(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 4) {
        mexErrMsgTxt("compute_kalman_gain: Expected 3 arguments (P, H, S)");
    }
    if (nlhs > 1) {
        mexErrMsgTxt("compute_kalman_gain: Only 1 output (K)");
    }

    // Get dimensions
    int N = (int)mxGetM(prhs[1]);  // P is NxN
    int M = (int)mxGetM(prhs[2]);  // H is MxN
    
    // Support common sizes
    if (N == 15 && M == 3) {
        cmath_fx::Matrix<15, 15, float> P;
        cmath_fx::Matrix<3, 15, float> H;
        cmath_fx::Matrix<3, 3, float> S;
        
        if (!matToMatrix(prhs[1], P)) mexErrMsgTxt("Failed to read P");
        if (!matToMatrix(prhs[2], H)) mexErrMsgTxt("Failed to read H");
        if (!matToMatrix(prhs[3], S)) mexErrMsgTxt("Failed to read S");
        
        auto K = kf::KalmanFilterCore::compute_kalman_gain<15, 3, float>(P, H, S);
        plhs[0] = matrixToMat(K);
    } else if (N == 15 && M == 1) {
        cmath_fx::Matrix<15, 15, float> P;
        cmath_fx::Matrix<1, 15, float> H;
        cmath_fx::Matrix<1, 1, float> S;
        
        if (!matToMatrix(prhs[1], P)) mexErrMsgTxt("Failed to read P");
        if (!matToMatrix(prhs[2], H)) mexErrMsgTxt("Failed to read H");
        if (!matToMatrix(prhs[3], S)) mexErrMsgTxt("Failed to read S");
        
        auto K = kf::KalmanFilterCore::compute_kalman_gain<15, 1, float>(P, H, S);
        plhs[0] = matrixToMat(K);
    } else {
        mexErrMsgIdAndTxt("kalman_filter_core:unsupported_size",
                          "Unsupported matrix dimensions N=%d, M=%d", N, M);
    }
}

// Main MEX entry point
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_kalman_filter_core(function_name, ...)");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a string (function name)");
    }
    
    char func_name[256];
    if (mxGetString(prhs[0], func_name, sizeof(func_name)) != 0) {
        mexErrMsgTxt("Failed to read function name");
    }
    
    if (strcmp(func_name, "compute_kalman_gain") == 0) {
        handle_compute_kalman_gain(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgIdAndTxt("kalman_filter_core:unknown_function",
                          "Unknown function: %s", func_name);
    }
}
