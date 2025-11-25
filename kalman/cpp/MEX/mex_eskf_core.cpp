// mex_eskf_core.cpp
// MATLAB MEX wrapper for ESKF core functions
//
// Usage:
//   [p,v,q,ba,bg] = mex_eskf_core('integrate_nominal', p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr)
//   P_new = mex_eskf_core('predict_covariance', P, q, a_meas, ba, w_meas, bg, Q, dt)

#include "mex.h"
#include "../ESKF/eskf_core.hpp"
#include <string>
#include <cstring>

using namespace eskf;

// Helper: MATLAB array -> Vector
template<int R, typename T = float>
static bool matToVector(const mxArray* arr, cmath_fx::Vector<R, T>& out) {
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr);
    mwSize cols = mxGetN(arr);
    if (rows != R || cols != 1) return false;
    double* pr = mxGetPr(arr);
    for (int i = 0; i < R; ++i) {
        out(i, 0) = static_cast<T>(pr[i]);
    }
    return true;
}

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

// Helper: Vector -> MATLAB array
template<int R, typename T>
static mxArray* vectorToMat(const cmath_fx::Vector<R, T>& v) {
    mxArray* out = mxCreateDoubleMatrix(R, 1, mxREAL);
    double* pr = mxGetPr(out);
    for (int i = 0; i < R; ++i) {
        pr[i] = static_cast<double>(v(i, 0));
    }
    return out;
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

// Helper: get scalar
static float getScalar(const mxArray* arr) {
    if (!mxIsDouble(arr) || mxIsComplex(arr) || mxGetNumberOfElements(arr) != 1) {
        mexErrMsgTxt("Expected scalar double");
    }
    return static_cast<float>(mxGetScalar(arr));
}

// Handler: integrate_nominal
static void handle_integrate_nominal(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 12) {
        mexErrMsgTxt("integrate_nominal: Expected 11 arguments after function name");
    }
    if (nlhs > 5) {
        mexErrMsgTxt("integrate_nominal: Maximum 5 outputs (p, v, q, ba, bg)");
    }
    
    Vector3 p, v, ba, bg, a_meas, w_meas, g, gyro_thr, accel_thr;
    Vector4 q;
    if (!matToVector(prhs[1], p)) mexErrMsgTxt("Failed to read p");
    if (!matToVector(prhs[2], v)) mexErrMsgTxt("Failed to read v");
    if (!matToVector(prhs[3], q)) mexErrMsgTxt("Failed to read q");
    if (!matToVector(prhs[4], ba)) mexErrMsgTxt("Failed to read ba");
    if (!matToVector(prhs[5], bg)) mexErrMsgTxt("Failed to read bg");
    if (!matToVector(prhs[6], a_meas)) mexErrMsgTxt("Failed to read a_meas");
    if (!matToVector(prhs[7], w_meas)) mexErrMsgTxt("Failed to read w_meas");
    float dt = getScalar(prhs[8]);
    if (!matToVector(prhs[9], g)) mexErrMsgTxt("Failed to read g");
    if (!matToVector(prhs[10], gyro_thr)) mexErrMsgTxt("Failed to read gyro_thr");
    if (!matToVector(prhs[11], accel_thr)) mexErrMsgTxt("Failed to read accel_thr");
    
    ESKFCore::integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr);
    
    plhs[0] = vectorToMat(p);
    if (nlhs > 1) plhs[1] = vectorToMat(v);
    if (nlhs > 2) plhs[2] = vectorToMat(q);
    if (nlhs > 3) plhs[3] = vectorToMat(ba);
    if (nlhs > 4) plhs[4] = vectorToMat(bg);
}

// Handler: predict_covariance
static void handle_predict_covariance(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 9) {
        mexErrMsgTxt("predict_covariance: Expected 8 arguments after function name");
    }
    if (nlhs > 1) {
        mexErrMsgTxt("predict_covariance: Maximum 1 output (P_new)");
    }
    
    Matrix15x15 P, Q, P_new;
    Vector4 q;
    Vector3 a_meas, ba, w_meas, bg;
    
    if (!matToMatrix(prhs[1], P)) mexErrMsgTxt("Failed to read P");
    if (!matToVector(prhs[2], q)) mexErrMsgTxt("Failed to read q");
    if (!matToVector(prhs[3], a_meas)) mexErrMsgTxt("Failed to read a_meas");
    if (!matToVector(prhs[4], ba)) mexErrMsgTxt("Failed to read ba");
    if (!matToVector(prhs[5], w_meas)) mexErrMsgTxt("Failed to read w_meas");
    if (!matToVector(prhs[6], bg)) mexErrMsgTxt("Failed to read bg");
    if (!matToMatrix(prhs[7], Q)) mexErrMsgTxt("Failed to read Q");
    float dt = getScalar(prhs[8]);
    
    ESKFCore::predict_covariance(P, q, a_meas, ba, w_meas, bg, Q, dt, P_new);
    
    plhs[0] = matrixToMat(P_new);
}

// Main MEX entry point
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_eskf_core(function_name, ...)");
    }
    
    // Get function name
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a string (function name)");
    }
    char func_name[256];
    if (mxGetString(prhs[0], func_name, sizeof(func_name)) != 0) {
        mexErrMsgTxt("Failed to read function name");
    }
    
    // Dispatch to appropriate handler
    if (strcmp(func_name, "integrate_nominal") == 0) {
        handle_integrate_nominal(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "predict_covariance") == 0) {
        handle_predict_covariance(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgIdAndTxt("mex_eskf_core:unknownFunction",
                          "Unknown function: %s", func_name);
    }
}
